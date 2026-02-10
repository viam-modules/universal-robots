#include "ur_arm.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <future>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include <json/json.h>

#include <grpcpp/server_context.h>

#include <boost/format.hpp>
#include <boost/io/ostream_joiner.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/combine.hpp>

#include <viam/sdk/components/component.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include <third_party/trajectories/Path.h>
#include <third_party/trajectories/Trajectory.h>

#include <viam/sdk/rpc/grpc_context_observer.hpp>
#include <viam/trajex/totg/totg.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/hertz.hpp>

#include "ur_arm_state.hpp"
#include "utils.hpp"

// this chunk of code uses the rust FFI to handle the spatialmath calculations to turn a UR vector to a pose or a pose to a UR vector
extern "C" void* quaternion_from_orientation_vector(void* ov);
extern "C" void* quaternion_from_axis_angle(double x, double y, double z, double theta);
extern "C" void free_quaternion_memory(void* q);

extern "C" void* new_orientation_vector(double ox, double oy, double oz, double theta);
extern "C" void* orientation_vector_from_quaternion(void* q);
extern "C" void free_orientation_vector_memory(void* ov);

extern "C" double* orientation_vector_get_components(void* ov);
extern "C" void free_orientation_vector_components(double* ds);

extern "C" double* axis_angle_from_quaternion(void* q);
extern "C" void free_axis_angles_memory(void* aa);

namespace {

constexpr double k_min_timestep_sec = 1e-2;  // determined experimentally, the arm appears to error when given timesteps ~2e-5 and lower

constexpr double k_min_duration_secs = 0.1;
constexpr double k_max_duration_secs = 3600.0;
constexpr double k_min_sampling_freq_hz = 1.0;
constexpr double k_max_sampling_freq_hz = 500.0;

// Convert Eigen waypoint list to xt::xarray for trajex/totg
xt::xarray<double> eigen_waypoints_to_xarray(const std::list<Eigen::VectorXd>& waypoints) {
    if (waypoints.empty()) {
        return xt::xarray<double>::from_shape({0, 0});
    }

    const size_t num_waypoints = waypoints.size();
    const size_t dof = static_cast<size_t>(waypoints.front().size());

    xt::xarray<double> result = xt::zeros<double>({num_waypoints, dof});

    size_t i = 0;
    for (const auto& waypoint : waypoints) {
        for (size_t j = 0; j < dof; ++j) {
            result(i, j) = waypoint[static_cast<Eigen::Index>(j)];
        }
        ++i;
    }

    return result;
}

// Type aliases for smart pointers managing FFI memory
using unique_orientation_vector = std::unique_ptr<void, decltype(&free_orientation_vector_memory)>;
using unique_quaternion = std::unique_ptr<void, decltype(&free_quaternion_memory)>;
using unique_axis_angles = std::unique_ptr<double[], decltype(&free_axis_angles_memory)>;
using unique_orientation_components = std::unique_ptr<double[], decltype(&free_orientation_vector_components)>;

pose ur_vector_to_pose(urcl::vector6d_t vec) {
    const double norm = std::hypot(vec[3], vec[4], vec[5]);
    if (std::isnan(norm) || (norm == 0)) {
        throw std::invalid_argument("Cannot normalize with NaN or zero norm");
    }

    auto q = unique_quaternion(quaternion_from_axis_angle(vec[3] / norm, vec[4] / norm, vec[5] / norm, norm), &free_quaternion_memory);

    auto ov = unique_orientation_vector(orientation_vector_from_quaternion(q.get()), &free_orientation_vector_memory);

    auto components = unique_orientation_components(orientation_vector_get_components(ov.get()), &free_orientation_vector_components);

    auto position = coordinates{1000 * vec[0], 1000 * vec[1], 1000 * vec[2]};
    auto orientation = pose_orientation{components[0], components[1], components[2]};
    auto theta = radians_to_degrees(components[3]);

    return {position, orientation, theta};
}

urcl::vector6d_t pose_to_ur_vector(const pose& p) {
    if (!std::isfinite(p.theta)) {
        throw std::invalid_argument("pose_to_ur_vector: theta is infinite or NaN");
    }

    urcl::vector6d_t v{};

    // convert millimeters to meters
    v[0] = p.coordinates.x / 1000.0;
    v[1] = p.coordinates.y / 1000.0;
    v[2] = p.coordinates.z / 1000.0;

    const double theta_rad = degrees_to_radians(p.theta);

    // If angle ~ 0, UR expects the axis-angle "vector" to be zero.
    constexpr double k_angle_epsilon = 1e-12;
    if (std::abs(theta_rad) < k_angle_epsilon) {
        v[3] = v[4] = v[5] = 0.0;
        return v;
    }

    // convert viam's orientation to axis angles which is what universal robots use
    // viam orientation vector -> quaternion -> axis angle -> universal robots orientation vector
    // create an orientation vector to use
    auto ov = unique_orientation_vector(new_orientation_vector(p.orientation.o_x, p.orientation.o_y, p.orientation.o_z, theta_rad),
                                        &free_orientation_vector_memory);
    if (!ov) {
        throw std::runtime_error("pose_to_ur_vector: failed to create orientation vector");
    }

    // create quaternion from the orientation vector
    auto q = unique_quaternion(quaternion_from_orientation_vector(ov.get()), &free_quaternion_memory);
    if (!q) {
        throw std::runtime_error("pose_to_ur_vector: failed to create quaternion");
    }

    // convert the quaternion to axis angles
    auto aa = unique_axis_angles(axis_angle_from_quaternion(q.get()), &free_axis_angles_memory);
    if (!aa) {
        throw std::runtime_error("pose_to_ur_vector: failed to compute axis-angle");
    }

    const double ax = aa[0];
    const double ay = aa[1];
    const double az = aa[2];
    const double th = aa[3];

    // robustness to tiny axes/angles
    const double n = std::hypot(ax, ay, az);
    constexpr double k_axis_epsilon = 1e-8;
    if (n < k_angle_epsilon || std::abs(th) < k_axis_epsilon) {
        v[3] = v[4] = v[5] = 0.0;
    } else {
        // universal robots orientation vector: r = axis * theta
        v[3] = ax * th;
        v[4] = ay * th;
        v[5] = az * th;
    }

    return v;
}

std::vector<std::string> validate_config_(const ResourceConfig& cfg) {
    if (!find_config_attribute<std::string>(cfg, "host")) {
        throw std::invalid_argument("attribute `host` is required");
    }

    parse_and_validate_joint_limits(cfg, "speed_degs_per_sec");
    parse_and_validate_joint_limits(cfg, "acceleration_degs_per_sec2");

    if (cfg.attributes().contains("max_speed_degs_per_sec")) {
        parse_and_validate_joint_limits(cfg, "max_speed_degs_per_sec");
    }

    if (cfg.attributes().contains("max_acceleration_degs_per_sec2")) {
        parse_and_validate_joint_limits(cfg, "max_acceleration_degs_per_sec2");
    }

    auto max_duration = find_config_attribute<double>(cfg, "max_trajectory_duration_secs");
    if (max_duration && (*max_duration < k_min_duration_secs || *max_duration > k_max_duration_secs)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `max_trajectory_duration_secs` should be between %1% and %2%, it is: %3% seconds") %
                       k_min_duration_secs % k_max_duration_secs % *max_duration));
    }

    auto sampling_freq = find_config_attribute<double>(cfg, "trajectory_sampling_freq_hz");
    if (sampling_freq && (*sampling_freq < k_min_sampling_freq_hz || *sampling_freq > k_max_sampling_freq_hz)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `trajectory_sampling_freq_hz` should be between %1% and %2%, it is: %3% Hz") %
                       k_min_sampling_freq_hz % k_max_sampling_freq_hz % *sampling_freq));
    }

    auto threshold = find_config_attribute<double>(cfg, "reject_move_request_threshold_deg");
    constexpr double k_min_threshold = 0.0;
    constexpr double k_max_threshold = 360.0;
    if (threshold && (*threshold < k_min_threshold || *threshold > k_max_threshold)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `reject_move_request_threshold_deg` should be between %1% and %2%, it is: %3% degrees") %
                       k_min_threshold % k_max_threshold % *threshold));
    }

    auto frequency = find_config_attribute<double>(cfg, "robot_control_freq_hz");
    constexpr double k_max_frequency = 1000.;
    if (frequency && (*frequency <= 0. || *frequency >= k_max_frequency)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `robot_control_freq_hz` should be a positive number less than %1%, it is: %2% Hz") %
                       k_max_frequency % *frequency));
    }

    auto path_tolerance_deg = find_config_attribute<double>(cfg, "path_tolerance_delta_deg");
    if (path_tolerance_deg && (*path_tolerance_deg <= 0 || *path_tolerance_deg > 12)) {
        throw std::invalid_argument(boost::str(
            boost::format("attribute `path_tolerance_delta_deg` must be > 0 and <= 12, it is: %1% degrees") % *path_tolerance_deg));
    }

    auto colinearization_ratio = find_config_attribute<double>(cfg, "path_colinearization_ratio");
    if (colinearization_ratio && (*colinearization_ratio < 0 || *colinearization_ratio > 2)) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `path_colinearization_ratio` must be >= 0 and <= 2, it is: %1%") % *colinearization_ratio));
    }

    // Validate telemetry_output_path is a string if present
    const auto telemetry_output_path = find_config_attribute<std::string>(cfg, "telemetry_output_path");

    // Also validate that `csv_output_path`, if present, is a string.
    //
    // TODO(RSDK-12929): When `csv_output_path` is removed, actively reject it (don't ignore it).
    const auto csv_output_path = find_config_attribute<std::string>(cfg, "csv_output_path");
    if (csv_output_path) {
        if (telemetry_output_path) {
            throw std::invalid_argument("Only one of `csv_output_path` (deprecated) or `telemetry_output_path` may be specified");
        }
    }

    // Validate telemetry_output_path_append_traceid: accepts bool (backward compat) or template string containing {trace_id}
    {
        const auto it = cfg.attributes().find("telemetry_output_path_append_traceid");
        if (it != cfg.attributes().end()) {
            if (const auto* s = it->second.get<std::string>()) {
                if (s->find("{trace_id}") == std::string::npos) {
                    throw std::invalid_argument("`telemetry_output_path_append_traceid` template string must contain `{trace_id}`");
                }
            } else if (!it->second.get<bool>()) {
                throw std::invalid_argument(
                    "`telemetry_output_path_append_traceid` must be a boolean or a template string containing `{trace_id}`");
            }
        }
    }

    return {};
}

template <typename Callable>
auto make_scope_guard(Callable&& cleanup) {
    struct guard {
       public:
        explicit guard(Callable&& cleanup) : cleanup_(std::move(cleanup)) {}
        void deactivate() {
            cleanup_ = [] {};
        }
        ~guard() {
            cleanup_();
        }

       private:
        std::function<void()> cleanup_;
    };
    return guard{std::forward<Callable>(cleanup)};
}

}  // namespace

void write_trajectory_to_file(const std::string& filepath, const std::vector<trajectory_sample_point>& samples) {
    std::ofstream of(filepath);
    of << "t(s),j0,j1,j2,j3,j4,j5,v0,v1,v2,v3,v4,v5\n";
    float time_traj = 0;
    for (size_t i = 0; i < samples.size(); i++) {
        time_traj += samples[i].timestep;
        of << time_traj;
        for (size_t j = 0; j < samples[i].p.size(); j++) {
            of << "," << samples[i].p[j];
        }
        for (size_t j = 0; j < samples[i].v.size(); j++) {
            of << "," << samples[i].v[j];
        }
        of << "\n";
    }

    of.close();
}

void write_pose_to_file(const std::string& filepath, const pose_sample& sample) {
    std::ofstream of(filepath);
    of << "x,y,z,rx,ry,rz\n";
    of << sample.p[0];
    for (size_t i = 1; i < sample.p.size(); i++) {
        of << "," << sample.p[i];
    }
    of << "\n";
    of.close();
}

void write_waypoints_to_csv(const std::string& filepath, const std::list<Eigen::VectorXd>& waypoints) {
    unsigned i;
    std::ofstream of(filepath);
    for (const auto& vec : waypoints) {
        i = 0;
        for (const auto& n : vec) {
            i++;
            if (i == vec.size()) {
                of << n;
            } else {
                of << n << ",";
            }
        }
        of << "\n";
    }
    of.close();
}

std::string waypoints_filename(const std::string& path, const std::string& resource_name, const std::string& unix_time) {
    constexpr char kWaypointsCsvNameTemplate[] = "/%1%_%2%_waypoints.csv";
    auto fmt = boost::format(path + kWaypointsCsvNameTemplate);
    return (fmt % unix_time % resource_name).str();
}

std::string trajectory_filename(const std::string& path, const std::string& resource_name, const std::string& unix_time) {
    constexpr char kTrajectoryCsvNameTemplate[] = "/%1%_%2%_trajectory.csv";
    auto fmt = boost::format(path + kTrajectoryCsvNameTemplate);
    return (fmt % unix_time % resource_name).str();
}

std::string move_to_position_pose_filename(const std::string& path, const std::string& resource_name, const std::string& unix_time) {
    constexpr char kPoseCsvNameTemplate[] = "/%1%_%2%_move_to_position_pose.csv";
    auto fmt = boost::format(path + kPoseCsvNameTemplate);
    return (fmt % unix_time % resource_name).str();
}

std::string arm_joint_positions_filename(const std::string& path, const std::string& resource_name, const std::string& unix_time) {
    constexpr char kArmJointPositionsCsvNameTemplate[] = "/%1%_%2%_arm_joint_positions.csv";
    auto fmt = boost::format(path + kArmJointPositionsCsvNameTemplate);
    return (fmt % unix_time % resource_name).str();
}

std::string failed_trajectory_filename(const std::string& path, const std::string& resource_name, const std::string& unix_time) {
    constexpr char kFailedTrajectoryJsonNameTemplate[] = "/%1%_%2%_failed_trajectory.json";
    auto fmt = boost::format(path + kFailedTrajectoryJsonNameTemplate);
    return (fmt % unix_time % resource_name).str();
}

std::string unix_time_iso8601() {
    namespace chrono = std::chrono;
    std::stringstream stream;

    const auto now = chrono::system_clock::now();
    const auto seconds_part = chrono::duration_cast<chrono::seconds>(now.time_since_epoch());
    const auto tt = chrono::system_clock::to_time_t(chrono::system_clock::time_point{seconds_part});
    const auto delta_us = chrono::duration_cast<chrono::microseconds>(now.time_since_epoch() - seconds_part);

    struct tm buf;
    auto* ret = gmtime_r(&tt, &buf);
    if (ret == nullptr) {
        throw std::runtime_error("failed to convert time to iso8601");
    }
    stream << std::put_time(&buf, "%FT%T");
    stream << "." << std::setw(6) << std::setfill('0') << delta_us.count() << "Z";

    return stream.str();
}

std::string serialize_failed_trajectory_to_json(const std::list<Eigen::VectorXd>& waypoints,
                                                const Eigen::VectorXd& max_velocity_vec,
                                                const Eigen::VectorXd& max_acceleration_vec,
                                                double path_tolerance_delta_rads,
                                                const std::optional<double>& path_colinearization_ratio) {
    namespace json = Json;

    json::Value root;
    root["timestamp"] = unix_time_iso8601();
    root["path_tolerance_delta_rads"] = path_tolerance_delta_rads;

    if (path_colinearization_ratio) {
        root["path_colinearization_ratio"] = *path_colinearization_ratio;
    }

    json::Value max_vel_array(json::arrayValue);
    std::ranges::for_each(max_velocity_vec, [&](double item) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::max_digits10) << item;
        max_vel_array.append(ss.str());
    });
    root["max_velocity_vec_rads_per_sec"] = std::move(max_vel_array);

    json::Value max_acc_array(json::arrayValue);
    std::ranges::for_each(max_acceleration_vec, [&](double item) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<double>::max_digits10) << item;
        max_acc_array.append(ss.str());
    });
    root["max_acceleration_vec_rads_per_sec2"] = std::move(max_acc_array);

    json::Value waypoints_array(json::arrayValue);
    waypoints_array.resize((json::ArrayIndex)waypoints.size());
    for (const auto& [waypoint, json_waypoint] : boost::combine(waypoints, waypoints_array)) {
        std::ranges::for_each(waypoint, [&](double item) {
            std::stringstream ss;
            ss << std::setprecision(std::numeric_limits<double>::max_digits10) << item;
            // NOLINTBEGIN(clang-analyzer-core.CallAndMessage): json_waypoint is initialized as a nullValue and then lazily initialized
            json_waypoint.append(ss.str());
            // NOLINTEND(clang-analyzer-core.CallAndMessage)
        });
    }
    root["waypoints_rads"] = std::move(waypoints_array);

    json::StreamWriterBuilder writer;
    writer["indentation"] = " ";

    return json::writeString(writer, root);
}

const ModelFamily& URArm::model_family() {
    // TODO: If ModelFamily had a constexpr constructor, we wouldn't need
    // this function at all and could just inline it into the class definition.
    static const auto family = ModelFamily{"viam", "universal-robots"};
    return family;
}

Model URArm::model(std::string model_name) {
    return {model_family(), std::move(model_name)};
}

std::vector<std::shared_ptr<ModelRegistration>> URArm::create_model_registrations() {
    const auto model_strings = {
        "ur3e",  //
        "ur5e",  //
        "ur7e",  //
        "ur20"   //
    };

    const auto arm = API::get<Arm>();
    const auto registration_factory = [&](auto m) {
        const auto model = URArm::model(m);
        return std::make_shared<ModelRegistration>(
            arm,
            model,
            // NOLINTNEXTLINE(performance-unnecessary-value-param): Signature is fixed by ModelRegistration.
            [model](auto deps, auto config) { return std::make_unique<URArm>(model, deps, config); },
            [](auto const& config) { return validate_config_(config); });
    };

    auto registrations = model_strings | boost::adaptors::transformed(registration_factory);
    return {std::make_move_iterator(begin(registrations)), std::make_move_iterator(end(registrations))};
}

URArm::URArm(Model model, const Dependencies& deps, const ResourceConfig& cfg) : Arm(cfg.name()), model_(std::move(model)) {
    VIAM_SDK_LOG(info) << "instantiating URArm driver for arm model: " << model_.to_string();
    arm_name_to_model_parts_ = {
        {"ur5e", {"base_link", "ee_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}},
        {"ur20", {"base_link", "wrist_3_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}},
    };
    const std::unique_lock wlock(config_mutex_);
    // TODO: prevent multiple calls to configure_logger
    configure_logger(cfg);
    configure_(wlock, deps, cfg);
}

void URArm::configure_(const std::unique_lock<std::shared_mutex>& lock, const Dependencies&, const ResourceConfig& cfg) {
    if (current_state_) {
        throw std::logic_error("URArm::configure_ was called for a currently configured instance");
    }

    // check model type is valid, map to ur_client data type
    // https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/bff7bf2e2a85c17fa3f88adda241763040596ff1/include/ur_client_library/ur/datatypes.h#L204
    const std::string configured_model_type = [&] {
        if (model_ == URArm::model("ur3e")) {
            return "UR3";
        } else if ((model_ == URArm::model("ur5e")) || (model_ == URArm::model("ur7e"))) {
            return "UR5";
        } else if (model_ == URArm::model("ur20")) {
            return "UR20";
        } else {
            std::ostringstream buffer;
            buffer << "unsupported model type: `" << model_.to_string() << "`";
            throw std::invalid_argument(buffer.str());
        }
    }();

    // If we fail to make it through the startup sequence, execute the shutdown code. The
    // shutdown code must be prepared to be called from any intermediate state that this
    // function may have constructed due to partial execution.
    auto failure_handler = make_scope_guard([&] {
        VIAM_SDK_LOG(warn) << "URArm startup failed - shutting down";
        shutdown_(lock);
    });

    VIAM_SDK_LOG(debug) << "URArm starting up";
    current_state_ = state_::create(configured_model_type, name(), cfg, ports_);

    VIAM_SDK_LOG(info) << "URArm startup complete";
    failure_handler.deactivate();
}

template <template <typename> typename lock_type>
void URArm::check_configured_(const lock_type<std::shared_mutex>&) {
    if (!current_state_) {
        std::ostringstream buffer;
        buffer << "Arm is not currently configured; reconfiguration likely failed";
        throw std::runtime_error(buffer.str());
    }
}

void URArm::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    const std::unique_lock wlock{config_mutex_};
    VIAM_SDK_LOG(warn) << "Reconfigure called: shutting down current state";
    shutdown_(wlock);
    VIAM_SDK_LOG(warn) << "Reconfigure called: configuring new state";
    configure_(wlock, deps, cfg);
    VIAM_SDK_LOG(info) << "Reconfigure completed OK";
}

std::vector<double> URArm::get_joint_positions(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    auto joint_rads = get_joint_positions_rad_(rlock);
    auto joint_position_degree = joint_rads | boost::adaptors::transformed(radians_to_degrees<const double&>);
    return {std::begin(joint_position_degree), std::end(joint_position_degree)};
}

vector6d_t URArm::get_joint_positions_rad_(const std::shared_lock<std::shared_mutex>&) {
    return current_state_->read_joint_positions();
}

void URArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    auto next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), boost::numeric_cast<Eigen::Index>(positions.size()));
    auto next_waypoint_rad = degrees_to_radians(std::move(next_waypoint_deg));
    std::list<Eigen::VectorXd> waypoints;
    waypoints.emplace_back(std::move(next_waypoint_rad));

    const auto unix_time = unix_time_iso8601();
    const auto filename = waypoints_filename(current_state_->telemetry_output_path(), current_state_->resource_name(), unix_time);

    write_waypoints_to_csv(filename, waypoints);

    // move will throw if an error occurs
    move_joint_space_(std::move(rlock), std::move(waypoints), MoveOptions{}, unix_time);
}

void URArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                         const MoveOptions& options,
                                         const viam::sdk::ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    if (positions.empty()) {
        return;
    }

    // Convert from vector<vector<double>> (degrees) to list<Eigen::VectorXd> (radians)
    constexpr auto to_radians = [](const auto& position) {
        auto deg = Eigen::VectorXd::Map(position.data(), boost::numeric_cast<Eigen::Index>(position.size()));
        return degrees_to_radians(std::move(deg)).eval();
    };
    auto waypoints_range = positions | std::views::transform(to_radians);
    std::list<Eigen::VectorXd> waypoints(std::ranges::begin(waypoints_range), std::ranges::end(waypoints_range));

    const auto unix_time = unix_time_iso8601();
    const auto filename = waypoints_filename(current_state_->telemetry_output_path(), current_state_->resource_name(), unix_time);
    write_waypoints_to_csv(filename, waypoints);

    move_joint_space_(std::move(rlock), std::move(waypoints), options, unix_time);
}

pose URArm::get_end_position(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    return ur_vector_to_pose(current_state_->read_tcp_pose());
}

bool URArm::is_moving() {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    return current_state_->is_moving();
}

void URArm::move_to_position(const pose& p, const ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    const auto unix_time = unix_time_iso8601();
    move_tool_space_(std::move(rlock), p, unix_time);
}

URArm::KinematicsData URArm::get_kinematics(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    // The `Model` class absurdly lacks accessors
    const std::string model_string = [&] {
        if (model_ == model("ur3e")) {
            return "ur3e";
        } else if (model_ == model("ur5e")) {
            return "ur5e";
        } else if (model_ == model("ur7e")) {
            return "ur7e";
        } else if (model_ == model("ur20")) {
            return "ur20";
        }
        throw std::runtime_error(str(boost::format("no kinematics file known for model '%1%'") % model_.to_string()));
    }();

    constexpr char kSvaFileTemplate[] = "kinematics/%1%.json";
    const auto sva_file_path = current_state_->resource_root() / str(boost::format(kSvaFileTemplate) % model_string);

    // Open the file in binary mode
    std::ifstream sva_file(sva_file_path, std::ios::binary);
    if (!sva_file) {
        throw std::runtime_error(str(boost::format("unable to open kinematics file '%1%'") % sva_file_path));
    }

    // Read the entire file into a vector without computing size ahead of time
    std::vector<char> temp_bytes(std::istreambuf_iterator<char>(sva_file), {});
    if (sva_file.bad()) {
        throw std::runtime_error(str(boost::format("error reading kinematics file '%1%'") % sva_file_path));
    }

    // Convert to unsigned char vector
    return KinematicsDataSVA({temp_bytes.begin(), temp_bytes.end()});
}

// Unknown arm models should return an empty map. Arm models that do not have all expected parts in the map should return as much as they
// have
std::map<std::string, mesh> URArm::get_3d_models(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    const auto model_name = model_.model_name();

    const auto where = arm_name_to_model_parts_.find(model_name);
    if (where == arm_name_to_model_parts_.end()) {
        return {};
    }
    const auto& parts_to_load = where->second;

    std::map<std::string, mesh> result_model_parts;
    constexpr char threeDModelFileTemplate[] = "3d_models/%1%/%2%.glb";

    for (const auto& part : parts_to_load) {
        const std::filesystem::path model_file_path =
            current_state_->resource_root() / str(boost::format(threeDModelFileTemplate) % model_name % part);

        // Open the file in binary mode
        std::ifstream model_file(model_file_path, std::ios::binary);
        if (!model_file) {
            throw std::runtime_error(str(boost::format("unable to open 3d model file '%1%'") % model_file_path));
        }

        // Read the entire file into a vector without computing size ahead of time
        std::vector<char> temp_bytes(std::istreambuf_iterator<char>(model_file), {});
        if (model_file.bad()) {
            throw std::runtime_error(str(boost::format("error reading 3d model file '%1%'") % model_file_path));
        }

        // Convert to unsigned char vector
        std::vector<unsigned char> temp_bytes_unsigned(temp_bytes.begin(), temp_bytes.end());
        result_model_parts.emplace(part, mesh{"model/gltf-binary", std::move(temp_bytes_unsigned)});
    }

    return result_model_parts;
}

void URArm::stop(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    stop_(rlock);
}

ProtoStruct URArm::do_command(const ProtoStruct& command) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    ProtoStruct resp = ProtoStruct{};

    // NOTE: Changes to these values will not be effective for any
    // trajectory currently being planned, and will only affect
    // trajectory planning initiated after these values have been
    // changed. Note that torn reads are also possible, since
    // `::move_` loads from these values independently.
    constexpr char k_vel_key[] = "set_vel";
    constexpr char k_acc_key[] = "set_acc";
    constexpr char k_vel_degs_key[] = "set_vel_degs_per_sec";
    constexpr char k_acc_degs_key[] = "set_accel_degs_per_sec2";
    constexpr char k_get_tcp_forces_base_key[] = "get_tcp_forces_base";
    constexpr char k_get_tcp_forces_tool_key[] = "get_tcp_forces_tool";
    constexpr char k_clear_pstop[] = "clear_pstop";
    constexpr char k_is_controllable[] = "is_controllable_state";
    constexpr char k_get_state_description[] = "get_state_description";

    // Cache TCP state to ensure atomic read of pose and forces from same timestamp
    std::optional<decltype(current_state_->read_tcp_state_snapshot())> cached_tcp_state;

    // cache state descriptors to ensure atomic read from the same timestamp
    struct controlled_info {
        bool controlled;
        std::string description;
    };
    std::optional<controlled_info> cached_controlled_info;

    const auto add_limits_response = [&resp](const std::string& key, const vector6d_t& limits_deg) {
        std::vector<ProtoValue> arr;
        arr.reserve(limits_deg.size());
        for (size_t i = 0; i < limits_deg.size(); ++i) {
            arr.push_back(ProtoValue{limits_deg[i]});
        }
        resp.emplace(key, arr);
    };

    for (const auto& kv : command) {
        if (kv.first == k_vel_key || kv.first == k_vel_degs_key) {
            const auto limits_rad = parse_and_validate_joint_limits(kv.second, kv.first);
            auto result = current_state_->set_velocity_limits(limits_rad);
            add_limits_response(kv.first, radians_to_degrees(result));
        } else if (kv.first == k_acc_key || kv.first == k_acc_degs_key) {
            const auto limits_rad = parse_and_validate_joint_limits(kv.second, kv.first);
            auto result = current_state_->set_acceleration_limits(limits_rad);
            add_limits_response(kv.first, radians_to_degrees(result));
        } else if (kv.first == k_get_tcp_forces_base_key) {
            if (!cached_tcp_state) {
                cached_tcp_state = current_state_->read_tcp_state_snapshot();
            }
            const auto& tcp_force = cached_tcp_state->forces_at_base;
            ProtoStruct tcp_forces_base;
            tcp_forces_base.emplace("Fx_N", tcp_force[0]);
            tcp_forces_base.emplace("Fy_N", tcp_force[1]);
            tcp_forces_base.emplace("Fz_N", tcp_force[2]);
            tcp_forces_base.emplace("TRx_Nm", tcp_force[3]);
            tcp_forces_base.emplace("TRy_Nm", tcp_force[4]);
            tcp_forces_base.emplace("TRz_Nm", tcp_force[5]);
            resp.emplace("tcp_forces_base", std::move(tcp_forces_base));
        } else if (kv.first == k_get_tcp_forces_tool_key) {
            if (!cached_tcp_state) {
                cached_tcp_state = current_state_->read_tcp_state_snapshot();
            }
            const auto tcp_force = convert_tcp_force_to_tool_frame(cached_tcp_state->pose, cached_tcp_state->forces_at_base);
            ProtoStruct tcp_forces_tool;
            tcp_forces_tool.emplace("Fx_N", tcp_force[0]);
            tcp_forces_tool.emplace("Fy_N", tcp_force[1]);
            tcp_forces_tool.emplace("Fz_N", tcp_force[2]);
            tcp_forces_tool.emplace("TRx_Nm", tcp_force[3]);
            tcp_forces_tool.emplace("TRy_Nm", tcp_force[4]);
            tcp_forces_tool.emplace("TRz_Nm", tcp_force[5]);
            resp.emplace("tcp_forces_tool", std::move(tcp_forces_tool));
        } else if (kv.first == k_clear_pstop) {
            current_state_->clear_pstop();
            resp.emplace(k_clear_pstop, "protective stop cleared");
        } else if (kv.first == k_is_controllable) {
            if (!cached_controlled_info) {
                cached_controlled_info = controlled_info{};
                cached_controlled_info->controlled = current_state_->is_current_state_controlled(&cached_controlled_info->description);
            }
            resp.emplace(k_is_controllable, cached_controlled_info->controlled);
        } else if (kv.first == k_get_state_description) {
            if (!cached_controlled_info) {
                cached_controlled_info = controlled_info{};
                cached_controlled_info->controlled = current_state_->is_current_state_controlled(&cached_controlled_info->description);
            }
            resp.emplace(k_get_state_description, cached_controlled_info->description);
        } else {
            throw std::runtime_error("unsupported do_command key: " + kv.first);
        }
    }

    return resp;
}

void URArm::move_tool_space_(std::shared_lock<std::shared_mutex> config_rlock, pose p, const std::string& unix_time) {
    auto our_config_rlock = std::move(config_rlock);

    auto async_cancellation_monitor = [observer = GrpcContextObserver::current()]() {
        if (!observer) {
            return false;
        }

        return observer->context().IsCancelled();
    };

    // Capture the current movement epoch, so we can later detect if another caller
    // slipped in while we were planning.
    auto current_move_epoch = current_state_->get_move_epoch();

    VIAM_SDK_LOG(debug) << "move tool space: start unix_time_ms " << unix_time << " p: " << p;
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(info) << "move tool space: end unix_time " << unix_time; });

    // get current pose
    auto current_pose = current_state_->read_tcp_pose();

    // convert viam pose to universal robots pose
    auto target_pose = pose_to_ur_vector(p);

    // if we are already at the desired pose, there is nothing to do
    constexpr double k_position_tolerance_m = 1e-3;  // 1 mm
    bool already_there = true;
    for (size_t i = 0; i != 3; ++i) {  // check XYZ
        if (std::abs(current_pose[i] - target_pose[i]) > k_position_tolerance_m) {
            already_there = false;
            break;
        }
    }
    constexpr double k_orientation_tolerance_rad = 1e-3;  // ~0.057 degrees
    for (size_t i = 3; i != 6 && already_there; ++i) {    // check orientation (rx, ry, rz)
        if (std::abs(current_pose[i] - target_pose[i]) > k_orientation_tolerance_rad) {
            already_there = false;
            break;
        }
    }
    if (already_there) {
        VIAM_SDK_LOG(debug) << "Already at desired pose; skipping movement.";
        return;
    }

    // create a pose_sample for tool-space movement
    const pose_sample ps{target_pose};

    // Write ur pose to file for debugging purposes
    const std::string& path = current_state_->telemetry_output_path();
    write_pose_to_file(move_to_position_pose_filename(path, current_state_->resource_name(), unix_time), ps);

    // For pose-space moves, we don't log joint data since we only have the target pose
    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock)]() mutable {
        return current_state_->enqueue_move_request(current_move_epoch, std::nullopt, std::move(async_cancellation_monitor), ps);
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.
    trajectory_completion_future.get();
}

void URArm::move_joint_space_(std::shared_lock<std::shared_mutex> config_rlock,
                              std::list<Eigen::VectorXd> waypoints,
                              const MoveOptions& options,
                              const std::string& unix_time) {
    auto our_config_rlock = std::move(config_rlock);

    auto async_cancellation_monitor = [observer = GrpcContextObserver::current()]() {
        if (!observer) {
            return false;
        }

        auto result = observer->context().IsCancelled();
        return result;
    };

    // Capture the current movement epoch, so we can later detect if another caller
    // slipped in while we were planning.
    auto current_move_epoch = current_state_->get_move_epoch();

    VIAM_SDK_LOG(debug) << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size();
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(debug) << "move: end unix_time " << unix_time; });

    // get current joint position and add that as starting pose to waypoints
    auto curr_joint_pos = get_joint_positions_rad_(our_config_rlock);
    auto curr_joint_pos_rad = Eigen::Map<Eigen::VectorXd>(curr_joint_pos.data(), curr_joint_pos.size());

    // Unconditionally prepend current position, then deduplicate
    waypoints.emplace_front(curr_joint_pos_rad);
    deduplicate_waypoints(waypoints, current_state_->get_waypoint_deduplication_tolerance_rad());

    if (waypoints.size() == 1) {  // this tells us if we are already at the goal
        VIAM_SDK_LOG(debug) << "arm is already at the desired joint positions";
        return;
    }

    if (const auto& threshold = current_state_->get_reject_move_request_threshold_rad()) {
        auto current_joint_position = waypoints.begin();
        auto first_waypoint = std::next(current_joint_position);

        auto delta_pos = (*first_waypoint - *current_joint_position);

        if (delta_pos.lpNorm<Eigen::Infinity>() > *threshold) {
            std::stringstream err_string;

            err_string << "rejecting move request : difference between starting trajectory position [(";
            boost::copy(*first_waypoint | boost::adaptors::transformed(radians_to_degrees<const double&>),
                        boost::io::make_ostream_joiner(err_string, ", "));
            err_string << ")] and joint position [(";
            boost::copy(*current_joint_position | boost::adaptors::transformed(radians_to_degrees<const double&>),
                        boost::io::make_ostream_joiner(err_string, ", "));
            err_string << ")] is above threshold " << radians_to_degrees(delta_pos.lpNorm<Eigen::Infinity>()) << " > "
                       << radians_to_degrees(*threshold);
            VIAM_SDK_LOG(error) << err_string.str();
            throw std::runtime_error(err_string.str());
        }
    }

    VIAM_SDK_LOG(debug) << "move: compute_trajectory start " << unix_time;

    // Walk all interior points of the waypoints list, if any. If the
    // point of current interest is the cusp of a direction reversal
    // w.r.t. the points immediately before and after it, then splice
    // all waypoints up to and including the cusp point into a new
    // segment, and then begin accumulating a new segment starting at
    // the cusp point. The cusp point is duplicated, forming both the
    // end of one segment and the beginning of the next segment. After
    // exiting the loop, any remaining waypoints form the last (and if
    // no cusps were identified the only) segment. If one or more cusp
    // points were identified, the waypoints list will always have at
    // least two residual waypoints, since the last waypoint is never
    // examined, and the splice call never removes the waypoint being
    // visited.
    //
    // NOTE: This assumes waypoints have been de-duplicated to avoid
    // zero-length segments that would cause numerical issues in
    // normalized() calculations.
    std::vector<decltype(waypoints)> segments;
    for (auto where = next(begin(waypoints)); where != prev(end(waypoints)); ++where) {
        const auto segment_ab = *where - *prev(where);
        const auto segment_bc = *next(where) - *where;
        const auto dot = segment_ab.normalized().dot(segment_bc.normalized());
        if (std::fabs(dot + 1.0) < 1e-3) {
            segments.emplace_back();
            segments.back().splice(segments.back().begin(), waypoints, waypoints.begin(), where);
            segments.back().push_back(*where);
        }
    }
    segments.push_back(std::move(waypoints));

    auto velocity_limits_data = current_state_->get_velocity_limits();
    // TODO(RSDK-12375) Remove 0 velocity check when RDK stops sending 0 velocities
    if (options.max_vel_degs_per_sec && options.max_vel_degs_per_sec.get() > 0) {
        velocity_limits_data.fill(degrees_to_radians(options.max_vel_degs_per_sec.get()));
        velocity_limits_data = current_state_->clamp_velocity_limits(velocity_limits_data);
    }
    const auto velocity_limits = Eigen::Map<const Eigen::VectorXd>(velocity_limits_data.data(), velocity_limits_data.size());

    auto acceleration_limits_data = current_state_->get_acceleration_limits();
    // TODO(RSDK-12375) Remove 0 acc check when RDK stops sending 0 velocities
    if (options.max_acc_degs_per_sec2 && options.max_acc_degs_per_sec2.get() > 0) {
        acceleration_limits_data.fill(degrees_to_radians(options.max_acc_degs_per_sec2.get()));
        acceleration_limits_data = current_state_->clamp_acceleration_limits(acceleration_limits_data);
    }
    const auto acceleration_limits = Eigen::Map<const Eigen::VectorXd>(acceleration_limits_data.data(), acceleration_limits_data.size());

    const auto new_trajectory = [&]() -> std::optional<std::vector<trajectory_sample_point>> {
        if (!current_state_->use_new_trajectory_planner()) {
            return std::nullopt;
        }

        size_t total_waypoints = 0;
        double total_duration = 0.0;
        viam::trajex::arc_length total_arc_length{0.0};

        try {
            using namespace viam::trajex;

            totg::trajectory::options trajex_opts;
            trajex_opts.max_velocity = xt::xarray<double>::from_shape({velocity_limits_data.size()});
            trajex_opts.max_acceleration = xt::xarray<double>::from_shape({acceleration_limits_data.size()});
            std::ranges::copy(velocity_limits_data, trajex_opts.max_velocity.begin());
            std::ranges::copy(acceleration_limits_data, trajex_opts.max_acceleration.begin());

            std::vector<trajectory_sample_point> all_trajex_samples;

            const auto generation_start = std::chrono::steady_clock::now();

            for (const auto& segment : segments) {
                try {
                    const auto segment_xarray = eigen_waypoints_to_xarray(segment);
                    const totg::waypoint_accumulator trajex_waypoints(segment_xarray);

                    auto path_opts = totg::path::options{}.set_max_blend_deviation(current_state_->get_path_tolerance_delta_rads());

                    if (auto ratio = current_state_->get_path_colinearization_ratio()) {
                        path_opts.set_max_linear_deviation(current_state_->get_path_tolerance_delta_rads() * (*ratio));
                    }

                    auto trajex_path = totg::path::create(trajex_waypoints, path_opts);

                    // Create a copy of trajex_opts for each segment since it's consumed by create()
                    totg::trajectory::options segment_opts = trajex_opts;
                    auto trajex_trajectory = totg::trajectory::create(std::move(trajex_path), std::move(segment_opts));

                    auto sampler = totg::uniform_sampler::quantized_for_trajectory(
                        trajex_trajectory, types::hertz{current_state_->get_trajectory_sampling_freq_hz()});

                    double previous_time = 0.0;
                    for (const auto& sample : trajex_trajectory.samples(sampler) | std::views::drop(1)) {
                        const double current_time = sample.time.count();
                        const float timestep = boost::numeric_cast<float>(current_time - previous_time);

                        trajectory_sample_point point;
                        for (size_t i = 0; i < point.p.size(); ++i) {
                            point.p[i] = sample.configuration(i);
                            point.v[i] = sample.velocity(i);
                        }
                        point.timestep = timestep;

                        all_trajex_samples.push_back(point);
                        previous_time = current_time;
                    }

                    total_waypoints += segment.size();
                    total_duration += trajex_trajectory.duration().count();
                    total_arc_length += trajex_trajectory.path().length();

                    if (total_duration > current_state_->get_max_trajectory_duration_secs()) {
                        throw std::runtime_error("total trajectory duration exceeds maximum allowed duration");
                    }

                    VIAM_SDK_LOG(info) << "trajex/totg segment generated successfully, waypoints: " << segment.size()
                                       << ", duration: " << trajex_trajectory.duration().count()
                                       << "s, samples: " << all_trajex_samples.size()
                                       << ", arc length: " << trajex_trajectory.path().length();
                } catch (...) {
                    const std::string json_content = serialize_failed_trajectory_to_json(segment,
                                                                                         velocity_limits,
                                                                                         acceleration_limits,
                                                                                         current_state_->get_path_tolerance_delta_rads(),
                                                                                         current_state_->get_path_colinearization_ratio());

                    const std::string& path_dir = current_state_->telemetry_output_path();
                    const std::string filename =
                        failed_trajectory_filename(path_dir, current_state_->resource_name() + "_trajex", unix_time);
                    std::ofstream json_file(filename);
                    json_file << json_content;
                    json_file.close();

                    throw;
                }
            }

            if (total_duration < k_min_timestep_sec) {
                VIAM_SDK_LOG(debug) << "duration of move is too small, assuming arm is at goal";
                return std::vector<trajectory_sample_point>{};
            }

            const auto generation_end = std::chrono::steady_clock::now();
            const auto generation_time = std::chrono::duration<double>(generation_end - generation_start).count();

            VIAM_SDK_LOG(info) << "trajex/totg trajectory generated successfully, total waypoints: " << total_waypoints
                               << ", total duration: " << total_duration << "s, total samples: " << all_trajex_samples.size()
                               << ", total arc length: " << total_arc_length << ", generation_time: " << generation_time << "s";

            return all_trajex_samples;
        } catch (const std::exception& e) {
            VIAM_SDK_LOG(warn) << "trajex/totg trajectory generation failed, waypoints: " << total_waypoints << ", exception: " << e.what();
            return std::nullopt;
        }
    }();

    const auto legacy_generation_start = std::chrono::steady_clock::now();

    VIAM_SDK_LOG(debug) << "generating trajectory with max speed: " << radians_to_degrees(velocity_limits[0]);

    std::vector<trajectory_sample_point> samples;
    double total_duration = 0.0;
    double total_arc_length = 0.0;
    size_t total_waypoints = 0;

    for (auto& segment : segments) {
        // Apply colinearization to reduce waypoints
        if (auto ratio = current_state_->get_path_colinearization_ratio()) {
            apply_colinearization(segment, current_state_->get_path_tolerance_delta_rads() * (*ratio));
        }

        total_waypoints += segment.size();
        const Path path(segment, current_state_->get_path_tolerance_delta_rads());
        total_arc_length += path.getLength();
        const Trajectory trajectory(path, velocity_limits, acceleration_limits);
        if (!trajectory.isValid()) {
            // When the trajectory cannot be generated save the all the waypoints, velocity, acceleration and path tolerance in a
            // JSON. We should be able to feed it back to the trajectory generator and confirm it is failing
            const std::string json_content = serialize_failed_trajectory_to_json(segment,
                                                                                 velocity_limits,
                                                                                 acceleration_limits,
                                                                                 current_state_->get_path_tolerance_delta_rads(),
                                                                                 current_state_->get_path_colinearization_ratio());

            const std::string& path_dir = current_state_->telemetry_output_path();
            const std::string filename = failed_trajectory_filename(path_dir, current_state_->resource_name(), unix_time);
            std::ofstream json_file(filename);
            json_file << json_content;
            json_file.close();

            throw std::runtime_error(boost::str(boost::format("trajectory generation failed - details saved to: %1%") % filename));
        }

        const double duration = trajectory.getDuration();

        if (!std::isfinite(duration)) {
            throw std::runtime_error("trajectory.getDuration() was not a finite number");
        }

        total_duration += duration;
        if (total_duration > current_state_->get_max_trajectory_duration_secs()) {
            throw std::runtime_error("total trajectory duration exceeds maximum allowed duration");
        }

        trajectory.outputPhasePlaneTrajectory();

        sampling_func(samples, duration, current_state_->get_trajectory_sampling_freq_hz(), [&](const double t, const double step) {
            auto p_eigen = trajectory.getPosition(t);
            auto v_eigen = trajectory.getVelocity(t);
            return trajectory_sample_point{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                           {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                           boost::numeric_cast<float>(step)};
        });
    }

    if (total_duration < k_min_timestep_sec) {
        VIAM_SDK_LOG(debug) << "duration of move is too small, assuming arm is at goal";
        return;
    }

    const auto legacy_generation_end = std::chrono::steady_clock::now();
    const auto legacy_generation_time = std::chrono::duration<double>(legacy_generation_end - legacy_generation_start).count();

    if (current_state_->use_new_trajectory_planner()) {
        VIAM_SDK_LOG(info) << "legacy trajectory generated successfully, waypoints: " << total_waypoints << ", duration: " << total_duration
                           << "s, samples: " << samples.size() << ", arc length: " << total_arc_length
                           << ", generation_time: " << legacy_generation_time << "s";
    } else {
        VIAM_SDK_LOG(debug) << "legacy trajectory generated successfully, waypoints: " << total_waypoints
                            << ", duration: " << total_duration << "s, samples: " << samples.size() << ", arc length: " << total_arc_length
                            << ", generation_time: " << legacy_generation_time << "s";
    }

    if (new_trajectory) {
        samples = *new_trajectory;
    }

    VIAM_SDK_LOG(debug) << "move: compute_trajectory end " << unix_time << " samples.size() " << samples.size() << " segments "
                        << segments.size() - 1;

    const std::string& path = current_state_->telemetry_output_path();
    write_trajectory_to_file(trajectory_filename(path, current_state_->resource_name(), unix_time), samples);

    // For joint-space moves, we log the actual joint positions/velocities during execution
    std::ofstream ajp_of(arm_joint_positions_filename(path, current_state_->resource_name(), unix_time));
    ajp_of << "time_ms,read_attempt,"
              "joint_0_pos,joint_1_pos,joint_2_pos,joint_3_pos,joint_4_pos,joint_5_pos,"
              "joint_0_vel,joint_1_vel,joint_2_vel,joint_3_vel,joint_4_vel,joint_5_vel\n";

    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock), ajp_of = std::move(ajp_of)]() mutable {
        return current_state_->enqueue_move_request(
            current_move_epoch, std::optional<std::ofstream>{std::move(ajp_of)}, std::move(async_cancellation_monitor), samples);
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.
    trajectory_completion_future.get();
}

// Define the destructor
URArm::~URArm() {
    VIAM_SDK_LOG(info) << "Shutting down URArm driver instance for arm model: " << model_.to_string();
    const std::unique_lock wlock{config_mutex_};
    shutdown_(wlock);
}

template <template <typename> typename lock_type>
void URArm::stop_(const lock_type<std::shared_mutex>&) {
    if (auto cancel_future = current_state_->cancel_move_request()) {
        cancel_future->get();
    }
}

void URArm::shutdown_(const std::unique_lock<std::shared_mutex>& lock) noexcept {
    try {
        if (current_state_) {
            const auto destroy_state = make_scope_guard([&] { current_state_.reset(); });

            stop_(lock);
            current_state_->shutdown();
        }
        VIAM_SDK_LOG(info) << "URArm shutdown complete";

    } catch (...) {
        const auto unconditional_abort = make_scope_guard([] { std::abort(); });
        try {
            throw;
        } catch (const std::exception& ex) {
            VIAM_SDK_LOG(error) << "URArm shutdown failed with a std::exception - module service will terminate: " << ex.what();
        } catch (...) {
            VIAM_SDK_LOG(error) << "URArm shutdown failed with an unknown exception - module service will terminate";
        }
    }
}

// We need to requisition different ports for each independent URArm
// instance, otherwise they will all try to use the same ports and
// only one of them will work.
URArm::ports_::ports_() {
    static std::atomic<std::uint32_t> counter{50001};
    reverse_port = counter.fetch_add(4);
    script_sender_port = reverse_port + 1;
    trajectory_port = script_sender_port + 1;
    script_command_port = trajectory_port + 1;
}
