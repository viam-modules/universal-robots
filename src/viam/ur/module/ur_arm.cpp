#include "ur_arm.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <boost/format.hpp>
#include <boost/io/ostream_joiner.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>

#include <viam/sdk/components/component.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include <third_party/trajectories/Trajectory.h>

#include "ur_arm_state.hpp"
#include "utils.hpp"

// this chunk of code uses the rust FFI to handle the spatialmath calculations to turn a UR vector to a pose or a pose to a UR vector
extern "C" void* quaternion_from_axis_angle(double x, double y, double z, double theta);
extern "C" void free_quaternion_memory(void* q);

extern "C" void* orientation_vector_from_quaternion(void* q);
extern "C" void free_orientation_vector_memory(void* ov);

extern "C" double* orientation_vector_get_components(void* ov);
extern "C" void free_orientation_vector_components(double* ds);

// OrientationVector
extern "C" void* new_orientation_vector(double ox, double oy, double oz, double theta);
extern "C" void* quaternion_from_orientation_vector(void* ov);
extern "C" void free_orientation_vector_components(double* ds);  // you already have this
// Quaternion
extern "C" void free_quaternion_memory(void* q);  // you already have this
// AxisAngle
extern "C" double* axis_angle_from_quaternion(void* q);
extern "C" void free_axis_angles_memory(void* aa);

pose ur_vector_to_pose(urcl::vector6d_t vec) {
    const double norm = std::hypot(vec[3], vec[4], vec[5]);
    if (std::isnan(norm) || (norm == 0)) {
        throw std::invalid_argument("Cannot normalize with NaN or zero norm");
    }

    auto q = std::unique_ptr<void, decltype(&free_quaternion_memory)>(
        quaternion_from_axis_angle(vec[3] / norm, vec[4] / norm, vec[5] / norm, norm), &free_quaternion_memory);

    auto ov = std::unique_ptr<void, decltype(&free_orientation_vector_memory)>(orientation_vector_from_quaternion(q.get()),
                                                                               &free_orientation_vector_memory);

    auto components = std::unique_ptr<double[], decltype(&free_orientation_vector_components)>(orientation_vector_get_components(ov.get()),
                                                                                               &free_orientation_vector_components);

    auto position = coordinates{1000 * vec[0], 1000 * vec[1], 1000 * vec[2]};
    auto orientation = pose_orientation{components[0], components[1], components[2]};
    auto theta = radians_to_degrees(components[3]);

    return {position, orientation, theta};
}

inline double degrees_to_radians(double deg) {
    return deg * M_PI / 180.0;
}

urcl::vector6d_t pose_to_ur_vector(const pose& p) {
    urcl::vector6d_t v{};

    // 1) Position: mm → meters
    v[0] = p.coordinates.x / 1000.0;
    v[1] = p.coordinates.y / 1000.0;
    v[2] = p.coordinates.z / 1000.0;

    // 2) Orientation vector (o_x, o_y, o_z, theta [deg]) → quaternion → axis-angle
    constexpr double k_angle_epsilon = 1e-12;

    if (std::isnan(p.theta)) {
        throw std::invalid_argument("pose_to_ur_vector: theta is NaN");
    }

    const double theta_rad = degrees_to_radians(p.theta);
    // If angle ~ 0, the UR axis-angle vector is just (0,0,0)
    if (std::abs(theta_rad) < k_angle_epsilon) {
        v[3] = v[4] = v[5] = 0.0;
        return v;
    }

    // Build an OrientationVector in rust-utils, then convert:
    // new_orientation_vector(ox, oy, oz, theta_radians)
    void* ov = new_orientation_vector(p.orientation.o_x, p.orientation.o_y, p.orientation.o_z, theta_rad);
    if (!ov) {
        throw std::runtime_error("pose_to_ur_vector: failed to create orientation vector");
    }

    // quaternion_from_orientation_vector(ov)
    void* q = quaternion_from_orientation_vector(ov);
    if (!q) {
        free_orientation_vector_memory(ov);
        throw std::runtime_error("pose_to_ur_vector: failed to create quaternion");
    }

    // axis_angle_from_quaternion(q) -> returns [ax, ay, az, theta] (theta in radians)
    double* aa = axis_angle_from_quaternion(q);
    if (!aa) {
        free_quaternion_memory(q);
        free_orientation_vector_memory(ov);
        throw std::runtime_error("pose_to_ur_vector: failed to compute axis-angle");
    }

    // UR expects (rx, ry, rz) = axis * theta  (a.k.a. axis-angle "vector")
    const double ax = aa[0];
    const double ay = aa[1];
    const double az = aa[2];
    const double th = aa[3];

    // If the axis is near-zero or th ~ 0, treat as zero rotation (robust to numeric noise)
    const double n = std::hypot(ax, std::hypot(ay, az));
    if (n < k_angle_epsilon || std::abs(th) < k_angle_epsilon) {
        v[3] = v[4] = v[5] = 0.0;
    } else {
        v[3] = ax * th;
        v[4] = ay * th;
        v[5] = az * th;
    }

    // clean up FFI allocations
    free_axis_angles_memory(aa);
    free_quaternion_memory(q);
    free_orientation_vector_memory(ov);

    return v;
}

namespace {

constexpr double k_waypoint_equivalancy_epsilon_rad = 1e-4;
constexpr double k_min_timestep_sec = 1e-2;  // determined experimentally, the arm appears to error when given timesteps ~2e-5 and lower

std::vector<std::string> validate_config_(const ResourceConfig& cfg) {
    if (!find_config_attribute<std::string>(cfg, "host")) {
        throw std::invalid_argument("attribute `host` is required");
    }
    if (!find_config_attribute<double>(cfg, "speed_degs_per_sec")) {
        throw std::invalid_argument("attribute `speed_degs_per_sec` is required");
    }
    if (!find_config_attribute<double>(cfg, "acceleration_degs_per_sec2")) {
        throw std::invalid_argument("attribute `acceleration_degs_per_sec2` is required");
    }

    auto threshold = find_config_attribute<double>(cfg, "reject_move_request_threshold_deg");
    constexpr double k_min_threshold = 0.0;
    constexpr double k_max_threshold = 360.0;
    if (threshold && (*threshold < k_min_threshold || *threshold > k_max_threshold)) {
        std::stringstream sstream;
        sstream << "attribute `reject_move_request_threshold_deg` should be between " << k_min_threshold << " and " << k_max_threshold
                << ", it is : " << *threshold << " degrees";
        throw std::invalid_argument(sstream.str());
    }

    auto frequency = find_config_attribute<double>(cfg, "robot_control_freq_hz");
    constexpr double k_max_frequency = 1000.;
    if (frequency && (*frequency <= 0. || *frequency >= k_max_frequency)) {
        std::stringstream sstream;
        sstream << "attribute `robot_control_freq_hz` should be a positive number less than " << k_max_frequency
                << ", it is : " << *frequency << " hz";

        throw std::invalid_argument(sstream.str());
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
        for (size_t j = 0; j < 6; j++) {
            of << "," << samples[i].p[j];
        }
        for (size_t j = 0; j < 6; j++) {
            of << "," << samples[i].v[j];
        }
        of << "\n";
    }

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

std::string waypoints_filename(const std::string& path, const std::string& unix_time) {
    constexpr char kWaypointsCsvNameTemplate[] = "/%1%_waypoints.csv";
    auto fmt = boost::format(path + kWaypointsCsvNameTemplate);
    return (fmt % unix_time).str();
}

std::string trajectory_filename(const std::string& path, const std::string& unix_time) {
    constexpr char kTrajectoryCsvNameTemplate[] = "/%1%_trajectory.csv";
    auto fmt = boost::format(path + kTrajectoryCsvNameTemplate);
    return (fmt % unix_time).str();
}

std::string arm_joint_positions_filename(const std::string& path, const std::string& unix_time) {
    constexpr char kArmJointPositionsCsvNameTemplate[] = "/%1%_arm_joint_positions.csv";
    auto fmt = boost::format(path + kArmJointPositionsCsvNameTemplate);
    return (fmt % unix_time).str();
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
    VIAM_SDK_LOG(info) << "URArm constructor called (model: " << model_.to_string() << ")";
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
        } else if (model_ == URArm::model("ur5e")) {
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

    VIAM_SDK_LOG(info) << "URArm starting up";
    current_state_ = state_::create(configured_model_type, cfg, ports_);

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
    const auto filename = waypoints_filename(current_state_->csv_output_path(), unix_time);

    write_waypoints_to_csv(filename, waypoints);

    // move will throw if an error occurs
    move_joint_space_(std::move(rlock), std::move(waypoints), unix_time);
}

void URArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                         const MoveOptions&,
                                         const viam::sdk::ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    // TODO: use options
    if (!positions.empty()) {
        std::list<Eigen::VectorXd> waypoints;
        for (const auto& position : positions) {
            auto next_waypoint_deg = Eigen::VectorXd::Map(position.data(), boost::numeric_cast<Eigen::Index>(position.size()));
            auto next_waypoint_rad = degrees_to_radians(std::move(next_waypoint_deg)).eval();
            if ((!waypoints.empty()) && (next_waypoint_rad.isApprox(waypoints.back(), k_waypoint_equivalancy_epsilon_rad))) {
                continue;
            }
            waypoints.emplace_back(std::move(next_waypoint_rad));
        }

        const auto unix_time = unix_time_iso8601();
        const auto filename = waypoints_filename(current_state_->csv_output_path(), unix_time);

        write_waypoints_to_csv(filename, waypoints);

        // move will throw if an error occurs
        move_joint_space_(std::move(rlock), std::move(waypoints), unix_time);
    }
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
    VIAM_SDK_LOG(info) << "move_to_position via on-robot IK + movel";

    const auto unix_time = unix_time_iso8601();

    move_tool_space_(std::move(rlock), std::move(p), unix_time);
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
        } else if (model_ == model("ur20")) {
            return "ur20";
        }
        throw std::runtime_error(str(boost::format("no kinematics file known for model '%1'") % model_.to_string()));
    }();

    constexpr char kSvaFileTemplate[] = "kinematics/%1%.json";
    const auto sva_file_path = current_state_->resource_root() / str(boost::format(kSvaFileTemplate) % model_string);

    // Open the file in binary mode
    std::ifstream sva_file(sva_file_path, std::ios::binary);
    if (!sva_file) {
        throw std::runtime_error(str(boost::format("unable to open kinematics file '%1'") % sva_file_path));
    }

    // Read the entire file into a vector without computing size ahead of time
    std::vector<char> temp_bytes(std::istreambuf_iterator<char>(sva_file), {});
    if (sva_file.bad()) {
        throw std::runtime_error(str(boost::format("error reading kinematics file '%1'") % sva_file_path));
    }

    // Convert to unsigned char vector
    return KinematicsDataSVA({temp_bytes.begin(), temp_bytes.end()});
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
    constexpr char k_get_tcp_forces_base_key[] = "get_tcp_forces_base";
    constexpr char k_get_tcp_forces_tool_key[] = "get_tcp_forces_tool";
    constexpr char k_clear_pstop[] = "clear_pstop";

    // Cache TCP state to ensure atomic read of pose and forces from same timestamp
    std::optional<decltype(current_state_->read_tcp_state_snapshot())> cached_tcp_state;

    for (const auto& kv : command) {
        if (kv.first == k_vel_key) {
            const double val = *kv.second.get<double>();
            current_state_->set_speed(degrees_to_radians(val));
            resp.emplace(k_vel_key, val);
        } else if (kv.first == k_acc_key) {
            const double val = *kv.second.get<double>();
            current_state_->set_acceleration(degrees_to_radians(val));
            resp.emplace(k_acc_key, val);
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
        } else {
            throw std::runtime_error("unsupported do_command key: " + kv.first);
        }
    }

    return resp;
}

void URArm::move_tool_space_(std::shared_lock<std::shared_mutex> config_rlock, pose p, const std::string& unix_time) {
    auto our_config_rlock = std::move(config_rlock);

    // Capture the current movement epoch, so we can later detect if another caller
    // slipped in while we were planning.
    auto current_move_epoch = current_state_->get_move_epoch();

    VIAM_SDK_LOG(info) << "move tool space: start unix_time_ms " << unix_time << " p " << p;
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(info) << "move tool space: end unix_time " << unix_time; });

    // get current pose and add that as a starting pose
    auto current_pose = current_state_->read_tcp_pose();
    // TODO: do check here to make sure that we are not already at the goal
    auto ur_pose = pose_to_ur_vector(p);

    // create a trajectory sample point with where we want to go to
    std::vector<trajectory_sample_point> samples;

    trajectory_sample_point point_start{
        current_pose,        // positions
        {0, 0, 0, 0, 0, 0},  // velocities
        0.0,                 // step
        false                // flag
    };

    samples.push_back(point_start);

    trajectory_sample_point point_end{
        ur_pose,             // positions
        {0, 0, 0, 0, 0, 0},  // velocities
        0.0,                 // step
        false                // flag
    };

    samples.push_back(point_end);

    // take the list of sample points and hand that over to the ur arm
    const std::string& path = current_state_->csv_output_path();
    write_trajectory_to_file(trajectory_filename(path, unix_time), samples);

    std::ofstream ajp_of(arm_joint_positions_filename(path, unix_time));
    ajp_of << "time_ms,read_attempt,"
              "joint_0_pos,joint_1_pos,joint_2_pos,joint_3_pos,joint_4_pos,joint_5_pos,"
              "joint_0_vel,joint_1_vel,joint_2_vel,joint_3_vel,joint_4_vel,joint_5_vel\n";

    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock), ajp_of = std::move(ajp_of)]() mutable {
        return current_state_->enqueue_move_request(current_move_epoch, std::move(samples), std::move(ajp_of));
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.
    trajectory_completion_future.get();
}

void URArm::move_joint_space_(std::shared_lock<std::shared_mutex> config_rlock,
                              std::list<Eigen::VectorXd> waypoints,
                              const std::string& unix_time) {
    auto our_config_rlock = std::move(config_rlock);

    // Capture the current movement epoch, so we can later detect if another caller
    // slipped in while we were planning.
    auto current_move_epoch = current_state_->get_move_epoch();

    VIAM_SDK_LOG(info) << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size();
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(info) << "move: end unix_time " << unix_time; });

    // get current joint position and add that as starting pose to waypoints
    VIAM_SDK_LOG(info) << "move: get_joint_positions start " << unix_time;
    auto curr_joint_pos = get_joint_positions_rad_(our_config_rlock);
    VIAM_SDK_LOG(info) << "move: get_joint_positions end " << unix_time;
    auto curr_joint_pos_rad = Eigen::Map<Eigen::VectorXd>(curr_joint_pos.data(), curr_joint_pos.size());

    if (const auto& threshold = current_state_->get_reject_move_request_threshold_rad()) {
        auto delta_pos = (waypoints.front() - curr_joint_pos_rad);

        if (delta_pos.lpNorm<Eigen::Infinity>() > *threshold) {
            std::stringstream err_string;

            err_string << "rejecting move request : difference between starting trajectory position [(";
            auto first_waypoint = waypoints.front();
            boost::copy(first_waypoint | boost::adaptors::transformed(radians_to_degrees<const double&>),
                        boost::io::make_ostream_joiner(err_string, ", "));
            err_string << ")] and joint position [(";
            boost::copy(curr_joint_pos_rad | boost::adaptors::transformed(radians_to_degrees<const double&>),
                        boost::io::make_ostream_joiner(err_string, ", "));
            err_string << ")] is above threshold " << radians_to_degrees(delta_pos.lpNorm<Eigen::Infinity>()) << " > "
                       << radians_to_degrees(*threshold);
            VIAM_SDK_LOG(error) << err_string.str();
            throw std::runtime_error(err_string.str());
        }
    }

    VIAM_SDK_LOG(info) << "move: compute_trajectory start " << unix_time;

    if (!curr_joint_pos_rad.isApprox(waypoints.front(), k_waypoint_equivalancy_epsilon_rad)) {
        waypoints.emplace_front(std::move(curr_joint_pos_rad));
    }
    if (waypoints.size() == 1) {  // this tells us if we are already at the goal
        VIAM_SDK_LOG(info) << "arm is already at the desired joint positions";
        return;
    }

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

    // set velocity/acceleration constraints
    const auto max_velocity = Eigen::VectorXd::Constant(6, current_state_->get_speed());
    const auto max_acceleration = Eigen::VectorXd::Constant(6, current_state_->get_acceleration());
    VIAM_SDK_LOG(info) << "generating trajectory with max speed: " << radians_to_degrees(max_velocity[0]);

    std::vector<trajectory_sample_point> samples;

    for (const auto& segment : segments) {
        const Trajectory trajectory(Path(segment, 0.1), max_velocity, max_acceleration);
        if (!trajectory.isValid()) {
            std::stringstream buffer;
            buffer << "trajectory generation failed for path:";
            for (const auto& position : segment) {
                buffer << "{";
                for (Eigen::Index j = 0; j < 6; j++) {
                    buffer << position[j] << " ";
                }
                buffer << "}";
            }
            throw std::runtime_error(buffer.str());
        }

        const double duration = trajectory.getDuration();

        if (!std::isfinite(duration)) {
            throw std::runtime_error("trajectory.getDuration() was not a finite number");
        }

        // TODO(RSDK-11069): Make this configurable
        // https://viam.atlassian.net/browse/RSDK-11069
        if (duration > 600) {  // if the duration is longer than 10 minutes
            throw std::runtime_error("trajectory.getDuration() exceeds 10 minutes");
        }

        if (duration < k_min_timestep_sec) {
            VIAM_SDK_LOG(info) << "duration of move is too small, assuming arm is at goal";
            return;
        }

        trajectory.outputPhasePlaneTrajectory();

        // desired sampling frequency. if the duration is small we will oversample but that should be fine.
        constexpr double k_sampling_freq_hz = 5;
        sampling_func(samples, duration, k_sampling_freq_hz, [&](const double t, const double step) {
            auto p_eigen = trajectory.getPosition(t);
            auto v_eigen = trajectory.getVelocity(t);
            return trajectory_sample_point{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                           {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                           boost::numeric_cast<float>(step),
                                           true};
        });
    }
    VIAM_SDK_LOG(info) << "move: compute_trajectory end " << unix_time << " samples.size() " << samples.size() << " segments "
                       << segments.size() - 1;

    const std::string& path = current_state_->csv_output_path();
    write_trajectory_to_file(trajectory_filename(path, unix_time), samples);

    std::ofstream ajp_of(arm_joint_positions_filename(path, unix_time));
    ajp_of << "time_ms,read_attempt,"
              "joint_0_pos,joint_1_pos,joint_2_pos,joint_3_pos,joint_4_pos,joint_5_pos,"
              "joint_0_vel,joint_1_vel,joint_2_vel,joint_3_vel,joint_4_vel,joint_5_vel\n";

    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock), ajp_of = std::move(ajp_of)]() mutable {
        return current_state_->enqueue_move_request(current_move_epoch, std::move(samples), std::move(ajp_of));
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.
    trajectory_completion_future.get();
}

// Define the destructor
URArm::~URArm() {
    VIAM_SDK_LOG(warn) << "URArm destructor called, shutting down";
    const std::unique_lock wlock{config_mutex_};
    shutdown_(wlock);
    VIAM_SDK_LOG(warn) << "URArm destroyed";
}

template <template <typename> typename lock_type>
void URArm::stop_(const lock_type<std::shared_mutex>&) {
    if (auto cancel_future = current_state_->cancel_move_request()) {
        cancel_future->get();
    }
}

void URArm::shutdown_(const std::unique_lock<std::shared_mutex>& lock) noexcept {
    try {
        VIAM_SDK_LOG(warn) << "URArm shutdown called";
        if (current_state_) {
            const auto destroy_state = make_scope_guard([&] { current_state_.reset(); });

            VIAM_SDK_LOG(info) << "URArm shutdown calling stop";
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
