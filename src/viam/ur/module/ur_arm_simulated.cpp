#include "ur_arm_simulated.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/format.hpp>

#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/resource/resource_api.hpp>

#include "dh_kinematics.hpp"
#include "utils.hpp"

namespace {

// The simulated arm shares the model family with the hardware arm so it appears under
// `viam:universal-robots:simulated-arm`.
Model simulated_arm_model(const std::string& name) {
    static const auto family = ModelFamily{"viam", "universal-robots"};
    return {family, name};
}

constexpr char k_model_name[] = "simulated-arm";

std::string supported_models_message() {
    std::string out;
    const auto& models = simulated_arm_models();
    for (size_t i = 0; i < models.size(); ++i) {
        out += models[i];
        if (i + 1 < models.size()) {
            out += ", ";
        }
    }
    return out;
}

std::vector<std::string> validate_simulated_config_(const ResourceConfig& cfg) {
    const auto arm_model = find_config_attribute<std::string>(cfg, "arm_model");
    if (!arm_model) {
        throw std::invalid_argument("attribute `arm_model` is required (one of: " + supported_models_message() + ")");
    }
    const auto& models = simulated_arm_models();
    if (std::find(models.begin(), models.end(), *arm_model) == models.end()) {
        throw std::invalid_argument("unsupported `arm_model`: `" + *arm_model + "` (expected one of: " + supported_models_message() + ")");
    }

    if (cfg.attributes().contains("speed_degs_per_sec")) {
        parse_and_validate_joint_limits(cfg, "speed_degs_per_sec");
    }

    // Validate types of the optional attributes if present.
    find_config_attribute<bool>(cfg, "simulate_time");

    const auto motion_name = find_config_attribute<std::string>(cfg, "motion").value_or(URArmSimulated::k_default_motion_service);

    // Declare the motion service as an implicit dependency so the SDK injects it for
    // `move_to_position` (Cartesian) planning.
    return {Name(API::get<Motion>(), "", motion_name).to_string()};
}

}  // namespace

const std::vector<std::string>& simulated_arm_models() {
    // ur5e and ur7e share a kinematic chain; ur12e shares the UR10e chain.
    static const std::vector<std::string> models = {"ur3e", "ur5e", "ur7e", "ur12e", "ur20"};
    return models;
}

urcl::vector6d_t simulated_interpolate_step(const urcl::vector6d_t& current,
                                            const urcl::vector6d_t& target,
                                            const urcl::vector6d_t& speed_rad_per_sec,
                                            double elapsed_secs,
                                            bool* reached) {
    constexpr double k_epsilon = 1e-9;

    // The whole motion takes as long as the joint that needs the most time. Scaling each
    // joint to finish at that same time keeps all joints arriving together, matching how
    // rdk's motion planner interpolates.
    double total_time = 0.0;
    for (size_t i = 0; i < current.size(); ++i) {
        const double distance = std::abs(target[i] - current[i]);
        const double speed = (speed_rad_per_sec[i] > k_epsilon) ? speed_rad_per_sec[i] : k_epsilon;
        total_time = std::max(total_time, distance / speed);
    }

    if (total_time < k_epsilon) {
        if (reached != nullptr) {
            *reached = true;
        }
        return target;
    }

    urcl::vector6d_t result = current;
    bool any_joint_still_moving = false;
    for (size_t i = 0; i < current.size(); ++i) {
        const double diff = target[i] - current[i];
        const double effective_speed = std::abs(diff) / total_time;
        const double to_travel = elapsed_secs * effective_speed;
        if (to_travel >= std::abs(diff) - k_epsilon) {
            result[i] = target[i];
        } else {
            result[i] = current[i] + ((diff < 0) ? -to_travel : to_travel);
            any_joint_still_moving = true;
        }
    }

    if (reached != nullptr) {
        *reached = !any_joint_still_moving;
    }
    return result;
}

std::vector<std::shared_ptr<ModelRegistration>> URArmSimulated::create_model_registrations() {
    const auto arm = API::get<Arm>();
    const auto model = simulated_arm_model(k_model_name);
    auto registration = std::make_shared<ModelRegistration>(
        arm,
        model,
        // NOLINTNEXTLINE(performance-unnecessary-value-param): Signature is fixed by ModelRegistration.
        [model](auto deps, auto config) { return std::make_unique<URArmSimulated>(model, deps, config); },
        [](auto const& config) { return validate_simulated_config_(config); });
    return {std::move(registration)};
}

URArmSimulated::URArmSimulated(Model model, const Dependencies& deps, const ResourceConfig& cfg)
    : Arm(cfg.name()), model_(std::move(model)) {
    VIAM_SDK_LOG(info) << "instantiating URArmSimulated for arm model: " << model_.to_string();
    configure_logger(cfg);
    arm_name_to_model_parts_ = {
        {"ur5e", {"base_link", "ee_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}},
        {"ur20", {"base_link", "wrist_3_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}},
    };
    configure_(deps, cfg);
}

URArmSimulated::~URArmSimulated() {
    shutdown_();
}

void URArmSimulated::configure_(const Dependencies& deps, const ResourceConfig& cfg) {
    {
        const std::lock_guard<std::mutex> lock(mutex_);

        arm_model_ = find_config_attribute<std::string>(cfg, "arm_model").value();
        resource_root_ = module_resource_root();

        if (cfg.attributes().contains("speed_degs_per_sec")) {
            speed_rad_per_sec_ = parse_and_validate_joint_limits(cfg, "speed_degs_per_sec");
        } else {
            speed_rad_per_sec_.fill(degrees_to_radians(k_default_speed_degs_per_sec));
        }

        simulate_time_ = find_config_attribute<bool>(cfg, "simulate_time").value_or(true);

        // Resolve the motion service dependency declared in validate_simulated_config_.
        motion_.reset();
        for (const auto& [dep_name, resource] : deps) {
            if (auto motion = std::dynamic_pointer_cast<Motion>(resource)) {
                motion_ = std::move(motion);
                break;
            }
        }
        if (!motion_) {
            VIAM_SDK_LOG(warn) << "URArmSimulated: no motion service dependency resolved; move_to_position will be unavailable";
        }

        joint_positions_rad_.fill(0.0);
        target_positions_rad_ = joint_positions_rad_;
        moving_ = false;
        last_update_ = std::chrono::steady_clock::now();
        shutdown_requested_ = false;
    }

    if (simulate_time_) {
        sim_thread_ = std::thread(&URArmSimulated::run_simulation_, this);
    }

    VIAM_SDK_LOG(info) << "URArmSimulated configured to emulate `" << arm_model_ << "`";
}

void URArmSimulated::shutdown_() noexcept {
    shutdown_requested_ = true;
    cv_.notify_all();
    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    moving_ = false;
    cv_.notify_all();
}

void URArmSimulated::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    VIAM_SDK_LOG(info) << "URArmSimulated reconfigure: shutting down and reconfiguring";
    shutdown_();
    configure_(deps, cfg);
}

void URArmSimulated::run_simulation_() {
    constexpr auto k_tick = std::chrono::milliseconds(10);
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_requested_) {
        cv_.wait_for(lock, k_tick, [this] { return shutdown_requested_.load(); });
        if (shutdown_requested_) {
            break;
        }
        update_for_time_(std::chrono::steady_clock::now());
    }
}

void URArmSimulated::update_for_time_(std::chrono::steady_clock::time_point now) {
    if (!moving_) {
        last_update_ = now;
        return;
    }
    const double elapsed_secs = std::chrono::duration<double>(now - last_update_).count();
    last_update_ = now;

    bool reached = false;
    joint_positions_rad_ =
        simulated_interpolate_step(joint_positions_rad_, target_positions_rad_, speed_rad_per_sec_, elapsed_secs, &reached);
    if (reached) {
        moving_ = false;
        cv_.notify_all();
    }
}

std::vector<double> URArmSimulated::get_joint_positions(const ProtoStruct&) {
    const std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> degrees(joint_positions_rad_.size());
    for (size_t i = 0; i < joint_positions_rad_.size(); ++i) {
        degrees[i] = radians_to_degrees(joint_positions_rad_[i]);
    }
    return degrees;
}

void URArmSimulated::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct&) {
    if (positions.size() != k_ur_arm_dof) {
        throw std::invalid_argument(
            boost::str(boost::format("move_to_joint_positions expects %1% joint values, got %2%") % k_ur_arm_dof % positions.size()));
    }

    vector6d_t target{};
    for (size_t i = 0; i < k_ur_arm_dof; ++i) {
        target[i] = degrees_to_radians(positions[i]);
    }

    std::unique_lock<std::mutex> lock(mutex_);
    target_positions_rad_ = target;

    // Without time simulation, jump straight to the target.
    if (!simulate_time_) {
        joint_positions_rad_ = target;
        moving_ = false;
        return;
    }

    moving_ = true;
    last_update_ = std::chrono::steady_clock::now();
    cv_.notify_all();  // wake the simulation thread to begin advancing
    cv_.wait(lock, [this] { return !moving_ || shutdown_requested_.load(); });
}

void URArmSimulated::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                                  const MoveOptions&,
                                                  const ProtoStruct&) {
    for (const auto& waypoint : positions) {
        const ProtoStruct extra{};
        move_to_joint_positions(waypoint, extra);
    }
}

pose URArmSimulated::get_end_position(const ProtoStruct&) {
    vector6d_t joints{};
    std::string arm_model;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        joints = joint_positions_rad_;
        arm_model = arm_model_;
    }
    return ur_vector_to_pose(forward_kinematics(arm_model, joints));
}

void URArmSimulated::move_to_position(const pose& p, const ProtoStruct& extra) {
    if (!motion_) {
        throw std::runtime_error(
            "move_to_position requires a motion service, which was not available; configure a `motion` service dependency");
    }
    const pose_in_frame destination{name() + "_origin", p};
    if (!motion_->move(destination, name(), nullptr, nullptr, extra)) {
        throw std::runtime_error("motion service failed to plan or execute the requested move_to_position");
    }
}

bool URArmSimulated::is_moving() {
    const std::lock_guard<std::mutex> lock(mutex_);
    return moving_;
}

void URArmSimulated::stop(const ProtoStruct&) {
    const std::lock_guard<std::mutex> lock(mutex_);
    moving_ = false;
    target_positions_rad_ = joint_positions_rad_;
    cv_.notify_all();
}

viam::sdk::KinematicsData URArmSimulated::get_kinematics(const ProtoStruct&) {
    constexpr char k_sva_file_template[] = "kinematics/%1%.json";
    const auto sva_file_path = resource_root_ / boost::str(boost::format(k_sva_file_template) % arm_model_);

    std::ifstream sva_file(sva_file_path, std::ios::binary);
    if (!sva_file) {
        throw std::runtime_error(boost::str(boost::format("unable to open kinematics file '%1%'") % sva_file_path));
    }

    std::vector<char> temp_bytes(std::istreambuf_iterator<char>(sva_file), {});
    if (sva_file.bad()) {
        throw std::runtime_error(boost::str(boost::format("error reading kinematics file '%1%'") % sva_file_path));
    }

    return viam::sdk::KinematicsDataSVA({temp_bytes.begin(), temp_bytes.end()});
}

std::map<std::string, mesh> URArmSimulated::get_3d_models(const ProtoStruct&) {
    const auto where = arm_name_to_model_parts_.find(arm_model_);
    if (where == arm_name_to_model_parts_.end()) {
        return {};
    }
    const auto& parts_to_load = where->second;

    std::map<std::string, mesh> result_model_parts;
    constexpr char k_three_d_model_file_template[] = "3d_models/%1%/%2%.glb";

    for (const auto& part : parts_to_load) {
        const std::filesystem::path model_file_path =
            resource_root_ / boost::str(boost::format(k_three_d_model_file_template) % arm_model_ % part);

        std::ifstream model_file(model_file_path, std::ios::binary);
        if (!model_file) {
            throw std::runtime_error(boost::str(boost::format("unable to open 3d model file '%1%'") % model_file_path));
        }

        std::vector<char> temp_bytes(std::istreambuf_iterator<char>(model_file), {});
        if (model_file.bad()) {
            throw std::runtime_error(boost::str(boost::format("error reading 3d model file '%1%'") % model_file_path));
        }

        std::vector<unsigned char> temp_bytes_unsigned(temp_bytes.begin(), temp_bytes.end());
        result_model_parts.emplace(part, mesh{"model/gltf-binary", std::move(temp_bytes_unsigned)});
    }

    return result_model_parts;
}

ProtoStruct URArmSimulated::do_command(const ProtoStruct&) {
    return ProtoStruct{};
}
