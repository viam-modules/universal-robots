#include "ur_arm.hpp"

#include <chrono>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <thread>

#include <boost/format.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>

#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include <third_party/trajectories/Trajectory.h>

// this chunk of code uses the rust FFI to handle the spatialmath calculations to turn a UR vector to a pose
extern "C" void* quaternion_from_axis_angle(double x, double y, double z, double theta);
extern "C" void free_quaternion_memory(void* q);

extern "C" void* orientation_vector_from_quaternion(void* q);
extern "C" void free_orientation_vector_memory(void* ov);

extern "C" double* orientation_vector_get_components(void* ov);
extern "C" void free_orientation_vector_components(double* ds);

namespace {

// locations of files necessary to build module, specified as relative paths
constexpr char k_output_recipe[] = "/src/control/rtde_output_recipe.txt";
constexpr char k_input_recipe[] = "/src/control/rtde_input_recipe.txt";

// constants for robot operation
constexpr auto k_noop_delay = std::chrono::milliseconds(2);     // 2 millisecond, 500 Hz
constexpr auto k_estop_delay = std::chrono::milliseconds(100);  // 100 millisecond, 10 Hz

constexpr double k_waypoint_equivalancy_epsilon_rad = 1e-4;
constexpr double k_min_timestep_sec = 1e-2;  // determined experimentally, the arm appears to error when given timesteps ~2e-5 and lower

// define callback function to be called by UR client library when program state changes
void reportRobotProgramState(bool program_running) {
    // Print the text in green so we see it better
    // TODO(RSDK-11048): verify side-effects on logstream, rm direct coloring
    VIAM_SDK_LOG(info) << "\033[1;32mUR program running: " << std::boolalpha << program_running << "\033[0m";
}

std::chrono::milliseconds unix_now_ms() {
    namespace chrono = std::chrono;
    return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
}

std::string to_iso_8601(std::chrono::milliseconds time) {
    std::stringstream stream;
    auto tp = std::chrono::time_point<std::chrono::system_clock>{} + time;
    auto tt = std::chrono::system_clock::to_time_t(tp);

    struct tm buf;
    gmtime_r(&tt, &buf);
    stream << std::put_time(&buf, "%FT%T");

    auto delta_us = time % 1000000;
    stream << "." << std::fixed << std::setw(6) << std::setfill('0') << delta_us.count() << "Z";

    return stream.str();
}

template <typename T>
[[nodiscard]] constexpr decltype(auto) degrees_to_radians(T&& degrees) {
    return std::forward<T>(degrees) * (M_PI / 180.0);
}

template <typename T>
[[nodiscard]] constexpr decltype(auto) radians_to_degrees(T&& radians) {
    return std::forward<T>(radians) * (180.0 / M_PI);
}

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

void write_joint_data(
    const vector6d_t& jp, const vector6d_t& jv, std::ostream& of, const std::chrono::milliseconds unix_time, unsigned attempt) {
    of << unix_time.count() << "," << attempt << ",";
    for (const double joint_pos : jp) {
        of << joint_pos << ",";
    }

    unsigned i = 0;
    for (const double joint_velocity : jv) {
        i++;
        if (i == jv.size()) {
            of << joint_velocity;
        } else {
            of << joint_velocity << ",";
        }
    }
    of << "\n";
}

// helper function to extract an attribute value from its key within a ResourceConfig
template <class T>
T find_config_attribute(const ResourceConfig& cfg, const std::string& attribute) {
    std::ostringstream buffer;
    auto key = cfg.attributes().find(attribute);
    if (key == cfg.attributes().end()) {
        buffer << "required attribute `" << attribute << "` not found in configuration";
        throw std::invalid_argument(buffer.str());
    }
    const auto* const val = key->second.get<T>();
    if (!val) {
        buffer << "required non-empty attribute `" << attribute << " could not be decoded";
        throw std::invalid_argument(buffer.str());
    }
    return *val;
}

// NOLINTNEXTLINE(performance-enum-size)
enum class TrajectoryStatus { k_running = 1, k_cancelled = 2, k_stopped = 3 };

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

// private variables to maintain connection and state
struct URArm::state_ {
    std::mutex mu;
    std::unique_ptr<UrDriver> driver;
    std::unique_ptr<DashboardClient> dashboard;

    // data from received robot
    vector6d_t joints_position;
    vector6d_t joints_velocity;
    vector6d_t tcp_state;

    std::atomic<bool> shutdown{false};
    std::atomic<TrajectoryStatus> trajectory_status{TrajectoryStatus::k_stopped};
    std::thread keep_alive_thread;

    // specified through APPDIR environment variable
    std::string appdir;

    // variables specified by ResourceConfig and set through reconfigure
    std::string host;
    std::atomic<double> speed{0};
    std::atomic<double> acceleration{0};
    std::atomic<bool> estop{false};
    std::atomic<bool> local_disconnect{false};

    std::mutex output_csv_dir_path_mu;
    // specified through VIAM_MODULE_DATA environment variable
    std::string output_csv_dir_path;
};

enum class URArm::UrDriverStatus : int8_t  // Only available on 3.10/5.4
{
    NORMAL = 1,
    ESTOPPED = 2,
    READ_FAILURE = 3,
    DASHBOARD_FAILURE = 4
};

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
    using namespace std::placeholders;

    constexpr auto arm_factory = [](Model model, const Dependencies& deps, const ResourceConfig& config) {
        return std::make_unique<URArm>(std::move(model), deps, config);
    };

    const auto arm = API::get<Arm>();
    const auto registration_factory = [&](const Model& model) {
        return std::make_shared<ModelRegistration>(arm, model, std::bind(arm_factory, model, _1, _2));
    };

    return {
        registration_factory(URArm::model("ur3e")),
        registration_factory(URArm::model("ur5e")),
        registration_factory(URArm::model("ur20")),
    };
}

URArm::URArm(Model model, const Dependencies& deps, const ResourceConfig& cfg) : Arm(cfg.name()), model_(std::move(model)) {
    VIAM_SDK_LOG(info) << "URArm constructor called (model: " << model_.to_string() << ")";
    startup_(deps, cfg);
}

void URArm::startup_(const Dependencies&, const ResourceConfig& cfg) {
    if (current_state_) {
        throw std::logic_error("URArm::startup_ was called for an already running instance");
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

    VIAM_SDK_LOG(info) << "URArm starting up";
    current_state_ = std::make_unique<state_>();

    // If we fail to make it through the startup sequence, execute the shutdown code. The
    // shutdown code must be prepared to be called from any intermediate state that this
    // function may have constructed due to partial execution.
    auto failure_handler = make_scope_guard([&] {
        VIAM_SDK_LOG(warn) << "URArm startup failed - shutting down";
        shutdown_();
    });

    // extract relevant attributes from config
    current_state_->host = find_config_attribute<std::string>(cfg, "host");
    current_state_->speed.store(degrees_to_radians(find_config_attribute<double>(cfg, "speed_degs_per_sec")));
    current_state_->acceleration.store(degrees_to_radians(find_config_attribute<double>(cfg, "acceleration_degs_per_sec2")));
    try {
        const std::lock_guard<std::mutex> guard{current_state_->output_csv_dir_path_mu};
        current_state_->output_csv_dir_path = find_config_attribute<std::string>(cfg, "csv_output_path");
    } catch (...) {  // NOLINT: TODO: What should actually happen if the attribute is missing?
    }

    // get the APPDIR environment variable
    auto* tmp = std::getenv("APPDIR");  // NOLINT: Yes, we know getenv isn't thread safe
    if (!tmp) {
        throw std::runtime_error("required environment variable APPDIR unset");
    }
    current_state_->appdir = std::string(tmp);
    VIAM_SDK_LOG(info) << "appdir" << current_state_->appdir;

    {
        const std::lock_guard<std::mutex> guard{current_state_->output_csv_dir_path_mu};
        if (current_state_->output_csv_dir_path.empty()) {
            tmp = std::getenv("VIAM_MODULE_DATA");  // NOLINT: Yes, we know getenv isn't thread safe
            if (!tmp) {
                throw std::runtime_error("required environment variable VIAM_MODULE_DATA unset");
            }
            current_state_->output_csv_dir_path = std::string(tmp);
            VIAM_SDK_LOG(info) << "VIAM_MODULE_DATA" << current_state_->output_csv_dir_path;
        }
    }

    // connect to the robot dashboard
    current_state_->dashboard.reset(new DashboardClient(current_state_->host));
    if (!current_state_->dashboard->connect()) {
        throw std::runtime_error("couldn't connect to dashboard");
    }

    // verify that the connected arm is the same model as the configured module
    std::string actual_model_type{};
    if (!current_state_->dashboard->commandGetRobotModel(actual_model_type)) {
        throw std::runtime_error("failed to get model info of connected arm");
    }

    if (configured_model_type != actual_model_type) {
        std::ostringstream buffer;
        buffer << "configured model type `" << configured_model_type << "` does not match connected arm `" << actual_model_type << "`";
        throw std::runtime_error(buffer.str());
    }

    // stop program, if there is one running
    if (!current_state_->dashboard->commandStop()) {
        throw std::runtime_error("couldn't stop program running on dashboard");
    }

    // if the robot is not powered on and ready
    std::string robotModeRunning("RUNNING");
    while (!current_state_->dashboard->commandRobotMode(robotModeRunning)) {
        // power cycle the arm
        if (!current_state_->dashboard->commandPowerOff()) {
            throw std::runtime_error("couldn't power off arm");
        }
        if (!current_state_->dashboard->commandPowerOn()) {
            throw std::runtime_error("couldn't power on arm");
        }
    }

    // Release the brakes
    if (!current_state_->dashboard->commandBrakeRelease()) {
        throw std::runtime_error("couldn't release the arm brakes");
    }

    constexpr char k_script_file[] = "/src/control/external_control.urscript";

    // Now the robot is ready to receive a program
    const urcl::UrDriverConfiguration ur_cfg = {current_state_->host,
                                                current_state_->appdir + k_script_file,
                                                current_state_->appdir + k_output_recipe,
                                                current_state_->appdir + k_input_recipe,
                                                &reportRobotProgramState,
                                                true,  // headless mode
                                                nullptr};
    current_state_->driver.reset(new UrDriver(ur_cfg));

    // define callback function to be called by UR client library when trajectory state changes
    current_state_->driver->registerTrajectoryDoneCallback(std::bind(&URArm::trajectory_done_cb_, this, std::placeholders::_1));

    // Once RTDE communication is started, we have to make sure to read from the interface buffer,
    // as otherwise we will get pipeline overflows. Therefore, do this directly before starting your
    // main loop
    current_state_->driver->startRTDECommunication();
    int retry_count = 100;
    while (read_joint_keep_alive_(false) != UrDriverStatus::NORMAL) {
        if (retry_count <= 0) {
            throw std::runtime_error("couldn't get joint positions; unable to establish communication with the arm");
        }
        retry_count--;
        std::this_thread::sleep_for(k_noop_delay);
    }

    // start background thread to continuously send no-ops and keep socket connection alive
    VIAM_SDK_LOG(info) << "starting background_thread";
    current_state_->keep_alive_thread = std::thread(&URArm::keep_alive_, this);

    VIAM_SDK_LOG(info) << "URArm startup complete";
    failure_handler.deactivate();
}

void URArm::check_configured_() {
    if (!current_state_) {
        std::ostringstream buffer;
        buffer << "Arm is not currently configured; reconfiguration likely failed";
        throw std::runtime_error(buffer.str());
    }
}

void URArm::trajectory_done_cb_(const control::TrajectoryResult state) {
    std::string report;
    switch (state) {
        case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
            report = "success";
            current_state_->trajectory_status.store(TrajectoryStatus::k_stopped);
            break;
        case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
            report = "canceled";
            current_state_->trajectory_status.store(TrajectoryStatus::k_cancelled);
            break;
        case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
        default:
            current_state_->trajectory_status.store(TrajectoryStatus::k_stopped);
            report = "failure";
    }
    VIAM_SDK_LOG(info) << "\033[1;32mtrajectory report: " << report << "\033[0m";
}

void URArm::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    VIAM_SDK_LOG(warn) << "Reconfigure called: shutting down current state";
    shutdown_();
    VIAM_SDK_LOG(warn) << "Reconfigure: starting up with new state";
    startup_(deps, cfg);
    VIAM_SDK_LOG(info) << "Reconfigure completed OK";
}

std::vector<double> URArm::get_joint_positions(const ProtoStruct&) {
    check_configured_();
    const std::lock_guard<std::mutex> guard{current_state_->mu};
    if (read_joint_keep_alive_(true) == UrDriverStatus::READ_FAILURE) {
        throw std::runtime_error("failed to read from arm");
    };
    std::vector<double> to_ret;
    for (const double joint_pos_rad : current_state_->joints_position) {
        const double joint_pos_deg = radians_to_degrees(joint_pos_rad);
        to_ret.push_back(joint_pos_deg);
    }
    return to_ret;
}

std::string waypoints_filename(const std::string& path, const std::chrono::milliseconds unix_time) {
    constexpr char kWaypointsCsvNameTemplate[] = "/%1%_waypoints.csv";
    auto fmt = boost::format(path + kWaypointsCsvNameTemplate);
    return (fmt % to_iso_8601(unix_time)).str();
}

std::string trajectory_filename(const std::string& path, const std::chrono::milliseconds unix_time) {
    constexpr char kTrajectoryCsvNameTemplate[] = "/%1%_trajectory.csv";
    auto fmt = boost::format(path + kTrajectoryCsvNameTemplate);
    return (fmt % to_iso_8601(unix_time)).str();
}

std::string arm_joint_positions_filename(const std::string& path, const std::chrono::milliseconds unix_time) {
    constexpr char kArmJointPositionsCsvNameTemplate[] = "/%1%_arm_joint_positions.csv";
    auto fmt = boost::format(path + kArmJointPositionsCsvNameTemplate);
    return (fmt % to_iso_8601(unix_time)).str();
}

std::string URArm::get_output_csv_dir_path() {
    std::string path;
    {
        const std::lock_guard<std::mutex> guard{current_state_->output_csv_dir_path_mu};
        path = current_state_->output_csv_dir_path;
    }
    return path;
}

void URArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct&) {
    check_configured_();

    if (current_state_->local_disconnect.load()) {
        throw std::runtime_error("arm is currently in local mode");
    }

    if (current_state_->estop.load()) {
        throw std::runtime_error("move_to_joint_positions cancelled -> emergency stop is currently active");
    }

    auto next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), boost::numeric_cast<Eigen::Index>(positions.size()));
    auto next_waypoint_rad = degrees_to_radians(std::move(next_waypoint_deg));
    std::list<Eigen::VectorXd> waypoints;
    waypoints.emplace_back(std::move(next_waypoint_rad));

    const auto unix_time = unix_now_ms();
    const auto filename = waypoints_filename(get_output_csv_dir_path(), unix_time);
    write_waypoints_to_csv(filename, waypoints);

    // move will throw if an error occurs
    move_(std::move(waypoints), unix_time);
}

void URArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                         const MoveOptions&,
                                         const viam::sdk::ProtoStruct&) {
    check_configured_();

    if (current_state_->local_disconnect.load()) {
        throw std::runtime_error("arm is currently in local mode");
    }
    if (current_state_->estop.load()) {
        throw std::runtime_error("move_through_joint_positions cancelled -> emergency stop is currently active");
    }
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

        const auto unix_time = unix_now_ms();
        const auto filename = waypoints_filename(get_output_csv_dir_path(), unix_time);
        write_waypoints_to_csv(filename, waypoints);

        // move will throw if an error occurs
        move_(std::move(waypoints), unix_time);
    }
}

pose URArm::get_end_position(const ProtoStruct&) {
    check_configured_();
    const std::lock_guard<std::mutex> guard{current_state_->mu};
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = current_state_->driver->getDataPackage();
    if (data_pkg == nullptr) {
        VIAM_SDK_LOG(warn) << "URArm::get_end_position got nullptr from driver->getDataPackage()";
        return pose();
    }
    if (!data_pkg->getData("actual_TCP_pose", current_state_->tcp_state)) {
        VIAM_SDK_LOG(warn) << "URArm::get_end_position driver->getDataPackage().getData(\"actual_TCP_pos\") returned false";
        return pose();
    }
    return ur_vector_to_pose(current_state_->tcp_state);
}

bool URArm::is_moving() {
    check_configured_();
    return current_state_->trajectory_status.load() == TrajectoryStatus::k_running;
}

URArm::KinematicsData URArm::get_kinematics(const ProtoStruct&) {
    check_configured_();

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

    constexpr char kSvaFileTemplate[] = "%1%/src/kinematics/%2%.json";

    const auto sva_file_path = str(boost::format(kSvaFileTemplate) % current_state_->appdir % model_string);

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
    check_configured_();

    if (current_state_->trajectory_status.load() == TrajectoryStatus::k_running) {
        const bool ok = current_state_->driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off());
        if (!ok) {
            VIAM_SDK_LOG(warn) << "URArm::stop driver->writeTrajectoryControlMessage returned false";
            throw std::runtime_error("failed to write trajectory control cancel message to URArm");
        }
    }
}

ProtoStruct URArm::do_command(const ProtoStruct& command) {
    check_configured_();

    ProtoStruct resp = ProtoStruct{};

    constexpr char k_acc_key[] = "set_acc";
    constexpr char k_vel_key[] = "set_vel";
    for (const auto& kv : command) {
        if (kv.first == k_vel_key) {
            const double val = *kv.second.get<double>();
            current_state_->speed.store(degrees_to_radians(val));
            resp.emplace(k_vel_key, val);
        }
        if (kv.first == k_acc_key) {
            const double val = *kv.second.get<double>();
            current_state_->acceleration.store(degrees_to_radians(val));
            resp.emplace(k_acc_key, val);
        }
    }

    return resp;
}

// Send no-ops and keep socket connection alive
void URArm::keep_alive_() {
    VIAM_SDK_LOG(info) << "keep_alive thread started";
    while (true) {
        if (current_state_->shutdown.load()) {
            break;
        }
        {
            const std::lock_guard<std::mutex> guard{current_state_->mu};
            try {
                read_joint_keep_alive_(true);
            } catch (const std::exception& ex) {
                VIAM_SDK_LOG(error) << "keep_alive failed Exception: " << std::string(ex.what());
            }
        }
        std::this_thread::sleep_for(k_noop_delay);
    }
    VIAM_SDK_LOG(info) << "keep_alive thread terminating";
}

void URArm::move_(std::list<Eigen::VectorXd> waypoints, std::chrono::milliseconds unix_time) {
    VIAM_SDK_LOG(info) << "move: start unix_time_ms " << unix_time.count() << " waypoints size " << waypoints.size();

    // get current joint position and add that as starting pose to waypoints
    VIAM_SDK_LOG(info) << "move: get_joint_positions start " << unix_time.count();
    std::vector<double> curr_joint_pos = get_joint_positions(ProtoStruct{});
    VIAM_SDK_LOG(info) << "move: get_joint_positions end" << unix_time.count();

    VIAM_SDK_LOG(info) << "move: compute_trajectory start " << unix_time.count();
    auto curr_waypoint_deg = Eigen::VectorXd::Map(curr_joint_pos.data(), boost::numeric_cast<Eigen::Index>(curr_joint_pos.size()));
    auto curr_waypoint_rad = degrees_to_radians(std::move(curr_waypoint_deg)).eval();
    if (!curr_waypoint_rad.isApprox(waypoints.front(), k_waypoint_equivalancy_epsilon_rad)) {
        waypoints.emplace_front(std::move(curr_waypoint_rad));
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
    const auto max_velocity = Eigen::VectorXd::Constant(6, current_state_->speed.load());
    const auto max_acceleration = Eigen::VectorXd::Constant(6, current_state_->acceleration.load());
    VIAM_SDK_LOG(info) << "generating trajectory with max speed: " << radians_to_degrees(max_velocity[0]);

    std::vector<trajectory_sample_point> samples;

    for (const auto& segment : segments) {
        const Trajectory trajectory(Path(segment, 0.1), max_velocity, max_acceleration);
        trajectory.outputPhasePlaneTrajectory();
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

        // desired sampling frequency. if the duration is small we will oversample but that should be fine.
        constexpr double k_sampling_freq_hz = 5;
        sampling_func(samples, duration, k_sampling_freq_hz, [&](const double t, const double step) {
            auto p_eigen = trajectory.getPosition(t);
            auto v_eigen = trajectory.getVelocity(t);
            return trajectory_sample_point{{p_eigen[0], p_eigen[1], p_eigen[2], p_eigen[3], p_eigen[4], p_eigen[5]},
                                           {v_eigen[0], v_eigen[1], v_eigen[2], v_eigen[3], v_eigen[4], v_eigen[5]},
                                           boost::numeric_cast<float>(step)};
        });
    }
    VIAM_SDK_LOG(info) << "move: compute_trajectory end " << unix_time.count() << " samples.size() " << samples.size() << " segments "
                       << segments.size() - 1;

    const std::string path = get_output_csv_dir_path();
    write_trajectory_to_file(trajectory_filename(path, unix_time), samples);
    {  // note the open brace which introduces a new variable scope
        // construct a lock_guard: locks the mutex on construction and unlocks on destruction
        const std::lock_guard<std::mutex> guard{current_state_->mu};
        {
            UrDriverStatus status;
            for (unsigned i = 0; i < 5; i++) {
                status = read_joint_keep_alive_(true);
                if (status == UrDriverStatus::NORMAL) {
                    break;
                }
            }
            if (status != UrDriverStatus::NORMAL) {
                throw std::runtime_error("unable to get arm state before send_trajectory");
            }
        }
        if (!send_trajectory_(samples)) {
            throw std::runtime_error("send_trajectory failed");
        };

        std::ofstream of(arm_joint_positions_filename(path, unix_time));

        of << "time_ms,read_attempt,"
              "joint_0_pos,joint_1_pos,joint_2_pos,joint_3_pos,joint_4_pos,joint_5_pos,"
              "joint_0_vel,joint_1_vel,joint_2_vel,joint_3_vel,joint_4_vel,joint_5_vel\n";
        unsigned attempt = 0;
        UrDriverStatus status;
        while ((current_state_->trajectory_status.load() == TrajectoryStatus::k_running) && !current_state_->shutdown.load()) {
            const auto unix_time = unix_now_ms();
            status = read_joint_keep_alive_(true);
            if (status != UrDriverStatus::NORMAL) {
                break;
            }
            write_joint_data(current_state_->joints_position, current_state_->joints_velocity, of, unix_time, attempt++);
        };
        if (current_state_->shutdown.load()) {
            of.close();
            throw std::runtime_error("interrupted by shutdown");
        }

        of.close();
        VIAM_SDK_LOG(info) << "move: end unix_time " << unix_time.count();

        if (current_state_->trajectory_status.load() == TrajectoryStatus::k_cancelled) {
            throw std::runtime_error("arm's current trajectory cancelled by code");
        }

        switch (status) {
            case UrDriverStatus::ESTOPPED:
                throw std::runtime_error("arm is estopped");
            case UrDriverStatus::READ_FAILURE:
                throw std::runtime_error("arm failed to retrieve data packet");
            case UrDriverStatus::DASHBOARD_FAILURE:
                throw std::runtime_error("arm dashboard is disconnected");
            case UrDriverStatus::NORMAL:
                return;
        }
    }
}

std::string URArm::status_to_string_(UrDriverStatus status) {
    switch (status) {
        case UrDriverStatus::ESTOPPED:
            return "ESTOPPED";
        case UrDriverStatus::READ_FAILURE:
            return "READ_FAILURE";
        case UrDriverStatus::DASHBOARD_FAILURE:
            return "DASHBOARD_FAILURE";
        case UrDriverStatus::NORMAL:
            return "NORMAL";
    }
    return "no status";
}

// Define the destructor
URArm::~URArm() {
    VIAM_SDK_LOG(warn) << "URArm destructor called, shutting down";
    shutdown_();
    VIAM_SDK_LOG(warn) << "URArm destroyed";
}

void URArm::shutdown_() noexcept {
    try {
        VIAM_SDK_LOG(warn) << "URArm shutdown called";
        if (current_state_) {
            const auto destroy_state = make_scope_guard([&] { current_state_.reset(); });

            current_state_->shutdown.store(true);
            // stop the robot
            VIAM_SDK_LOG(info) << "URArm shutdown calling stop";
            stop(ProtoStruct{});
            // disconnect from the dashboard
            if (current_state_->dashboard) {
                VIAM_SDK_LOG(info) << "URArm shutdown calling dashboard->disconnect()";
                current_state_->dashboard->disconnect();
            }
            if (current_state_->keep_alive_thread.joinable()) {
                VIAM_SDK_LOG(info) << "URArm shutdown waiting for keep_alive thread to terminate";
                current_state_->keep_alive_thread.join();
                VIAM_SDK_LOG(info) << "keep_alive thread terminated";
            }
            VIAM_SDK_LOG(info) << "URArm shutdown complete";
        }
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

// helper function to send time-indexed position, velocity, acceleration setpoints to the UR driver
bool URArm::send_trajectory_(const std::vector<trajectory_sample_point>& samples) {
    VIAM_SDK_LOG(info) << "URArm::send_trajectory start";
    auto point_number = static_cast<int>(samples.size());
    if (!current_state_->driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_START, point_number, RobotReceiveTimeout::off())) {
        VIAM_SDK_LOG(error) << "send_trajectory driver->writeTrajectoryControlMessage returned false";
        return false;
    };

    current_state_->trajectory_status.store(TrajectoryStatus::k_running);
    VIAM_SDK_LOG(info) << "URArm::send_trajectory sending " << samples.size() << " cubic writeTrajectorySplinePoint/3";
    for (size_t i = 0; i < samples.size(); i++) {
        if (!current_state_->driver->writeTrajectorySplinePoint(samples[i].p, samples[i].v, samples[i].timestep)) {
            VIAM_SDK_LOG(error) << "send_trajectory cubic driver->writeTrajectorySplinePoint returned false";
            return false;
        };
    }

    VIAM_SDK_LOG(info) << "URArm::send_trajectory end";
    return true;
}

// helper function to read a data packet and send a noop message
URArm::UrDriverStatus URArm::read_joint_keep_alive_(bool log) {
    // check to see if an estop has occurred.
    std::string status;
    try {
        if (current_state_->local_disconnect.load()) {
            if (current_state_->dashboard->commandIsInRemoteControl()) {
                // reconnect to the tablet. We have to do this, otherwise the client will assume that the arm is still in local mode.
                // yes, even though the client can already recognize that we are in remote control mode
                current_state_->dashboard->disconnect();
                if (!current_state_->dashboard->connect()) {
                    return UrDriverStatus::DASHBOARD_FAILURE;
                }
                // reset the driver client so we are not asking for data
                current_state_->driver->resetRTDEClient(current_state_->appdir + k_output_recipe, current_state_->appdir + k_input_recipe);
                // reset the primary client so the driver is aware the arm is back in remote mode
                // this has to happen, otherwise when an estop is triggered the arm will return error code C210A0
                current_state_->driver->stopPrimaryClientCommunication();
                current_state_->driver->startPrimaryClientCommunication();

                // check if we can still send trajectories. if not we need to send the robot program again.
                if (!current_state_->driver->isReverseInterfaceConnected() || !current_state_->driver->isTrajectoryInterfaceConnected()) {
                    // send control script again to complete the restart
                    if (!current_state_->driver->sendRobotProgram()) {
                        throw std::runtime_error(
                            "read_joint_keep_alive driver->sendRobotProgram() returned false when attempting to reconnect to the arm");
                    }
                }

                // turn communication with the RTDE client back on so we can receive data from the arm
                current_state_->driver->startRTDECommunication();

                // reset the flag and return to the "happy" path
                current_state_->local_disconnect.store(false);
                VIAM_SDK_LOG(info) << "recovered from local mode";
                return UrDriverStatus::NORMAL;
            }
            VIAM_SDK_LOG(error) << "arm is still in local mode";
        }
        if (!current_state_->dashboard->commandSafetyStatus(status)) {
            // We should not end up in here, as commandSafetyStatus will probably throw before returning false
            VIAM_SDK_LOG(error) << "read_joint_keep_alive dashboard->commandSafetyStatus() returned false when retrieving the status";
            return UrDriverStatus::DASHBOARD_FAILURE;
        }
    } catch (const std::exception& ex) {
        // if we end up here that means we can no longer talk to the arm. store the state so we can try to recover from this.
        current_state_->local_disconnect.store(true);

        // attempt to reconnect to the arm. Even if we reconnect, we will have to check that the tablet is not in local mode.
        // we reconnect in the catch to attempt to limit the amount of times we connect to the arm to avoid a "no controller" error
        // https://forum.universal-robots.com/t/no-controller-error-on-real-robot/3127
        current_state_->dashboard->disconnect();
        if (!current_state_->dashboard->connect()) {
            // we failed to reconnect to the tablet, so we might not even be able to talk to it.
            // return an error so we can attempt to reconnect again
            return UrDriverStatus::DASHBOARD_FAILURE;
        }

        // reset the driver client so we stop trying to ask for more data
        current_state_->driver->resetRTDEClient(current_state_->appdir + k_output_recipe, current_state_->appdir + k_input_recipe);

        // delay so we don't spam the dashboard client if disconnected
        std::this_thread::sleep_for(k_estop_delay);

        // start capturing data from the arm driver again
        current_state_->driver->startRTDECommunication();

        VIAM_SDK_LOG(error) << "failed to talk to the arm, is the tablet in local mode? : " << std::string(ex.what());
    }

    // check if the arm status is normal or not empty. the status will be empty if the arm is disconnected or when the arm is first switched
    // into local mode
    if ((status.find(urcl::safetyStatusString(urcl::SafetyStatus::NORMAL)) == std::string::npos) && !status.empty()) {
        VIAM_SDK_LOG(info) << "read_joint_keep_alive dashboard->commandSafetyStatus() arm status : " << status;
        // the arm is currently estopped. if the user is not in local mode, save this state.
        // if the user is in local mode, then we assume that they opted into this state.
        // the UR20 will always return a stop while in manual mode, unless the three position enabling button is held down
        if (!current_state_->local_disconnect.load()) {
            current_state_->estop.store(true);
        }

        // clear any currently running trajectory.
        current_state_->trajectory_status.store(TrajectoryStatus::k_stopped);

        // TODO: further investigate the need for this delay
        // sleep longer to prevent buffer error
        // Removing this will cause the RTDE client to move into an unrecoverable state
        std::this_thread::sleep_for(k_estop_delay);

    } else {
        // the arm is in a normal state.
        if (current_state_->estop.load()) {
            // if the arm was previously estopped, attempt to recover from the estop.
            // We should not enter this code without the user interacting with the arm in some way(i.e. resetting the estop)
            try {
                VIAM_SDK_LOG(info) << "recovering from e-stop";
                current_state_->driver->resetRTDEClient(current_state_->appdir + k_output_recipe, current_state_->appdir + k_input_recipe);

                VIAM_SDK_LOG(info) << "restarting arm";
                if (!current_state_->dashboard->commandPowerOff()) {
                    throw std::runtime_error(
                        "read_joint_keep_alive dashboard->commandPowerOff() returned false when attempting to restart the arm");
                }

                if (!current_state_->dashboard->commandPowerOn()) {
                    throw std::runtime_error(
                        "read_joint_keep_alive dashboard->commandPowerOn() returned false when attempting to restart the arm");
                }
                // Release the brakes
                if (!current_state_->dashboard->commandBrakeRelease()) {
                    throw std::runtime_error(
                        "read_joint_keep_alive dashboard->commandBrakeRelease() returned false when attempting to restart the arm");
                }

                // send control script again to complete the restart
                if (!current_state_->driver->sendRobotProgram()) {
                    throw std::runtime_error(
                        "read_joint_keep_alive driver->sendRobotProgram() returned false when attempting to restart the arm");
                }

            } catch (const std::exception& ex) {
                VIAM_SDK_LOG(info) << "failed to restart the arm: : " << std::string(ex.what());
                return UrDriverStatus::ESTOPPED;
            }

            VIAM_SDK_LOG(info) << "send robot program successful, restarting communication";
            current_state_->driver->startRTDECommunication();

            current_state_->estop.store(false);
            VIAM_SDK_LOG(info) << "arm successfully recovered from estop";
            return UrDriverStatus::NORMAL;
        }
    }

    std::unique_ptr<rtde_interface::DataPackage> data_pkg = current_state_->driver->getDataPackage();
    if (data_pkg == nullptr) {
        // we received no data packet, so our comms are down. reset the comms from the driver.
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->getDataPackage() returned nullptr. resetting RTDE client connection";
        }
        try {
            current_state_->driver->resetRTDEClient(current_state_->appdir + k_output_recipe, current_state_->appdir + k_input_recipe);
        } catch (const std::exception& ex) {
            if (log) {
                VIAM_SDK_LOG(error) << "read_joint_keep_alive driver RTDEClient failed to restart: " << std::string(ex.what());
            }
            return UrDriverStatus::READ_FAILURE;
        }
        current_state_->driver->startRTDECommunication();
        if (log) {
            VIAM_SDK_LOG(info) << "RTDE client connection successfully restarted";
        }
        // we restarted the RTDE client so the robot should be back to a normal state
        return UrDriverStatus::NORMAL;
    }

    // read current joint positions from robot data
    vector6d_t joints_position{};
    if (!data_pkg->getData("actual_q", joints_position)) {
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->getDataPackage()->data_pkg->getData(\"actual_q\") returned false";
        }
        return UrDriverStatus::READ_FAILURE;
    }

    // read current joint velocities from robot data
    vector6d_t joints_velocity{};
    if (!data_pkg->getData("actual_qd", joints_velocity)) {
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->getDataPackage()->data_pkg->getData(\"actual_qd\") returned false";
        }
        return UrDriverStatus::READ_FAILURE;
    }

    // for consistency, update cached data only after all getData calls succeed
    current_state_->joints_position = std::move(joints_position);
    current_state_->joints_velocity = std::move(joints_velocity);

    // send a noop to keep the connection alive
    if (!current_state_->driver->writeTrajectoryControlMessage(
            control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->writeTrajectoryControlMessage returned false";
        }
        // technically this should be driver error but I am gonna be lazy until we do the refactor here.
        return UrDriverStatus::DASHBOARD_FAILURE;
    }

    // check if we detect an estop. while estopped we could still retrieve data from the arm
    if (current_state_->estop.load()) {
        return UrDriverStatus::ESTOPPED;
    }
    return UrDriverStatus::NORMAL;
}
