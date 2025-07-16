#include "ur_arm.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <thread>

#include <boost/format.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/range/adaptor/transformed.hpp>

#include <ur_client_library/log.h>
#include <ur_client_library/types.h>
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

void write_joint_data(const vector6d_t& jp, const vector6d_t& jv, std::ostream& of, const std::string& unix_time, std::size_t attempt) {
    of << unix_time << "," << attempt << ",";
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

class URArmLogHandler : public urcl::LogHandler {
   public:
    URArmLogHandler() = default;

    void log(const char* file, int line, urcl::LogLevel loglevel, const char* log) override {
        switch (loglevel) {
            case urcl::LogLevel::INFO:
                VIAM_SDK_LOG(info) << "URCL - " << file << " " << line << ": " << log;
                break;
            case urcl::LogLevel::DEBUG:
                VIAM_SDK_LOG(debug) << "URCL - " << file << " " << line << ": " << log;
                break;
            case urcl::LogLevel::WARN:
                VIAM_SDK_LOG(warn) << "URCL - " << file << " " << line << ": " << log;
                break;
            case urcl::LogLevel::ERROR:
                VIAM_SDK_LOG(error) << "URCL - " << file << " " << line << ": " << log;
                break;
            case urcl::LogLevel::FATAL:
                VIAM_SDK_LOG(error) << "URCL - " << file << " " << line << ": " << log;
                break;
            default:
                break;
        }
    }
};

void configure_logger(const ResourceConfig& cfg) {
    std::string level_str{};
    try {
        level_str = find_config_attribute<std::string>(cfg, "log_level");
    } catch (...) {
        level_str = "debug";
    }
    const auto level = [&] {
        if (level_str == "info") {
            return urcl::LogLevel::INFO;
        } else if (level_str == "debug") {
            return urcl::LogLevel::DEBUG;
        } else if (level_str == "warn") {
            return urcl::LogLevel::WARN;
        } else if (level_str == "error") {
            return urcl::LogLevel::ERROR;
        } else if (level_str == "fatal") {
            return urcl::LogLevel::FATAL;
        } else {
            VIAM_SDK_LOG(error) << "invalid log_level: '" << level_str << "' - defaulting to 'debug'";
            return urcl::LogLevel::DEBUG;
        }
    }();
    urcl::setLogLevel(level);
    urcl::registerLogHandler(std::make_unique<URArmLogHandler>());
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

enum class URArm::UrDriverStatus : int8_t  // Only available on 3.10/5.4
{
    NORMAL = 1,
    ESTOPPED = 2,
    READ_FAILURE = 3,
    DASHBOARD_FAILURE = 4
};

// private variables to maintain connection and state
struct URArm::state_ {
    std::mutex state_mutex;
    std::unique_ptr<UrDriver> driver;
    std::unique_ptr<DashboardClient> dashboard;

    // data from received robot
    UrDriverStatus last_driver_status{UrDriverStatus::READ_FAILURE};
    std::optional<vector6d_t> joints_position;
    std::optional<vector6d_t> joints_velocity;
    std::optional<vector6d_t> tcp_state;

    std::atomic<bool> shutdown{false};
    std::atomic<TrajectoryStatus> trajectory_status{TrajectoryStatus::k_stopped};
    std::thread keep_alive_thread;

    struct move_request {
       public:
        explicit move_request(std::vector<trajectory_sample_point>&& samples, std::ofstream arm_joint_positions_stream)
            : samples(std::move(samples)), arm_joint_positions_stream(std::move(arm_joint_positions_stream)) {
            if (this->samples.empty()) {
                throw std::invalid_argument("no trajectory samples provided to move request");
            }
        }

        // TODO(RSDK-11290): Since the `get_future` here should really only be called once,
        // it might be better later to have `state` have a function to enqueue
        // a move request and return the associated future.
        auto get_completion_future() {
            return completion.get_future();
        }

        auto cancel() {
            if (!cancellation_request) {
                auto& cr = cancellation_request.emplace();
                cr.future = cr.promise.get_future().share();
            }
            return cancellation_request->future;
        }

        void complete_success() {
            // Mark the move_request as completed. If there
            // was a cancel request, it raced and lost, but it doesn't
            // need an error.
            completion.set_value();
            if (cancellation_request) {
                cancellation_request->promise.set_value();
            }
        }

        void complete_cancelled() {
            complete_error("arm's current trajectory cancelled");
        }

        void complete_failure() {
            complete_error("arm's current trajectory failed");
        }

        void complete_error(std::string_view message) {
            // The trajectory is being completed with an error of some sort. Set the completion result to an error,
            // and unblock any cancellation request.
            completion.set_exception(std::make_exception_ptr(std::runtime_error{std::string{message}}));
            if (cancellation_request) {
                cancellation_request->promise.set_value();
            }
            VIAM_SDK_LOG(warn) << "A trajectory completed with an error: " << message;
        }

        void cancel_error(std::string_view message) {
            std::exchange(cancellation_request, {})
                ->promise.set_exception(std::make_exception_ptr(std::runtime_error{std::string{message}}));
        }

        void write_joint_data(vector6d_t& position, vector6d_t& velocity) {
            ::write_joint_data(position, velocity, arm_joint_positions_stream, unix_time_iso8601(), arm_joint_positions_sample++);
        }

        std::vector<trajectory_sample_point> samples;
        std::ofstream arm_joint_positions_stream;
        std::size_t arm_joint_positions_sample{0};
        std::promise<void> completion;

        struct cancellation_request {
            // This constructor needs to be written this way for
            // std::optional::emplace with no arguments to work.
            cancellation_request() {}

            std::promise<void> promise;
            std::shared_future<void> future;
            bool issued{false};
        };

        std::optional<cancellation_request> cancellation_request;
    };
    std::optional<move_request> move_request;

    // specified through APPDIR environment variable
    std::string appdir;

    // variables specified by ResourceConfig and set through reconfigure
    std::string host;
    std::atomic<double> speed{0};
    std::atomic<double> acceleration{0};
    std::atomic<bool> estop{false};
    std::atomic<bool> local_disconnect{false};

    // variables derived from arm state on configuration
    bool is_sim;

    // specified through VIAM_MODULE_DATA environment variable
    std::string output_csv_dir_path;
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
            [model](auto deps, auto config) { return std::make_unique<URArm>(model, deps, config); });
    };

    auto registrations = model_strings | boost::adaptors::transformed(registration_factory);
    return {std::make_move_iterator(begin(registrations)), std::make_move_iterator(end(registrations))};
}

URArm::URArm(Model model, const Dependencies& deps, const ResourceConfig& cfg) : Arm(cfg.name()), model_(std::move(model)) {
    VIAM_SDK_LOG(info) << "URArm constructor called (model: " << model_.to_string() << ")";
    const std::unique_lock wlock(config_mutex_);
    configure_(wlock, deps, cfg);
    configure_logger(cfg);
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

    VIAM_SDK_LOG(info) << "URArm starting up";
    current_state_ = std::make_unique<state_>();

    // If we fail to make it through the startup sequence, execute the shutdown code. The
    // shutdown code must be prepared to be called from any intermediate state that this
    // function may have constructed due to partial execution.
    auto failure_handler = make_scope_guard([&] {
        VIAM_SDK_LOG(warn) << "URArm startup failed - shutting down";
        shutdown_(lock);
    });

    // extract relevant attributes from config
    current_state_->host = find_config_attribute<std::string>(cfg, "host");
    current_state_->speed.store(degrees_to_radians(find_config_attribute<double>(cfg, "speed_degs_per_sec")));
    current_state_->acceleration.store(degrees_to_radians(find_config_attribute<double>(cfg, "acceleration_degs_per_sec2")));
    try {
        current_state_->output_csv_dir_path = find_config_attribute<std::string>(cfg, "csv_output_path");
    } catch (...) {  // NOLINT: TODO(RSDK-11299): What should actually happen if the attribute is missing?
    }

    // get the APPDIR environment variable
    auto* tmp = std::getenv("APPDIR");  // NOLINT: Yes, we know getenv isn't thread safe
    if (!tmp) {
        throw std::runtime_error("required environment variable APPDIR unset");
    }
    current_state_->appdir = std::string(tmp);
    VIAM_SDK_LOG(info) << "appdir" << current_state_->appdir;

    if (current_state_->output_csv_dir_path.empty()) {
        tmp = std::getenv("VIAM_MODULE_DATA");  // NOLINT: Yes, we know getenv isn't thread safe
        if (!tmp) {
            throw std::runtime_error("required environment variable VIAM_MODULE_DATA unset");
        }
        current_state_->output_csv_dir_path = std::string(tmp);
        VIAM_SDK_LOG(info) << "VIAM_MODULE_DATA" << current_state_->output_csv_dir_path;
    }

    // connect to the robot dashboard
    current_state_->dashboard.reset(new DashboardClient(current_state_->host));
    if (!current_state_->dashboard->connect(1)) {
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

    // If we made it to this part of the code, then the arm is on and we can successfully talk to it.
    // A physical/real arm can only be controlled this way if the dashboard is in remote mode.
    // Conversely, a simulation arm will always report that the arm is in local mode.
    // Using this, we can determine whether the arm is a simulation arm by calling dashboard->commandIsInRemoteControl at this point.
    // we will capture this state to use within the worker thread.
    current_state_->is_sim = !current_state_->dashboard->commandIsInRemoteControl();

    constexpr char k_script_file[] = "/src/control/external_control.urscript";

    // Now the robot is ready to receive a program
    auto ur_cfg = urcl::UrDriverConfiguration{};
    ur_cfg.robot_ip = current_state_->host;
    ur_cfg.script_file = current_state_->appdir + k_script_file;
    ur_cfg.output_recipe_file = current_state_->appdir + k_output_recipe;
    ur_cfg.input_recipe_file = current_state_->appdir + k_input_recipe;
    ur_cfg.handle_program_state = &reportRobotProgramState;
    ur_cfg.headless_mode = true;
    ur_cfg.socket_reconnect_attempts = 1;

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

template <template <typename> typename lock_type>
void URArm::check_configured_(const lock_type<std::shared_mutex>&) {
    if (!current_state_) {
        std::ostringstream buffer;
        buffer << "Arm is not currently configured; reconfiguration likely failed";
        throw std::runtime_error(buffer.str());
    }
}

void URArm::trajectory_done_cb_(const control::TrajectoryResult state) {
    const std::lock_guard guard{current_state_->state_mutex};
    std::string report;

    // Take ownership of any move request so we open the slot for the next one.
    //
    // TODO(RSDK-11298): Could we move the completions out of the critical section?
    //
    // TODO(RSDK-11298): Do we still need `trajectory_status`
    auto move_request = std::exchange(current_state_->move_request, {});

    switch (state) {
        case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS: {
            report = "success";
            if (move_request) {
                move_request->complete_success();
            }
            current_state_->trajectory_status.store(TrajectoryStatus::k_stopped);
            break;
        }
        case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED: {
            report = "canceled";
            if (move_request) {
                move_request->complete_cancelled();
            }
            current_state_->trajectory_status.store(TrajectoryStatus::k_cancelled);
            break;
        }
        case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
        default: {
            report = "failure";
            if (move_request) {
                move_request->complete_failure();
            }
            current_state_->trajectory_status.store(TrajectoryStatus::k_stopped);
            break;
        }
    }

    VIAM_SDK_LOG(info) << "\033[1;32mtrajectory report: " << report << "\033[0m";
}

void URArm::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    const std::unique_lock wlock{config_mutex_};
    check_configured_(wlock);
    VIAM_SDK_LOG(warn) << "Reconfigure called: shutting down current state";
    shutdown_(wlock);
    VIAM_SDK_LOG(warn) << "Reconfigure called: configuring new state";
    configure_(wlock, deps, cfg);
    VIAM_SDK_LOG(info) << "Reconfigure completed OK";
}

std::vector<double> URArm::get_joint_positions(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    return get_joint_positions_(rlock);
}

std::vector<double> URArm::get_joint_positions_(const std::shared_lock<std::shared_mutex>&) {
    const std::lock_guard guard{current_state_->state_mutex};
    if (current_state_->last_driver_status == UrDriverStatus::READ_FAILURE) {
        // TODO(RSDK-11295): provide more context
        throw std::runtime_error("get_joint_positions: failed to read from arm");
    }

    if (!current_state_->joints_position) {
        // TODO(RSDK-11295): provide more context, when might this happen?
        throw std::runtime_error("get_joint_positions: joint position is not currently known");
    }

    std::vector<double> to_ret;
    for (const double joint_pos_rad : *current_state_->joints_position) {
        const double joint_pos_deg = radians_to_degrees(joint_pos_rad);
        to_ret.push_back(joint_pos_deg);
    }
    return to_ret;
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

void URArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

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

    const auto unix_time = unix_time_iso8601();
    const auto filename = waypoints_filename(current_state_->output_csv_dir_path, unix_time);

    write_waypoints_to_csv(filename, waypoints);

    // move will throw if an error occurs
    move_(std::move(rlock), std::move(waypoints), unix_time);
}

void URArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                         const MoveOptions&,
                                         const viam::sdk::ProtoStruct&) {
    std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

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

        const auto unix_time = unix_time_iso8601();
        const auto filename = waypoints_filename(current_state_->output_csv_dir_path, unix_time);

        write_waypoints_to_csv(filename, waypoints);

        // move will throw if an error occurs
        move_(std::move(rlock), std::move(waypoints), unix_time);
    }
}

pose URArm::get_end_position(const ProtoStruct&) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

    const std::lock_guard guard{current_state_->state_mutex};
    if (current_state_->last_driver_status == UrDriverStatus::READ_FAILURE) {
        // TODO(RSDK-11295): provide more context
        throw std::runtime_error("get_end_position: failed to read from arm");
    }

    if (!current_state_->tcp_state) {
        // TODO(RSDK-11295): provide more context, when might this happen?
        throw std::runtime_error("get_end_position: end position data are not currently known");
    }

    return ur_vector_to_pose(*current_state_->tcp_state);
}

bool URArm::is_moving() {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    return current_state_->trajectory_status.load() == TrajectoryStatus::k_running;
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
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);
    stop_(rlock);
}

ProtoStruct URArm::do_command(const ProtoStruct& command) {
    const std::shared_lock rlock{config_mutex_};
    check_configured_(rlock);

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
            const std::lock_guard guard{current_state_->state_mutex};
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

void URArm::move_(std::shared_lock<std::shared_mutex> config_rlock, std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
    auto our_config_rlock = std::move(config_rlock);

    VIAM_SDK_LOG(info) << "move: start unix_time_ms " << unix_time << " waypoints size " << waypoints.size();
    const auto log_move_end = make_scope_guard([&] { VIAM_SDK_LOG(info) << "move: end unix_time " << unix_time; });

    // get current joint position and add that as starting pose to waypoints
    VIAM_SDK_LOG(info) << "move: get_joint_positions start " << unix_time;
    std::vector<double> curr_joint_pos = get_joint_positions_(our_config_rlock);
    VIAM_SDK_LOG(info) << "move: get_joint_positions end " << unix_time;

    VIAM_SDK_LOG(info) << "move: compute_trajectory start " << unix_time;
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
    VIAM_SDK_LOG(info) << "move: compute_trajectory end " << unix_time << " samples.size() " << samples.size() << " segments "
                       << segments.size() - 1;

    const std::string& path = current_state_->output_csv_dir_path;
    write_trajectory_to_file(trajectory_filename(path, unix_time), samples);

    std::ofstream ajp_of(arm_joint_positions_filename(path, unix_time));
    ajp_of << "time_ms,read_attempt,"
              "joint_0_pos,joint_1_pos,joint_2_pos,joint_3_pos,joint_4_pos,joint_5_pos,"
              "joint_0_vel,joint_1_vel,joint_2_vel,joint_3_vel,joint_4_vel,joint_5_vel\n";

    auto trajectory_completion_future = [&, config_rlock = std::move(our_config_rlock), ajp_of = std::move(ajp_of)]() mutable {
        const std::lock_guard guard{current_state_->state_mutex};
        if (current_state_->move_request) {
            throw std::runtime_error("An actuation is already in progress");
        }
        // TODO(RSDK-11296): There is a race here, where if we planned, then
        // another move started and completed, and then we execute,
        // that the starting position we used is stale. See
        // https://github.com/viam-modules/universal-robots/pull/79/files#r2195721629
        // for more discussion. Some mechanism to prevent that case
        // should be implemented.
        return current_state_->move_request.emplace(std::move(samples), std::move(ajp_of)).get_completion_future();
    }();

    // NOTE: The configuration read lock is no longer held after the above statement. Do not interact
    // with the current state other than to wait on the result of this future.
    trajectory_completion_future.get();
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
    const std::unique_lock wlock{config_mutex_};
    shutdown_(wlock);
    VIAM_SDK_LOG(warn) << "URArm destroyed";
}

template <template <typename> typename lock_type>
void URArm::stop_(const lock_type<std::shared_mutex>&) {
    auto cancel_future = [&]() -> std::optional<std::shared_future<void>> {
        const std::lock_guard guard{current_state_->state_mutex};
        if (current_state_->move_request) {
            return std::make_optional(current_state_->move_request->cancel());
        }
        return {};
    }();

    if (cancel_future) {
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

            // stop the worker thread.
            // Do this first to prevent the worker thread from turning the dashboard client back on when the thread detects the disconnect.
            current_state_->shutdown.store(true);
            if (current_state_->keep_alive_thread.joinable()) {
                VIAM_SDK_LOG(info) << "URArm shutdown waiting for keep_alive thread to terminate";
                current_state_->keep_alive_thread.join();
                VIAM_SDK_LOG(info) << "keep_alive thread terminated";
            }

            // disconnect from the dashboard.
            if (current_state_->dashboard) {
                VIAM_SDK_LOG(info) << "URArm shutdown calling dashboard->disconnect()";
                current_state_->dashboard->disconnect();
            }
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

URArm::UrDriverStatus URArm::read_joint_keep_alive_(bool log) {
    current_state_->joints_position.reset();
    current_state_->joints_velocity.reset();
    current_state_->tcp_state.reset();
    current_state_->last_driver_status = read_joint_keep_alive_inner_(log);

    if (current_state_->move_request) {
        if (current_state_->last_driver_status == UrDriverStatus::NORMAL) {
            // If we are in a normal state, deal with issuing and canceling trajectories
            if (!current_state_->move_request->samples.empty() && !current_state_->move_request->cancellation_request) {
                // We have a move request, it has samples, and there is no pending cancel for that move. Issue the move.
                auto samples = std::move(current_state_->move_request->samples);
                if (!send_trajectory_(samples)) {
                    std::exchange(current_state_->move_request, {})->complete_error("failed to send trajectory to arm");
                };
            } else if (current_state_->move_request->samples.empty() && current_state_->move_request->cancellation_request &&
                       !current_state_->move_request->cancellation_request->issued) {
                // We have a move request, the samples have been forwarded,
                // and cancellation is requested but has not yet been issued. Issue a cancel.
                current_state_->move_request->cancellation_request->issued = true;
                if (!current_state_->driver->writeTrajectoryControlMessage(
                        urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off())) {
                    current_state_->move_request->cancel_error("failed to write trajectory control cancel message to URArm");
                }
            } else if (!current_state_->move_request->samples.empty() && current_state_->move_request->cancellation_request) {
                // We have a move request that we haven't issued but a
                // cancel is already pending. Don't issue it, just cancel it.
                std::exchange(current_state_->move_request, {})->complete_cancelled();
            } else {
                current_state_->move_request->write_joint_data(*current_state_->joints_position, *current_state_->joints_velocity);
            }
        } else {
            // We have entered an abnormal state. Complete the move request with an error.
            std::exchange(current_state_->move_request, {})->complete_error([&] {
                switch (current_state_->last_driver_status) {
                    case UrDriverStatus::ESTOPPED:
                        return "arm is estopped";
                    case UrDriverStatus::READ_FAILURE:
                        return "arm failed to retrieve current state";
                    case UrDriverStatus::DASHBOARD_FAILURE:
                        return "arm dashboard is disconnected";
                    default:
                        abort();  // impossible
                }
            }());
        }
    }

    return current_state_->last_driver_status;
}

// helper function to read a data packet and send a noop message
URArm::UrDriverStatus URArm::read_joint_keep_alive_inner_(bool log) {
    // check to see if an estop has occurred.
    std::string status;
    try {
        if (current_state_->local_disconnect.load()) {
            // check if the arm is in remote mode. Sim arms will always report in local mode.
            // if the arm is disconnected, commandIsInRemoteControl() will throw instead.
            if (current_state_->dashboard->commandIsInRemoteControl() || current_state_->is_sim) {
                // reconnect to the tablet. We have to do this, otherwise the client will assume that the arm is still in local mode.
                // yes, even though the client can already recognize that we are in remote control mode
                current_state_->dashboard->disconnect();
                if (!current_state_->dashboard->connect(1)) {
                    return UrDriverStatus::DASHBOARD_FAILURE;
                }
                // reset the driver client so we are not asking for data
                current_state_->driver->resetRTDEClient(current_state_->appdir + k_output_recipe, current_state_->appdir + k_input_recipe);
                // reset the primary client so the driver is aware the arm is back in remote mode
                // this has to happen, otherwise when an estop is triggered the arm will return error code C210A0
                current_state_->driver->stopPrimaryClientCommunication();
                current_state_->driver->startPrimaryClientCommunication();

                std::string robot_mode;
                if (!current_state_->dashboard->commandRobotMode(robot_mode)) {
                    throw std::runtime_error("read_joint_keep_alive dashboard->commandRobotMode() failed to retrieve robot mode");
                }

                // if the robot is not running, we want to restart the arm fully to make sure we are ready
                // we will cheat by going into the estop code
                if (robot_mode.find(urcl::robotModeString(urcl::RobotMode::RUNNING)) == std::string::npos) {
                    // we are not ready to run, go into the estop code to try to be ready
                    current_state_->estop.store(true);
                }

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
        if (current_state_->local_disconnect.load()) {
            // we still cannot connect to the arm, add a large delay to further reduce spam
            std::this_thread::sleep_for(10 * k_estop_delay);
        }
        // if we end up here that means we can no longer talk to the arm. store the state so we can try to recover from this.
        current_state_->local_disconnect.store(true);

        // attempt to reconnect to the arm. Even if we reconnect, we will have to check that the tablet is not in local mode.
        // we reconnect in the catch to attempt to limit the amount of times we connect to the arm to avoid a "no controller" error
        // https://forum.universal-robots.com/t/no-controller-error-on-real-robot/3127
        // check if we have a tcp connection to the arm, if we do, restart the connection
        if (current_state_->dashboard->getState() == urcl::comm::SocketState::Connected) {
            current_state_->dashboard->disconnect();
        }

        // if we think we are connected to the arm, try resetting the connection
        if (current_state_->driver->isReverseInterfaceConnected()) {
            current_state_->driver->resetRTDEClient(current_state_->appdir + k_output_recipe, current_state_->appdir + k_input_recipe);
        }

        // delay so we don't spam the dashboard client if disconnected
        std::this_thread::sleep_for(k_estop_delay);

        // connecting to the dashboard can hang when calling an arm that is off, which will cause issues on shutdown.
        if (!current_state_->dashboard->connect(1)) {
            // we failed to reconnect to the tablet, so we might not even be able to talk to it.
            // return an error so we can attempt to reconnect again
            return UrDriverStatus::DASHBOARD_FAILURE;
        }

        // start capturing data from the arm driver again
        current_state_->driver->startRTDECommunication();

        VIAM_SDK_LOG(error) << "failed to talk to the arm, is the tablet in local mode? : " << std::string(ex.what());
    }

    // check if the arm status is normal or not empty. the status will be empty if the arm is disconnected or when the arm is first
    // switched into local mode
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

        // TODO(RSDK-11297): further investigate the need for this delay
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

    vector6d_t tcp_state{};
    if (!data_pkg->getData("actual_TCP_pose", tcp_state)) {
        VIAM_SDK_LOG(warn) << "read_joint_keep_alive driver->getDataPackage().getData(\"actual_TCP_pos\") returned false";
        return UrDriverStatus::READ_FAILURE;
    }

    // for consistency, update cached data only after all getData calls succeed
    current_state_->joints_position = std::move(joints_position);
    current_state_->joints_velocity = std::move(joints_velocity);
    current_state_->tcp_state = std::move(tcp_state);

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
