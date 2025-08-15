#include "ur_arm.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>

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

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/rtde/data_package.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/types.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/datatypes.h>
#include <ur_client_library/ur/ur_driver.h>

#include <third_party/trajectories/Trajectory.h>

#include "utils.hpp"

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
constexpr auto k_disconnect_delay = std::chrono::seconds(1);

constexpr double k_waypoint_equivalancy_epsilon_rad = 1e-4;
constexpr double k_min_timestep_sec = 1e-2;  // determined experimentally, the arm appears to error when given timesteps ~2e-5 and lower

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

std::vector<std::string> validate_config_(const ResourceConfig& cfg) {
    static_cast<void>(find_config_attribute<std::string>(cfg, "host"));
    static_cast<void>(find_config_attribute<double>(cfg, "speed_degs_per_sec"));
    static_cast<void>(find_config_attribute<double>(cfg, "acceleration_degs_per_sec2"));
    {
        auto threshold = -1.0;
        try {
            threshold = find_config_attribute<double>(cfg, "reject_move_request_threshold_deg");
        } catch (...) {
            // will go away once find_config_attribute returns an optional
            return {};
        }
        if (threshold < 0 || threshold > 360) {
            std::stringstream sstream;
            sstream << "reject_move_request_threshold_deg should be between 0 and 360, it is : " << threshold;
            throw std::invalid_argument(sstream.str());
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

// private variables to maintain connection and state
class URArm::state_ {
    struct private_ {};

   public:
    explicit state_(private_,
                    std::string configured_model_type,
                    std::string host,
                    std::string app_dir,
                    std::string csv_output_path,
                    std::optional<double> reject_move_request_threshold_rad,
                    const struct ports_& ports);
    ~state_();

    static std::unique_ptr<state_> create(std::string configured_model_type, const ResourceConfig& config, const struct ports_& ports);
    void shutdown();

    const std::optional<double>& get_reject_move_request_threshold_rad() const;
    vector6d_t read_joint_positions() const;
    vector6d_t read_tcp_pose() const;

    const std::string& csv_output_path() const;
    const std::string& app_dir() const;

    void set_speed(double speed);
    double get_speed() const;

    void set_acceleration(double acceleration);
    double get_acceleration() const;

    size_t get_move_epoch() const;

    std::future<void> enqueue_move_request(size_t current_move_epoch,
                                           std::vector<trajectory_sample_point>&& samples,
                                           std::ofstream arm_joint_positions_stream);

    bool is_moving() const;

    std::optional<std::shared_future<void>> cancel_move_request();

   private:
    struct state_disconnected_;
    friend struct state_disconnected_;

    struct state_controlled_;
    friend struct state_controlled_;

    struct state_independent_;
    friend struct state_independent_;

    using state_variant_ = std::variant<state_disconnected_, state_controlled_, state_independent_>;

    struct event_connection_established_;
    struct event_connection_lost_;
    struct event_estop_detected_;
    struct event_estop_cleared_;
    struct event_local_mode_detected_;
    struct event_remote_mode_restored_;

    using event_variant_ = std::variant<event_connection_established_,
                                        event_connection_lost_,
                                        event_estop_detected_,
                                        event_estop_cleared_,
                                        event_local_mode_detected_,
                                        event_remote_mode_restored_>;

    template <typename T>
    class state_event_handler_base_ {
       public:
        // This is the common catch-all handler for `Event`s for
        // which the actual state classes lack a more specific
        // overload (e.g., an event handler specifically for
        // `event_stop_detected_`). It will log a warning, since this
        // probably indicates that one of the states below is missing
        // a handler for an event that it actually emits.
        template <typename Event>
        std::optional<state_variant_> handle_event(Event event);

       private:
        friend T;
        state_event_handler_base_() = default;
    };

    struct state_disconnected_ : public state_event_handler_base_<state_disconnected_> {
        static std::string_view name();
        std::string describe() const;
        std::chrono::milliseconds get_timeout() const;

        std::optional<event_variant_> recv_arm_data(state_&);
        std::optional<event_variant_> upgrade_downgrade(state_& state);
        std::optional<event_variant_> handle_move_request(state_& state);
        std::optional<event_variant_> send_noop();

        std::optional<state_variant_> handle_event(event_connection_established_ event);

        using state_event_handler_base_<state_disconnected_>::handle_event;
    };

    struct arm_connection_ {
        ~arm_connection_();

        std::unique_ptr<DashboardClient> dashboard;
        std::unique_ptr<UrDriver> driver;
        std::unique_ptr<rtde_interface::DataPackage> data_package;
        std::optional<std::bitset<4>> robot_status_bits;
        std::optional<std::bitset<11>> safety_status_bits;

        // TODO(RSDK-11620): Check if we still need this flag. We may
        // not, now that we examine status bits that include letting
        // us know whether the program is running.
        std::atomic<bool> program_running_flag{false};
    };

    struct state_connected_ {
        explicit state_connected_(std::unique_ptr<arm_connection_> arm_conn);

        std::optional<event_variant_> recv_arm_data(state_& state) const;
        std::optional<event_variant_> send_noop() const;

        std::unique_ptr<arm_connection_> arm_conn_;
    };

    struct state_controlled_ : public state_event_handler_base_<state_controlled_>, public state_connected_ {
        explicit state_controlled_(std::unique_ptr<arm_connection_> arm_conn);

        static std::string_view name();
        std::string describe() const;
        std::chrono::milliseconds get_timeout() const;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_& state);
        using state_connected_::send_noop;

        std::optional<state_variant_> handle_event(event_connection_lost_);
        std::optional<state_variant_> handle_event(event_estop_detected_);
        std::optional<state_variant_> handle_event(event_local_mode_detected_);

        using state_event_handler_base_<state_controlled_>::handle_event;
    };

    struct state_independent_ : public state_event_handler_base_<state_independent_>, public state_connected_ {
        enum class reason : std::uint8_t { k_estopped, k_local_mode, k_both };

        explicit state_independent_(std::unique_ptr<arm_connection_> arm_conn, reason r);

        static std::string_view name();
        std::string describe() const;
        std::chrono::milliseconds get_timeout() const;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_& state);
        std::optional<event_variant_> send_noop();

        bool estopped() const;
        bool local_mode() const;

        std::optional<state_variant_> handle_event(event_connection_lost_);
        std::optional<state_variant_> handle_event(event_estop_detected_);
        std::optional<state_variant_> handle_event(event_local_mode_detected_);
        std::optional<state_variant_> handle_event(event_estop_cleared_);
        std::optional<state_variant_> handle_event(event_remote_mode_restored_);

        using state_event_handler_base_<state_independent_>::handle_event;

        reason reason_;
    };

    struct event_connection_established_ {
        static std::string_view name();
        auto describe() const;
        std::unique_ptr<arm_connection_> payload;
    };

    struct event_connection_lost_ {
        static std::string_view name();
        auto describe() const;
    };

    struct event_estop_detected_ {
        static std::string_view name();
        auto describe() const;
    };

    struct event_estop_cleared_ {
        static std::string_view name();
        auto describe() const;
    };

    struct event_local_mode_detected_ {
        static std::string_view name();
        auto describe() const;
    };

    struct event_remote_mode_restored_ {
        static std::string_view name();
        auto describe() const;
    };

    // TODO: Arguably, this should be a class since it has some
    // non-trivial members. But the state_ class needs pretty deep
    // access. When I tried to turn it into a class, it ended up with a
    // profusion of accessors and mutators, and it ended up seeming more
    // confusing. So, for now at least, it remains a struct. It is
    // private to `state_` so that should offer some protection against
    // URArm misusing it.
    struct move_request {
       public:
        explicit move_request(std::vector<trajectory_sample_point>&& samples, std::ofstream arm_joint_positions_stream);

        std::shared_future<void> cancel();

        void complete_success();
        void complete_cancelled();
        void complete_failure();
        void complete_error(std::string_view message);
        void cancel_error(std::string_view message);

        void write_joint_data(vector6d_t& position, vector6d_t& velocity);

        std::vector<trajectory_sample_point> samples;
        std::ofstream arm_joint_positions_stream;
        std::size_t arm_joint_positions_sample{0};
        std::promise<void> completion;

        struct cancellation_request {
            // This constructor needs to be written this way for
            // std::optional::emplace with no arguments to work.
            cancellation_request();

            std::promise<void> promise;
            std::shared_future<void> future;
            bool issued{false};
        };

        std::optional<cancellation_request> cancellation_request;
    };

    void emit_event_(event_variant_&& event);

    template <typename Event>
    void emit_event_(Event&& event);

    std::chrono::milliseconds get_timeout_() const;
    std::string describe_() const;
    static std::string describe_state_(const state_variant_& state);

    void clear_ephemeral_values_();
    void recv_arm_data_();
    void upgrade_downgrade_();
    void handle_move_request_();
    void send_noop_();

    void run_();
    void trajectory_done_callback_(control::TrajectoryResult trajectory_result);

    const std::string configured_model_type_;
    const std::string host_;
    const std::string app_dir_;
    const std::string csv_output_path_;

    // If this field ever becomes mutable, the accessors for it must
    // start taking the lock and returning a copy.
    const std::optional<double> reject_move_request_threshold_rad_;

    const struct ports_ ports_;

    std::atomic<double> speed_{0};
    std::atomic<double> acceleration_{0};

    mutable std::mutex mutex_;
    state_variant_ current_state_{state_disconnected_{}};
    std::thread worker_thread_;
    std::condition_variable worker_wakeup_cv_;
    bool shutdown_requested_{false};

    std::atomic<std::size_t> move_epoch_{0};
    std::optional<move_request> move_request_;

    struct ephemeral_ {
        vector6d_t joint_positions;
        vector6d_t joint_velocities;
        vector6d_t tcp_state;
    };
    std::optional<struct ephemeral_> ephemeral_;
};

URArm::state_::state_(private_,
                      std::string configured_model_type,
                      std::string host,
                      std::string app_dir,
                      std::string csv_output_path,
                      std::optional<double> reject_move_request_threshold_rad,
                      const struct ports_& ports)
    : configured_model_type_{std::move(configured_model_type)},
      host_{std::move(host)},
      app_dir_{std::move(app_dir)},
      csv_output_path_{std::move(csv_output_path)},
      reject_move_request_threshold_rad_(std::move(reject_move_request_threshold_rad)),
      ports_{ports} {}

URArm::state_::~state_() {
    shutdown();
}

std::unique_ptr<URArm::state_> URArm::state_::create(std::string configured_model_type,
                                                     const ResourceConfig& config,
                                                     const struct ports_& ports) {
    // get the APPDIR environment variable
    auto* const app_dir = std::getenv("APPDIR");  // NOLINT: Yes, we know getenv isn't thread safe
    if (!app_dir) {
        throw std::runtime_error("required environment variable `APPDIR` unset");
    }
    VIAM_SDK_LOG(info) << "APPDIR: " << app_dir;

    auto host = find_config_attribute<std::string>(config, "host");

    // If the config contains `csv_output_path`, use that, otherwise,
    // fall back to `VIAM_MOUDLE_DATA` as the output path, which must
    // be set.
    auto csv_output_path = [&] {
        try {
            return find_config_attribute<std::string>(config, "csv_output_path");
        } catch (...) {
            // If we threw, but we have the attribute, then it failed to
            // convert and we should report that error, since that is an
            // actual user error.
            if (config.attributes().count("csv_output_path") != 0) {
                throw;
            }

            auto* const viam_module_data = std::getenv("VIAM_MODULE_DATA");  // NOLINT: Yes, we know getenv isn't thread safe
            if (!viam_module_data) {
                throw std::runtime_error("required environment variable `VIAM_MODULE_DATA` unset");
            }
            VIAM_SDK_LOG(info) << "VIAM_MODULE_DATA: " << viam_module_data;

            return std::string{viam_module_data};
        }
    }();

    auto threshold = [&] {
        try {
            return std::make_optional(find_config_attribute<double>(config, "reject_move_request_threshold_deg"));
        } catch (...) {
            return std::optional<double>();
        }
    }();

    auto state = std::make_unique<state_>(private_{},
                                          std::move(configured_model_type),
                                          std::move(host),
                                          std::string{app_dir},
                                          std::move(csv_output_path),
                                          std::move(threshold),
                                          ports);

    state->set_speed(degrees_to_radians(find_config_attribute<double>(config, "speed_degs_per_sec")));
    state->set_acceleration(degrees_to_radians(find_config_attribute<double>(config, "acceleration_degs_per_sec2")));

    const std::lock_guard lock(state->mutex_);
    state->worker_thread_ = std::thread{&state_::run_, state.get()};

    // Attempt to manually drive the state machine out of the
    // disconnected state. For now, just try a few times with the
    // natural recovery cycle.
    for (size_t i = 0; i != 5; ++i) {
        try {
            state->upgrade_downgrade_();
            return state;
        } catch (const std::exception& xcp) {
            VIAM_SDK_LOG(warn) << "Failed to establish working connection to arm after " << (i + 1) << " attempts: `" << xcp.what() << "`";
            const auto timeout = state->get_timeout_();
            VIAM_SDK_LOG(warn) << "Retrying after " << timeout.count() << " milliseconds";
            std::this_thread::sleep_for(timeout);
        }
    }
    state->upgrade_downgrade_();
    return state;
}

void URArm::state_::shutdown() {
    auto worker = [this] {
        const std::lock_guard lock{mutex_};
        if (!shutdown_requested_) {
            shutdown_requested_ = true;
            worker_wakeup_cv_.notify_all();
        }
        return std::move(worker_thread_);
    }();

    if (worker.joinable()) {
        VIAM_SDK_LOG(info) << "URArm shutdown waiting for worker thread to terminate";
        worker.join();
        VIAM_SDK_LOG(info) << "worker thread terminated";
    }
}

const std::optional<double>& URArm::state_::get_reject_move_request_threshold_rad() const {
    // NOTE: It is OK to return this as a reference and without taking the lock, as this field is immutable inside `state_`.
    return reject_move_request_threshold_rad_;
}

vector6d_t URArm::state_::read_joint_positions() const {
    const std::lock_guard lock{mutex_};
    if (!ephemeral_) {
        std::ostringstream buffer;
        buffer << "read_joint_positions: joint positions is not currently known; current state: " << describe_();
        throw std::runtime_error(buffer.str());
    }
    return ephemeral_->joint_positions;
}

vector6d_t URArm::state_::read_tcp_pose() const {
    const std::lock_guard lock{mutex_};
    if (!ephemeral_) {
        std::ostringstream buffer;
        buffer << "read_tcp_pose: tcp pose is not currently known; current state: " << describe_();
        throw std::runtime_error(buffer.str());
    }
    return ephemeral_->tcp_state;
}

const std::string& URArm::state_::csv_output_path() const {
    return csv_output_path_;
}

const std::string& URArm::state_::app_dir() const {
    return app_dir_;
}

void URArm::state_::set_speed(double speed) {
    speed_.store(speed);
}

double URArm::state_::get_speed() const {
    return speed_.load();
}

void URArm::state_::set_acceleration(double acceleration) {
    acceleration_.store(acceleration);
}

double URArm::state_::get_acceleration() const {
    return acceleration_.load();
}

size_t URArm::state_::get_move_epoch() const {
    return move_epoch_.load(std::memory_order_acquire);
}

std::future<void> URArm::state_::enqueue_move_request(size_t current_move_epoch,
                                                      std::vector<trajectory_sample_point>&& samples,
                                                      std::ofstream arm_joint_positions_stream) {
    // Use CAS to increment the epoch and detect if another move
    // operation occurred between when we obtained a value with
    // `get_move_epoch` and when `enqueue_move_request` was called
    // (presumably, while we were planning). If so, we have to fail
    // this operation, since our starting waypoint information is no
    // longer valid.
    if (!move_epoch_.compare_exchange_strong(current_move_epoch, current_move_epoch + 1, std::memory_order_acq_rel)) {
        throw std::runtime_error("move operation was superseded by a newer operation");
    }

    const std::lock_guard lock{mutex_};
    if (move_request_) {
        throw std::runtime_error("an actuation is already in progress");
    }
    return move_request_.emplace(std::move(samples), std::move(arm_joint_positions_stream)).completion.get_future();
}

bool URArm::state_::is_moving() const {
    const std::lock_guard lock{mutex_};
    // If we have a move request, but the samples are gone, it means we
    // sent them to the arm, so as far as we are concerned, the arm is
    // moving, though that move might fail later.
    return (move_request_ && move_request_->samples.empty());
}

std::optional<std::shared_future<void>> URArm::state_::cancel_move_request() {
    const std::lock_guard lock{mutex_};
    if (!move_request_) {
        return std::nullopt;
    }

    return std::make_optional(move_request_->cancel());
}

template <typename T>
template <typename Event>
std::optional<URArm::state_::state_variant_> URArm::state_::state_event_handler_base_<T>::handle_event(Event event) {
    const auto current_state_desc = static_cast<T*>(this)->describe();
    const auto event_desc = event.describe();

    VIAM_SDK_LOG(warn) << "In state `" << current_state_desc << "`, received an event `" << event_desc
                       << "` for which there is no declared handler; state will not be changed";

    // No transition
    return std::nullopt;
}

// Many of the state machine types get false positive clang-tidy
// warnings suggesting that they should be made static. They can't
// really though. Suppress that tidy warning from here until we are
// done defining the states and events.
//
// NOLINTBEGIN(readability-convert-member-functions-to-static)

std::string_view URArm::state_::state_disconnected_::name() {
    using namespace std::literals::string_view_literals;
    return "disconnected"sv;
}

std::string URArm::state_::state_disconnected_::describe() const {
    return std::string{name()};
}

std::chrono::milliseconds URArm::state_::state_disconnected_::get_timeout() const {
    return k_disconnect_delay;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::recv_arm_data(state_&) {
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::upgrade_downgrade(state_& state) {
    VIAM_SDK_LOG(info) << "disconnected: attempting recovery";
    auto arm_connection = std::make_unique<arm_connection_>();
    arm_connection->dashboard = std::make_unique<DashboardClient>(state.host_);

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: trying to connect to dashboard";
    if (!arm_connection->dashboard->connect(1)) {
        std::ostringstream buffer;
        buffer << "Failed trying to connect to UR dashboard on host " << state.host_;
        throw std::runtime_error(buffer.str());
    }
    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: connected to dashboard";

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: validating model";
    std::string actual_model_type{};
    if (!arm_connection->dashboard->commandGetRobotModel(actual_model_type)) {
        throw std::runtime_error("failed to get model info of connected arm");
    }

    if (state.configured_model_type_ != actual_model_type) {
        std::ostringstream buffer;
        buffer << "configured model type `" << state.configured_model_type_ << "` does not match connected arm `" << actual_model_type
               << "`";
        throw std::runtime_error(buffer.str());
    }

    // If the arm is in remote control mode, we need to stop any
    // running program, since it might not be ours.
    //
    // TODO(11619): See if we can fully eliminate use of the
    // DashboardClient and use only the `UrDriver`.
    if (arm_connection->dashboard->commandIsInRemoteControl()) {
        VIAM_SDK_LOG(info) << "disconnected: attempting recovery: stopping any currently running program";
        if (!arm_connection->dashboard->commandStop()) {
            throw std::runtime_error("couldn't stop program running on arm_connection->dashboard");
        }
    }

    constexpr char k_script_file[] = "/src/control/external_control.urscript";

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: instantiating new UrDriver";
    // Now the robot is ready to receive a program
    auto ur_cfg = urcl::UrDriverConfiguration{};
    ur_cfg.robot_ip = state.host_;
    ur_cfg.script_file = state.app_dir_ + k_script_file;
    ur_cfg.output_recipe_file = state.app_dir_ + k_output_recipe;
    ur_cfg.input_recipe_file = state.app_dir_ + k_input_recipe;
    ur_cfg.handle_program_state = [&running_flag = arm_connection->program_running_flag](bool running) {
        VIAM_SDK_LOG(info) << "UR program is " << (running ? "running" : "not running");
        running_flag.store(running, std::memory_order_release);
    };
    ur_cfg.headless_mode = true;
    ur_cfg.socket_reconnect_attempts = 1;

    ur_cfg.reverse_port = state.ports_.reverse_port;
    ur_cfg.script_sender_port = state.ports_.script_sender_port;
    ur_cfg.trajectory_port = state.ports_.trajectory_port;
    ur_cfg.script_command_port = state.ports_.script_command_port;
    VIAM_SDK_LOG(info) << "using reverse_port " << ur_cfg.reverse_port;
    VIAM_SDK_LOG(info) << "using script_sender_port " << ur_cfg.script_sender_port;
    VIAM_SDK_LOG(info) << "using trajectory_port " << ur_cfg.trajectory_port;
    VIAM_SDK_LOG(info) << "using script_command_port " << ur_cfg.script_command_port;

    arm_connection->driver = std::make_unique<UrDriver>(ur_cfg);

    arm_connection->driver->registerTrajectoryDoneCallback(
        std::bind(&URArm::state_::trajectory_done_callback_, &state, std::placeholders::_1));

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: starting RTDE communication";
    arm_connection->driver->startRTDECommunication();

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: attempting to read a data package from the arm";
    if (!arm_connection->driver->getDataPackage()) {
        throw std::runtime_error("could not read data package from newly established driver connection ");
    }

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: recovery appears to have been successful; transitioning to independent mode";
    return event_connection_established_{std::move(arm_connection)};
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::handle_move_request(state_& state) {
    if (state.move_request_) {
        std::exchange(state.move_request_, {})->complete_error("no connection to arm");
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::send_noop() {
    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_disconnected_::handle_event(event_connection_established_ event) {
    // If we are leaving disconnected mode for independent mode, we
    // don't know, really, in what state we will find the arm. Assume
    // the worst case scenario, that we are both estopped and in local
    // mode, and let the natural recovery process sort out how to get
    // back to a good place.
    return state_independent_{std::move(event.payload), state_independent_::reason::k_both};
}

URArm::state_::arm_connection_::~arm_connection_() {
    data_package.reset();
    VIAM_SDK_LOG(info) << "destroying current UrDriver instance";
    driver.reset();
    VIAM_SDK_LOG(info) << "destroying current DashboardClient instance";
    dashboard.reset();
}

URArm::state_::state_connected_::state_connected_(std::unique_ptr<arm_connection_> arm_conn) : arm_conn_{std::move(arm_conn)} {}

std::optional<URArm::state_::event_variant_> URArm::state_::state_connected_::send_noop() const {
    if (!arm_conn_->driver->writeTrajectoryControlMessage(
            control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        return event_estop_detected_{};
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_connected_::recv_arm_data(state_& state) const {
    const auto prior_robot_status_bits = std::exchange(arm_conn_->robot_status_bits, std::nullopt);
    const auto prior_safety_status_bits = std::exchange(arm_conn_->safety_status_bits, std::nullopt);

    arm_conn_->data_package = arm_conn_->driver->getDataPackage();
    if (!arm_conn_->data_package) {
        VIAM_SDK_LOG(error) << "Failed to read a data package from the arm: dropping connection";
        return event_connection_lost_{};
    }

    static const std::string k_robot_status_bits_key = "robot_status_bits";
    std::bitset<4> robot_status_bits;
    if (!arm_conn_->data_package->getData<std::uint32_t>(k_robot_status_bits_key, robot_status_bits)) {
        VIAM_SDK_LOG(error) << "Data package did not contain the expected `robot_status_bits` information; dropping connection";
        return event_connection_lost_{};
    }

    static const std::string k_safety_status_bits_key = "safety_status_bits";
    std::bitset<11> safety_status_bits;
    if (!arm_conn_->data_package->getData<std::uint32_t>(k_safety_status_bits_key, safety_status_bits)) {
        VIAM_SDK_LOG(error) << "Data package did not contain the expected `safety_status_bits` information; dropping connection";
        return event_connection_lost_{};
    }

    if (!prior_robot_status_bits || (*prior_robot_status_bits != robot_status_bits)) {
        VIAM_SDK_LOG(info) << "Updated robot status bits: `" << robot_status_bits << "`";
    }
    arm_conn_->robot_status_bits = robot_status_bits;

    if (!prior_safety_status_bits || (*prior_safety_status_bits != safety_status_bits)) {
        VIAM_SDK_LOG(info) << "Updated safety status bits: `" << safety_status_bits << "`";
    }
    arm_conn_->safety_status_bits = safety_status_bits;

    static const std::string k_joints_position_key = "actual_q";
    static const std::string k_joints_velocity_key = "actual_qd";
    static const std::string k_tcp_key = "actual_TCP_pose";

    bool data_good = true;
    vector6d_t joint_positions{};
    if (!arm_conn_->data_package->getData(k_joints_position_key, joint_positions)) {
        VIAM_SDK_LOG(error) << "getData(\"actual_q\") returned false - joint positions will not be available";
        data_good = false;
    }

    // read current joint velocities from robot data
    vector6d_t joint_velocities{};
    if (!arm_conn_->data_package->getData(k_joints_velocity_key, joint_velocities)) {
        VIAM_SDK_LOG(error) << "getData(\"actual_qd\") returned false - joint velocities will not be available";
        data_good = false;
    }

    vector6d_t tcp_state{};
    if (!arm_conn_->data_package->getData(k_tcp_key, tcp_state)) {
        VIAM_SDK_LOG(warn) << "getData(\"actual_TCP_pos\") returned false - end effector pose will not be available";
        data_good = false;
    }

    // For consistency, update cached data only after all getData
    // calls succeed.
    if (data_good) {
        state.ephemeral_ = {std::move(joint_positions), std::move(joint_velocities), std::move(tcp_state)};
    }

    return std::nullopt;
}

URArm::state_::state_controlled_::state_controlled_(std::unique_ptr<arm_connection_> arm_conn) : state_connected_(std::move(arm_conn)) {}

std::string_view URArm::state_::state_controlled_::name() {
    using namespace std::literals::string_view_literals;
    return "controlled"sv;
}

std::string URArm::state_::state_controlled_::describe() const {
    return std::string{name()};
}

std::chrono::milliseconds URArm::state_::state_controlled_::get_timeout() const {
    return k_noop_delay;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_controlled_::upgrade_downgrade(state_&) {
    namespace urtde = urcl::rtde_interface;

    if (!arm_conn_->safety_status_bits || !arm_conn_->robot_status_bits) {
        VIAM_SDK_LOG(warn) << "While in controlled state, robot and safety status bits were not available; disconnecting";
        return event_connection_lost_{};
    }

    if (!arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        return event_estop_detected_{};
    }

    constexpr auto power_on_bit = 1ULL << static_cast<int>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON);
    constexpr auto program_running_bit = 1ULL << static_cast<int>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING);
    constexpr std::bitset<4> k_power_on_and_running{power_on_bit | program_running_bit};

    if (*(arm_conn_->robot_status_bits) != k_power_on_and_running) {
        return event_estop_detected_{};
    }

    try {
        if (!arm_conn_->dashboard->commandIsInRemoteControl()) {
            VIAM_SDK_LOG(warn) << "While in controlled state, detected that dashboard is no longer in remote mode";
            return event_local_mode_detected_{};
        }
    } catch (...) {
        VIAM_SDK_LOG(warn) << "While in controlled state, could not communicate with dashboard to determine remote control state";
        // We want to go to local mode first so we can try to recover
        // by reconnecting to the dashboard. If local mode can't make
        // that happen, then it will further downgrade to
        // disconnected.
        return event_local_mode_detected_{};
    }

    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_controlled_::handle_move_request(state_& state) {
    if (!state.move_request_) {
        return std::nullopt;
    }

    if (!state.move_request_->samples.empty() && !state.move_request_->cancellation_request) {
        // We have a move request, it has samples, and there is no pending cancel for that move. Issue the move.

        VIAM_SDK_LOG(info) << "URArm sending trajectory";

        // By moving the samples out, we indicate that the trajectory is considered started.
        auto samples = std::move(state.move_request_->samples);
        const auto num_samples = samples.size();

        VIAM_SDK_LOG(info) << "URArm::send_trajectory sending TRAJECTORY_START for " << num_samples << " samples";
        if (!arm_conn_->driver->writeTrajectoryControlMessage(
                urcl::control::TrajectoryControlMessage::TRAJECTORY_START, static_cast<int>(num_samples), RobotReceiveTimeout::off())) {
            VIAM_SDK_LOG(error) << "send_trajectory driver->writeTrajectoryControlMessage returned false";
            std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start message to arm");

            // Unfortunately, we can't differentiate given the `bool`
            // return from `writeTrajectoryControlMessage` above
            // whether the failure here is due to full connectivity
            // loss, or just the arm being in local mode. If we
            // interpreted the latter as the former, we would drop and
            // re-form connections every time the arm went into local
            // mode, which is too aggressive. So, instead, we
            // interpret this as meaning local mode, and then let
            // `upgrade_downgrade` for local mode make a subsequent
            // determination about whether we have really lost
            // connectivity such that we should enter
            // `state_disconnected_`. The same log applies to the
            // other cases below.
            return event_local_mode_detected_{};
        }

        VIAM_SDK_LOG(info) << "URArm::send_trajectory sending " << num_samples << " cubic writeTrajectorySplinePoint";
        for (size_t i = 0; i < num_samples; i++) {
            if (!arm_conn_->driver->writeTrajectorySplinePoint(samples[i].p, samples[i].v, samples[i].timestep)) {
                VIAM_SDK_LOG(error) << "send_trajectory cubic driver->writeTrajectorySplinePoint returned false";
                std::exchange(state.move_request_, {})->complete_error("failed to send trajectory spline point to arm");
                return event_estop_detected_{};
            };
        }

        VIAM_SDK_LOG(info) << "URArm trajectory sent";

    } else if (state.move_request_->samples.empty() && state.move_request_->cancellation_request &&
               !state.move_request_->cancellation_request->issued) {
        // We have a move request, the samples have been forwarded,
        // and cancellation is requested but has not yet been issued. Issue a cancel.
        state.move_request_->cancellation_request->issued = true;
        if (!arm_conn_->driver->writeTrajectoryControlMessage(
                urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off())) {
            state.move_request_->cancel_error("failed to write trajectory control cancel message to URArm");
            return event_estop_detected_{};
        }
    } else if (!state.move_request_->samples.empty() && state.move_request_->cancellation_request) {
        // We have a move request that we haven't issued but a
        // cancel is already pending. Don't issue it, just cancel it.
        std::exchange(state.move_request_, {})->complete_cancelled();
    } else {
        // TODO: is it assured that we have positions/velocities here?
        state.move_request_->write_joint_data(state.ephemeral_->joint_positions, state.ephemeral_->joint_velocities);
    }

    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_connection_lost_) {
    return state_disconnected_{};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_estop_detected_) {
    return state_independent_{std::move(arm_conn_), state_independent_::reason::k_estopped};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_local_mode_detected_) {
    return state_independent_{std::move(arm_conn_), state_independent_::reason::k_local_mode};
}

URArm::state_::state_independent_::state_independent_(std::unique_ptr<arm_connection_> arm_conn, reason r)
    : state_connected_(std::move(arm_conn)), reason_(r) {}

std::string_view URArm::state_::state_independent_::name() {
    using namespace std::literals::string_view_literals;
    return "independent"sv;
}

std::string URArm::state_::state_independent_::describe() const {
    std::ostringstream buffer;
    buffer << name() << "(";
    switch (reason_) {
        case reason::k_estopped: {
            buffer << "estop)";
            break;
        }
        case reason::k_local_mode: {
            buffer << "local)";
            break;
        }
        case reason::k_both: {
            buffer << "estop|local)";
            break;
        }
        default: {
            buffer << "unknown)";
            break;
        }
    }
    return buffer.str();
}

std::chrono::milliseconds URArm::state_::state_independent_::get_timeout() const {
    // TODO(RSDK-11622): I'm not sure this actually makes sense: even if we are
    // estopped, we still want high frequency service for the rtde
    // interface.
    if (estopped()) {
        return k_estop_delay;
    }
    return k_noop_delay;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::upgrade_downgrade(state_&) {
    namespace urtde = urcl::rtde_interface;

    if (!arm_conn_->safety_status_bits || !arm_conn_->robot_status_bits) {
        VIAM_SDK_LOG(warn) << "While in independent state, robot and safety status bits were not available; disconnecting";
        return event_connection_lost_{};
    }

    // If we aren't estopped, but the safety flags say we are, become estopped immediately.
    if (!estopped() && !arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        return event_estop_detected_{};
    }

    // If we aren't estopped, but the robot program is not running, become estopped.
    if (!estopped() && !arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
        return event_estop_detected_{};
    }

    if (local_mode()) {
        // If we aren't connected to the dashboard, try to reconnect, so
        // we can get an honest answer to `commandIsInRemoteControl`.
        if (arm_conn_->dashboard->getState() != urcl::comm::SocketState::Connected) {
            VIAM_SDK_LOG(info) << "While in independent state, dashboard client is disconnected. Attempting to recover";
            arm_conn_->dashboard->disconnect();
            if (!arm_conn_->dashboard->connect(1)) {
                return event_connection_lost_{};
            }
        }

        // Try to use the dashboard connection to determine if the arm
        // is now in remote mode. If we fail to communicate with the
        // dashboard, downgrade to disconnected.
        // TODO(RSDK-11619) We currently only test this if we have already detected local mode. This is because when an estop occurs,
        // and a user switches into local mode without recovering from the stop, the dashboard client cannot reach the arm.
        // since the driver client does not appear to have this issue, we should revaluate where this check should live when we go to remove
        // the dashboard client.
        try {
            if (!arm_conn_->dashboard->commandIsInRemoteControl()) {
                VIAM_SDK_LOG(warn) << "While in independent state, waiting for arm to re-enter remote mode";
                return std::nullopt;
            }
        } catch (...) {
            VIAM_SDK_LOG(warn)
                << "While in independent state, could not communicate with dashboard to determine remote control state; disconnecting";
            return event_connection_lost_{};
        }

        // reset the clients to clear the state that blocks commands.
        try {
            VIAM_SDK_LOG(info) << "Arm has exited local control - cycling primary client";
            arm_conn_->dashboard->disconnect();
            if (!arm_conn_->dashboard->connect(1)) {
                return event_connection_lost_{};
            }
            arm_conn_->driver->stopPrimaryClientCommunication();
            arm_conn_->driver->startPrimaryClientCommunication();
        } catch (...) {
            VIAM_SDK_LOG(warn) << "While in independent state, failed cycling primary client; disconnecting";
            return event_connection_lost_{};
        }
        return event_remote_mode_restored_{};
    }

    // On the other hand, if we are estopped, and that condition has been resolved, clear it.
    //
    // TODO(RSDK-11621): Deal with three position enabling stops.
    if (estopped() && arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        // ensure the arm is powered on
        if (!arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON))) {
            VIAM_SDK_LOG(info) << "While in independent state, arm is not powered on; attempting to power on arm";
            try {
                if (!arm_conn_->dashboard->commandPowerOn()) {
                    return std::nullopt;
                }
            } catch (...) {
                VIAM_SDK_LOG(warn) << "While in independent state, could not communicate with dashboard to power on arm; disconnecting";
                return event_connection_lost_{};
            }
        }

        try {
            // TODO(RSDK-11645) find a way to detect if the breaks are locked
            VIAM_SDK_LOG(info) << "While in independent state: releasing brakes since no longer estopped";
            if (!arm_conn_->dashboard->commandBrakeRelease()) {
                VIAM_SDK_LOG(warn) << "While in independent state, could not release brakes";
                return std::nullopt;
            }
        } catch (...) {
            VIAM_SDK_LOG(warn) << "While in independent state, could not communicate with dashboard to release brakes; disconnecting";
            return event_connection_lost_{};
        }

        // resend the robot program if the control script is not running on the arm.
        // This has been found to occur on some estops and when
        // controlling the arm directly while in local mode.
        if (!arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
            arm_conn_->program_running_flag.store(false, std::memory_order_release);
            VIAM_SDK_LOG(info) << "While in independent state, program is not running on arm; attempting to resend program";
            try {
                if (!arm_conn_->driver->sendRobotProgram()) {
                    VIAM_SDK_LOG(warn) << "While in independent state, could not send program to robot via driver; disconnecting";
                    return event_connection_lost_{};
                }
            } catch (...) {
                VIAM_SDK_LOG(warn) << "While in independent state, failed sending program to robot via driver; disconnecting";
                return event_connection_lost_{};
            }
        }

        // TODO(RSDK-11622): Should we give up after a while rather
        // than getting stuck here indefinitely? But see also
        // RSDK-11620 which suggest removing this flag entirely.
        if (!arm_conn_->program_running_flag.load(std::memory_order_acquire)) {
            VIAM_SDK_LOG(info) << "While in independent state, waiting for callback to toggle program state to running";
            return std::nullopt;
        }

        return event_estop_cleared_{};
    }

    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::handle_move_request(state_& state) {
    if (state.move_request_) {
        std::exchange(state.move_request_, {})->complete_error([this]() -> std::string {
            switch (reason_) {
                case reason::k_estopped: {
                    return "arm is estopped";
                }
                case reason::k_local_mode: {
                    return "arm is in local mode";
                }
                case reason::k_both: {
                    return "arm is in local mode and estopped";
                }
                default: {
                    return "arm is in an unknown and uncontrolled state";
                }
            }
        }());
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::send_noop() {
    // If we are in local mode, there is no point trying to spam NOOPs
    // because each one will just get rejected and we will hammer the
    // state machine with meaningless `event_local_mode_detected_` events.
    if (local_mode()) {
        return std::nullopt;
    }

    return state_connected_::send_noop();
}

bool URArm::state_::state_independent_::estopped() const {
    return reason_ != reason::k_local_mode;
}

bool URArm::state_::state_independent_::local_mode() const {
    return reason_ != reason::k_estopped;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_estop_cleared_) {
    if (reason_ == reason::k_local_mode) {
        return std::move(*this);
    } else if (reason_ == reason::k_both) {
        return state_independent_{std::move(arm_conn_), reason::k_local_mode};
    } else {
        return state_controlled_{std::move(arm_conn_)};
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_remote_mode_restored_) {
    if (reason_ == reason::k_estopped) {
        return std::move(*this);
    } else if (reason_ == reason::k_both) {
        return state_independent_{std::move(arm_conn_), reason::k_estopped};
    } else {
        return state_controlled_{std::move(arm_conn_)};
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_estop_detected_) {
    if (reason_ == reason::k_local_mode) {
        return state_independent_{std::move(arm_conn_), reason::k_both};
    } else {
        return std::move(*this);
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_local_mode_detected_) {
    if (reason_ == reason::k_estopped) {
        return state_independent_{std::move(arm_conn_), reason::k_both};
    } else {
        return std::move(*this);
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_connection_lost_) {
    return state_disconnected_{};
}

std::string_view URArm::state_::event_connection_established_::name() {
    using namespace std::literals::string_view_literals;
    return "connection_established"sv;
}

auto URArm::state_::event_connection_established_::describe() const {
    return name();
}

std::string_view URArm::state_::event_connection_lost_::name() {
    using namespace std::literals::string_view_literals;
    return "connection_lost"sv;
}

auto URArm::state_::event_connection_lost_::describe() const {
    return name();
}

std::string_view URArm::state_::event_estop_detected_::name() {
    using namespace std::literals::string_view_literals;
    return "estop_detected"sv;
}

auto URArm::state_::event_estop_detected_::describe() const {
    return name();
}

std::string_view URArm::state_::event_estop_cleared_::name() {
    using namespace std::literals::string_view_literals;
    return "estop_cleared"sv;
}

auto URArm::state_::event_estop_cleared_::describe() const {
    return name();
}

std::string_view URArm::state_::event_local_mode_detected_::name() {
    using namespace std::literals::string_view_literals;
    return "local_mode_detected"sv;
}

auto URArm::state_::event_local_mode_detected_::describe() const {
    return name();
}

std::string_view URArm::state_::event_remote_mode_restored_::name() {
    using namespace std::literals::string_view_literals;
    return "remote_mode_restored"sv;
}

auto URArm::state_::event_remote_mode_restored_::describe() const {
    return name();
}

// NOLINTEND(readability-convert-member-functions-to-static)

URArm::state_::move_request::move_request(std::vector<trajectory_sample_point>&& samples, std::ofstream arm_joint_positions_stream)
    : samples(std::move(samples)), arm_joint_positions_stream(std::move(arm_joint_positions_stream)) {
    if (this->samples.empty()) {
        throw std::invalid_argument("no trajectory samples provided to move request");
    }
}

std::shared_future<void> URArm::state_::move_request::cancel() {
    if (!cancellation_request) {
        auto& cr = cancellation_request.emplace();
        cr.future = cr.promise.get_future().share();
    }
    return cancellation_request->future;
}

void URArm::state_::move_request::complete_success() {
    // Mark the move_request as completed. If there
    // was a cancel request, it raced and lost, but it doesn't
    // need an error.
    completion.set_value();
    if (cancellation_request) {
        cancellation_request->promise.set_value();
    }
}

void URArm::state_::move_request::complete_cancelled() {
    complete_error("arm's current trajectory cancelled");
}

void URArm::state_::move_request::complete_failure() {
    complete_error("arm's current trajectory failed");
}

void URArm::state_::move_request::complete_error(std::string_view message) {
    // The trajectory is being completed with an error of some sort. Set the completion result to an error,
    // and unblock any cancellation request.
    completion.set_exception(std::make_exception_ptr(std::runtime_error{std::string{message}}));
    if (cancellation_request) {
        cancellation_request->promise.set_value();
    }
    VIAM_SDK_LOG(warn) << "A trajectory completed with an error: " << message;
}

void URArm::state_::move_request::cancel_error(std::string_view message) {
    std::exchange(cancellation_request, {})->promise.set_exception(std::make_exception_ptr(std::runtime_error{std::string{message}}));
}

void URArm::state_::move_request::write_joint_data(vector6d_t& position, vector6d_t& velocity) {
    ::write_joint_data(position, velocity, arm_joint_positions_stream, unix_time_iso8601(), arm_joint_positions_sample++);
}

URArm::state_::move_request::cancellation_request::cancellation_request() {}

void URArm::state_::emit_event_(event_variant_&& event) {
    std::visit([this](auto&& event) { this->emit_event_(std::forward<decltype(event)>(event)); }, std::move(event));
}

template <typename T>
void URArm::state_::emit_event_(T&& event) {
    auto new_state = std::visit(
        [&, event_description = event.describe()](auto& current_state) {
            // Get the description of the current state before it handles
            // the event. Then, forward the event. If we got a new state
            // back, ask it to describe itself. Otherwise, ask the original
            // state to describe itself again, as it may present differently
            // after interpreting the event.
            const auto state_preimage_desc = current_state.describe();
            auto new_state = current_state.handle_event(std::forward<T>(event));
            const auto state_postimage_desc = new_state ? describe_state_(*new_state) : current_state.describe();
            VIAM_SDK_LOG(info) << "URArm state transition from state `" << state_preimage_desc << "` to state `" << state_postimage_desc
                               << "` due to event `" << event_description << "`";

            return new_state;
        },
        current_state_);

    if (new_state) {
        current_state_ = std::move(*new_state);
    }
}

std::chrono::milliseconds URArm::state_::get_timeout_() const {
    return std::visit([](auto& state) { return state.get_timeout(); }, current_state_);
}

std::string URArm::state_::describe_() const {
    return describe_state_(current_state_);
}

std::string URArm::state_::describe_state_(const state_variant_& state) {
    return std::visit([](auto& state) { return state.describe(); }, state);
}

void URArm::state_::upgrade_downgrade_() {
    if (auto event = std::visit([this](auto& state) { return state.upgrade_downgrade(*this); }, current_state_)) {
        emit_event_(*std::move(event));
    }
}

void URArm::state_::clear_ephemeral_values_() {
    ephemeral_.reset();
}

void URArm::state_::recv_arm_data_() {
    if (auto event = std::visit([this](auto& state) { return state.recv_arm_data(*this); }, current_state_)) {
        emit_event_(*std::move(event));
    }
}

void URArm::state_::handle_move_request_() {
    if (auto event = std::visit([this](auto& state) { return state.handle_move_request(*this); }, current_state_)) {
        emit_event_(*std::move(event));
    }
}

void URArm::state_::send_noop_() {
    if (auto event = std::visit([](auto& state) { return state.send_noop(); }, current_state_)) {
        emit_event_(*std::move(event));
    }
}

void URArm::state_::run_() {
    VIAM_SDK_LOG(info) << "worker thread started";
    while (true) {
        std::unique_lock lock(mutex_);
        if (worker_wakeup_cv_.wait_for(lock, get_timeout_(), [this] { return shutdown_requested_; })) {
            VIAM_SDK_LOG(info) << "worker thread signaled to terminate";
            break;
        }

        try {
            clear_ephemeral_values_();
            recv_arm_data_();
            upgrade_downgrade_();
            handle_move_request_();
            send_noop_();
        } catch (const std::exception& ex) {
            VIAM_SDK_LOG(warn) << "Exception in worker thread: " << ex.what();
        } catch (...) {
            VIAM_SDK_LOG(warn) << "Unknown exception in worker thread";
        }
    }

    VIAM_SDK_LOG(info) << "worker thread emitting disconnection event";
    emit_event_(event_connection_lost_{});
    VIAM_SDK_LOG(info) << "worker thread terminating";
}

void URArm::state_::trajectory_done_callback_(const control::TrajectoryResult trajectory_result) {
    const char* report;

    // Take ownership of any move request so we open the slot for the next one.
    auto move_request = [this] {
        const std::lock_guard guard{mutex_};
        return std::exchange(move_request_, {});
    }();

    switch (trajectory_result) {
        case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS: {
            report = "success";
            if (move_request) {
                move_request->complete_success();
            }
            break;
        }
        case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED: {
            report = "canceled";
            if (move_request) {
                move_request->complete_cancelled();
            }
            break;
        }
        case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
        default: {
            report = "failure";
            if (move_request) {
                move_request->complete_failure();
            }
            break;
        }
    }

    VIAM_SDK_LOG(info) << "trajectory report: " << report;
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
    move_(std::move(rlock), std::move(waypoints), unix_time);
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
        move_(std::move(rlock), std::move(waypoints), unix_time);
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

    const auto sva_file_path = str(boost::format(kSvaFileTemplate) % current_state_->app_dir() % model_string);

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
    constexpr char k_acc_key[] = "set_acc";
    constexpr char k_vel_key[] = "set_vel";
    for (const auto& kv : command) {
        if (kv.first == k_vel_key) {
            const double val = *kv.second.get<double>();
            current_state_->set_speed(degrees_to_radians(val));
            resp.emplace(k_vel_key, val);
        }
        if (kv.first == k_acc_key) {
            const double val = *kv.second.get<double>();
            current_state_->set_acceleration(degrees_to_radians(val));
            resp.emplace(k_acc_key, val);
        }
    }

    return resp;
}

void URArm::move_(std::shared_lock<std::shared_mutex> config_rlock, std::list<Eigen::VectorXd> waypoints, const std::string& unix_time) {
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
