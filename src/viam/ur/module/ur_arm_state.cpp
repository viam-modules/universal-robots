#include "ur_arm_state.hpp"

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/dll/runtime_symbol_info.hpp>

#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/rpc/grpc_context_observer.hpp>

#include "ur_arm_config.hpp"
#include "utils.hpp"

namespace {

// Extracts trace-id from W3C Trace Context traceparent header.
// Format: <version>-<trace-id>-<parent-id>-<trace-flags>
// Example: 00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01
// Returns std::nullopt if parsing fails or format is invalid.
std::optional<std::string> extract_trace_id_from_traceparent(std::string_view traceparent) {
    std::vector<std::string_view> parts;
    boost::split(parts, traceparent, boost::is_any_of("-"));

    // W3C Trace Context format requires exactly 4 components
    if (parts.size() != 4) {
        return std::nullopt;
    }

    // Validate version field (parts[0]) - should be "00" for current spec
    if (parts[0] != "00") {
        return std::nullopt;
    }

    // Validate trace-id (parts[1]) - must be 32 hex characters
    if (parts[1].size() != 32) {
        return std::nullopt;
    }

    // Validate parent-id (parts[2]) - must be 16 hex characters
    if (parts[2].size() != 16) {
        return std::nullopt;
    }

    // Validate trace-flags (parts[3]) - must be 2 hex characters
    if (parts[3].size() != 2) {
        return std::nullopt;
    }

    // Could add validation that characters are actually hex, but not strictly necessary
    // since an invalid trace-id subdirectory name is harmless

    return std::string(parts[1]);
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

}  // namespace

URArm::state_::state_(private_,
                      std::string configured_model_type,
                      std::string resource_name,
                      std::string host,
                      std::filesystem::path resource_root,
                      std::filesystem::path urcl_resource_root,
                      std::filesystem::path telemetry_output_path,
                      std::optional<double> reject_move_request_threshold_rad,
                      double waypoint_deduplication_tolerance_rad,
                      std::optional<double> robot_control_freq_hz,
                      double path_tolerance_delta_rads,
                      std::optional<double> path_colinearization_ratio,
                      bool use_new_trajectory_planner,
                      double max_trajectory_duration_secs,
                      double trajectory_sampling_freq_hz,
                      bool telemetry_output_path_append_traceid,
                      const struct ports_& ports)
    : configured_model_type_{std::move(configured_model_type)},
      resource_name_{std::move(resource_name)},
      host_{std::move(host)},
      resource_root_{std::move(resource_root)},
      urcl_resource_root_{std::move(urcl_resource_root)},
      telemetry_output_path_{std::move(telemetry_output_path)},
      robot_control_freq_hz_(robot_control_freq_hz.value_or(URArm::k_default_robot_control_freq_hz)),
      reject_move_request_threshold_rad_(std::move(reject_move_request_threshold_rad)),
      ports_{ports},
      waypoint_deduplication_tolerance_rad_(std::move(waypoint_deduplication_tolerance_rad)),
      path_tolerance_delta_rads_(path_tolerance_delta_rads),
      path_colinearization_ratio_(path_colinearization_ratio),
      use_new_trajectory_planner_(use_new_trajectory_planner),
      max_trajectory_duration_secs_(max_trajectory_duration_secs),
      trajectory_sampling_freq_hz_(trajectory_sampling_freq_hz),
      telemetry_output_path_append_traceid_(telemetry_output_path_append_traceid) {}

URArm::state_::~state_() {
    shutdown();
}

std::unique_ptr<URArm::state_> URArm::state_::create(std::string configured_model_type,
                                                     std::string resource_name,
                                                     const ResourceConfig& config,
                                                     const struct ports_& ports) {
    auto host = find_config_attribute<std::string>(config, "host").value();

    const auto module_executable_path = boost::dll::program_location();
    const auto module_executable_directory = module_executable_path.parent_path();
    auto resource_root = std::filesystem::canonical(module_executable_directory / k_relpath_bindir_to_datadir / "universal-robots");
    VIAM_SDK_LOG(debug) << "Universal robots module executable found in `" << module_executable_path << "; resources will be found in `"
                        << resource_root << "`";

    auto urcl_resource_root = std::filesystem::canonical(module_executable_directory / k_relpath_bindir_to_urcl_resources);
    VIAM_SDK_LOG(debug) << "URCL resources will be found in `" << urcl_resource_root << "`";

    // If the config contains `telemetry_output_path`, use that, otherwise,
    // fall back to `VIAM_MODULE_DATA` as the output path, which must
    // be set.
    auto telemetry_output_path = [&] {
        auto path = find_config_attribute<std::string>(config, "telemetry_output_path");
        if (path) {
            return path.value();
        }

        // TODO(RSDK-12929): When `csv_output_path` is removed, delete this.
        path = find_config_attribute<std::string>(config, "csv_output_path");
        if (path) {
            VIAM_SDK_LOG(warn) << "The `csv_output_path` configuration parameter is deprecated and will be removed; please use "
                                  "`telemetry_output_path` instead";
            return path.value();
        }

        auto* const viam_module_data = std::getenv("VIAM_MODULE_DATA");  // NOLINT: Yes, we know getenv isn't thread safe
        if (!viam_module_data) {
            throw std::runtime_error("required environment variable `VIAM_MODULE_DATA` unset");
        }
        VIAM_SDK_LOG(debug) << "VIAM_MODULE_DATA: " << viam_module_data;

        return std::string{viam_module_data};
    }();

    auto threshold_deg = find_config_attribute<double>(config, "reject_move_request_threshold_deg");
    std::optional<double> threshold_rad;
    if (threshold_deg) {
        threshold_rad = degrees_to_radians(*threshold_deg);
    }

    auto waypoint_dedup_tolerance_deg = find_config_attribute<double>(config, "waypoint_deduplication_tolerance_deg");
    double waypoint_dedup_tolerance_rad = waypoint_dedup_tolerance_deg ? degrees_to_radians(*waypoint_dedup_tolerance_deg)
                                                                       : URArm::k_default_waypoint_deduplication_tolerance_rads;

    auto frequency = find_config_attribute<double>(config, "robot_control_freq_hz");
    auto use_new_planner = find_config_attribute<bool>(config, "enable_new_trajectory_planner").value_or(false);

    auto path_tolerance_deg = find_config_attribute<double>(config, "path_tolerance_delta_deg");
    auto path_tolerance_rad = path_tolerance_deg ? degrees_to_radians(*path_tolerance_deg) : URArm::k_default_path_tolerance_delta_rads;

    auto colinearization_ratio = find_config_attribute<double>(config, "path_colinearization_ratio");

    const double max_trajectory_duration_secs =
        find_config_attribute<double>(config, "max_trajectory_duration_secs").value_or(URArm::k_default_max_trajectory_duration_secs);

    const double trajectory_sampling_freq_hz =
        find_config_attribute<double>(config, "trajectory_sampling_freq_hz").value_or(URArm::k_default_trajectory_sampling_freq_hz);

    const bool telemetry_output_path_append_traceid =
        find_config_attribute<bool>(config, "telemetry_output_path_append_traceid").value_or(false);

    auto state = std::make_unique<state_>(private_{},
                                          std::move(configured_model_type),
                                          std::move(resource_name),
                                          std::move(host),
                                          std::move(resource_root),
                                          std::move(urcl_resource_root),
                                          std::move(telemetry_output_path),
                                          std::move(threshold_rad),
                                          std::move(waypoint_dedup_tolerance_rad),
                                          std::move(frequency),
                                          path_tolerance_rad,
                                          colinearization_ratio,
                                          use_new_planner,
                                          max_trajectory_duration_secs,
                                          trajectory_sampling_freq_hz,
                                          telemetry_output_path_append_traceid,
                                          ports);

    state->set_max_velocity(parse_and_validate_joint_limits(config, "speed_degs_per_sec"));
    state->set_max_acceleration(parse_and_validate_joint_limits(config, "acceleration_degs_per_sec2"));

    // Hold the mutex while we start the worker thread. It will not be
    // able to advance, but will be ready to take over work as soon as
    // we release the lock, minimizing the delay between establishing
    // a connection and allowing the worker thread to begin meeting
    // its service obligations.
    const std::lock_guard lock(state->mutex_);
    state->worker_thread_ = std::thread{&state_::run_, state.get()};

    // Attempt to manually drive the state machine out of the
    // disconnected state. For now, just try a few times with the
    // natural recovery cycle.
    size_t connection_failures = 0;
    constexpr size_t max_connection_failures = 5;
    while (std::holds_alternative<state_disconnected_>(state->current_state_)) {
        try {
            state->upgrade_downgrade_();
        } catch (const std::exception& xcp) {
            VIAM_SDK_LOG(warn) << "Failed to establish working connection to arm after " << ++connection_failures << " attempts: `"
                               << xcp.what() << "`";
            if (connection_failures >= max_connection_failures) {
                throw;
            }
            VIAM_SDK_LOG(info) << "Retrying after " << state->get_timeout_().count() << " milliseconds";
        }
        std::this_thread::sleep_for(state->get_timeout_());
    }

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
        VIAM_SDK_LOG(debug) << "URArm shutdown waiting for worker thread to terminate";
        worker.join();
        VIAM_SDK_LOG(debug) << "worker thread terminated";
    }
}

const std::optional<double>& URArm::state_::get_reject_move_request_threshold_rad() const {
    // NOTE: It is OK to return this as a reference and without taking the lock, as this field is immutable inside `state_`.
    return reject_move_request_threshold_rad_;
}

double URArm::state_::get_waypoint_deduplication_tolerance_rad() const {
    return waypoint_deduplication_tolerance_rad_;
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

vector6d_t URArm::state_::read_tcp_forces_at_base() const {
    const std::lock_guard lock{mutex_};
    if (!ephemeral_) {
        std::ostringstream buffer;
        buffer << "read_tcp_forces_at_base: tcp forces are not currently known; current state: " << describe_();
        throw std::runtime_error(buffer.str());
    }

    return ephemeral_->tcp_forces;
}

URArm::state_::tcp_state_snapshot URArm::state_::read_tcp_state_snapshot() const {
    const std::lock_guard lock{mutex_};
    if (!ephemeral_) {
        std::ostringstream buffer;
        buffer << "read_tcp_state_snapshot: tcp state not currently available; current state: " << describe_();
        throw std::runtime_error(buffer.str());
    }

    return {ephemeral_->tcp_state, ephemeral_->tcp_forces};
}

std::filesystem::path URArm::state_::telemetry_output_path() const {
    // If trace-id appending is not enabled, return the base path
    if (!telemetry_output_path_append_traceid_) {
        return telemetry_output_path_;
    }

    // Try to get the trace-id from the current gRPC context
    const auto& observer = viam::sdk::GrpcContextObserver::current();
    if (!observer) {
        // No gRPC context available, return base path
        return telemetry_output_path_;
    }

    const auto traceparent_values = observer->get_client_metadata_field_values("traceparent");
    if (traceparent_values.empty()) {
        // No traceparent header, return base path
        return telemetry_output_path_;
    }

    const auto trace_id = extract_trace_id_from_traceparent(traceparent_values[0]);
    if (!trace_id) {
        // Failed to parse trace-id, return base path
        return telemetry_output_path_;
    }

    // Append trace-id as a subdirectory
    auto result = telemetry_output_path_ / *trace_id;

    // Ensure the directory exists before returning
    std::error_code ec;
    std::filesystem::create_directories(result, ec);
    if (ec) {
        VIAM_SDK_LOG(warn) << "Failed to create telemetry output directory '" << result << "': " << ec.message()
                           << " - falling back to base path";
        return telemetry_output_path_;
    }

    return result;
}

const std::filesystem::path& URArm::state_::resource_root() const {
    return resource_root_;
}

const std::filesystem::path& URArm::state_::urcl_resource_root() const {
    return urcl_resource_root_;
}

const std::string& URArm::state_::resource_name() const {
    return resource_name_;
}

bool URArm::state_::telemetry_output_path_append_traceid() const {
    return telemetry_output_path_append_traceid_;
}

void URArm::state_::set_max_velocity(const vector6d_t& velocity) {
    const std::lock_guard lock(mutex_);
    max_velocity_ = velocity;
}

void URArm::state_::set_max_velocity(double velocity) {
    vector6d_t v;
    v.fill(velocity);
    set_max_velocity(v);
}

vector6d_t URArm::state_::get_max_velocity() const {
    const std::lock_guard lock(mutex_);
    return max_velocity_;
}

void URArm::state_::set_max_acceleration(const vector6d_t& acceleration) {
    const std::lock_guard lock(mutex_);
    max_acceleration_ = acceleration;
}

void URArm::state_::set_max_acceleration(double acceleration) {
    vector6d_t a;
    a.fill(acceleration);
    set_max_acceleration(a);
}

vector6d_t URArm::state_::get_max_acceleration() const {
    const std::lock_guard lock(mutex_);
    return max_acceleration_;
}

double URArm::state_::get_path_tolerance_delta_rads() const {
    return path_tolerance_delta_rads_;
}

const std::optional<double>& URArm::state_::get_path_colinearization_ratio() const {
    return path_colinearization_ratio_;
}

bool URArm::state_::use_new_trajectory_planner() const {
    return use_new_trajectory_planner_;
}

double URArm::state_::get_max_trajectory_duration_secs() const {
    return max_trajectory_duration_secs_;
}

double URArm::state_::get_trajectory_sampling_freq_hz() const {
    return trajectory_sampling_freq_hz_;
}

size_t URArm::state_::get_move_epoch() const {
    return move_epoch_.load(std::memory_order_acquire);
}

bool URArm::state_::is_moving() const {
    const std::lock_guard lock{mutex_};
    if (!move_request_) {
        return false;
    }
    return std::visit(
        [](const auto& cmd) -> bool {
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, std::vector<trajectory_sample_point>>) {
                // If we have an empty vector of trajectory_sample_point it means we have sent them to the arm
                // so, as far as we are concerned, the arm is moving, though it may fail later.
                return cmd.empty();
            } else if constexpr (std::is_same_v<T, std::optional<pose_sample>>) {
                // If we have nullopt it means we have sent it to the arm
                // so, as far as we are concerned, the arm is moving, though it may fail later.
                return !cmd.has_value();
            }
        },
        move_request_->move_command);
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

URArm::state_::arm_connection_::~arm_connection_() {
    data_package.reset();
    if (log_destructor) {
        VIAM_SDK_LOG(debug) << "destroying current UrDriver instance";
    }
    driver.reset();
    if (log_destructor) {
        VIAM_SDK_LOG(debug) << "destroying current DashboardClient instance";
    }
    dashboard.reset();
}

URArm::state_::move_request::move_request(std::optional<std::ofstream> arm_joint_positions_stream, move_command_data&& move_command)
    : arm_joint_positions_stream(std::move(arm_joint_positions_stream)), move_command(std::move(move_command)) {
    // Validate the move command based on its type
    std::visit(
        [](const auto& cmd) {
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, std::vector<trajectory_sample_point>>) {
                if (cmd.empty()) {
                    throw std::invalid_argument("no trajectory samples provided to move request");
                }
            } else if constexpr (std::is_same_v<T, std::optional<pose_sample>>) {
                if (!cmd.has_value()) {
                    throw std::invalid_argument("no pose provided to move request");
                }
            }
        },
        this->move_command);
}

URArm::state_::move_request::move_request(std::optional<std::ofstream> arm_joint_positions_stream,
                                          std::vector<trajectory_sample_point>&& tsps)
    : move_request(std::move(arm_joint_positions_stream), move_command_data{std::move(tsps)}) {}

URArm::state_::move_request::move_request(std::optional<std::ofstream> arm_joint_positions_stream, pose_sample ps)
    : move_request(std::move(arm_joint_positions_stream), move_command_data{std::optional<pose_sample>{std::move(ps)}}) {}

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
    if (arm_joint_positions_stream) {
        ::write_joint_data(position, velocity, *arm_joint_positions_stream, unix_time_iso8601(), arm_joint_positions_sample++);
    }
}

URArm::state_::move_request::cancellation_request::cancellation_request() {}

void URArm::state_::emit_event_(event_variant_&& event) {
    std::visit([this](auto&& event) { this->emit_event_(std::forward<decltype(event)>(event)); }, std::move(event));
}

void URArm::state_::clear_pstop() const {
    const std::lock_guard lock{mutex_};
    std::visit([](auto& state) { state.clear_pstop(); }, current_state_);
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

std::string URArm::state_::describe() const {
    const std::lock_guard lock{mutex_};
    return describe_();
}

std::string URArm::state_::describe_state_(const state_variant_& state) {
    return std::visit([](auto& state) { return state.describe(); }, state);
}

bool URArm::state_::is_current_state_controlled(std::string* description /*= nullptr*/) const {
    const std::lock_guard lock{mutex_};
    if (description != nullptr) {
        *description = describe_();
    }
    return std::holds_alternative<state_controlled_>(current_state_);
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
    VIAM_SDK_LOG(debug) << "worker thread started";

    // Periodically, collect a limited number of samples of the
    // duration of our wait latency on the condition variable. We
    // expect this to fairly well respect the configured
    // `robot_control_freq_hz`. Report this in the log so that we can
    // know how well we are doing staying a reliable consumer of arm
    // data. The logging also serves as a proof of forward progress
    // for the worker thread.
    constexpr std::size_t k_num_samples = 100;
    constexpr auto k_sampling_interval = std::chrono::minutes(5);
    auto last_sampling_point = std::chrono::steady_clock::now();

    namespace bacc = ::boost::accumulators;
    std::optional<bacc::accumulator_set<double, bacc::stats<bacc::tag::median, bacc::tag::variance>>> accumulator;

    while (true) {
        std::unique_lock lock(mutex_);

        const auto wait_start = std::chrono::steady_clock::now();
        if (worker_wakeup_cv_.wait_for(lock, get_timeout_(), [this] { return shutdown_requested_; })) {
            VIAM_SDK_LOG(debug) << "worker thread signaled to terminate";
            break;
        }

        if (!accumulator && ((wait_start - last_sampling_point) > k_sampling_interval)) {
            last_sampling_point = wait_start;
            accumulator.emplace();
        }

        if (accumulator) {
            auto wait_end = std::chrono::steady_clock::now();
            (*accumulator)(std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(wait_end - wait_start).count());
            if (bacc::count(*accumulator) == k_num_samples) {
                const auto accumulated = std::exchange(accumulator, std::nullopt);
                VIAM_SDK_LOG(info) << "URArm worker thread median wait between control cycles is " << bacc::median(*accumulated)
                                   << " milliseconds, with variance " << bacc::variance(*accumulated);
            }
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

    VIAM_SDK_LOG(debug) << "worker thread emitting disconnection event";
    emit_event_(event_connection_lost_::module_shutdown());
    VIAM_SDK_LOG(debug) << "worker thread terminating";
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

    VIAM_SDK_LOG(debug) << "trajectory report: " << report;
}

void URArm::state_::program_running_callback_(bool running) {
    program_running_flag.store(running, std::memory_order_release);
    if (running) {
        VIAM_SDK_LOG(debug) << "UR program is running";
        return;
    }
    VIAM_SDK_LOG(warn) << "UR program is not running";
}
