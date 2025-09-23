#include "ur_arm_state.hpp"

#include <filesystem>
#include <stdexcept>
#include <thread>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/dll/runtime_symbol_info.hpp>

// The `ur_arm_config.hpp` header is only generated for non-AppImage
// builds. If the header exists, include it and note that we aren't
// using appimage. Otherwise, we are. Name a constant to distinguish
// the cases.
#if __has_include("ur_arm_config.hpp")

#include "ur_arm_config.hpp"

namespace {

std::filesystem::path find_resource_root() {
    const auto module_executable_path = boost::dll::program_location();
    const auto module_executable_directory = module_executable_path.parent_path();
    auto data_directory = std::filesystem::canonical(module_executable_directory / k_relpath_bindir_to_datadir / "universal-robots");
    VIAM_SDK_LOG(info) << "Universal robots module executable found in `" << module_executable_path << "; resources will be found in `"
                       << data_directory << "`";
    return data_directory;
}

}  // namespace

#else

namespace {

std::filesystem::path find_resource_root() {
    // get the APPDIR environment variable
    auto* const app_dir = std::getenv("APPDIR");  // NOLINT: Yes, we know getenv isn't thread safe
    if (!app_dir) {
        throw std::runtime_error("required environment variable `APPDIR` unset");
    }
    auto data_directory = std::filesystem::path{app_dir} / "src";
    VIAM_SDK_LOG(info) << "APPDIR is `" << app_dir << "`; resources will be found in `" << data_directory << "`";
    return data_directory;
}

}  // namespace

#endif

#include "utils.hpp"

namespace {

constexpr auto k_default_robot_control_freq_hz = 100.;

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
                      std::string host,
                      std::filesystem::path resource_root,
                      std::filesystem::path csv_output_path,
                      std::optional<double> reject_move_request_threshold_rad,
                      std::optional<double> robot_control_freq_hz,
                      const struct ports_& ports)
    : configured_model_type_{std::move(configured_model_type)},
      host_{std::move(host)},
      resource_root_{std::move(resource_root)},
      csv_output_path_{std::move(csv_output_path)},
      robot_control_freq_hz_(robot_control_freq_hz.value_or(k_default_robot_control_freq_hz)),
      reject_move_request_threshold_rad_(std::move(reject_move_request_threshold_rad)),
      ports_{ports} {}

URArm::state_::~state_() {
    shutdown();
}

std::unique_ptr<URArm::state_> URArm::state_::create(std::string configured_model_type,
                                                     const ResourceConfig& config,
                                                     const struct ports_& ports) {
    auto host = find_config_attribute<std::string>(config, "host").value();

    auto resource_root = find_resource_root();

    // If the config contains `csv_output_path`, use that, otherwise,
    // fall back to `VIAM_MODULE_DATA` as the output path, which must
    // be set.
    auto csv_output_path = [&] {
        auto path = find_config_attribute<std::string>(config, "csv_output_path");
        if (path) {
            return path.value();
        }

        auto* const viam_module_data = std::getenv("VIAM_MODULE_DATA");  // NOLINT: Yes, we know getenv isn't thread safe
        if (!viam_module_data) {
            throw std::runtime_error("required environment variable `VIAM_MODULE_DATA` unset");
        }
        VIAM_SDK_LOG(info) << "VIAM_MODULE_DATA: " << viam_module_data;

        return std::string{viam_module_data};
    }();

    auto threshold = find_config_attribute<double>(config, "reject_move_request_threshold_deg");
    auto frequency = find_config_attribute<double>(config, "robot_control_freq_hz");

    auto state = std::make_unique<state_>(private_{},
                                          std::move(configured_model_type),
                                          std::move(host),
                                          std::move(resource_root),
                                          std::move(csv_output_path),
                                          std::move(threshold),
                                          std::move(frequency),
                                          ports);

    state->set_speed(degrees_to_radians(find_config_attribute<double>(config, "speed_degs_per_sec").value()));
    state->set_acceleration(degrees_to_radians(find_config_attribute<double>(config, "acceleration_degs_per_sec2").value()));

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
            VIAM_SDK_LOG(warn) << "Retrying after " << state->get_timeout_().count() << " milliseconds";
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

const std::filesystem::path& URArm::state_::csv_output_path() const {
    return csv_output_path_;
}

const std::filesystem::path& URArm::state_::resource_root() const {
    return resource_root_;
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

URArm::state_::arm_connection_::~arm_connection_() {
    data_package.reset();
    if (log_destructor) {
        VIAM_SDK_LOG(info) << "destroying current UrDriver instance";
    }
    driver.reset();
    if (log_destructor) {
        VIAM_SDK_LOG(info) << "destroying current DashboardClient instance";
    }
    dashboard.reset();
}

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
            VIAM_SDK_LOG(info) << "worker thread signaled to terminate";
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

    VIAM_SDK_LOG(info) << "worker thread emitting disconnection event";
    emit_event_(event_connection_lost_::module_shutdown());
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
