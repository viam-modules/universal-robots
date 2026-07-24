#include "ur_arm_state.hpp"

#include <algorithm>
#include <filesystem>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/io/ostream_joiner.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/uuid/random_generator.hpp>

#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/rpc/grpc_context_observer.hpp>

#include "ur_arm_config.hpp"
#include "utils.hpp"

URArm::state_::state_(private_,
                      UrArmModel configured_model,
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
                      double segmentation_threshold,
                      bool use_new_trajectory_planner,
                      bool prefer_precomputed_accelerations,
                      bool segment_for_trajex,
                      double max_trajectory_duration_secs,
                      std::optional<vector6d_t> max_velocity_limits,
                      std::optional<vector6d_t> max_acceleration_limits,
                      double trajectory_sampling_freq_hz,
                      std::string telemetry_output_path_append_traceid_template,
                      std::string traceid_metadata_key,
                      const struct ports_& ports)
    : configured_model_{std::move(configured_model)},
      resource_name_{std::move(resource_name)},
      host_{std::move(host)},
      resource_root_{std::move(resource_root)},
      urcl_resource_root_{std::move(urcl_resource_root)},
      telemetry_output_path_{std::move(telemetry_output_path)},
      robot_control_freq_hz_(robot_control_freq_hz.value_or(URArm::k_default_robot_control_freq_hz)),
      reject_move_request_threshold_rad_(std::move(reject_move_request_threshold_rad)),
      ports_{ports},
      waypoint_deduplication_tolerance_rad_(waypoint_deduplication_tolerance_rad),
      path_tolerance_delta_rads_(path_tolerance_delta_rads),
      path_colinearization_ratio_(path_colinearization_ratio),
      segmentation_threshold_(segmentation_threshold),
      use_new_trajectory_planner_(use_new_trajectory_planner),
      prefer_precomputed_accelerations_(prefer_precomputed_accelerations),
      segment_for_trajex_(segment_for_trajex),
      max_trajectory_duration_secs_(max_trajectory_duration_secs),
      max_velocity_limits_(std::move(max_velocity_limits)),
      max_acceleration_limits_(std::move(max_acceleration_limits)),
      trajectory_sampling_freq_hz_(trajectory_sampling_freq_hz),
      telemetry_output_path_append_traceid_template_(std::move(telemetry_output_path_append_traceid_template)),
      traceid_metadata_key_(std::move(traceid_metadata_key)) {
    // Initialize the kinematics promise/future pair so that
    // `kinematics_future_.valid()` is unconditionally true for the rest of
    // `state_`'s lifetime. The producer side (see
    // `state_connected_::recv_arm_data`) adopts this slot on the first tick
    // of each connection period, fulfilling and (if needed) replacing it
    // across connections; readers never have to check `valid()`.
    kinematics_promise_.emplace();
    kinematics_future_ = kinematics_promise_->get_future().share();
}

URArm::state_::~state_() {
    shutdown();
}

std::unique_ptr<URArm::state_> URArm::state_::create(UrArmModel configured_model,
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
    const auto waypoint_dedup_tolerance_rad = waypoint_dedup_tolerance_deg ? degrees_to_radians(*waypoint_dedup_tolerance_deg)
                                                                           : URArm::k_default_waypoint_deduplication_tolerance_rads;

    auto frequency = find_config_attribute<double>(config, "robot_control_freq_hz");
    auto use_new_planner = find_config_attribute<bool>(config, "enable_new_trajectory_planner").value_or(true);
    auto prefer_precomputed_accels = find_config_attribute<bool>(config, "prefer_precomputed_accelerations").value_or(true);
    const auto segment_for_trajex = find_config_attribute<bool>(config, "segment_for_trajex").value_or(false);

    auto path_tolerance_deg = find_config_attribute<double>(config, "path_tolerance_delta_deg");
    auto path_tolerance_rad = path_tolerance_deg ? degrees_to_radians(*path_tolerance_deg) : URArm::k_default_path_tolerance_delta_rads;

    auto colinearization_ratio = find_config_attribute<double>(config, "path_colinearization_ratio");
    const double segmentation_threshold =
        find_config_attribute<double>(config, "segmentation_threshold").value_or(URArm::k_default_segmentation_threshold);

    const double max_trajectory_duration_secs =
        find_config_attribute<double>(config, "max_trajectory_duration_secs").value_or(URArm::k_default_max_trajectory_duration_secs);

    const double trajectory_sampling_freq_hz =
        find_config_attribute<double>(config, "trajectory_sampling_freq_hz").value_or(URArm::k_default_trajectory_sampling_freq_hz);

    // Parse `telemetry_output_path_append_traceid` — accepts bool (backward compat) or template string with {trace_id}
    const auto telemetry_output_path_append_traceid_template = [&]() -> std::string {
        const auto it = config.attributes().find("telemetry_output_path_append_traceid");
        if (it == config.attributes().end()) {
            return {};
        }
        if (const auto* s = it->second.get<std::string>()) {
            return *s;
        }
        if (const auto* b = it->second.get<bool>(); b && *b) {
            return "{trace_id}";
        }
        return {};
    }();

    // Parse `traceid_metadata_key` — the request-metadata field the trace id is read from. When
    // unset, the trace id is derived from the traceparent header (see telemetry_output_path).
    const auto traceid_metadata_key = find_config_attribute<std::string>(config, "traceid_metadata_key").value_or("");

    // Look for the optional settings to place an upper bound on what velocity and acceleration limits may be
    // configured, or set via DoCommand or applied via MoveOptions. Note that the parse/validate function automatically
    // converts from degrees to radians.

    std::optional<vector6d_t> max_velocity_limits;
    if (config.attributes().contains("max_speed_degs_per_sec")) {
        max_velocity_limits = parse_and_validate_joint_limits(config, "max_speed_degs_per_sec");
    }

    std::optional<vector6d_t> max_acceleration_limits;
    if (config.attributes().contains("max_acceleration_degs_per_sec2")) {
        max_acceleration_limits = parse_and_validate_joint_limits(config, "max_acceleration_degs_per_sec2");
    }

    auto state = std::make_unique<state_>(private_{},
                                          std::move(configured_model),
                                          std::move(resource_name),
                                          std::move(host),
                                          std::move(resource_root),
                                          std::move(urcl_resource_root),
                                          std::move(telemetry_output_path),
                                          std::move(threshold_rad),
                                          waypoint_dedup_tolerance_rad,
                                          std::move(frequency),
                                          path_tolerance_rad,
                                          colinearization_ratio,
                                          segmentation_threshold,
                                          use_new_planner,
                                          prefer_precomputed_accels,
                                          segment_for_trajex,
                                          max_trajectory_duration_secs,
                                          std::move(max_velocity_limits),
                                          std::move(max_acceleration_limits),
                                          trajectory_sampling_freq_hz,
                                          telemetry_output_path_append_traceid_template,
                                          traceid_metadata_key,
                                          ports);

    state->set_velocity_limits(parse_and_validate_joint_limits(config, "speed_degs_per_sec"));
    state->set_acceleration_limits(parse_and_validate_joint_limits(config, "acceleration_degs_per_sec2"));

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
    // If no template is set, trace-id appending is disabled
    if (telemetry_output_path_append_traceid_template_.empty()) {
        return telemetry_output_path_;
    }

    // Try to get the trace-id from the current gRPC context
    const auto& observer = viam::sdk::GrpcContextObserver::current();
    if (!observer) {
        // No gRPC context available, return base path
        return telemetry_output_path_;
    }

    // Resolve the trace-id: read it directly from the configured request-metadata field if one is
    // set, otherwise parse it from the traceparent header (the default).
    std::optional<std::string> trace_id;
    if (!traceid_metadata_key_.empty()) {
        const auto values = observer->get_client_metadata_field_values(traceid_metadata_key_);
        if (!values.empty()) {
            trace_id = values[0];
        }
    } else {
        const auto traceparent_values = observer->get_client_metadata_field_values("traceparent");
        if (!traceparent_values.empty()) {
            trace_id = extract_trace_id_from_traceparent(traceparent_values[0]);
        }
    }
    if (!trace_id) {
        // No trace-id available, return base path
        return telemetry_output_path_;
    }

    // Expand the template by replacing {trace_id} with the actual trace-id
    auto result = expand_telemetry_path(telemetry_output_path_, telemetry_output_path_append_traceid_template_, *trace_id);

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

const std::string& URArm::state_::telemetry_output_path_append_traceid() const {
    return telemetry_output_path_append_traceid_template_;
}

vector6d_t URArm::state_::set_velocity_limits(vector6d_t velocity) {
    const std::lock_guard lock(mutex_);
    return velocity_limits_ = clamp_velocity_limits(velocity);
}

vector6d_t URArm::state_::set_velocity_limits(double velocity) {
    vector6d_t v;
    v.fill(velocity);
    return set_velocity_limits(v);
}

vector6d_t URArm::state_::get_velocity_limits() const {
    const std::lock_guard lock(mutex_);
    return velocity_limits_;
}

vector6d_t URArm::state_::clamp_velocity_limits(vector6d_t desired_velocity_limits) {
    const auto result = clamp_limits_(desired_velocity_limits, max_velocity_limits_);
    if (max_velocity_limits_ && result != desired_velocity_limits) {
        std::ostringstream msg;
        msg << "Velocity limits clamped from [";
        boost::copy(desired_velocity_limits | boost::adaptors::transformed(radians_to_degrees<const double&>),
                    boost::io::make_ostream_joiner(msg, ", "));
        msg << "] to [";
        boost::copy(result | boost::adaptors::transformed(radians_to_degrees<const double&>), boost::io::make_ostream_joiner(msg, ", "));
        msg << "] deg/s";
        VIAM_SDK_LOG(debug) << msg.str();
    }
    return result;
}

vector6d_t URArm::state_::set_acceleration_limits(vector6d_t acceleration) {
    const std::lock_guard lock(mutex_);
    return acceleration_limits_ = clamp_acceleration_limits(acceleration);
}

vector6d_t URArm::state_::set_acceleration_limits(double acceleration) {
    vector6d_t a;
    a.fill(acceleration);
    return set_acceleration_limits(a);
}

vector6d_t URArm::state_::get_acceleration_limits() const {
    const std::lock_guard lock(mutex_);
    return acceleration_limits_;
}

vector6d_t URArm::state_::clamp_acceleration_limits(vector6d_t desired_acceleration_limits) {
    const auto result = clamp_limits_(desired_acceleration_limits, max_acceleration_limits_);
    if (max_acceleration_limits_ && result != desired_acceleration_limits) {
        std::ostringstream msg;
        msg << "Acceleration limits clamped from [";
        boost::copy(desired_acceleration_limits | boost::adaptors::transformed(radians_to_degrees<const double&>),
                    boost::io::make_ostream_joiner(msg, ", "));
        msg << "] to [";
        boost::copy(result | boost::adaptors::transformed(radians_to_degrees<const double&>), boost::io::make_ostream_joiner(msg, ", "));
        msg << "] deg/s/s";
        VIAM_SDK_LOG(debug) << msg.str();
    }
    return result;
}

double URArm::state_::get_path_tolerance_delta_rads() const {
    return path_tolerance_delta_rads_;
}

const std::optional<double>& URArm::state_::get_path_colinearization_ratio() const {
    return path_colinearization_ratio_;
}

double URArm::state_::get_segmentation_threshold() const {
    return segmentation_threshold_;
}

bool URArm::state_::use_new_trajectory_planner() const {
    return use_new_trajectory_planner_;
}

bool URArm::state_::prefer_precomputed_accelerations() const {
    return prefer_precomputed_accelerations_;
}

bool URArm::state_::segment_for_trajex() const {
    return segment_for_trajex_;
}

double URArm::state_::get_max_trajectory_duration_secs() const {
    return max_trajectory_duration_secs_;
}

double URArm::state_::get_trajectory_sampling_freq_hz() const {
    return trajectory_sampling_freq_hz_;
}

URArm::move_id URArm::state_::allocate_move_id() const {
    // We snapshot the `generation` now. The `uuid` is freshly generated and has no
    // ordering relationship to any other allocation.
    static thread_local boost::uuids::random_generator generator;
    return {generator(), move_epoch_.load(std::memory_order_acquire)};
}

bool URArm::state_::is_moving() const {
    const std::lock_guard lock{mutex_};
    if (!move_request_) {
        return false;
    }
    return std::visit(
        [](const auto& cmd) -> bool {
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, sample_stream>) {
                // We only count as moving once URCL has been told to start,
                // meaning STREAM_START has been sent. In `k_open` or `k_buffered` the
                // points have not reached the controller yet, so we don't claim
                // the arm is moving.
                return cmd.current_phase != sample_stream::phase::k_open && cmd.current_phase != sample_stream::phase::k_buffered;
            } else if constexpr (std::is_same_v<T, std::optional<pose_sample>>) {
                // Disengaged optional means we have already sent the pose to
                // the arm; we are moving (or about to be).
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

bool URArm::state_::extend_move_request(const boost::uuids::uuid& id, trajectory_samples batch) {
    const std::lock_guard lock{mutex_};
    if (!move_request_ || move_request_->id != id) {
        // Our move is no longer the active one: the worker finished it, or a newer
        // move has claimed the slot. Report that rather than throw; the producer
        // treats it as the worker finishing early.
        return false;
    }

    // Streams can only be extended before the producer has closed.
    auto* const stream = std::get_if<sample_stream>(&move_request_->move_command);
    if (!stream) {
        throw std::runtime_error("extend_move_request: the in-flight move is not a stream");
    }
    if (stream->current_phase != sample_stream::phase::k_open && stream->current_phase != sample_stream::phase::k_streaming) {
        throw std::runtime_error("extend_move_request: stream has been closed and cannot be extended");
    }

    // Append the incoming batch to `pending`. The first extend picks PV or PVA by
    // which variant it holds, and later extends must match. The producer already
    // rejects a client mismatch as a protocol error, so a mismatch here would be
    // a bug on our side, and we treat it as one.
    if (!stream->pending) {
        stream->pending = std::move(batch);
    } else {
        std::visit(
            [&](auto& dest_vec) {
                using DestVec = std::decay_t<decltype(dest_vec)>;
                auto* src_vec = std::get_if<DestVec>(&batch);
                if (!src_vec) {
                    throw std::logic_error("extend_move_request: PV/PVA arm mismatch within a stream");
                }
                dest_vec.insert(dest_vec.end(), std::make_move_iterator(src_vec->begin()), std::make_move_iterator(src_vec->end()));
            },
            *stream->pending);
    }

    return true;
}

bool URArm::state_::close_move_request(const boost::uuids::uuid& id) {
    const std::lock_guard lock{mutex_};
    if (!move_request_ || move_request_->id != id) {
        // See `extend_move_request`: our move is gone, so report it rather than throw.
        return false;
    }

    auto* const stream = std::get_if<sample_stream>(&move_request_->move_command);
    if (!stream) {
        throw std::runtime_error("close_move_request: the in-flight move is not a stream");
    }

    switch (stream->current_phase) {
        case sample_stream::phase::k_open:
            stream->current_phase = sample_stream::phase::k_buffered;
            break;
        case sample_stream::phase::k_streaming:
            stream->current_phase = sample_stream::phase::k_draining;
            break;
        case sample_stream::phase::k_buffered:
        case sample_stream::phase::k_draining:
        case sample_stream::phase::k_ended:
            throw std::runtime_error("close_move_request: stream has already been closed");
    }

    return true;
}

std::optional<std::shared_future<void>> URArm::state_::cancel_move_request(const boost::uuids::uuid& id) {
    const std::lock_guard lock{mutex_};
    if (!move_request_ || move_request_->id != id) {
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

URArm::state_::move_request::move_request(boost::uuids::uuid id,
                                          std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                                          async_cancellation_monitor monitor,
                                          move_command_data&& move_command)
    : id(std::move(id)),
      trajectory_logger(std::move(trajectory_logger)),
      async_cancel_monitor(std::move(monitor)),
      move_command(std::move(move_command)) {
    // Validate the move command based on its type.
    std::visit(
        [](const auto& cmd) {
            using T = std::decay_t<decltype(cmd)>;
            if constexpr (std::is_same_v<T, sample_stream>) {
                // A `sample_stream` must arrive in one of its two valid initial shapes,
                // with nothing written yet: freshly opened for streaming (`k_open`,
                // nothing pending; the producer adds points later via
                // `state_::extend_move_request`), or a buffered one-shot (`k_buffered`
                // with the whole trajectory already pending).
                const bool freshly_open = (cmd.current_phase == sample_stream::phase::k_open) && !cmd.pending.has_value();
                const bool buffered_one_shot = (cmd.current_phase == sample_stream::phase::k_buffered) && cmd.pending.has_value();
                if (cmd.points_written != 0 || !(freshly_open || buffered_one_shot)) {
                    throw std::invalid_argument("sample_stream must be freshly opened or a buffered one-shot when handed to move_request");
                }

                // A buffered one-shot must actually carry points; an empty trajectory is not a move.
                if (buffered_one_shot && std::visit([](const auto& v) { return v.empty(); }, *cmd.pending)) {
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

URArm::state_::move_request::move_request(boost::uuids::uuid id,
                                          std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                                          async_cancellation_monitor monitor)
    : move_request(std::move(id), std::move(trajectory_logger), std::move(monitor), move_command_data{sample_stream{}}) {}

URArm::state_::move_request::move_request(boost::uuids::uuid id,
                                          std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                                          async_cancellation_monitor monitor,
                                          pose_sample ps)
    : move_request(
          std::move(id), std::move(trajectory_logger), std::move(monitor), move_command_data{std::optional<pose_sample>{std::move(ps)}}) {}

URArm::state_::move_request::move_request(boost::uuids::uuid id,
                                          std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                                          async_cancellation_monitor monitor,
                                          trajectory_samples samples)
    : move_request(std::move(id),
                   std::move(trajectory_logger),
                   std::move(monitor),
                   move_command_data{sample_stream{sample_stream::phase::k_buffered, std::move(samples)}}) {}

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

void URArm::state_::move_request::write_realtime_sample(const ephemeral_data& data,
                                                        std::optional<uint32_t> robot_status_bits,
                                                        std::optional<uint32_t> safety_status_bits) const {
    if (trajectory_logger) {
        const auto now_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        trajectory_logger->append_realtime_sample(now_us, data, robot_status_bits, safety_status_bits);
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

void URArm::state_::zero_ftsensor() const {
    const std::lock_guard lock{mutex_};
    std::visit([](auto& state) { state.zero_ftsensor(); }, current_state_);
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

vector6d_t URArm::state_::clamp_limits_(vector6d_t desired, const std::optional<vector6d_t>& limits) {
    if (limits) {
        std::ranges::transform(desired, *limits, begin(desired), [](auto dd, auto ll) { return std::min(dd, ll); });
    }
    return desired;
}

urcl::primary_interface::KinematicsInfo URArm::state_::get_calibrated_kinematics_info(std::chrono::steady_clock::duration wait_duration) {
    // Copy the shared_future under the lock so we can release before
    // potentially blocking. The copy bumps the shared-state refcount;
    // any concurrent producer replacement of `kinematics_future_` does not
    // affect our local handle.
    std::shared_future<cached_kinematics_payload> fut;
    {
        const std::lock_guard lock{mutex_};
        fut = kinematics_future_;
    }
    if (fut.wait_for(wait_duration) != std::future_status::ready) {
        throw std::runtime_error{"kinematics info not available within deadline"};
    }
    // `.get()` rethrows any producer-side exception forwarded via the
    // promise; otherwise it returns `const cached_kinematics_payload&` into
    // the persistent shared state.
    return fut.get().info;
}

std::string URArm::state_::get_dh_kinematics_json(std::chrono::steady_clock::duration wait_duration) {
    std::shared_future<cached_kinematics_payload> fut;
    {
        const std::lock_guard lock{mutex_};
        fut = kinematics_future_;
    }
    if (fut.wait_for(wait_duration) != std::future_status::ready) {
        throw std::runtime_error{"kinematics info not available within deadline"};
    }
    const auto& payload = fut.get();

    // The first JSON-wanting caller builds the string under `call_once`;
    // subsequent callers reuse the memoized value. `json_once`/`json` are
    // `mutable` on `cached_kinematics_payload` precisely so this lazy build
    // is legal through the `const&` returned by `shared_future::get()`;
    // `json_once` is a `unique_ptr<once_flag>` (dereferenced here) because
    // `std::once_flag` is neither copyable nor movable.
    std::call_once(*payload.json_once, [&] {
        payload.json = payload.arm_model.load_kinematics((resource_root_ / "kinematics" / payload.arm_model.sdk_name()).concat(".json"))
                           .apply_calibration({payload.info.dh_a_, payload.info.dh_d_, payload.info.dh_alpha_, payload.info.dh_theta_})
                           .to_sva_json();
    });
    return payload.json;
}
