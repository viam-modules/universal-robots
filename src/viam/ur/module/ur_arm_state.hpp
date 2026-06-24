#include "ur_arm.hpp"
#include "ur_arm_model.hpp"

#include <bitset>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <variant>

#include <boost/uuid/uuid.hpp>

#include "trajectory_logger.hpp"

#include <ur_client_library/primary/robot_state/kinematics_info.h>
#include <ur_client_library/types.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>

class URArm::state_ {
    struct private_ {};

   public:
    explicit state_(private_,
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
                    const struct ports_& ports);
    ~state_();

    static std::unique_ptr<state_> create(UrArmModel configured_model,
                                          std::string resource_name,
                                          const ResourceConfig& config,
                                          const struct ports_& ports);
    void shutdown();

    struct tcp_state_snapshot {
        vector6d_t pose;
        vector6d_t forces_at_base;
    };

    const std::optional<double>& get_reject_move_request_threshold_rad() const;
    double get_waypoint_deduplication_tolerance_rad() const;
    vector6d_t read_joint_positions() const;
    vector6d_t read_tcp_pose() const;
    vector6d_t read_tcp_forces_at_base() const;
    tcp_state_snapshot read_tcp_state_snapshot() const;

    std::filesystem::path telemetry_output_path() const;
    const std::filesystem::path& resource_root() const;
    const std::filesystem::path& urcl_resource_root() const;

    const std::string& resource_name() const;
    const std::string& telemetry_output_path_append_traceid() const;

    // All values here are in radians/s.
    vector6d_t set_velocity_limits(vector6d_t velocity);
    vector6d_t set_velocity_limits(double velocity);
    vector6d_t get_velocity_limits() const;
    vector6d_t clamp_velocity_limits(vector6d_t desired_velocity_limits);

    // All values here are in radians/s^2.
    vector6d_t set_acceleration_limits(vector6d_t acceleration);
    vector6d_t set_acceleration_limits(double acceleration);
    vector6d_t get_acceleration_limits() const;
    vector6d_t clamp_acceleration_limits(vector6d_t desired_acceleration_limits);

    double get_path_tolerance_delta_rads() const;
    const std::optional<double>& get_path_colinearization_ratio() const;
    double get_segmentation_threshold() const;

    bool use_new_trajectory_planner() const;
    bool prefer_precomputed_accelerations() const;
    bool segment_for_trajex() const;

    double get_max_trajectory_duration_secs() const;
    double get_trajectory_sampling_freq_hz() const;

    void clear_pstop() const;

    ///
    /// Allocate a `move_id` (UUID + epoch snapshot) for a new move. The caller plumbs
    /// the result through whatever planning happens before `start_move_request`, then
    /// presents it as proof at start time. The generation snapshot is validated then;
    /// see start_move_request for the semantics.
    ///
    URArm::move_id allocate_move_id() const;

    ///
    /// Claim the move slot for the move identified by `id` and emplace a fresh
    /// `move_request` constructed from the forwarded `args`. Validates the
    /// generation snapshot against `move_epoch_` (rejecting plans whose
    /// starting arm state has been superseded), then takes `mutex_` and
    /// requires the slot to be empty. Returns the completion future on
    /// success; throws on supersession or on slot-busy. Notifies the worker.
    ///
    template <typename... Args>
    std::future<void> start_move_request(URArm::move_id id, Args&&... args);

    ///
    /// Append a batch of trajectory samples to the open stream identified by
    /// `id`. The batch's `trajectory_samples` variant arm must match the
    /// arm already engaged in `sample_stream::pending` (or, on the first
    /// extend, become the engaged arm). Throws if no in-flight request
    /// matches `id`, or if the stream's phase forbids further extension
    /// (anything past `k_streaming`), or on PV/PVA arm mismatch. Notifies
    /// the worker.
    ///
    void extend_move_request(const boost::uuids::uuid& id, trajectory_samples batch);

    ///
    /// Signal end-of-stream for the move identified by `id`. Transitions
    /// `k_open` to `k_buffered` or `k_streaming` to `k_draining`. Throws if
    /// no in-flight request matches `id`, or if the stream has already been
    /// closed (`k_buffered` / `k_draining` / `k_ended`). Notifies the worker.
    ///
    void close_move_request(const boost::uuids::uuid& id);

    ///
    /// Cancel the move identified by `id`. Identity-validated overload used
    /// by the streaming override's response-sink-said-stop path. Throws if no
    /// in-flight request matches `id`. Routes through the existing
    /// cancellation_request slot on `move_request`; the worker observes the
    /// cancellation at the top of the visitor. Notifies the worker.
    ///
    std::shared_future<void> cancel_move_request(const boost::uuids::uuid& id);

    bool is_moving() const;
    std::string describe() const;
    bool is_current_state_controlled(std::string* description = nullptr) const;

    // Returns the calibrated DH parameters for the currently-connected arm
    // (controller's nominal DH combined with the per-arm calibration deltas).
    //
    // Blocks up to `wait_duration` waiting for the controller to deliver
    // KinematicsInfo on the primary stream. May return a value populated by
    // a *previous* connection while the arm is currently disconnected: this
    // is intentional, because calibration is durable across transient
    // disconnects and the last known value remains the best answer until
    // the controller can be re-queried.
    //
    // Throws std::runtime_error if no KinematicsInfo has yet been observed
    // and `wait_duration` elapses without one arriving. Rethrows any
    // producer-side exception (e.g., a failure surfaced from URCL while
    // attempting to read KinematicsInfo) forwarded via the underlying
    // promise.
    urcl::primary_interface::KinematicsInfo get_calibrated_kinematics_info(std::chrono::steady_clock::duration wait_duration);

    // Returns the synthesized kinematics JSON (SVA form) built from the
    // calibrated DH parameters of the currently-connected arm. Returns an
    // empty string for models that ship static `kinematics/<model>.json`
    // files (the non-UR20 cases are gated early and do not wait on the
    // controller).
    //
    // For models that synthesize the JSON (currently only UR20), blocks up
    // to `wait_duration` waiting for KinematicsInfo. May return a value
    // built from a *previous* connection while currently disconnected, with
    // the same rationale as `get_calibrated_kinematics_info`. The JSON
    // itself is built once per payload via `std::call_once` and reused
    // thereafter.
    //
    // Throws std::runtime_error on `wait_duration` expiry for a model that
    // does synthesize JSON, and rethrows any producer-side exception
    // forwarded via the underlying promise.
    std::string get_dh_kinematics_json(std::chrono::steady_clock::duration wait_duration);

    std::optional<std::shared_future<void>> cancel_move_request();

   private:
    struct arm_connection_;
    struct state_disconnected_;
    friend struct state_disconnected_;

    struct state_controlled_;
    friend struct state_controlled_;

    struct state_independent_;
    friend struct state_independent_;

    using state_variant_ = std::variant<state_disconnected_, state_controlled_, state_independent_>;

    struct event_connection_established_;
    class event_connection_lost_;
    struct event_stop_detected_;
    struct event_stop_cleared_;
    struct event_local_mode_detected_;
    struct event_remote_mode_detected_;

    using event_variant_ = std::variant<event_connection_established_,
                                        event_connection_lost_,
                                        event_stop_detected_,
                                        event_stop_cleared_,
                                        event_local_mode_detected_,
                                        event_remote_mode_detected_>;

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
        state_disconnected_() = default;
        explicit state_disconnected_(event_connection_lost_ triggering_event);

        static std::string_view name();
        std::string describe() const;
        std::chrono::milliseconds get_timeout() const;
        void clear_pstop() const;

        std::optional<event_variant_> recv_arm_data(state_&);
        std::optional<event_variant_> upgrade_downgrade(state_& state);
        std::optional<event_variant_> handle_move_request(state_& state) const;
        std::optional<event_variant_> send_noop();

        std::optional<state_variant_> handle_event(event_connection_lost_ event);
        std::optional<state_variant_> handle_event(event_connection_established_ event);

        using state_event_handler_base_<state_disconnected_>::handle_event;

        std::unique_ptr<arm_connection_> connect_(state_& state);

        // track how often we attempt to reconnect.
        // We will use this to limit how often logs spam during expected behaviors.
        int reconnect_attempts{-1};
        std::optional<std::future<std::unique_ptr<arm_connection_>>> pending_connection;

        // The event that caused us to enter the disconnected state (if any).
        // Using unique_ptr instead of optional because event_connection_lost_ is incomplete here.
        std::unique_ptr<event_connection_lost_> triggering_event_;
    };

    // Fulfillment payload for the `kinematics_future_` cache slot.
    //
    // `mutable` is intentional on `json_once`/`json`: the shared state is
    // accessed via `shared_future<T>::get()` returning `const T&`, and the
    // JSON is a deterministic function of `info` and `arm_model`. Every
    // caller observes the same logical value; `std::call_once` synchronizes
    // the first JSON-wanting caller's build with later reuses.
    //
    // `json_once` is held via `unique_ptr` because `std::once_flag` is
    // neither copyable nor movable, and the payload must be at least
    // move-constructible to flow through `std::promise::set_value`. Wrapping
    // the once_flag (rather than the whole payload) keeps the future's
    // value type a plain `cached_kinematics_payload` and confines the heap
    // indirection to a single field. A default member initializer for
    // `json_once` lets the payload remain a C++20 aggregate so the producer
    // can still construct it with designated initializers.
    struct cached_kinematics_payload {
        urcl::primary_interface::KinematicsInfo info;
        UrArmModel arm_model;
        mutable std::unique_ptr<std::once_flag> json_once{std::make_unique<std::once_flag>()};
        mutable std::string json{};  // NOLINT(readability-redundant-member-init)
    };

    struct arm_connection_ {
        static constexpr size_t k_num_robot_status_bits = 4;
        static constexpr size_t k_num_safety_status_bits = 11;

        ~arm_connection_();

        std::unique_ptr<DashboardClient> dashboard;
        std::unique_ptr<UrDriver> driver;
        std::unique_ptr<rtde_interface::DataPackage> data_package;
        std::optional<std::bitset<k_num_robot_status_bits>> robot_status_bits;
        std::optional<std::bitset<k_num_safety_status_bits>> safety_status_bits;

        bool log_destructor{false};

        // True once `state_connected_::recv_arm_data` has adopted the
        // `state_::kinematics_promise_` / `state_::kinematics_future_` slot
        // on behalf of this connection period. Resets to false implicitly by
        // virtue of being a per-connection field: each fresh
        // `arm_connection_` starts with `false`, so the first tick of every
        // new connection period reattempts adoption. This is what gives the
        // future-based cache its identity tracking without raw-pointer
        // comparisons.
        bool kinematics_claimed_{false};
    };

    struct state_connected_ {
        explicit state_connected_(std::unique_ptr<arm_connection_> arm_conn);

        std::chrono::milliseconds get_timeout() const;

        std::optional<event_variant_> recv_arm_data(state_& state);
        std::optional<event_variant_> send_noop() const;

        std::unique_ptr<arm_connection_> arm_conn_;

        int consecutive_missed_packets{0};
    };

    struct state_controlled_ : public state_event_handler_base_<state_controlled_>, public state_connected_ {
        explicit state_controlled_(std::unique_ptr<arm_connection_> arm_conn);

        static std::string_view name();
        std::string describe() const;
        using state_connected_::get_timeout;
        void clear_pstop() const;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_& state);
        using state_connected_::send_noop;

        std::optional<state_variant_> handle_event(event_connection_lost_ event);
        std::optional<state_variant_> handle_event(event_stop_detected_);

        using state_event_handler_base_<state_controlled_>::handle_event;
    };

    struct state_independent_ : public state_event_handler_base_<state_independent_>, public state_connected_ {
        enum class reason : std::uint8_t { k_stopped, k_local_mode, k_both };

        explicit state_independent_(std::unique_ptr<arm_connection_> arm_conn, reason r);

        static std::string_view name();
        std::string describe() const;
        using state_connected_::get_timeout;
        void clear_pstop() const;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_& state);
        std::optional<event_variant_> send_noop();

        bool stopped() const;
        bool local_mode() const;

        std::optional<state_variant_> handle_event(event_connection_lost_ event);
        std::optional<state_variant_> handle_event(event_stop_detected_);
        std::optional<state_variant_> handle_event(event_local_mode_detected_);
        std::optional<state_variant_> handle_event(event_stop_cleared_);
        std::optional<state_variant_> handle_event(event_remote_mode_detected_);

        using state_event_handler_base_<state_independent_>::handle_event;

        reason reason_;

        // track how often we attempt to reconnect.
        // We will use this to limit how often logs spam during expected behaviors.
        int local_reconnect_attempts{-1};

        // When a stop event interrupts a move, the trajectory logger is
        // moved out of the move_request and kept here so it continues to
        // record realtime samples (capturing the robot's deceleration
        // behavior during the stop) for a cooldown period.
        static constexpr auto k_post_stop_recording_duration = std::chrono::seconds(1);
        std::unique_ptr<RealtimeTrajectoryLogger> post_stop_logger_;
        std::optional<std::chrono::steady_clock::time_point> post_stop_recording_deadline_;
    };

    struct event_connection_established_ {
        static std::string_view name();
        std::string_view describe() const;
        std::unique_ptr<arm_connection_> payload;
    };

    class event_connection_lost_ {
       public:
        static event_connection_lost_ data_communication_failure();
        static event_connection_lost_ dashboard_communication_failure();
        static event_connection_lost_ dashboard_command_failure();
        static event_connection_lost_ dashboard_control_mode_change();
        static event_connection_lost_ robot_program_failure();
        static event_connection_lost_ trajectory_control_failure();
        static event_connection_lost_ module_shutdown();

        static std::string_view name();
        std::string_view describe() const;

       private:
        enum reason : std::uint8_t {
            k_data_communication_failure,
            k_dashboard_communication_failure,
            k_dashboard_command_failure,
            k_dashboard_control_mode_change,
            k_robot_program_failure,
            k_trajectory_control_failure,
            k_module_shutdown
        };

        explicit event_connection_lost_(reason r);

        reason reason_code;
    };

    struct event_stop_detected_ {
        static std::string_view name();
        std::string_view describe() const;
    };

    struct event_stop_cleared_ {
        static std::string_view name();
        std::string_view describe() const;
    };

    struct event_local_mode_detected_ {
        static std::string_view name();
        std::string_view describe() const;
    };

    struct event_remote_mode_detected_ {
        static std::string_view name();
        std::string_view describe() const;
    };

    // Producer/worker-shared state describing the lifecycle of an
    // open-ended trajectory stream. Default construction yields the
    // freshly-opened state (k_open, no pending samples, zero points
    // written); every other configuration is reached by mutators
    // running under `state_::mutex_`. The phase enum is a forward-only
    // state machine; see `state_controlled_::handle_move_request` for
    // the transition rules and the corresponding URCL traffic.
    struct sample_stream {
        enum class phase {
            // STREAM_START not sent; producer can still extend or close.
            k_open,
            // STREAM_START has been sent; producer can still extend or close.
            k_streaming,
            // Producer has closed; STREAM_START not yet sent. Covers the race
            // where the producer hands us a complete trajectory before the
            // worker has had a chance to tick (the unary path always hits this
            // state; bidi can hit it on a single-batch-then-close stream).
            k_buffered,
            // Producer has closed and STREAM_START has been sent. Worker is
            // draining `pending`; STREAM_END will be sent when it empties.
            k_draining,
            // STREAM_END has been sent. Awaiting trajectory_done_callback_.
            k_ended,
        };

        phase current_phase = phase::k_open;

        // PV-vs-PVA is decided by the producer at the first extend and locked:
        // the engaged variant arm IS the decision. `nullopt` is the
        // pre-decision state; engaged-but-inner-empty is the post-drain
        // (mid-stream) state.
        std::optional<trajectory_samples> pending;

        // Spline points written to the URCL trajectory socket since the most
        // recent STREAM_START. URCL requires the producer to report this total
        // back on STREAM_END so URScript can drain the remaining unread tail
        // out of its socket buffer (see resources/external_control.urscript in
        // the local URCL fork). Reset to zero whenever STREAM_START is sent.
        std::size_t points_written = 0;
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
        using move_command_data = std::variant<sample_stream, std::optional<pose_sample>>;

        using async_cancellation_monitor = std::function<bool()>;

        explicit move_request(boost::uuids::uuid id,
                              std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                              async_cancellation_monitor monitor,
                              move_command_data&& move_command);

        // Construct a streaming move_request in the freshly-opened state:
        // a default-constructed `sample_stream`, no batches yet. The producer
        // subsequently drives the request via `state_::extend_move_request` /
        // `close_move_request` / `cancel_move_request(uuid)`.
        explicit move_request(boost::uuids::uuid id,
                              std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                              async_cancellation_monitor monitor);

        explicit move_request(boost::uuids::uuid id,
                              std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger,
                              async_cancellation_monitor monitor,
                              pose_sample ps);

        std::shared_future<void> cancel();

        void complete_success();
        void complete_cancelled();
        void complete_failure();
        void complete_error(std::string_view message);
        void cancel_error(std::string_view message);

        void write_realtime_sample(const ephemeral_data& data,
                                   std::optional<uint32_t> robot_status_bits,
                                   std::optional<uint32_t> safety_status_bits) const;

        boost::uuids::uuid id;
        std::unique_ptr<RealtimeTrajectoryLogger> trajectory_logger;
        async_cancellation_monitor async_cancel_monitor;
        move_command_data move_command;
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
    void program_running_callback_(bool running);

    static vector6d_t clamp_limits_(vector6d_t desired, const std::optional<vector6d_t>& limits);

    // TODO(RSDK-11620): Check if we still need this flag. We may
    // not, now that we examine status bits that include letting
    // us know whether the program is running.
    std::atomic<bool> program_running_flag{false};

    const UrArmModel configured_model_;
    const std::string resource_name_;
    const std::string host_;
    const std::filesystem::path resource_root_;
    const std::filesystem::path urcl_resource_root_;
    const std::filesystem::path telemetry_output_path_;
    const double robot_control_freq_hz_;

    // If this field ever becomes mutable, the accessors for it must
    // start taking the lock and returning a copy.
    const std::optional<double> reject_move_request_threshold_rad_;

    const struct ports_ ports_;

    const double waypoint_deduplication_tolerance_rad_;

    vector6d_t velocity_limits_{};
    vector6d_t acceleration_limits_{};

    const double path_tolerance_delta_rads_;
    const std::optional<double> path_colinearization_ratio_;
    const double segmentation_threshold_;

    const bool use_new_trajectory_planner_;
    const bool prefer_precomputed_accelerations_;
    const bool segment_for_trajex_;
    const double max_trajectory_duration_secs_;
    const std::optional<vector6d_t> max_velocity_limits_;
    const std::optional<vector6d_t> max_acceleration_limits_;
    const double trajectory_sampling_freq_hz_;
    const std::string telemetry_output_path_append_traceid_template_;

    mutable std::mutex mutex_;
    state_variant_ current_state_{state_disconnected_{}};
    std::thread worker_thread_;
    std::condition_variable worker_wakeup_cv_;
    bool shutdown_requested_{false};

    std::atomic<std::size_t> move_epoch_{0};
    std::optional<move_request> move_request_;

    std::optional<ephemeral_data> ephemeral_;

    // Cache slot for `get_calibrated_kinematics_info()` /
    // `get_dh_kinematics_json()`, protected by `mutex_`. The future is
    // initialized at construction (so `kinematics_future_.valid()` is an
    // invariant) and replaced only at the first tick of a new connection
    // period whose worker observes that the previous period's promise has
    // already been fulfilled (see `state_connected_::recv_arm_data`).
    //
    // The promise is engaged while a fetch is outstanding and reset (via
    // `.reset()`) after `set_value` or `set_exception`. An empty promise
    // therefore means "the current future is already fulfilled and the next
    // connection period should install a fresh promise/future pair before
    // attempting to populate it."
    std::optional<std::promise<cached_kinematics_payload>> kinematics_promise_;
    std::shared_future<cached_kinematics_payload> kinematics_future_;
};

template <typename... Args>
std::future<void> URArm::state_::start_move_request(URArm::move_id id, Args&&... args) {
    // CAS both validates the generation snapshot from `allocate_move_id` and claims the
    // slot for this caller. If another move came through (successful or failed) between
    // the snapshot and this call, the generation has advanced; the caller's plan started
    // from arm state that may no longer be current, and we reject. On success the slot
    // is ours regardless of whether the previous move had actually been emplaced — failed
    // emplaces (slot-occupied) still bump the generation, which is acceptable.
    auto expected_generation = id.generation;
    if (!move_epoch_.compare_exchange_strong(expected_generation, expected_generation + 1, std::memory_order_acq_rel)) {
        throw std::runtime_error("move operation was superseded by a newer operation");
    }

    const std::lock_guard lock{mutex_};
    if (move_request_) {
        throw std::runtime_error("an actuation is already in progress");
    }
    auto future = move_request_.emplace(std::move(id).uuid, std::forward<Args>(args)...).completion.get_future();
    // Wake the worker so it picks up this request on its next iteration
    // rather than waiting out the remainder of its current tick interval.
    worker_wakeup_cv_.notify_one();
    return future;
}
