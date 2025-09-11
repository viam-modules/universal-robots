#include "ur_arm.hpp"

#include <bitset>
#include <condition_variable>
#include <future>
#include <thread>
#include <variant>

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>

class URArm::state_ {
    struct private_ {};

   public:
    explicit state_(private_,
                    std::string configured_model_type,
                    std::string host,
                    std::string app_dir,
                    std::string csv_output_path,
                    std::optional<double> reject_move_request_threshold_rad,
                    std::optional<double> robot_control_freq_hz,
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
    struct event_stop_detected_;
    struct event_stop_cleared_;
    struct event_local_mode_detected_;
    struct event_remote_mode_restored_;

    using event_variant_ = std::variant<event_connection_established_,
                                        event_connection_lost_,
                                        event_stop_detected_,
                                        event_stop_cleared_,
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

        // track how often we attempt to reconnect.
        // We will use this to limit how often logs spam during expected behaviors.
        int reconnect_attempts{-1};
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

        // TODO(RSDK-11620): Check if we still need this flag. We may
        // not, now that we examine status bits that include letting
        // us know whether the program is running.
        std::atomic<bool> program_running_flag{false};
        bool log_destructor{false};
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

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_& state);
        using state_connected_::send_noop;

        std::optional<state_variant_> handle_event(event_connection_lost_);
        std::optional<state_variant_> handle_event(event_stop_detected_);
        std::optional<state_variant_> handle_event(event_local_mode_detected_);

        using state_event_handler_base_<state_controlled_>::handle_event;
    };

    struct state_independent_ : public state_event_handler_base_<state_independent_>, public state_connected_ {
        enum class reason : std::uint8_t { k_stopped, k_local_mode, k_both };

        explicit state_independent_(std::unique_ptr<arm_connection_> arm_conn, reason r);

        static std::string_view name();
        std::string describe() const;
        using state_connected_::get_timeout;

        using state_connected_::recv_arm_data;
        std::optional<event_variant_> upgrade_downgrade(state_&);
        std::optional<event_variant_> handle_move_request(state_& state);
        std::optional<event_variant_> send_noop();

        bool stopped() const;
        bool local_mode() const;

        std::optional<state_variant_> handle_event(event_connection_lost_);
        std::optional<state_variant_> handle_event(event_stop_detected_);
        std::optional<state_variant_> handle_event(event_local_mode_detected_);
        std::optional<state_variant_> handle_event(event_stop_cleared_);
        std::optional<state_variant_> handle_event(event_remote_mode_restored_);

        using state_event_handler_base_<state_independent_>::handle_event;

        reason reason_;

        // track how often we attempt to reconnect.
        // We will use this to limit how often logs spam during expected behaviors.
        int local_reconnect_attempts{-1};
    };

    struct event_connection_established_ {
        static std::string_view name();
        std::string_view describe() const;
        std::unique_ptr<arm_connection_> payload;
    };

    struct event_connection_lost_ {
        static std::string_view name();
        std::string_view describe() const;
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

    struct event_remote_mode_restored_ {
        static std::string_view name();
        std::string_view describe() const;
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
    const double robot_control_freq_hz_;

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
