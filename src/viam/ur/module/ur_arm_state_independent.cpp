#include "ur_arm_state.hpp"

#include <algorithm>

#include <boost/io/ostream_joiner.hpp>
#include <boost/range/adaptor/filtered.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/empty.hpp>
#include <boost/range/irange.hpp>

// NOLINTBEGIN(readability-convert-member-functions-to-static)

URArm::state_::state_independent_::state_independent_(std::unique_ptr<arm_connection_> arm_conn, reason r)
    : state_connected_(std::move(arm_conn)), reason_(r) {}

std::string_view URArm::state_::state_independent_::name() {
    using namespace std::literals::string_view_literals;
    return "independent"sv;
}

std::string URArm::state_::state_independent_::describe() const {
    std::ostringstream buffer;
    buffer << name() << "(";

    const auto describe_stoppage = [this](std::ostream& stream) -> std::ostream& {
        // TODO: We should ask URCL to provide more of this for us.
        constexpr std::string_view k_safety_status_field_names[] =  //
            {"NORMAL_MODE",
             "REDUCED_MODE",
             "PROTECTIVE_STOPPED",
             "RECOVERY_MODE",
             "SAFEGUARD_STOPPED",
             "SYSTEM_EMERGENCY_STOPPED",
             "ROBOT_EMERGENCY_STOPPED",
             "EMERGENCY_STOPPED",
             "VIOLATION",
             "FAULT",
             "STOPPED_DUE_TO_SAFETY",
             "3PE_INPUT_ACTIVE"};

        static_assert(arm_connection_::k_num_safety_status_bits == std::size(k_safety_status_field_names));

        stream << "stop{";

        if (!arm_conn_->safety_status_bits.has_value()) {
            stream << "<safety-status-unavailable>";
        } else {
            const auto& bits = arm_conn_->safety_status_bits.value();
            const auto set_bit_names = boost::irange(size_t{0}, std::size(k_safety_status_field_names)) |
                                       boost::adaptors::filtered([&](size_t i) { return bits[i]; }) |
                                       boost::adaptors::transformed([&](size_t i) { return k_safety_status_field_names[i]; });

            if (!boost::empty(set_bit_names)) {
                std::copy(set_bit_names.begin(), set_bit_names.end(), boost::io::make_ostream_joiner(stream, ","));
            }
        }

        return stream << "}";
    };

    switch (reason_) {
        case reason::k_stopped: {
            describe_stoppage(buffer) << ")";
            break;
        }
        case reason::k_local_mode: {
            buffer << "local)";
            break;
        }
        case reason::k_both: {
            describe_stoppage(buffer) << "|local)";
            break;
        }
        default: {
            buffer << "unknown)";
            break;
        }
    }
    return buffer.str();
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::upgrade_downgrade(state_& state) {
    namespace urtde = urcl::rtde_interface;

    if (!arm_conn_->safety_status_bits || !arm_conn_->robot_status_bits) {
        VIAM_RESOURCE_LOG(state.log_resource_, debug)
            << "While in state " << describe() << ", robot and safety status bits were not available; dropping connection";
        return event_connection_lost_::data_communication_failure();
    }

    // If we aren't stopped, but the safety flags say we are, become stopped immediately.
    if (!stopped() && !arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        return event_stop_detected_{};
    }

    // If we aren't stopped, but the robot is not powered on, become stopped.
    if (!stopped() && !arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON))) {
        return event_stop_detected_{};
    }

    // If we aren't stopped, but the robot program is not running, become stopped.
    if (!stopped() && !arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
        return event_stop_detected_{};
    }

    // If we aren't in local mode, but the arm says we are, enter local mode.
    if (!local_mode()) {
        try {
            if (!arm_conn_->dashboard->commandIsInRemoteControl()) {
                return event_local_mode_detected_{};
            }
        } catch (...) {
            VIAM_RESOURCE_LOG(state.log_resource_, debug)
                << "While in state " << describe()
                << ", could not communicate with dashboard to determine remote control state; dropping connection";
            return event_connection_lost_::dashboard_communication_failure();
        }
    }

    if (local_mode()) {
        local_reconnect_attempts++;
        try {
            if (!arm_conn_->dashboard->commandIsInRemoteControl()) {
                // only log the failure every 100 attempts to be less spammy
                if (local_reconnect_attempts % 100000 == 0) {
                    VIAM_RESOURCE_LOG(state.log_resource_, warn)
                        << "While in state " << describe() << ", waiting for arm to re-enter remote mode";
                }
                return std::nullopt;
            }
        } catch (...) {
            VIAM_RESOURCE_LOG(state.log_resource_, warn)
                << "While in state " << describe()
                << ", could not communicate with dashboard to determine remote control state; dropping connection";
            return event_connection_lost_::dashboard_communication_failure();
        }

        // reset the clients to clear the state that blocks commands.
        return event_remote_mode_detected_{};
    }

    // On the other hand, if we are stopped, and that condition has been resolved, clear it.
    if (stopped()) {
        if (!arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
            // A new stop during recovery invalidates it; recovery restarts from power-on once the stop clears.
            program_start_deadline_.reset();
            pending_mode_transition_.reset();
            return std::nullopt;
        }

        if (program_start_deadline_) {
            if (program_ready()) {
                program_start_deadline_.reset();
                return event_stop_cleared_{};
            }
            if (std::chrono::steady_clock::now() < *program_start_deadline_) {
                return std::nullopt;
            }
            VIAM_RESOURCE_LOG(state.log_resource_, warn)
                << "While in state " << describe() << ", robot program did not start within "
                << std::chrono::duration_cast<std::chrono::milliseconds>(k_program_start_timeout).count() << "ms; dropping connection";
            return event_connection_lost_::robot_program_failure();
        }

        if (arm_conn_->is_polyscope_x) {
            // Drive the robot to RUNNING one request at a time, based on the
            // robot mode reported over RTDE, before touching the program.
            static const std::string k_robot_mode_key = "robot_mode";
            std::int32_t raw_robot_mode{};
            if (!arm_conn_->data_package->getData<std::int32_t>(k_robot_mode_key, raw_robot_mode)) {
                VIAM_RESOURCE_LOG(state.log_resource_, error)
                    << "While in state " << describe()
                    << ", the data package did not contain the expected `robot_mode` information; dropping connection";
                return event_connection_lost_::data_communication_failure();
            }
            const auto robot_mode = static_cast<urcl::RobotMode>(raw_robot_mode);

            if (pending_mode_transition_ && pending_mode_transition_->from != robot_mode) {
                VIAM_RESOURCE_LOG(state.log_resource_, debug)
                    << "While in state " << describe() << ", arm left " << urcl::robotModeString(pending_mode_transition_->from) << " for "
                    << urcl::robotModeString(robot_mode);
                pending_mode_transition_.reset();
            }

            switch (robot_mode) {
                case urcl::RobotMode::POWER_OFF: {
                    return request_mode_transition_(
                        state, robot_mode, "power on arm", [this] { return arm_conn_->dashboard->commandPowerOn(); });
                }
                case urcl::RobotMode::IDLE: {
                    return request_mode_transition_(
                        state, robot_mode, "release brakes", [this] { return arm_conn_->dashboard->commandBrakeRelease(); });
                }
                case urcl::RobotMode::RUNNING: {
                    break;
                }
                default: {
                    // Booting, powering on, or otherwise mid-transition.
                    return std::nullopt;
                }
            }
        } else {
            // ensure the arm is powered on
            if (!arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON))) {
                VIAM_RESOURCE_LOG(state.log_resource_, debug)
                    << "While in state " << describe()
                    << ", arm is not powered on; attempting to power on arm - ensure the dashboard is in remote mode";
                if (auto failure =
                        run_dashboard_command_(state, "power on arm", [this] { return arm_conn_->dashboard->commandPowerOn(); })) {
                    return failure;
                }
            }

            VIAM_RESOURCE_LOG(state.log_resource_, debug)
                << "While in state " << describe() << ": releasing brakes since no longer stopped - ensure the dashboard is in remote mode";
            if (auto failure =
                    run_dashboard_command_(state, "release brakes", [this] { return arm_conn_->dashboard->commandBrakeRelease(); })) {
                return failure;
            }
        }

        // resend the robot program if the control script is not running on the arm.
        // This has been found to occur on some stops and when
        // controlling the arm directly while in local mode.
        if (!arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
            VIAM_RESOURCE_LOG(state.log_resource_, debug)
                << "While in state " << describe() << ", program is not running on arm; attempting to resend program";
            try {
                if (!arm_conn_->driver->sendRobotProgram()) {
                    VIAM_RESOURCE_LOG(state.log_resource_, warn)
                        << "While in state " << describe() << ", could not send program to robot via driver; dropping connection";
                    return event_connection_lost_::robot_program_failure();
                }
            } catch (...) {
                VIAM_RESOURCE_LOG(state.log_resource_, warn)
                    << "While in state " << describe() << ", failed sending program to robot via driver; dropping connection";
                return event_connection_lost_::robot_program_failure();
            }
        }

        // Don't wait here: the worker holds `mutex_` for the whole tick, and the status bits
        // only refresh in `recv_arm_data`. Later ticks check `program_ready` against fresh data.
        VIAM_RESOURCE_LOG(state.log_resource_, debug)
            << "While in state " << describe() << ", waiting for robot program to run and connect to the reverse interface";
        program_start_deadline_ = std::chrono::steady_clock::now() + k_program_start_timeout;
    }

    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::run_dashboard_command_(
    state_& state, std::string_view action, const std::function<bool()>& command) const {
    try {
        if (!command()) {
            VIAM_RESOURCE_LOG(state.log_resource_, debug)
                << "While in state " << describe() << ", unable to " << action << "; dropping connection";
            return event_connection_lost_::dashboard_command_failure();
        }
    } catch (...) {
        VIAM_RESOURCE_LOG(state.log_resource_, debug)
            << "While in state " << describe() << ", could not communicate with dashboard to " << action << "; dropping connection";
        return event_connection_lost_::dashboard_communication_failure();
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::request_mode_transition_(
    state_& state, urcl::RobotMode from, std::string_view action, const std::function<bool()>& command) {
    const auto from_name = urcl::robotModeString(from);

    if (pending_mode_transition_) {
        if (std::chrono::steady_clock::now() < pending_mode_transition_->deadline) {
            return std::nullopt;
        }
        VIAM_RESOURCE_LOG(state.log_resource_, warn) << "While in state " << describe() << ", arm is still " << from_name << " "
                                                     << std::chrono::duration_cast<std::chrono::seconds>(k_mode_transition_timeout).count()
                                                     << "s after the request to " << action << "; dropping connection";
        return event_connection_lost_::dashboard_command_failure();
    }

    VIAM_RESOURCE_LOG(state.log_resource_, debug)
        << "While in state " << describe() << ", arm is " << from_name << "; requesting to " << action;
    auto failure = run_dashboard_command_(state, action, [&] {
        if (command()) {
            return true;
        }
        // The RTDE sample this tick acted on can predate the robot leaving
        // `from`, in which case the controller rightly rejects the request.
        // Only treat the rejection as a failure if the robot is still in `from`.
        std::string current_mode;
        if (arm_conn_->dashboard->commandRobotMode(current_mode) && current_mode != from_name) {
            VIAM_RESOURCE_LOG(state.log_resource_, debug)
                << "While in state " << describe() << ", request to " << action << " was rejected because the arm already left "
                << from_name << " for " << current_mode;
            return true;
        }
        return false;
    });
    if (failure) {
        return failure;
    }

    pending_mode_transition_ = mode_transition_request{from, std::chrono::steady_clock::now() + k_mode_transition_timeout};
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::handle_move_request(state_& state) {
    // If there is a pending move request, move the logger out, complete
    // the move with an error, and start the post-stop recording cooldown.
    if (state.move_request_) {
        auto logger = std::move(state.move_request_->trajectory_logger);

        std::exchange(state.move_request_, {})->complete_error([this]() -> std::string {
            switch (reason_) {
                case reason::k_stopped: {
                    return "arm is stopped";
                }
                case reason::k_local_mode: {
                    return "arm is in local mode";
                }
                case reason::k_both: {
                    return "arm is in local mode and stopped";
                }
                default: {
                    return "arm is in an unknown and uncontrolled state";
                }
            }
        }());

        if (logger) {
            post_stop_logger_ = std::move(logger);
            post_stop_recording_deadline_ = std::chrono::steady_clock::now() + k_post_stop_recording_duration;
            VIAM_RESOURCE_LOG(state.log_resource_, debug)
                << "post-stop telemetry recording started; will record for "
                << std::chrono::duration_cast<std::chrono::milliseconds>(k_post_stop_recording_duration).count() << "ms";
        }
    }

    // Continue recording post-stop samples until the deadline expires.
    if (post_stop_logger_) {
        if (std::chrono::steady_clock::now() < *post_stop_recording_deadline_) {
            if (state.ephemeral_) {
                const auto now_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                post_stop_logger_->append_realtime_sample(
                    now_us,
                    *state.ephemeral_,
                    arm_conn_->robot_status_bits ? std::optional<uint32_t>(static_cast<uint32_t>(arm_conn_->robot_status_bits->to_ulong()))
                                                 : std::nullopt,
                    arm_conn_->safety_status_bits
                        ? std::optional<uint32_t>(static_cast<uint32_t>(arm_conn_->safety_status_bits->to_ulong()))
                        : std::nullopt);
            }
        } else {
            VIAM_RESOURCE_LOG(state.log_resource_, debug) << "post-stop telemetry recording complete";
            post_stop_logger_.reset();
            post_stop_recording_deadline_.reset();
        }
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

    // Similarly, if we are stopped, don't send the noop unless our
    // program can receive it.
    if (stopped() && !program_ready()) {
        return std::nullopt;
    }

    return state_connected_::send_noop();
}

bool URArm::state_::state_independent_::stopped() const {
    return reason_ != reason::k_local_mode;
}

// The RTDE bit alone is not sufficient: it can report a running program before our
// external_control script has connected back to the driver, and NOOPs or trajectories sent in
// that window fail.
bool URArm::state_::state_independent_::program_ready() const {
    namespace urtde = urcl::rtde_interface;
    return arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING)) &&
           arm_conn_->driver->isReverseInterfaceConnected();
}

bool URArm::state_::state_independent_::local_mode() const {
    return reason_ != reason::k_stopped;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_stop_cleared_) {
    if (reason_ == reason::k_local_mode) {
        return std::move(*this);
    } else if (reason_ == reason::k_both) {
        return state_independent_{std::move(arm_conn_), reason::k_local_mode};
    } else {
        return state_controlled_{std::move(arm_conn_)};
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_remote_mode_detected_) {
    if (reason_ == reason::k_stopped) {
        return std::move(*this);
    } else if (reason_ == reason::k_both) {
        return state_independent_{std::move(arm_conn_), reason::k_stopped};
    } else {
        return state_controlled_{std::move(arm_conn_)};
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_stop_detected_) {
    if (reason_ == reason::k_local_mode) {
        return state_independent_{std::move(arm_conn_), reason::k_both};
    } else {
        return std::move(*this);
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_local_mode_detected_) {
    if (reason_ == reason::k_stopped) {
        return state_independent_{std::move(arm_conn_), reason::k_both};
    } else {
        return std::move(*this);
    }
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_connection_lost_ event) {
    return state_disconnected_{std::move(event)};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_independent_::handle_event(event_enter_freedrive_) {
    // Matches state_disconnected_'s throw-only handler: no `this` access, so it
    // stays non-const (consistent with the other handlers and the catch-all
    // dispatch) without tripping readability-make-member-function-const.
    throw std::runtime_error("cannot enter manual mode: arm is not in a controllable state (protective stop or local mode)");
}

void URArm::state_::state_independent_::clear_pstop() const {
    namespace urtde = urcl::rtde_interface;

    // make sure the do_command is being used correctly. We should only be clearing the protective stop if the protective stop is set
    if (!arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_PROTECTIVE_STOPPED))) {
        throw std::runtime_error("cannot clear the protective stop, arm is not currently pstopped");
    }
    // use a try catch, in case the arm was disconnected but the state machine has not detected it yet.
    try {
        if (arm_conn_->dashboard->commandUnlockProtectiveStop()) {
            return;
        }
    } catch (const std::exception& ex) {
        std::ostringstream buffer;
        buffer << "cannot clear the protective stop due to: " << ex.what();
        throw std::runtime_error(buffer.str());
    } catch (...) {
        // throw an error but do not try to change the state, let the worker thread handle that.
        throw std::runtime_error("cannot clear the protective stop due to an unknown exception");
    }
    // if we made it this far, we failed to clear the pstop. throw an error
    throw std::runtime_error("failed to clear the protective stop");
}

void URArm::state_::state_independent_::zero_ftsensor() const {
    throw std::runtime_error(
        "cannot zero the force-torque sensor, arm is not in a controllable state "
        "(it is stopped or in local mode; the external control script must be running)");
}

// NOLINTEND(readability-convert-member-functions-to-static)
