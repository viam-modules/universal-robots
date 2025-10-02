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
             "STOPPED_DUE_TO_SAFETY"};

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

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::upgrade_downgrade(state_&) {
    namespace urtde = urcl::rtde_interface;

    if (!arm_conn_->safety_status_bits || !arm_conn_->robot_status_bits) {
        VIAM_SDK_LOG(warn) << "While in state " << describe() << ", robot and safety status bits were not available; dropping connection";
        return event_connection_lost_::data_communication_failure();
    }

    if (arm_conn_->dashboard->getState() != urcl::comm::SocketState::Connected) {
        VIAM_SDK_LOG(info) << "While in state " << describe() << ", dashboard client is disconnected; dropping connection";
        return event_connection_lost_::dashboard_communication_failure();
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
            VIAM_SDK_LOG(warn) << "While in state " << describe()
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
                    VIAM_SDK_LOG(warn) << "While in state " << describe() << ", waiting for arm to re-enter remote mode";
                }
                return std::nullopt;
            }
        } catch (...) {
            VIAM_SDK_LOG(warn) << "While in state " << describe()
                               << ", could not communicate with dashboard to determine remote control state; dropping connection";
            return event_connection_lost_::dashboard_communication_failure();
        }

        // reset the clients to clear the state that blocks commands.
        return event_remote_mode_detected_{};
    }

    // On the other hand, if we are stopped, and that condition has been resolved, clear it.
    //
    // TODO(RSDK-11621): Deal with three position enabling stops.
    if (stopped() && arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        // ensure the arm is powered on
        if (!arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON))) {
            VIAM_SDK_LOG(info) << "While in state " << describe()
                               << ", arm is not powered on; attempting to power on arm - ensure the dashboard is in remote mode";
            try {
                if (!arm_conn_->dashboard->commandPowerOn()) {
                    VIAM_SDK_LOG(warn) << "While in state " << describe() << ", unable to power on arm; dropping connection";

                    return event_connection_lost_::dashboard_command_failure();
                }
            } catch (...) {
                VIAM_SDK_LOG(warn) << "While in state " << describe()
                                   << ", could not communicate with dashboard to power on arm; dropping connection";
                return event_connection_lost_::dashboard_communication_failure();
            }
        }

        try {
            // TODO(RSDK-11645) find a way to detect if the brakes are locked
            VIAM_SDK_LOG(info) << "While in state " << describe() << ": releasing brakes since no longer stopped";
            if (!arm_conn_->dashboard->commandBrakeRelease()) {
                VIAM_SDK_LOG(warn) << "While in state " << describe()
                                   << ", could not release brakes - ensure the dashboard is in remote mode; dropping connection";
                return event_connection_lost_::dashboard_command_failure();
            }
        } catch (...) {
            VIAM_SDK_LOG(warn) << "While in state " << describe()
                               << ", could not communicate with dashboard to release brakes; dropping connection";
            return event_connection_lost_::dashboard_communication_failure();
        }

        // resend the robot program if the control script is not running on the arm.
        // This has been found to occur on some stops and when
        // controlling the arm directly while in local mode.
        if (!arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
            arm_conn_->program_running_flag.store(false, std::memory_order_release);
            VIAM_SDK_LOG(info) << "While in state " << describe() << ", program is not running on arm; attempting to resend program";
            try {
                if (!arm_conn_->driver->sendRobotProgram()) {
                    VIAM_SDK_LOG(warn) << "While in state " << describe()
                                       << ", could not send program to robot via driver; dropping connection";
                    return event_connection_lost_::robot_program_failure();
                }
            } catch (...) {
                VIAM_SDK_LOG(warn) << "While in state " << describe()
                                   << ", failed sending program to robot via driver; dropping connection";
                return event_connection_lost_::robot_program_failure();
            }
        }

        // "Wait" for the robot program to start running.
        //
        // TODO(RSDK-11620) Check if we still need this flag.
        VIAM_SDK_LOG(info) << "While in state " << describe() << ", waiting for callback to toggle program state to running";
        int retry_count = 100;
        while (!arm_conn_->program_running_flag.load(std::memory_order_acquire)) {
            if (retry_count <= 0) {
                VIAM_SDK_LOG(warn) << "While in state " << describe() << ", program state never loaded; dropping connection";
                return event_connection_lost_::robot_program_failure();
            }
            retry_count--;
            std::this_thread::sleep_for(get_timeout());
        }

        return event_stop_cleared_{};
    }

    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_independent_::handle_move_request(state_& state) {
    if (state.move_request_) {
        // if we switched to independent mode, store the remaining trajectory so it can be resumed
        auto trajectory_end = std::chrono::steady_clock::now();
        auto traj_duration =
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(trajectory_end - state.move_request_->trajectory_start)
                .count() /
            1000.0;
        const auto num_samples = state.move_request_->samples.size();
        double time_traj = 0.0;
        size_t traj_end_index = 0;
        for (size_t i = 0; i < num_samples; i++) {
            time_traj += boost::numeric_cast<double>(state.move_request_->samples[i].timestep);
            if (time_traj > traj_duration) {
                traj_end_index = i;
                break;
            }
        }
        state.pending_samples_from_failure.insert(state.pending_samples_from_failure.end(),
                                                  std::make_move_iterator(state.move_request_->samples.begin() + traj_end_index),
                                                  std::make_move_iterator(state.move_request_->samples.end()));
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

    // Similarly, if we are stopped, don't send the noop if the
    // program isn't running.
    namespace urtde = urcl::rtde_interface;
    if (stopped() && !arm_conn_->robot_status_bits->test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
        return std::nullopt;
    }

    return state_connected_::send_noop();
}

bool URArm::state_::state_independent_::stopped() const {
    return reason_ != reason::k_local_mode;
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

// NOLINTEND(readability-convert-member-functions-to-static)
