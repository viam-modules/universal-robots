#include "ur_arm_state.hpp"

// NOLINTBEGIN(readability-convert-member-functions-to-static)

URArm::state_::state_controlled_::state_controlled_(std::unique_ptr<arm_connection_> arm_conn) : state_connected_(std::move(arm_conn)) {}

std::string_view URArm::state_::state_controlled_::name() {
    using namespace std::literals::string_view_literals;
    return "controlled"sv;
}

std::string URArm::state_::state_controlled_::describe() const {
    return std::string{name()};
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_controlled_::upgrade_downgrade(state_&) {
    namespace urtde = urcl::rtde_interface;

    if (!arm_conn_->safety_status_bits || !arm_conn_->robot_status_bits) {
        VIAM_SDK_LOG(warn) << "While in state " << describe() << ", robot and safety status bits were not available; dropping connection";
        return event_connection_lost_::data_communication_failure();
    }

    if (!arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        return event_stop_detected_{};
    }

    constexpr auto power_on_bit = 1ULL << static_cast<int>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON);
    constexpr auto program_running_bit = 1ULL << static_cast<int>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING);
    constexpr std::bitset<4> k_power_on_and_running{power_on_bit | program_running_bit};

    if (*(arm_conn_->robot_status_bits) != k_power_on_and_running) {
        return event_stop_detected_{};
    }

    if (arm_conn_->dashboard->getState() != urcl::comm::SocketState::Connected) {
        VIAM_SDK_LOG(warn) << "While in state " << describe() << ", dashboard client is disconnected; dropping connection";
        return event_connection_lost_::dashboard_communication_failure();
    }

    // If we get anything but a positive answer from the dashboard
    // that we are in remote control, assume that we need to
    // completely re-create our connection, since sockets aren't
    // reliable across local/remote mode transitions.
    try {
        if (!arm_conn_->dashboard->commandIsInRemoteControl()) {
            VIAM_SDK_LOG(warn) << "While in state " << describe()
                               << ", detected that dashboard is no longer in remote mode; dropping connection";
            return event_connection_lost_::dashboard_control_mode_change();
        }
    } catch (...) {
        VIAM_SDK_LOG(warn) << "While in state " << describe()
                           << ", could not communicate with dashboard to determine remote control state; dropping connection";
        return event_connection_lost_::dashboard_communication_failure();
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

        // if joint
        if (samples[0].is_joint_space) {
            VIAM_SDK_LOG(info) << "URArm::send_trajectory sending TRAJECTORY_START for " << num_samples << " samples";
            if (!arm_conn_->driver->writeTrajectoryControlMessage(
                    urcl::control::TrajectoryControlMessage::TRAJECTORY_START, static_cast<int>(num_samples), RobotReceiveTimeout::off())) {
                VIAM_SDK_LOG(error) << "send_trajectory driver->writeTrajectoryControlMessage returned false; dropping connection";
                std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start message to arm");
                return event_connection_lost_::trajectory_control_failure();
            }

            VIAM_SDK_LOG(info) << "URArm::send_trajectory sending " << num_samples << " cubic writeTrajectorySplinePoint";
            for (size_t i = 0; i < num_samples; i++) {
                if (!arm_conn_->driver->writeTrajectorySplinePoint(samples[i].p, samples[i].v, samples[i].timestep)) {
                    VIAM_SDK_LOG(error) << "send_trajectory cubic driver->writeTrajectorySplinePoint returned false; dropping connection";
                    std::exchange(state.move_request_, {})->complete_error("failed to send trajectory spline point to arm");
                    return event_connection_lost_::trajectory_control_failure();
                }
            }
        }

        if (!samples[0].is_joint_space) {
            VIAM_SDK_LOG(info) << "WE ARE IN NOT IN JOINT SPACE";
            float duration = 5.0;
            float velocity = 2.0;
            float acceleration = 2.5;
            float blend_radius = 0;

            // Start trajectory (4 was hard-coded; use n unless you truly want to send 4)
            if (!arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                                                  static_cast<int>(num_samples))) {
                VIAM_SDK_LOG(error) << "writeTrajectoryControlMessage(START) failed";
                std::exchange(state.move_request_, {})->complete_error("failed to start trajectory");
                return event_connection_lost_::trajectory_control_failure();
            }

            // First pass: duration + blend
            for (size_t i = 0; i < num_samples; ++i) {
                // interpret as Cartesian pose (movel)
                const bool cartesian = true;
                if (!arm_conn_->driver->writeTrajectoryPoint(samples[i].p, cartesian, duration, blend_radius)) {
                    VIAM_SDK_LOG(error) << "writeTrajectoryPoint (duration form) failed at i=" << i;
                    std::exchange(state.move_request_, {})->complete_error("failed to send trajectory point (duration)");
                    return event_connection_lost_::trajectory_control_failure();
                }
            }

            // Second pass: param by accel/vel + blend (duration 0)
            duration = 0.0;
            for (size_t i = 0; i < num_samples; ++i) {
                const bool cartesian = true;
                if (!arm_conn_->driver->writeTrajectoryPoint(samples[i].p, acceleration, velocity, cartesian, duration, blend_radius)) {
                    VIAM_SDK_LOG(error) << "writeTrajectoryPoint (acc/vel form) failed at i=" << i;
                    std::exchange(state.move_request_, {})->complete_error("failed to send trajectory point (acc/vel)");
                    return event_connection_lost_::trajectory_control_failure();
                }
            }

            // Replace the infinite loop with a bounded NOOP poll or remove entirely if not required
            for (int k = 0; k < 10; ++k) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // Best-effort NOOP; ignore return if not critical
                arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
            }
        }

        VIAM_SDK_LOG(info) << "URArm trajectory sent";

    } else if (state.move_request_->samples.empty() && state.move_request_->cancellation_request &&
               !state.move_request_->cancellation_request->issued) {
        VIAM_SDK_LOG(info) << "else case 1";
        // We have a move request, the samples have been forwarded,
        // and cancellation is requested but has not yet been issued. Issue a cancel.
        state.move_request_->cancellation_request->issued = true;
        if (!arm_conn_->driver->writeTrajectoryControlMessage(
                urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off())) {
            state.move_request_->cancel_error("failed to write trajectory control cancel message to URArm");
            VIAM_SDK_LOG(error) << "While in state " << describe()
                                << ", failed to write trajectory control cancel message; dropping connection";
            return event_connection_lost_::trajectory_control_failure();
        }
    } else if (!state.move_request_->samples.empty() && state.move_request_->cancellation_request) {
        VIAM_SDK_LOG(info) << "else case 2";
        // We have a move request that we haven't issued but a
        // cancel is already pending. Don't issue it, just cancel it.
        std::exchange(state.move_request_, {})->complete_cancelled();
    } else {
        VIAM_SDK_LOG(info) << "else case 3";
        // TODO: is it assured that we have positions/velocities here?
        state.move_request_->write_joint_data(state.ephemeral_->joint_positions, state.ephemeral_->joint_velocities);
    }

    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_connection_lost_ event) {
    return state_disconnected_{std::move(event)};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_stop_detected_) {
    return state_independent_{std::move(arm_conn_), state_independent_::reason::k_stopped};
}
void URArm::state_::state_controlled_::clear_pstop() const {
    throw std::runtime_error("cannot clear the protective stop, arm is not currently pstopped");
}

// NOLINTEND(readability-convert-member-functions-to-static)
