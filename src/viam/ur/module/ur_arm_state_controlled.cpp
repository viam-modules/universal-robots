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
        VIAM_SDK_LOG(warn) << "While in controlled state, robot and safety status bits were not available; disconnecting";
        return event_connection_lost_{};
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
                return event_stop_detected_{};
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
            return event_stop_detected_{};
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

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_stop_detected_) {
    return state_independent_{std::move(arm_conn_), state_independent_::reason::k_stopped};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_controlled_::handle_event(event_local_mode_detected_) {
    return state_independent_{std::move(arm_conn_), state_independent_::reason::k_local_mode};
}

// NOLINTEND(readability-convert-member-functions-to-static)
