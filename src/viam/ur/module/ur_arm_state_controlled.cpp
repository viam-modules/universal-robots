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

    auto& mr = *state.move_request_;

    return std::visit(
        [this, &state, &mr](auto& cmd) -> std::optional<event_variant_> {
            using T = std::decay_t<decltype(cmd)>;

            if constexpr (std::is_same_v<T, std::vector<trajectory_sample_point>>) {
                // Operating in joint-space

                if (!cmd.empty() && !mr.cancellation_request) {
                    // Have samples to send, no cancellation requested
                    VIAM_SDK_LOG(info) << "URArm sending trajectory";

                    auto to_send = std::move(cmd);
                    const auto num_samples = to_send.size();

                    if (!arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                                                          static_cast<int>(num_samples),
                                                                          RobotReceiveTimeout::off())) {
                        VIAM_SDK_LOG(error) << "send_trajectory: start failed; dropping connection";
                        std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start");
                        return event_connection_lost_::trajectory_control_failure();
                    }

                    for (size_t i = 0; i < num_samples; ++i) {
                        if (!arm_conn_->driver->writeTrajectorySplinePoint(to_send[i].p, to_send[i].v, to_send[i].timestep)) {
                            VIAM_SDK_LOG(error) << "send_trajectory: spline point failed; dropping connection";
                            std::exchange(state.move_request_, {})->complete_error("failed to send trajectory spline point");
                            return event_connection_lost_::trajectory_control_failure();
                        }
                    }

                    mr.move_command = std::vector<trajectory_sample_point>{};
                    return std::nullopt;

                } else if (cmd.empty() && mr.cancellation_request && !mr.cancellation_request->issued) {
                    // We have a move request, the samples have been forwarded,
                    // and cancellation is requested but has not yet been issued. Issue a cancel.
                    mr.cancellation_request->issued = true;
                    if (!arm_conn_->driver->writeTrajectoryControlMessage(
                            urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off())) {
                        mr.cancel_error("failed to send trajectory cancel");
                        VIAM_SDK_LOG(error) << "cancel failed; dropping connection";
                        return event_connection_lost_::trajectory_control_failure();
                    }
                    return std::nullopt;
                } else if (!cmd.empty() && mr.cancellation_request) {
                    // We have a move request that we haven't issued but a
                    // cancel is already pending. Don't issue it, just cancel it.
                    std::exchange(state.move_request_, {})->complete_cancelled();
                    return std::nullopt;

                } else {
                    // TODO: is it assured that we have positions/velocities here?
                    mr.write_joint_data(state.ephemeral_->joint_positions, state.ephemeral_->joint_velocities);
                    return std::nullopt;
                }

            } else if constexpr (std::is_same_v<T, std::optional<pose_sample>>) {
                // Operating in pose-space

                if (cmd.has_value() && !mr.cancellation_request) {
                    // Send the single pose command
                    if (!arm_conn_->driver->writeTrajectoryControlMessage(
                            urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1, RobotReceiveTimeout::off())) {
                        VIAM_SDK_LOG(error) << "single pose: start failed; dropping connection";
                        std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start for single pose");
                        return event_connection_lost_::trajectory_control_failure();
                    }

                    auto ps = std::move(*cmd);
                    VIAM_SDK_LOG(info) << "URArm sending single pose (tool-space)";

                    // TODO(RSDK-12294): determine how to set the velocity and acceleration so they are not hardcoded
                    const float velocity = 0.25F;
                    const float acceleration = 0.5F;
                    const float timestep = 0;
                    const float blend_radius = 0;
                    const bool cartesian = true;

                    if (!arm_conn_->driver->writeTrajectoryPoint(ps.p, acceleration, velocity, cartesian, timestep, blend_radius)) {
                        VIAM_SDK_LOG(error) << "single pose: writeTrajectoryPoint failed; dropping connection";
                        std::exchange(state.move_request_, {})->complete_error("failed to send single pose point");
                        return event_connection_lost_::trajectory_control_failure();
                    }

                    // Mark as sent by setting to nullopt
                    cmd = std::nullopt;
                    VIAM_SDK_LOG(info) << "URArm single pose sent";
                    return std::nullopt;

                } else if (!cmd.has_value()) {
                    // If already sent (nullopt), nothing to do
                    return std::nullopt;
                } else if (mr.cancellation_request) {
                    // Check for cancellation
                    std::exchange(state.move_request_, {})->complete_cancelled();
                    return std::nullopt;
                }

                // Unreachable: all cases should be handled above
                throw std::logic_error("handle_move_request: unexpected tool-space state");
            }
        },
        mr.move_command);
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
