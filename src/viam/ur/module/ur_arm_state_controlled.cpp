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

    // If we don't already have a cancel request, but we do have an async cancellation event,
    // note the cancellation so we act on it below.
    if (!state.move_request_->cancellation_request && state.move_request_->async_cancel_monitor()) {
        state.move_request_->cancel();
    }

    return std::visit(
        [this, &state](auto& cmd) -> std::optional<event_variant_> {
            using T = std::decay_t<decltype(cmd)>;

            if constexpr (std::is_same_v<T, sample_stream>) {
                // Joint-space streaming. The phase drives what we send to URCL;
                // see `sample_stream` for what each phase means. The phase only
                // moves forward:
                //   from `k_open`, first points pending: send STREAM_START, go to `k_streaming`.
                //   from `k_open`, on close: go to `k_buffered`.
                //   from `k_streaming`, on close: go to `k_draining`.
                //   from `k_buffered`: send START, drain, and END in one tick, go to `k_ended`.
                //   from `k_draining`, once drained: send STREAM_END, go to `k_ended`.

                const auto emit_realtime_sample = [&] {
                    state.move_request_->write_realtime_sample(
                        *state.ephemeral_,
                        arm_conn_->robot_status_bits
                            ? std::optional<uint32_t>(static_cast<uint32_t>(arm_conn_->robot_status_bits->to_ulong()))
                            : std::nullopt,
                        arm_conn_->safety_status_bits
                            ? std::optional<uint32_t>(static_cast<uint32_t>(arm_conn_->safety_status_bits->to_ulong()))
                            : std::nullopt);
                };

                // Returns `nullopt` on success, or a connection-lost event on
                // URCL write failure. On failure the `move_request` is
                // completed with an error and the slot is cleared.
                const auto drain_pending = [&]() -> std::optional<event_variant_> {
                    if (!cmd.pending) {
                        return std::nullopt;
                    }
                    return std::visit(
                        [&](auto& pts) -> std::optional<event_variant_> {
                            const auto write_point = [&](const auto& pt) {
                                if constexpr (requires { pt.a; }) {
                                    return arm_conn_->driver->writeTrajectorySplinePoint(pt.p, pt.v, pt.a, pt.timestep);
                                } else {
                                    return arm_conn_->driver->writeTrajectorySplinePoint(pt.p, pt.v, pt.timestep);
                                }
                            };
                            for (const auto& pt : pts) {
                                if (!write_point(pt)) {
                                    VIAM_SDK_LOG(error) << "send_trajectory: spline point failed; dropping connection";
                                    std::exchange(state.move_request_, {})->complete_error("failed to send trajectory spline point");
                                    return event_connection_lost_::trajectory_control_failure();
                                }
                                ++cmd.points_written;
                            }
                            // Drop the points we just sent but keep the variant
                            // itself, so the PV or PVA choice holds for later
                            // extends.
                            pts.clear();
                            return std::nullopt;
                        },
                        *cmd.pending);
                };

                const bool cancel_pending = state.move_request_->cancellation_request.has_value();
                const bool cancel_issued = cancel_pending && state.move_request_->cancellation_request->issued;

                // Cancellation handling. Discriminate on phase so we know
                // whether to send a TRAJECTORY_CANCEL (URCL has been told
                // about this move) or to complete the request locally
                // (STREAM_START was never sent).
                if (cancel_pending && !cancel_issued &&
                    (cmd.current_phase == sample_stream::phase::k_open || cmd.current_phase == sample_stream::phase::k_buffered)) {
                    std::exchange(state.move_request_, {})->complete_cancelled();
                    return std::nullopt;
                }
                if (cancel_pending && !cancel_issued) {
                    state.move_request_->cancellation_request->issued = true;
                    if (!arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL,
                                                                          static_cast<int>(cmd.points_written),
                                                                          RobotReceiveTimeout::off())) {
                        state.move_request_->cancel_error("failed to send trajectory cancel");
                        VIAM_SDK_LOG(error) << "cancel failed; dropping connection";
                        return event_connection_lost_::trajectory_control_failure();
                    }
                    return std::nullopt;
                }
                if (cancel_pending) {
                    // Cancel already issued; just heartbeat realtime samples
                    // until URScript fires the `trajectory_done_callback`.
                    emit_realtime_sample();
                    return std::nullopt;
                }

                // No cancellation in flight. Drive the phase state machine.
                const bool pending_empty = !cmd.pending || std::visit([](const auto& v) { return v.empty(); }, *cmd.pending);

                switch (cmd.current_phase) {
                    case sample_stream::phase::k_open: {
                        if (pending_empty) {
                            emit_realtime_sample();
                            return std::nullopt;
                        }
                        // First non-empty drain; open the stream on URCL and
                        // continue into the drain in this same tick.
                        VIAM_SDK_LOG(debug) << "URArm sending stream start";
                        if (!arm_conn_->driver->writeTrajectoryControlMessage(
                                urcl::control::TrajectoryControlMessage::TRAJECTORY_STREAM_START, 0, RobotReceiveTimeout::off())) {
                            VIAM_SDK_LOG(error) << "send_trajectory: stream start failed; dropping connection";
                            std::exchange(state.move_request_, {})->complete_error("failed to send trajectory stream start");
                            return event_connection_lost_::trajectory_control_failure();
                        }
                        cmd.points_written = 0;
                        cmd.current_phase = sample_stream::phase::k_streaming;
                        if (auto err = drain_pending()) {
                            return err;
                        }
                        return std::nullopt;
                    }

                    case sample_stream::phase::k_streaming: {
                        if (pending_empty) {
                            emit_realtime_sample();
                            return std::nullopt;
                        }
                        if (auto err = drain_pending()) {
                            return err;
                        }
                        return std::nullopt;
                    }

                    case sample_stream::phase::k_buffered: {
                        // The producer closed before we sent anything, so we have the whole
                        // trajectory in hand. That is the finite case, so send it with the
                        // plain TRAJECTORY_START path that the unary move has always used
                        // rather than the streaming primitives: no dependency on the streaming
                        // URScript, and the proven terminal-deceleration behavior. Empty
                        // pending means the producer closed without any points, so there is
                        // nothing to send.
                        if (pending_empty) {
                            std::exchange(state.move_request_, {})->complete_success();
                            return std::nullopt;
                        }
                        const auto num_points = std::visit([](const auto& v) { return v.size(); }, *cmd.pending);
                        VIAM_SDK_LOG(debug) << "URArm sending buffered trajectory (" << num_points << " points)";
                        if (!arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                                                              static_cast<int>(num_points),
                                                                              RobotReceiveTimeout::off())) {
                            VIAM_SDK_LOG(error) << "send_trajectory: start failed; dropping connection";
                            std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start");
                            return event_connection_lost_::trajectory_control_failure();
                        }
                        if (auto err = drain_pending()) {
                            return err;
                        }
                        VIAM_SDK_LOG(debug) << "URArm trajectory sent";
                        cmd.current_phase = sample_stream::phase::k_ended;
                        return std::nullopt;
                    }

                    case sample_stream::phase::k_draining: {
                        if (!pending_empty) {
                            if (auto err = drain_pending()) {
                                return err;
                            }
                            return std::nullopt;
                        }
                        VIAM_SDK_LOG(debug) << "URArm sending stream end";
                        // TODO: `points_written` can exceed int on a long stream; this narrowing is silent.
                        if (!arm_conn_->driver->writeTrajectoryControlMessage(
                                urcl::control::TrajectoryControlMessage::TRAJECTORY_STREAM_END,
                                static_cast<int>(cmd.points_written),
                                RobotReceiveTimeout::off())) {
                            VIAM_SDK_LOG(error) << "send_trajectory: stream end failed; dropping connection";
                            std::exchange(state.move_request_, {})->complete_error("failed to send trajectory stream end");
                            return event_connection_lost_::trajectory_control_failure();
                        }
                        cmd.current_phase = sample_stream::phase::k_ended;
                        return std::nullopt;
                    }

                    case sample_stream::phase::k_ended: {
                        // STREAM_END on the wire; URScript is finishing.
                        // Emit realtime samples while we wait for the
                        // `trajectory_done_callback` to fire.
                        emit_realtime_sample();
                        return std::nullopt;
                    }
                }

                // Unreachable: the switch covers every phase.
                throw std::logic_error("handle_move_request: unhandled sample_stream::phase");

            } else if constexpr (std::is_same_v<T, std::optional<pose_sample>>) {
                // Operating in pose-space

                if (cmd && !state.move_request_->cancellation_request) {
                    // Send the single pose command
                    if (!arm_conn_->driver->writeTrajectoryControlMessage(
                            urcl::control::TrajectoryControlMessage::TRAJECTORY_START, 1, RobotReceiveTimeout::off())) {
                        VIAM_SDK_LOG(error) << "single pose: start failed; dropping connection";
                        std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start for single pose");
                        return event_connection_lost_::trajectory_control_failure();
                    }

                    auto ps = std::exchange(cmd, std::nullopt);
                    VIAM_SDK_LOG(debug) << "URArm sending single pose (tool-space)";

                    // TODO(RSDK-12294): determine how to set the velocity and acceleration so they are not hardcoded
                    const float velocity = 0.25F;
                    const float acceleration = 0.5F;
                    const float timestep = 0;
                    const float blend_radius = 0;
                    const bool cartesian = true;

                    if (!arm_conn_->driver->writeTrajectoryPoint(ps->p, acceleration, velocity, cartesian, timestep, blend_radius)) {
                        VIAM_SDK_LOG(error) << "single pose: writeTrajectoryPoint failed; dropping connection";
                        std::exchange(state.move_request_, {})->complete_error("failed to send single pose point");
                        return event_connection_lost_::trajectory_control_failure();
                    }

                    VIAM_SDK_LOG(debug) << "URArm single pose sent";
                    return std::nullopt;

                } else if (!cmd && state.move_request_->cancellation_request && !state.move_request_->cancellation_request->issued) {
                    // We have a move request, the samples have been forwarded,
                    // and cancellation is requested but has not yet been issued. Issue a cancel.
                    state.move_request_->cancellation_request->issued = true;
                    if (!arm_conn_->driver->writeTrajectoryControlMessage(
                            urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 1, RobotReceiveTimeout::off())) {
                        state.move_request_->cancel_error("failed to send trajectory cancel");
                        VIAM_SDK_LOG(error) << "cancel failed; dropping connection";
                        return event_connection_lost_::trajectory_control_failure();
                    }
                    return std::nullopt;
                } else if (cmd && state.move_request_->cancellation_request) {
                    // We have a move request that we haven't issued but a
                    // cancel is already pending. Don't issue it, just cancel it.
                    std::exchange(state.move_request_, {})->complete_cancelled();
                    return std::nullopt;

                } else {
                    // Pose sent, waiting for completion — record realtime sample
                    state.move_request_->write_realtime_sample(
                        *state.ephemeral_,
                        arm_conn_->robot_status_bits
                            ? std::optional<uint32_t>(static_cast<uint32_t>(arm_conn_->robot_status_bits->to_ulong()))
                            : std::nullopt,
                        arm_conn_->safety_status_bits
                            ? std::optional<uint32_t>(static_cast<uint32_t>(arm_conn_->safety_status_bits->to_ulong()))
                            : std::nullopt);
                    return std::nullopt;
                }
            }

            // Unreachable: all move_command_data variants should be handled above
            throw std::logic_error("handle_move_request: unexpected move_command_data type");
        },
        state.move_request_->move_command);
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

void URArm::state_::state_controlled_::zero_ftsensor() const {
    if (!arm_conn_->driver->zeroFTSensor()) {
        throw std::runtime_error(
            "failed to zero the force-torque sensor "
            "(requires an e-Series robot with the external control script running)");
    }
}

// NOLINTEND(readability-convert-member-functions-to-static)
