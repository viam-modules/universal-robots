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
        // if (!samples[0].is_joint_space) {

        //     // assume samples is a std::vector<trajectory_sample_point>
        //     const auto& s0 = samples[0];

        //     // 1. Build a UR pose from p (x, y, z, rx, ry, rz)
        //     urcl::Pose target_pose(s0.p[0], s0.p[1], s0.p[2], s0.p[3], s0.p[4], s0.p[5]);

        //     // 2. Choose motion parameters (you can adjust these)
        //     double blend_radius = 0.0;
        //     double acceleration = 0.25;
        //     double velocity = 0.10;  // could also derive from s0.v if you want
        //     std::chrono::duration<double> duration(0);

        //     // 3. Create the primitive
        //     // MoveLPrimitive moveL(target_pose, blend_radius, duration, acceleration, velocity);
        //     if (!arm_conn_->driver->writeTrajectoryControlMessage(
        //             urcl::control::TrajectoryControlMessage::TRAJECTORY_START, static_cast<int>(1), RobotReceiveTimeout::off())) {
        //         VIAM_SDK_LOG(error) << "send_trajectory driver->writeTrajectoryControlMessage returned false; dropping connection";
        //         std::exchange(state.move_request_, {})->complete_error("failed to send trajectory start message to arm");
        //         return event_connection_lost_::trajectory_control_failure();
        //     }
        //     auto motion_primitive = std::make_shared<urcl::control::MotionPrimitive>(
        //         urcl::control::MoveLPrimitive(target_pose, blend_radius, duration, acceleration, velocity));

        //     // 4. Send it to the driver

        //     arm_conn_->driver->writeMotionPrimitive(motion_primitive);
        // }

        // if (!samples[0].is_joint_space) {
        //     const bool g_trajectory_done = false;
        //     // Trajectory definition
        //     // std::vector<urcl::vector6d_t> points{
        //     //     {samples[0][0], samples[0][1], samples[0][2], samples[0][3], samples[0][4], samples[0][5]},
        //     //     {samples[1][0], samples[1][1], samples[1][2], samples[1][3], samples[1][4], samples[1][5]}};
        //     // std::vector<urcl::vector6d_t> points{{samples[0].p}, {samples[1].p}};
        //     std::vector<double> motion_durations{5.0, 5.0};
        //     std::vector<double> velocities{2.0, 2.3};
        //     std::vector<double> accelerations{2.5, 2.5};
        //     std::vector<double> blend_radii{0.0, 0.0};

        //     // Trajectory execution of the path that goes through the points twice.
        //     arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
        //                                                      static_cast<int>(4));
        //     for (size_t i = 0; i < num_samples; i++) {
        //         // setting the cartesian parameter makes it interpret the 6d vector as a pose and use movel
        //         arm_conn_->driver->writeTrajectoryPoint(samples[i].p, true, motion_durations[i], blend_radii[i]);
        //     }

        //     // Same motion, but parametrized with acceleration and velocity
        //     motion_durations = {0.0, 0.0};
        //     for (size_t i = 0; i < num_samples; i++) {
        //         arm_conn_->driver->writeTrajectoryPoint(
        //             samples[i].p, accelerations[i], velocities[i], true, motion_durations[i], blend_radii[i]);
        //     }

        //     while (!g_trajectory_done) {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //         arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
        //     }
        // }

        if (!samples[0].is_joint_space) {
            // Ensure parameter vectors match the number of samples and are float
            const size_t n = num_samples;

            std::vector<float> motion_durations(n, 5.0f);
            std::vector<float> velocities(n, 2.0f);
            std::vector<float> accelerations(n, 2.5f);
            std::vector<float> blend_radii(n, 0.0f);

            // Start trajectory (4 was hard-coded; use n unless you truly want to send 4)
            if (!arm_conn_->driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                                                  static_cast<int>(n))) {
                VIAM_SDK_LOG(error) << "writeTrajectoryControlMessage(START) failed";
                std::exchange(state.move_request_, {})->complete_error("failed to start trajectory");
                return event_connection_lost_::trajectory_control_failure();
            }

            // First pass: duration + blend
            for (size_t i = 0; i < n; ++i) {
                // interpret as Cartesian pose (movel)
                const bool cartesian = true;
                if (!arm_conn_->driver->writeTrajectoryPoint(samples[i].p, cartesian, motion_durations[i], blend_radii[i])) {
                    VIAM_SDK_LOG(error) << "writeTrajectoryPoint (duration form) failed at i=" << i;
                    std::exchange(state.move_request_, {})->complete_error("failed to send trajectory point (duration)");
                    return event_connection_lost_::trajectory_control_failure();
                }
            }

            // Second pass: param by accel/vel + blend (duration 0)
            std::fill(motion_durations.begin(), motion_durations.end(), 0.0f);
            for (size_t i = 0; i < n; ++i) {
                const bool cartesian = true;
                if (!arm_conn_->driver->writeTrajectoryPoint(
                        samples[i].p, accelerations[i], velocities[i], cartesian, motion_durations[i], blend_radii[i])) {
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

        // if tool

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
