#include "ur_arm_state.hpp"

// NOLINTEND(readability-convert-member-functions-to-static)

std::chrono::milliseconds URArm::state_::state_connected_::get_timeout() const {
    return std::chrono::milliseconds{1000UL / arm_conn_->driver->getControlFrequency()};
}

URArm::state_::state_connected_::state_connected_(std::unique_ptr<arm_connection_> arm_conn) : arm_conn_{std::move(arm_conn)} {}

std::optional<URArm::state_::event_variant_> URArm::state_::state_connected_::send_noop() const {
    if (!arm_conn_->driver->writeTrajectoryControlMessage(
            control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        VIAM_SDK_LOG(error) << "While in a connected state, failed to write a NOOP trajectory control message; dropping connection";
        return event_connection_lost_::trajectory_control_failure();
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_connected_::recv_arm_data(state_& state) {
    const auto prior_robot_status_bits = std::exchange(arm_conn_->robot_status_bits, std::nullopt);
    const auto prior_safety_status_bits = std::exchange(arm_conn_->safety_status_bits, std::nullopt);

    // On failure, getDataPackage leaves data_package unmodified, so we retain the last good packet.
    if (!arm_conn_->driver->getDataPackage(*arm_conn_->data_package)) {
        consecutive_missed_packets++;
        // how many packets we can drop before restarting the connection
        if (consecutive_missed_packets > 3) {
            VIAM_SDK_LOG(error) << "Failed to read a data package from the arm: dropping connection";
            return event_connection_lost_::data_communication_failure();
        }
        VIAM_SDK_LOG(warn) << "Failed to read a data package from the arm: missed a packet: " << consecutive_missed_packets;
    } else {
        consecutive_missed_packets = 0;
    }

    static const std::string k_robot_status_bits_key = "robot_status_bits";
    decltype(arm_conn_->robot_status_bits)::value_type robot_status_bits;
    if (!arm_conn_->data_package->getData<std::uint32_t>(k_robot_status_bits_key, robot_status_bits)) {
        VIAM_SDK_LOG(error) << "While in a connected state, the data package did not contain the expected `robot_status_bits` information; "
                               "dropping connection";
        return event_connection_lost_::data_communication_failure();
    }

    static const std::string k_safety_status_bits_key = "safety_status_bits";
    decltype(arm_conn_->safety_status_bits)::value_type safety_status_bits;
    if (!arm_conn_->data_package->getData<std::uint32_t>(k_safety_status_bits_key, safety_status_bits)) {
        VIAM_SDK_LOG(error) << "While in connected state, the data package did not contain the expected `safety_status_bits` information; "
                               "dropping connection";
        return event_connection_lost_::data_communication_failure();
    }

    if (!prior_robot_status_bits) {
        VIAM_SDK_LOG(debug) << "Obtained robot status bits: `" << robot_status_bits;
    } else if (*prior_robot_status_bits != robot_status_bits) {
        VIAM_SDK_LOG(debug) << "Updated robot status bits: `" << robot_status_bits << "` (previously `" << *prior_robot_status_bits
                            << "`)`";
    }
    arm_conn_->robot_status_bits = robot_status_bits;

    if (!prior_safety_status_bits) {
        VIAM_SDK_LOG(debug) << "Obtained safety status bits: `" << safety_status_bits << "`";
    } else if (*prior_safety_status_bits != safety_status_bits) {
        VIAM_SDK_LOG(debug) << "Updated safety status bits: `" << safety_status_bits << "` (previously `" << *prior_safety_status_bits
                            << "`)`";
    }
    arm_conn_->safety_status_bits = safety_status_bits;

    static const std::string k_joints_position_key = "actual_q";
    static const std::string k_joints_velocity_key = "actual_qd";
    static const std::string k_tcp_key = "actual_TCP_pose";
    static const std::string k_tcp_force_key = "actual_TCP_force";
    static const std::string k_target_q_key = "target_q";
    static const std::string k_target_qd_key = "target_qd";
    static const std::string k_target_qdd_key = "target_qdd";
    static const std::string k_target_current_key = "target_current";
    static const std::string k_target_moment_key = "target_moment";
    static const std::string k_target_tcp_speed_key = "target_TCP_speed";
    static const std::string k_actual_tcp_speed_key = "actual_TCP_speed";
    static const std::string k_joint_temperatures_key = "joint_temperatures";
    static const std::string k_joint_control_output_key = "joint_control_output";
    static const std::string k_safety_status_key = "safety_status";

    bool data_good = true;
    vector6d_t joint_positions{};
    if (!arm_conn_->data_package->getData(k_joints_position_key, joint_positions)) {
        VIAM_SDK_LOG(error) << "getData(\"actual_q\") returned false - joint positions will not be available";
        data_good = false;
    }

    // read current joint velocities from robot data
    vector6d_t joint_velocities{};
    if (!arm_conn_->data_package->getData(k_joints_velocity_key, joint_velocities)) {
        VIAM_SDK_LOG(error) << "getData(\"actual_qd\") returned false - joint velocities will not be available";
        data_good = false;
    }

    vector6d_t tcp_state{};
    if (!arm_conn_->data_package->getData(k_tcp_key, tcp_state)) {
        VIAM_SDK_LOG(warn) << "getData(\"actual_TCP_pos\") returned false - end effector pose will not be available";
        data_good = false;
    }

    vector6d_t tcp_force{};
    if (!arm_conn_->data_package->getData(k_tcp_force_key, tcp_force)) {
        VIAM_SDK_LOG(warn) << "getData(\"actual_TCP_force\") returned false - end effector force will not be available";
        data_good = false;
    }

    // Read new telemetry fields (best-effort: don't fail if missing)
    vector6d_t target_q{};
    arm_conn_->data_package->getData(k_target_q_key, target_q);

    vector6d_t target_qd{};
    arm_conn_->data_package->getData(k_target_qd_key, target_qd);

    vector6d_t target_qdd{};
    arm_conn_->data_package->getData(k_target_qdd_key, target_qdd);

    vector6d_t target_current{};
    arm_conn_->data_package->getData(k_target_current_key, target_current);

    vector6d_t target_moment{};
    arm_conn_->data_package->getData(k_target_moment_key, target_moment);

    vector6d_t target_tcp_speed{};
    arm_conn_->data_package->getData(k_target_tcp_speed_key, target_tcp_speed);

    vector6d_t actual_tcp_speed{};
    arm_conn_->data_package->getData(k_actual_tcp_speed_key, actual_tcp_speed);

    vector6d_t joint_temperatures{};
    arm_conn_->data_package->getData(k_joint_temperatures_key, joint_temperatures);

    vector6d_t joint_control_output{};
    arm_conn_->data_package->getData(k_joint_control_output_key, joint_control_output);

    std::optional<uint32_t> safety_status_val;
    int32_t safety_status_raw{};
    if (arm_conn_->data_package->getData(k_safety_status_key, safety_status_raw)) {
        safety_status_val = static_cast<uint32_t>(safety_status_raw);
    }

    // For consistency, update cached data only after all critical getData
    // calls succeed.
    if (data_good) {
        state.ephemeral_ = {
            std::move(joint_positions),
            std::move(joint_velocities),
            std::move(tcp_state),
            std::move(tcp_force),
            std::move(target_q),
            std::move(target_qd),
            std::move(target_qdd),
            std::move(target_current),
            std::move(target_moment),
            std::move(target_tcp_speed),
            std::move(actual_tcp_speed),
            std::move(joint_temperatures),
            std::move(joint_control_output),
            safety_status_val,
        };
    }

    return std::nullopt;
}

// NOLINTEND(readability-convert-member-functions-to-static)
