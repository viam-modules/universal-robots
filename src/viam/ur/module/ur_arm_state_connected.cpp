#include "ur_arm_state.hpp"

// NOLINTEND(readability-convert-member-functions-to-static)

std::chrono::milliseconds URArm::state_::state_connected_::get_timeout() const {
    return std::chrono::milliseconds{1000UL / arm_conn_->driver->getControlFrequency()};
}

URArm::state_::state_connected_::state_connected_(std::unique_ptr<arm_connection_> arm_conn) : arm_conn_{std::move(arm_conn)} {}

std::optional<URArm::state_::event_variant_> URArm::state_::state_connected_::send_noop() const {
    if (!arm_conn_->driver->writeTrajectoryControlMessage(
            control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        return event_connection_lost_{};
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_connected_::recv_arm_data(state_& state) {
    const auto prior_robot_status_bits = std::exchange(arm_conn_->robot_status_bits, std::nullopt);
    const auto prior_safety_status_bits = std::exchange(arm_conn_->safety_status_bits, std::nullopt);

    auto new_packet = arm_conn_->driver->getDataPackage();
    if (!new_packet) {
        consecutive_missed_packets++;
        // how many packets we can drop before restarting the connection
        if (consecutive_missed_packets > 3) {
            VIAM_SDK_LOG(error) << "Failed to read a data package from the arm: dropping connection";
            return event_connection_lost_{};
        }
        VIAM_SDK_LOG(warn) << "Failed to read a data package from the arm: missed a packet: " << consecutive_missed_packets;
    } else {
        arm_conn_->data_package = std::move(new_packet);
        consecutive_missed_packets = 0;
    }

    static const std::string k_robot_status_bits_key = "robot_status_bits";
    decltype(arm_conn_->robot_status_bits)::value_type robot_status_bits;
    if (!arm_conn_->data_package->getData<std::uint32_t>(k_robot_status_bits_key, robot_status_bits)) {
        VIAM_SDK_LOG(error) << "Data package did not contain the expected `robot_status_bits` information; dropping connection";
        return event_connection_lost_{};
    }

    static const std::string k_safety_status_bits_key = "safety_status_bits";
    decltype(arm_conn_->safety_status_bits)::value_type safety_status_bits;
    if (!arm_conn_->data_package->getData<std::uint32_t>(k_safety_status_bits_key, safety_status_bits)) {
        VIAM_SDK_LOG(error) << "Data package did not contain the expected `safety_status_bits` information; dropping connection";
        return event_connection_lost_{};
    }

    if (!prior_robot_status_bits) {
        VIAM_SDK_LOG(info) << "Obtained robot status bits: `" << robot_status_bits;
    } else if (*prior_robot_status_bits != robot_status_bits) {
        VIAM_SDK_LOG(info) << "Updated robot status bits: `" << robot_status_bits << "` (previously `" << *prior_robot_status_bits << "`)`";
    }
    arm_conn_->robot_status_bits = robot_status_bits;

    if (!prior_safety_status_bits) {
        VIAM_SDK_LOG(info) << "Obtained safety status bits: `" << safety_status_bits << "`";
    } else if (*prior_safety_status_bits != safety_status_bits) {
        VIAM_SDK_LOG(info) << "Updated safety status bits: `" << safety_status_bits << "` (previously `" << *prior_safety_status_bits
                           << "`)`";
    }
    arm_conn_->safety_status_bits = safety_status_bits;

    static const std::string k_joints_position_key = "actual_q";
    static const std::string k_joints_velocity_key = "actual_qd";
    static const std::string k_tcp_key = "actual_TCP_pose";

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

    // For consistency, update cached data only after all getData
    // calls succeed.
    if (data_good) {
        state.ephemeral_ = {std::move(joint_positions), std::move(joint_velocities), std::move(tcp_state)};
    }

    return std::nullopt;
}

// NOLINTEND(readability-convert-member-functions-to-static)
