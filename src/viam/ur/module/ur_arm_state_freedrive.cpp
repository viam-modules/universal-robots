#include "ur_arm_state.hpp"

#include <stdexcept>
#include <utility>

#include <viam/sdk/log/logging.hpp>

// NOLINTBEGIN(readability-convert-member-functions-to-static)

URArm::state_::state_freedrive_::state_freedrive_(std::unique_ptr<arm_connection_> arm_conn,
                                                  std::optional<std::chrono::steady_clock::time_point> deadline)
    : state_connected_(std::move(arm_conn)), deadline_(deadline) {}

std::string_view URArm::state_::state_freedrive_::name() {
    using namespace std::literals::string_view_literals;
    return "freedrive"sv;
}

std::string URArm::state_::state_freedrive_::describe() const {
    return std::string{name()};
}

void URArm::state_::state_freedrive_::stop_freedrive_() const {
    // Best effort: we are leaving freedrive regardless. A failure here just
    // means the control script will drop freedrive on its own once it sees the
    // next (non-freedrive) control message.
    if (!arm_conn_->driver->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_STOP, RobotReceiveTimeout::off())) {
        VIAM_SDK_LOG(warn) << "While leaving " << describe() << ", failed to send a freedrive stop message to the arm";
    }
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_freedrive_::upgrade_downgrade(state_&) {
    namespace urtde = urcl::rtde_interface;

    if (!arm_conn_->safety_status_bits || !arm_conn_->robot_status_bits) {
        VIAM_SDK_LOG(warn) << "While in state " << describe() << ", robot and safety status bits were not available; dropping connection";
        return event_connection_lost_::data_communication_failure();
    }

    if (!arm_conn_->safety_status_bits->test(static_cast<size_t>(urtde::UrRtdeSafetyStatusBits::IS_NORMAL_MODE))) {
        return event_stop_detected_{};
    }

    // Unlike the controlled state, the teach-button-pressed bit is expected here
    // (freedrive engages the arm's teach function), so we check the bits we
    // require are set rather than for an exact match.
    const auto& robot_status_bits = *(arm_conn_->robot_status_bits);
    if (!robot_status_bits.test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_POWER_ON)) ||
        !robot_status_bits.test(static_cast<size_t>(urtde::UrRtdeRobotStatusBits::IS_PROGRAM_RUNNING))) {
        return event_stop_detected_{};
    }

    if (deadline_ && std::chrono::steady_clock::now() >= *deadline_) {
        VIAM_SDK_LOG(info) << "manual mode time limit reached; leaving freedrive";
        return event_freedrive_ended_{};
    }

    // As in the controlled state, treat any loss of remote control as a reason
    // to rebuild the connection, since sockets aren't reliable across
    // local/remote transitions.
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

std::optional<URArm::state_::event_variant_> URArm::state_::state_freedrive_::handle_move_request(state_& state) {
    // Actuation is not possible while hand-guiding; reject any pending move.
    if (state.move_request_) {
        std::exchange(state.move_request_, {})->complete_error("cannot move: arm is in manual mode (freedrive)");
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_freedrive_::send_noop() {
    // Keep freedrive alive. A regular trajectory NOOP (MODE_FORWARD) would be
    // read by the control script as a control-mode change and drop freedrive.
    if (!arm_conn_->driver->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP, RobotReceiveTimeout::off())) {
        VIAM_SDK_LOG(error) << "While in " << describe() << ", failed to write a freedrive control message; dropping connection";
        return event_connection_lost_::trajectory_control_failure();
    }
    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_freedrive_::handle_event(event_connection_lost_ event) {
    // The connection is gone; the arm leaves freedrive on its own.
    return state_disconnected_{std::move(event)};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_freedrive_::handle_event(event_stop_detected_) {
    stop_freedrive_();
    return state_independent_{std::move(arm_conn_), state_independent_::reason::k_stopped};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_freedrive_::handle_event(event_freedrive_ended_) {
    stop_freedrive_();
    return state_controlled_{std::move(arm_conn_)};
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_freedrive_::handle_event(event_enter_freedrive_ event) {
    // Already in freedrive; just refresh the auto-exit deadline, no transition.
    deadline_ = event.deadline;
    return std::nullopt;
}

void URArm::state_::state_freedrive_::clear_pstop() const {
    throw std::runtime_error("cannot clear the protective stop, arm is not currently pstopped");
}

void URArm::state_::state_freedrive_::zero_ftsensor() const {
    throw std::runtime_error("cannot zero the force-torque sensor while in manual mode; exit manual mode first");
}

// NOLINTEND(readability-convert-member-functions-to-static)
