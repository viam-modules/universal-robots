#include "ur_arm_state.hpp"

using namespace std::literals::string_view_literals;

// NOLINTBEGIN(readability-convert-member-functions-to-static)

std::string_view URArm::state_::event_connection_established_::name() {
    return "connection_established"sv;
}

std::string_view URArm::state_::event_connection_established_::describe() const {
    return name();
}

URArm::state_::event_connection_lost_::event_connection_lost_(reason r) : reason_code{r} {}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::data_communication_failure() {
    return event_connection_lost_{k_data_communication_failure};
}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::dashboard_communication_failure() {
    return event_connection_lost_{k_dashboard_communication_failure};
}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::dashboard_command_failure() {
    return event_connection_lost_{k_dashboard_command_failure};
}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::dashboard_control_mode_change() {
    return event_connection_lost_{k_dashboard_control_mode_change};
}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::robot_program_failure() {
    return event_connection_lost_{k_robot_program_failure};
}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::trajectory_control_failure() {
    return event_connection_lost_{k_trajectory_control_failure};
}

URArm::state_::event_connection_lost_ URArm::state_::event_connection_lost_::module_shutdown() {
    return event_connection_lost_{k_module_shutdown};
}

std::string_view URArm::state_::event_connection_lost_::name() {
    return "connection_lost"sv;
}

std::string_view URArm::state_::event_connection_lost_::describe() const {
    static constexpr auto reason_to_string = [](reason r) -> std::string_view {
        switch (r) {
            case reason::k_data_communication_failure:
                return "connection_lost(data_communication_failure)"sv;
            case reason::k_dashboard_communication_failure:
                return "connection_lost(dashboard_communication_failure)"sv;
            case reason::k_dashboard_command_failure:
                return "connection_lost(dashboard_command_failure)"sv;
            case reason::k_dashboard_control_mode_change:
                return "connection_lost(dashboard_control_mode_change)"sv;
            case reason::k_robot_program_failure:
                return "connection_lost(robot_program_failure)"sv;
            case reason::k_trajectory_control_failure:
                return "connection_lost(trajectory_control_failure)"sv;
            case reason::k_module_shutdown:
                return "connection_lost(module_shutdown)"sv;
            default:
                return "connection_lost(unknown_reason)"sv;
        }
    };

    return reason_to_string(reason_code);
}

std::string_view URArm::state_::event_stop_detected_::name() {
    return "stop_detected"sv;
}

std::string_view URArm::state_::event_stop_detected_::describe() const {
    return name();
}

std::string_view URArm::state_::event_stop_cleared_::name() {
    return "stop_cleared"sv;
}

std::string_view URArm::state_::event_stop_cleared_::describe() const {
    return name();
}

std::string_view URArm::state_::event_local_mode_detected_::name() {
    return "local_mode_detected"sv;
}

std::string_view URArm::state_::event_local_mode_detected_::describe() const {
    return name();
}

std::string_view URArm::state_::event_remote_mode_detected_::name() {
    return "remote_mode_restored"sv;
}

std::string_view URArm::state_::event_remote_mode_detected_::describe() const {
    return name();
}

// NOLINTEND(readability-convert-member-functions-to-static)
