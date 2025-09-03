#include "ur_arm_state.hpp"

using namespace std::literals::string_view_literals;

// NOLINTBEGIN(readability-convert-member-functions-to-static)

std::string_view URArm::state_::event_connection_established_::name() {
    return "connection_established"sv;
}

std::string_view URArm::state_::event_connection_established_::describe() const {
    return name();
}

std::string_view URArm::state_::event_connection_lost_::name() {
    return "connection_lost"sv;
}

std::string_view URArm::state_::event_connection_lost_::describe() const {
    return name();
}

std::string_view URArm::state_::event_estop_detected_::name() {
    return "estop_detected"sv;
}

std::string_view URArm::state_::event_estop_detected_::describe() const {
    return name();
}

std::string_view URArm::state_::event_estop_cleared_::name() {
    return "estop_cleared"sv;
}

std::string_view URArm::state_::event_estop_cleared_::describe() const {
    return name();
}

std::string_view URArm::state_::event_local_mode_detected_::name() {
    return "local_mode_detected"sv;
}

std::string_view URArm::state_::event_local_mode_detected_::describe() const {
    return name();
}

std::string_view URArm::state_::event_remote_mode_restored_::name() {
    return "remote_mode_restored"sv;
}

std::string_view URArm::state_::event_remote_mode_restored_::describe() const {
    return name();
}

// NOLINTEND(readability-convert-member-functions-to-static)
