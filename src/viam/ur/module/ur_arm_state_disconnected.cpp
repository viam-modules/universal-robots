#include "ur_arm_state.hpp"

namespace {

// locations of files necessary to build module, specified as relative paths
constexpr char k_output_recipe[] = "/src/control/rtde_output_recipe.txt";
constexpr char k_input_recipe[] = "/src/control/rtde_input_recipe.txt";

}  // namespace

// NOLINTBEGIN(readability-convert-member-functions-to-static)

std::string_view URArm::state_::state_disconnected_::name() {
    using namespace std::literals::string_view_literals;
    return "disconnected"sv;
}

std::string URArm::state_::state_disconnected_::describe() const {
    return std::string{name()};
}

std::chrono::milliseconds URArm::state_::state_disconnected_::get_timeout() const {
    return std::chrono::seconds(1);
}

bool URArm::state_::state_disconnected_::do_command_close_safety_popup() {
    throw std::runtime_error("cannot close popup, arm is currently disconnected");
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::recv_arm_data(state_&) {
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::upgrade_downgrade(state_& state) {
    reconnect_attempts++;
    const auto log_at_n_attempts = 100;
    if (reconnect_attempts % log_at_n_attempts == 0) {
        VIAM_SDK_LOG(info) << "disconnected: attempting recovery";
    }
    auto arm_connection = std::make_unique<arm_connection_>();
    arm_connection->dashboard = std::make_unique<DashboardClient>(state.host_);

    if (reconnect_attempts % log_at_n_attempts == 0) {
        VIAM_SDK_LOG(info) << "disconnected: attempting recovery: trying to connect to dashboard";
    }
    if (!arm_connection->dashboard->connect(1)) {
        std::ostringstream buffer;
        buffer << "Failed trying to connect to UR dashboard on host " << state.host_;
        throw std::runtime_error(buffer.str());
    }
    arm_connection->log_destructor = true;

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: connected to dashboard";

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: validating model";
    std::string actual_model_type{};
    if (!arm_connection->dashboard->commandGetRobotModel(actual_model_type)) {
        throw std::runtime_error("failed to get model info of connected arm");
    }

    if (state.configured_model_type_ != actual_model_type) {
        std::ostringstream buffer;
        buffer << "configured model type `" << state.configured_model_type_ << "` does not match connected arm `" << actual_model_type
               << "`";
        throw std::runtime_error(buffer.str());
    }

    // If the arm is in remote control mode, we need to stop any
    // running program, since it might not be ours.
    //
    // TODO(11619): See if we can fully eliminate use of the
    // DashboardClient and use only the `UrDriver`.
    if (arm_connection->dashboard->commandIsInRemoteControl()) {
        VIAM_SDK_LOG(info) << "disconnected: attempting recovery: stopping any currently running program";
        if (!arm_connection->dashboard->commandStop()) {
            throw std::runtime_error("couldn't stop program running on arm_connection->dashboard");
        }
    }

    constexpr char k_script_file[] = "/src/control/external_control.urscript";

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: instantiating new UrDriver";
    // Now the robot is ready to receive a program
    auto ur_cfg = urcl::UrDriverConfiguration{};
    ur_cfg.robot_ip = state.host_;
    ur_cfg.script_file = state.app_dir_ + k_script_file;
    ur_cfg.output_recipe_file = state.app_dir_ + k_output_recipe;
    ur_cfg.input_recipe_file = state.app_dir_ + k_input_recipe;
    ur_cfg.handle_program_state = [&running_flag = arm_connection->program_running_flag](bool running) {
        VIAM_SDK_LOG(info) << "UR program is " << (running ? "running" : "not running");
        running_flag.store(running, std::memory_order_release);
    };
    ur_cfg.headless_mode = true;
    ur_cfg.socket_reconnect_attempts = 1;

    ur_cfg.reverse_port = state.ports_.reverse_port;
    ur_cfg.script_sender_port = state.ports_.script_sender_port;
    ur_cfg.trajectory_port = state.ports_.trajectory_port;
    ur_cfg.script_command_port = state.ports_.script_command_port;
    VIAM_SDK_LOG(info) << "using reverse_port " << ur_cfg.reverse_port;
    VIAM_SDK_LOG(info) << "using script_sender_port " << ur_cfg.script_sender_port;
    VIAM_SDK_LOG(info) << "using trajectory_port " << ur_cfg.trajectory_port;
    VIAM_SDK_LOG(info) << "using script_command_port " << ur_cfg.script_command_port;

    arm_connection->driver = std::make_unique<UrDriver>(ur_cfg);

    // A little weird, but there doesn't seem to be a way to directly
    // set the target frequency, so we need to do a `resetRTDEClient`
    // call to achieve what we want.
    arm_connection->driver->resetRTDEClient(
        ur_cfg.output_recipe_file, ur_cfg.input_recipe_file, static_cast<std::uint32_t>(std::ceil(state.robot_control_freq_hz_)));

    arm_connection->driver->registerTrajectoryDoneCallback(
        std::bind(&URArm::state_::trajectory_done_callback_, &state, std::placeholders::_1));

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: starting RTDE communication";
    arm_connection->driver->startRTDECommunication();

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: attempting to read a data package from the arm";
    if (!arm_connection->driver->getDataPackage()) {
        throw std::runtime_error("could not read data package from newly established driver connection ");
    }

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: recovery appears to have been successful; transitioning to independent mode";
    return event_connection_established_{std::move(arm_connection)};
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::handle_move_request(state_& state) {
    if (state.move_request_) {
        std::exchange(state.move_request_, {})->complete_error("no connection to arm");
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::send_noop() {
    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_disconnected_::handle_event(event_connection_established_ event) {
    // If we are leaving disconnected mode for independent mode, we
    // don't know, really, in what state we will find the arm. Assume
    // the worst case scenario, that we are both estopped and in local
    // mode, and let the natural recovery process sort out how to get
    // back to a good place.
    return state_independent_{std::move(event.payload), state_independent_::reason::k_both};
}

// NOLINTEND(readability-convert-member-functions-to-static)
