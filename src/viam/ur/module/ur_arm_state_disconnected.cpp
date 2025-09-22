#include "ur_arm_state.hpp"

// NOLINTBEGIN(readability-convert-member-functions-to-static)

URArm::state_::state_disconnected_::state_disconnected_(event_connection_lost_ triggering_event)
    : triggering_event_{std::make_unique<event_connection_lost_>(std::move(triggering_event))} {}

std::string_view URArm::state_::state_disconnected_::name() {
    using namespace std::literals::string_view_literals;
    return "disconnected"sv;
}

std::string URArm::state_::state_disconnected_::describe() const {
    if (triggering_event_) {
        return std::string{name()} + "(" + std::string{triggering_event_->describe()} + ")";
    } else {
        return std::string{name()} + "(awaiting connection)";
    }
}

std::chrono::milliseconds URArm::state_::state_disconnected_::get_timeout() const {
    // If we have a pending connection, we are polling for completion,
    // so increase the sampling rate. Otherwise we want to wait a more
    // reasonable length of time before attempting to connect again.
    return pending_connection ? std::chrono::milliseconds(5) : std::chrono::seconds(1);
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::recv_arm_data(state_&) {
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::upgrade_downgrade(state_& state) {
    if (pending_connection) {
        // There is a pending connection attempt. Poll it for
        // completion. If it has completed, get the result, which will
        // either cause an exception to be thrown or return the arm
        // connection that we can wrap up into a connection established
        // event. If the future isn't ready, there is no event to
        // return, and we will come back to poll again.
        switch (pending_connection->wait_for(std::chrono::seconds(0))) {
            case std::future_status::ready: {
                return event_connection_established_{std::exchange(pending_connection, std::nullopt)->get()};
            }
            case std::future_status::timeout: {
                break;
            }
            case std::future_status::deferred: {
                // Impossible, due to `std::launch::async` below.
                std::abort();
            }
        }
    } else {
        // Otherwise, there is no pending connection attempt. Start a new one, and do not advance the state machine.
        pending_connection.emplace(std::async(std::launch::async, [this, &state] { return connect_(state); }));
    }
    return std::nullopt;
}

std::unique_ptr<URArm::state_::arm_connection_> URArm::state_::state_disconnected_::connect_(state_& state) {
    auto arm_connection = std::make_unique<arm_connection_>();

    constexpr auto k_log_at_n_attempts = 100;
    if (++reconnect_attempts % k_log_at_n_attempts == 0) {
        if (triggering_event_) {
            VIAM_SDK_LOG(warn) << "disconnected: the connection to the arm was lost due to a " << triggering_event_->describe()
                               << " event; attempting automatic recovery which may take some time";
        }
        VIAM_SDK_LOG(info) << "disconnected: attempting recovery";
    }

    arm_connection->dashboard = std::make_unique<DashboardClient>(state.host_);
    if (reconnect_attempts % k_log_at_n_attempts == 0) {
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

    // locations of files necessary to build module, specified as relative paths
    constexpr char k_script_file[] = "control/external_control.urscript";
    constexpr char k_output_recipe[] = "control/rtde_output_recipe.txt";
    constexpr char k_input_recipe[] = "control/rtde_input_recipe.txt";

    VIAM_SDK_LOG(info) << "disconnected: attempting recovery: instantiating new UrDriver";
    // Now the robot is ready to receive a program
    auto ur_cfg = urcl::UrDriverConfiguration{};
    ur_cfg.robot_ip = state.host_;
    ur_cfg.script_file = state.resource_root_ / k_script_file;
    ur_cfg.output_recipe_file = state.resource_root_ / k_output_recipe;
    ur_cfg.input_recipe_file = state.resource_root_ / k_input_recipe;

    // TODO: Change how this works. It ends up logging with this
    // disconnected.cpp filename state when we have a connection.
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
    return arm_connection;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::handle_move_request(state_& state) const {
    if (state.move_request_) {
        const std::string error_message = "move request failed: no connection to arm; current state: " + describe();
        std::exchange(state.move_request_, {})->complete_error(error_message);
    }
    return std::nullopt;
}

std::optional<URArm::state_::event_variant_> URArm::state_::state_disconnected_::send_noop() {
    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_disconnected_::handle_event(event_connection_lost_) {
    return std::nullopt;
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_disconnected_::handle_event(event_connection_established_ event) {
    // If we are leaving disconnected mode for independent mode, we
    // don't know, really, in what state we will find the arm. Assume
    // the worst case scenario, that we are both stopped and in local
    // mode, and let the natural recovery process sort out how to get
    // back to a good place.
    return state_independent_{std::move(event.payload), state_independent_::reason::k_both};
}

// NOLINTEND(readability-convert-member-functions-to-static)
