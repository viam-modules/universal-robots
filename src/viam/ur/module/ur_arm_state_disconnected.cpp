#include "ur_arm_state.hpp"

#include <boost/algorithm/string/case_conv.hpp>

namespace {

// PolyScope X (controller software 10.x) has no dashboard server on port
// 29999; its dashboard is a REST API. The primary interface reports the
// software version on every controller generation, so ask it which dashboard
// client policy to use.
//
// PolyScope X releases before 10.12 lack Robot API endpoints we depend on
// (e.g. remote control mode), so URCL would throw partway through
// reconnecting. Reject them up front with an actionable message instead.
DashboardClient::ClientPolicy detect_dashboard_policy(const std::string& host) {
    comm::INotifier notifier;
    primary_interface::PrimaryClient primary{host, notifier};
    primary.start(1);
    const auto version = primary.getRobotVersion();
    if (!version) {
        throw std::runtime_error("failed to read the controller software version from host " + host);
    }
    constexpr std::uint32_t k_first_polyscope_x_major = 10;
    if (version->major < k_first_polyscope_x_major) {
        return DashboardClient::ClientPolicy::G5;
    }
    static const auto k_min_supported_polyscope_x = urcl::VersionInformation::fromString("10.12.0");
    if (*version < k_min_supported_polyscope_x) {
        std::ostringstream buffer;
        buffer << "PolyScope X " << version->toString() << " on host " << host << " is not supported; version "
               << k_min_supported_polyscope_x.toString() << " or newer is required";
        throw std::runtime_error(buffer.str());
    }
    return DashboardClient::ClientPolicy::POLYSCOPE_X;
}

// Whether a PolyScope X program is playing or paused. The Robot API program
// state only tracks PolyScope X programs, not the headless URScript this
// module sends over the primary interface, so our own control script never
// counts. Any other state, including ones a future Robot API may add, is
// treated as inactive.
bool polyscope_x_program_is_active(DashboardClient& dashboard) {
    std::string program_state;
    if (!dashboard.commandProgramState(program_state)) {
        throw std::runtime_error("couldn't read program state from the PolyScope X Robot API");
    }
    return program_state == "PLAYING" || program_state == "PAUSED";
}

// PolyScope X rejects a stop request when no program is active, so only stop
// there when one is. The PolyScope 5 / CB3 stop command succeeds either way.
bool should_stop_polyscope_program(DashboardClient& dashboard, bool is_polyscope_x) {
    return !is_polyscope_x || polyscope_x_program_is_active(dashboard);
}

// PolyScope X accepts a stop request before the program has stopped, whereas
// the PolyScope 5 / CB3 dashboard client waits for STOPPED itself. Wait here
// too, so the control script isn't sent while the old program is still
// stopping. A timeout throws, and the reconnect loop retries from scratch.
void wait_for_polyscope_x_program_stop(DashboardClient& dashboard) {
    constexpr auto k_poll_period = std::chrono::milliseconds{100};
    constexpr auto k_timeout = std::chrono::seconds{5};
    const auto deadline = std::chrono::steady_clock::now() + k_timeout;
    while (polyscope_x_program_is_active(dashboard)) {
        if (std::chrono::steady_clock::now() >= deadline) {
            throw std::runtime_error("PolyScope X program did not stop within " + std::to_string(k_timeout.count()) +
                                     "s of the stop request");
        }
        std::this_thread::sleep_for(k_poll_period);
    }
}

}  // namespace

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
    auto arm_connection = std::make_unique<arm_connection_>(state.log_resource_);

    constexpr auto k_log_at_n_attempts = 100;
    if (++reconnect_attempts % k_log_at_n_attempts == 0) {
        if (triggering_event_) {
            VIAM_RESOURCE_LOG(state.log_resource_, warn)
                << "disconnected: the connection to the arm was lost due to a " << triggering_event_->describe()
                << " event; attempting automatic recovery which may take some time";
        }
        VIAM_RESOURCE_LOG(state.log_resource_, info) << "While in state " << describe() << ", attempting to reconnect";
    }

    const auto dashboard_policy = detect_dashboard_policy(state.host_);
    const bool is_polyscope_x = dashboard_policy == DashboardClient::ClientPolicy::POLYSCOPE_X;
    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "While in state " << describe() << ", attempting to reconnect: detected "
                                                  << (is_polyscope_x ? "PolyScope X" : "PolyScope 5 / CB3") << " controller";

    arm_connection->is_polyscope_x = is_polyscope_x;
    arm_connection->dashboard = std::make_unique<DashboardClient>(state.host_, dashboard_policy);
    if (is_polyscope_x) {
        // The PolyScope X client opens a new TCP connection for every request,
        // and requests run on the worker thread, so an unreachable controller
        // stalls the tick for the full connect timeout (URCL default 5s).
        // Connecting takes milliseconds on a working network.
        constexpr auto k_polyscope_x_connect_timeout = std::chrono::seconds{1};
        arm_connection->dashboard->setConnectTimeout(k_polyscope_x_connect_timeout);
    }
    if (reconnect_attempts % k_log_at_n_attempts == 0) {
        VIAM_RESOURCE_LOG(state.log_resource_, debug)
            << "While in state " << describe() << ", attempting to reconnect: trying to connect to dashboard";
    }
    if (!arm_connection->dashboard->connect(1)) {
        std::ostringstream buffer;
        buffer << "Failed trying to connect to UR " << (is_polyscope_x ? "PolyScope X Robot API" : "dashboard") << " on host "
               << state.host_;
        throw std::runtime_error(buffer.str());
    }
    arm_connection->log_destructor = true;

    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "While in state " << describe() << ", attempting to reconnect: connected to dashboard";

    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "While in state " << describe() << ", attempting to reconnect: validating model";
    std::string actual_model_type{};
    if (!arm_connection->dashboard->commandGetRobotModel(actual_model_type)) {
        throw std::runtime_error("failed to get model info of connected arm");
    }
    // The dashboard reports the model in uppercase (e.g. "UR20"), while our
    // configured model carries the SDK name in lowercase. The PolyScope 5 /
    // CB3 dashboard uses the coarser URCL category form ("UR5" covers both
    // ur5e and ur7e); the PolyScope X Robot API has its own naming. Lower-case
    // the dashboard side and compare against the matching descriptor field.
    boost::algorithm::to_lower(actual_model_type);

    const auto& descriptor = state.configured_model_.descriptor();
    const auto& expected_model_type = is_polyscope_x ? descriptor.polyscope_x_model : descriptor.urcl_category;
    if (expected_model_type != actual_model_type) {
        std::ostringstream buffer;
        buffer << "configured model type `" << state.configured_model_.sdk_name() << "` does not match connected arm `" << actual_model_type
               << "`";
        throw std::runtime_error(buffer.str());
    }

    // If the arm is in remote control mode, we need to stop any
    // running program, since it might not be ours.
    //
    // TODO(11619): See if we can fully eliminate use of the
    // DashboardClient and use only the `UrDriver`.
    if (arm_connection->dashboard->commandIsInRemoteControl() &&
        should_stop_polyscope_program(*arm_connection->dashboard, is_polyscope_x)) {
        VIAM_RESOURCE_LOG(state.log_resource_, debug)
            << "While in state " << describe() << ", attempting to reconnect: stopping any currently running program";
        if (!arm_connection->dashboard->commandStop()) {
            throw std::runtime_error("couldn't stop program running on arm_connection->dashboard");
        }
        if (is_polyscope_x) {
            wait_for_polyscope_x_program_stop(*arm_connection->dashboard);
        }
    }

    // locations of files necessary to build module, specified as relative paths
    constexpr char k_script_file[] = "external_control.urscript";
    constexpr char k_output_recipe[] = "control/rtde_output_recipe.txt";
    constexpr char k_input_recipe[] = "control/rtde_input_recipe.txt";

    VIAM_RESOURCE_LOG(state.log_resource_, debug)
        << "While in state " << describe() << ", attempting to reconnect: instantiating new UrDriver";
    // Now the robot is ready to receive a program
    auto ur_cfg = urcl::UrDriverConfiguration{};
    ur_cfg.robot_ip = state.host_;
    ur_cfg.script_file = state.urcl_resource_root() / k_script_file;
    ur_cfg.output_recipe_file = state.resource_root() / k_output_recipe;
    ur_cfg.input_recipe_file = state.resource_root() / k_input_recipe;
    ur_cfg.headless_mode = true;
    ur_cfg.socket_reconnect_attempts = 1;

    ur_cfg.reverse_port = state.ports_.reverse_port;
    ur_cfg.script_sender_port = state.ports_.script_sender_port;
    ur_cfg.trajectory_port = state.ports_.trajectory_port;
    ur_cfg.script_command_port = state.ports_.script_command_port;
    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "using reverse_port " << ur_cfg.reverse_port;
    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "using script_sender_port " << ur_cfg.script_sender_port;
    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "using trajectory_port " << ur_cfg.trajectory_port;
    VIAM_RESOURCE_LOG(state.log_resource_, debug) << "using script_command_port " << ur_cfg.script_command_port;

    arm_connection->driver = std::make_unique<UrDriver>(ur_cfg);

    // A little weird, but there doesn't seem to be a way to directly
    // set the target frequency, so we need to do a `resetRTDEClient`
    // call to achieve what we want.
    arm_connection->driver->resetRTDEClient(
        ur_cfg.output_recipe_file, ur_cfg.input_recipe_file, static_cast<std::uint32_t>(std::ceil(state.robot_control_freq_hz_)));

    arm_connection->driver->registerTrajectoryDoneCallback(
        std::bind(&URArm::state_::trajectory_done_callback_, &state, std::placeholders::_1));

    VIAM_RESOURCE_LOG(state.log_resource_, debug)
        << "While in state " << describe() << ", attempting to reconnect: starting RTDE communication";
    arm_connection->driver->startRTDECommunication();

    VIAM_RESOURCE_LOG(state.log_resource_, debug)
        << "While in state " << describe() << ", attempting to reconnect: attempting to read a data package from the arm";
    arm_connection->data_package = std::make_unique<urcl::rtde_interface::DataPackage>(arm_connection->driver->getRTDEOutputRecipe());
    if (!arm_connection->driver->getDataPackage(*arm_connection->data_package)) {
        throw std::runtime_error("could not read data package from newly established driver connection ");
    }

    VIAM_RESOURCE_LOG(state.log_resource_, debug)
        << "While in state " << describe() << ", recovery appears to have been successful; transitioning to independent mode";
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

std::optional<URArm::state_::state_variant_> URArm::state_::state_disconnected_::handle_event(event_enter_freedrive_) {
    throw std::runtime_error("cannot enter manual mode: arm is not connected");
}

std::optional<URArm::state_::state_variant_> URArm::state_::state_disconnected_::handle_event(event_connection_established_ event) {
    // If we are leaving disconnected mode for independent mode, we
    // don't know, really, in what state we will find the arm. Assume
    // the worst case scenario, that we are both stopped and in local
    // mode, and let the natural recovery process sort out how to get
    // back to a good place.
    return state_independent_{std::move(event.payload), state_independent_::reason::k_both};
}

void URArm::state_::state_disconnected_::clear_pstop() const {
    throw std::runtime_error("cannot clear the protective stop, arm is currently disconnected");
}

void URArm::state_::state_disconnected_::zero_ftsensor() const {
    throw std::runtime_error("cannot zero the force-torque sensor, arm is currently disconnected");
}

// NOLINTEND(readability-convert-member-functions-to-static)
