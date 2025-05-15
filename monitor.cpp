#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>

#include <csignal>
#include <iostream>

#include "embed.h"

const std::string SCRIPT_FILE = "/src/control/external_control.urscript";
const std::string OUTPUT_RECIPE = "/src/control/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "/src/control/rtde_input_recipe.txt";
std::atomic<bool> exit_signal_received;

void signal_handler(int signal) {
    exit_signal_received.store(true);
}

struct Monitor {
    std::string host;
    std::string appdir;

    void monitor();
    void read_joint_keep_alive(unsigned long long int iteration);

    std::unique_ptr<urcl::UrDriver> driver;
    std::unique_ptr<urcl::DashboardClient> dashboard;
    std::atomic<bool> trajectory_running;
    urcl::vector6d_t joint_state, tcp_state;
};

void Monitor::monitor() {
    // connect to the robot dashboard
    dashboard.reset(new urcl::DashboardClient(host));
    if (!dashboard->connect()) {
        throw std::runtime_error("couldn't connect to dashboard");
    }

    // // stop program, if there is one running
    // if (!dashboard->commandStop()) {
    //     throw std::runtime_error("couldn't stop program running on dashboard");
    // }
    std::string robotModeRunning("RUNNING");
    if (!dashboard->commandRobotMode(robotModeRunning)) {
        throw std::runtime_error("arm is not running");
    }

    // // Release the brakes
    // if (!dashboard->commandBrakeRelease()) {
    //     throw std::runtime_error("couldn't release the arm brakes");
    // }

    std::function<void(bool)> logProgramState = [=](bool program_running) {
        std::cerr << "\033[1;32mUR program running: " << std::boolalpha << program_running << "\033[0m\n";
    };
    // Now the robot is ready to receive a program
    urcl::UrDriverConfiguration ur_cfg = {.robot_ip = host,
                                          .script_file = appdir + SCRIPT_FILE,
                                          .output_recipe_file = appdir + OUTPUT_RECIPE,
                                          .input_recipe_file = appdir + INPUT_RECIPE,
                                          .handle_program_state = logProgramState,
                                          .headless_mode = true};
    driver.reset(new urcl::UrDriver(ur_cfg));

    std::function<void(urcl::control::TrajectoryResult)> monitorTrajectoryState = [=](urcl::control::TrajectoryResult state) {
        trajectory_running.store(false);
        std::string report;
        switch (state) {
            case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
                report = "success";
                break;
            case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
                report = "canceled";
                break;
            case urcl::control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
            default:
                report = "failure";
        }
        std::cerr << "\033[1;32mtrajectory report: " << report << "\033[0m\n";
    };
    driver->registerTrajectoryDoneCallback(monitorTrajectoryState);

    // Once RTDE communication is started, we have to make sure to read from the interface buffer,
    // as otherwise we will get pipeline overflows. Therefore, do this directly before starting your
    // main loop
    driver->startRTDECommunication();
    unsigned long long int iteration = 0;
    while (!exit_signal_received.load()) {
        read_joint_keep_alive(iteration);
        iteration++;
    }

    dashboard->disconnect();
}
std::string formatvector6d_t(urcl::vector6d_t data, std::string name, unsigned long long int iteration) {
    std::ostringstream buffer;
    buffer << iteration << ", " << name;
    for (auto i : data) {
        buffer << ", " << i;
    }
    buffer << "\n";
    return buffer.str();
}

void Monitor::read_joint_keep_alive(unsigned long long int iteration) {
    std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg == nullptr) {
        // we received no data packet, so our comms are down. reset the comms from the driver.
        std::cerr << "read_joint_keep_alive driver->getDataPackage() returned nullptr. resetting RTDE client connection\n";
        try {
            driver->resetRTDEClient(appdir + OUTPUT_RECIPE, appdir + INPUT_RECIPE);
        } catch (const std::exception& ex) {
            std::cerr << "read_joint_keep_alive driver RTDEClient failed to restart: " << std::string(ex.what()) << "\n";
            return;
        }
        driver->startRTDECommunication();
        std::cerr << "RTDE client connection successfully restarted\n";
        return;
    }

    // read current joint positions from robot data
    if (!data_pkg->getData("actual_q", joint_state)) {
        std::cerr << "read_joint_keep_alive driver->getDataPackage()->data_pkg->getData(\"actual_q\") returned false\n";
        return;
    }

    std::cout << formatvector6d_t(joint_state, "actual_q", iteration);

    if (!data_pkg->getData("actual_TCP_pose", tcp_state)) {
        std::cerr << "UR5eArm::get_end_position driver->getDataPackage().getData(\"actual_TCP_pos\") returned false\n";
        return;
    }

    std::cout << formatvector6d_t(tcp_state, "actual_TCP_pose", iteration);

    // send a noop to keep the connection alive
    if (!driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, urcl::RobotReceiveTimeout::off())) {
        std::cerr << "read_joint_keep_alive driver->writeTrajectoryControlMessage returned false\n";
        return;
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGKILL, signal_handler);
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << "<ur_robot_ip>\n";
        return EXIT_FAILURE;
    }
    std::string host = std::string(argv[1]);

    auto tmp = std::getenv("APPDIR");
    if (!tmp) {
        std::cerr << "must be run as an appimage APPDIR env var not set\n";
        return EXIT_FAILURE;
    }
    std::string appdir = std::string(tmp);
    std::cout << "host: " << host << " appdir " << appdir;
    Monitor m = {.host = host, .appdir = appdir};
    try {
        m.monitor();
    } catch (const std::exception& ex) {
        std::cerr << "monitor failed - err: " << ex.what() << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
};
