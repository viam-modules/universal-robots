#include <boost/log/trivial.hpp>
#include <csignal>
#include <iostream>

#include "src/ur5e_arm.hpp"
std::atomic<bool> exit_signal_received;
void signal_handler(int signal) {
    exit_signal_received.store(true);
}

struct Monitor {
    std::string host;
    std::string appdir;

    void monitor();
    void read_joint_keep_alive();

    std::unique_ptr<UrDriver> driver;
    std::unique_ptr<DashboardClient> dashboard;
    std::atomic<bool> trajectory_running;
    vector6d_t joint_state, tcp_state;
};

void Monitor::monitor() {
    // connect to the robot dashboard
    dashboard.reset(new DashboardClient(host));
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
        BOOST_LOG_TRIVIAL(info) << "\033[1;32mUR program running: " << std::boolalpha << program_running << "\033[0m\n";
    };
    // Now the robot is ready to receive a program
    urcl::UrDriverConfiguration ur_cfg = {.robot_ip = host,
                                          .script_file = appdir + SCRIPT_FILE,
                                          .output_recipe_file = appdir + OUTPUT_RECIPE,
                                          .input_recipe_file = appdir + INPUT_RECIPE,
                                          .handle_program_state = logProgramState,
                                          .headless_mode = true};
    driver.reset(new UrDriver(ur_cfg));

    std::function<void(control::TrajectoryResult)> monitorTrajectoryState = [=](control::TrajectoryResult state) {
        trajectory_running.store(false);
        std::string report;
        switch (state) {
            case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
                report = "success";
                break;
            case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
                report = "canceled";
                break;
            case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
            default:
                report = "failure";
        }
        BOOST_LOG_TRIVIAL(info) << "\033[1;32mtrajectory report: " << report << "\033[0m\n";
    };
    driver->registerTrajectoryDoneCallback(monitorTrajectoryState);

    // Once RTDE communication is started, we have to make sure to read from the interface buffer,
    // as otherwise we will get pipeline overflows. Therefore, do this directly before starting your
    // main loop
    driver->startRTDECommunication();
    while (!exit_signal_received.load()) {
        read_joint_keep_alive();
    }

    dashboard->disconnect();
}

void Monitor::read_joint_keep_alive() {
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg == nullptr) {
        // we received no data packet, so our comms are down. reset the comms from the driver.
        BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver->getDataPackage() returned nullptr. resetting RTDE client connection";
        try {
            driver->resetRTDEClient(appdir + OUTPUT_RECIPE, appdir + INPUT_RECIPE);
        } catch (const std::exception& ex) {
            BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver RTDEClient failed to restart: " << std::string(ex.what());
            return;
        }
        driver->startRTDECommunication();
        BOOST_LOG_TRIVIAL(info) << "RTDE client connection successfully restarted";
        return;
    }

    // read current joint positions from robot data
    if (!data_pkg->getData("actual_q", joint_state)) {
        BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver->getDataPackage()->data_pkg->getData(\"actual_q\") returned false";
        return;
    }

    if (!data_pkg->getData("actual_TCP_pose", tcp_state)) {
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_end_position driver->getDataPackage().getData(\"actual_TCP_pos\") returned false";
        return;
    }

    // send a noop to keep the connection alive
    if (!driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver->writeTrajectoryControlMessage returned false";
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
    appdir = std::string(tmp);
    std::cout << "host: " << host << " appdir " << appdir;
    Monitor m = {.host = host, .appdir = appdir};
    // m.start();
    return EXIT_SUCCESS;
};
