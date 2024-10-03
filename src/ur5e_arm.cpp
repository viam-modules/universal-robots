#include "ur5e_arm.hpp"

// global used to track if a trajectory is in progress
std::atomic<bool> trajectory_running(false);

// define callback function to be called by UR client library when program state changes
void reportRobotProgramState(bool program_running) {
    // Print the text in green so we see it better
    std::cout << "\033[1;32mprogram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

// define callback function to be called by UR client library when trajectory state changes
void reportTrajectoryState(control::TrajectoryResult state) {
    trajectory_running = false;
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
    std::cout << "\033[1;32mtrajectory report: " << report << "\033[0m\n" << std::endl;
}

UR5eArm::UR5eArm(Dependencies deps, const ResourceConfig& cfg) : Arm(cfg.name()) {
    this->reconfigure(deps, cfg);

    // get the APPDIR environment variable
    const char* path_offset = std::getenv("APPDIR");
    if (!path_offset) {
        throw std::runtime_error("required environment variable APPDIR unset");
    }

    // connect to the robot dashboard
    dashboard.reset(new DashboardClient(host));
    if (!dashboard->connect()) {
        throw std::runtime_error("couldn't connect to dashboard");
    }

    // stop program, if there is one running
    if (!dashboard->commandStop()) {
        throw std::runtime_error("couldn't stop program running on dashboard");
    }

    // if the robot is not powered on and ready
    std::string robotModeRunning("RUNNING");
    while (!dashboard->commandRobotMode(robotModeRunning)) {
        // power cycle the arm
        if (!dashboard->commandPowerOff()) {
            throw std::runtime_error("couldn't power off arm");
        }
        if (!dashboard->commandPowerOn()) {
            throw std::runtime_error("couldn't power on arm");
        }
    }

    // Release the brakes
    if (!dashboard->commandBrakeRelease()) {
        throw std::runtime_error("couldn't release the arm brakes");
    }

    // Now the robot is ready to receive a program
    std::unique_ptr<ToolCommSetup> tool_comm_setup;
    driver.reset(new UrDriver(host,
                              path_offset + SCRIPT_FILE,
                              path_offset + OUTPUT_RECIPE,
                              path_offset + INPUT_RECIPE,
                              &reportRobotProgramState,
                              true,  // headless mode
                              std::move(tool_comm_setup),
                              CALIBRATION_CHECKSUM));
    driver->registerTrajectoryDoneCallback(&reportTrajectoryState);

    // Once RTDE communication is started, we have to make sure to read from the interface buffer,
    // as otherwise we will get pipeline overflows. Therefore, do this directly before starting your
    // main loop
    driver->startRTDECommunication();
    read_and_noop();

    // start background thread to continuously send no-ops and keep socket connection alive
    std::thread t(&UR5eArm::keep_alive, this);
    t.detach();
}

// helper function to extract an attribute value from its key within a ResourceConfig
template <class T>
T find_config_attribute(const ResourceConfig& cfg, std::string attribute) {
    std::ostringstream buffer;
    auto motor = cfg.attributes()->find(attribute);
    if (motor == cfg.attributes()->end()) {
        buffer << "required attribute `" << attribute << "` not found in configuration";
        throw std::invalid_argument(buffer.str());
    }
    const auto* const val = motor->second->get<T>();
    if (!val) {
        buffer << "required non-empty attribute `" << attribute << " could not be decoded";
        throw std::invalid_argument(buffer.str());
    }
    return *val;
}

void UR5eArm::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    // extract relevant attributes from config
    host = find_config_attribute<std::string>(cfg, "host");
    speed = find_config_attribute<double>(cfg, "speed_degs_per_sec") * (M_PI / 180.0);
}

std::vector<double> UR5eArm::get_joint_positions(const AttributeMap& extra) {
    mu.lock();
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg && data_pkg->getData("actual_q", joint_state)) {
        std::vector<double> to_ret;
        for (double joint_pos_rad : joint_state) {
            double joint_pos_deg = 180.0 / M_PI * joint_pos_rad;
            to_ret.push_back(joint_pos_deg);
        }
        mu.unlock();
        return to_ret;
    }
    mu.unlock();
    return std::vector<double>();
}

void UR5eArm::move_to_joint_positions(const std::vector<double>& positions, const AttributeMap& extra) {
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), positions.size());
    Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
    waypoints.push_back(next_waypoint_rad);
    move(waypoints);
}

bool UR5eArm::is_moving() {
    return trajectory_running;
}

UR5eArm::KinematicsData UR5eArm::get_kinematics(const AttributeMap& extra) {
    // Open the file in binary mode
    std::ifstream file(URDF_FILE, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Unable to open file");
    }

    // Determine the file size
    file.seekg(0, std::ios::end);
    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // Create a buffer to hold the file contents
    std::vector<unsigned char> urdf_bytes(fileSize);

    // Read the file contents into the buffer
    if (!file.read(reinterpret_cast<char*>(urdf_bytes.data()), fileSize)) {
        throw std::runtime_error("Error reading file");
    }

    return KinematicsDataURDF(std::move(urdf_bytes));
}

void UR5eArm::stop(const AttributeMap& extra) {
    if (trajectory_running) {
        trajectory_running = false;
        driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off());
    }
}

AttributeMap UR5eArm::do_command(const AttributeMap& command) {
    if (!command) {
        throw std::runtime_error("command is null\n");
    }

    // move through a number of waypoints (specified as a 2D array of floats (degrees)) that were
    // sent in a batch
    if (command->count("waypoints") > 0) {
        std::vector<Eigen::VectorXd> waypoints;
        std::vector<std::shared_ptr<ProtoType>>* vec_protos = (*command)["waypoints"]->get<std::vector<std::shared_ptr<ProtoType>>>();
        if (vec_protos) {
            for (std::shared_ptr<ProtoType>& vec_proto : *vec_protos) {
                std::vector<std::shared_ptr<ProtoType>>* vec_joint_pos = vec_proto->get<std::vector<std::shared_ptr<ProtoType>>>();
                if (vec_joint_pos) {
                    vector6d_t to_add;
                    int counter = 0;
                    for (std::shared_ptr<ProtoType> joint_pos : *vec_joint_pos) {
                        double* joint_pos_deg = joint_pos->get<double>();
                        to_add[counter] = *joint_pos_deg;
                        counter += 1;
                    }
                    Eigen::VectorXd waypoint_deg = Eigen::VectorXd::Map(to_add.data(), to_add.size());
                    Eigen::VectorXd waypoint_rad = waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
                    waypoints.push_back(waypoint_rad);
                }
            }
        }
        move(waypoints);
    }

    return NULL;
}

// Send no-ops and keep socket connection alive
void UR5eArm::keep_alive() {
    while (true) {
        mu.lock();
        read_and_noop();
        mu.unlock();
        usleep(NOOP_DELAY);
    }
}

void UR5eArm::move(std::vector<Eigen::VectorXd> waypoints) {
    // get current joint position and add that as starting pose to waypoints
    std::vector<double> curr_joint_pos = get_joint_positions(NULL);
    Eigen::VectorXd curr_waypoint_deg = Eigen::VectorXd::Map(curr_joint_pos.data(), curr_joint_pos.size());
    Eigen::VectorXd curr_waypoint_rad = curr_waypoint_deg * (M_PI / 180.0);
    waypoints.insert(waypoints.begin(), curr_waypoint_rad);

    // calculate dot products and identify any consecutive segments with dot product == -1
    std::vector<int> segments;
    segments.push_back(0);
    for (int i = 2; i < waypoints.size(); i++) {
        Eigen::VectorXd segment_AB = (waypoints[i - 1] - waypoints[i - 2]);
        Eigen::VectorXd segment_BC = (waypoints[i] - waypoints[i - 1]);
        segment_AB.normalize();
        segment_BC.normalize();
        double dot = segment_BC.dot(segment_AB);
        if (dot == -1) {
            segments.push_back(i - 1);
        }
    }
    segments.push_back(waypoints.size() - 1);

    // set velocity/acceleration constraints
    std::cout << "generating trajectory with max speed: " << speed * (180.0 / M_PI) << std::endl;
    Eigen::VectorXd max_acceleration(6);
    Eigen::VectorXd max_velocity(6);
    max_acceleration << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    max_velocity << speed, speed, speed, speed, speed, speed;

    std::vector<vector6d_t> p;
    std::vector<vector6d_t> v;
    std::vector<vector6d_t> a;
    std::vector<double> time;

    for (int i = 0; i < segments.size() - 1; i++) {
        int start = segments[i];
        int end = segments[i + 1] + 1;
        std::list<Eigen::VectorXd> positions_subset(waypoints.begin() + start, waypoints.begin() + end);
        Trajectory trajectory(Path(positions_subset, 0.1), max_velocity, max_acceleration);
        trajectory.outputPhasePlaneTrajectory();
        if (trajectory.isValid()) {
            double duration = trajectory.getDuration();
            for (double t = 0.0; t < duration; t += TIMESTEP) {
                Eigen::VectorXd position = trajectory.getPosition(t);
                Eigen::VectorXd velocity = trajectory.getVelocity(t);
                p.push_back(vector6d_t{position[0], position[1], position[2], position[3], position[4], position[5]});
                v.push_back(vector6d_t{velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]});
                time.push_back(TIMESTEP);
            }
        } else {
            throw std::runtime_error("trajectory generation failed\n");
        }
    }

    mu.lock();
    send_trajectory(p, v, a, time, true);
    trajectory_running = true;
    while (trajectory_running) {
        read_and_noop();
    }
    mu.unlock();
}

// helper function to send time-indexed position, velocity, acceleration setpoints to the UR driver
void UR5eArm::send_trajectory(const std::vector<vector6d_t>& p_p,
                              const std::vector<vector6d_t>& p_v,
                              const std::vector<vector6d_t>& p_a,
                              const std::vector<double>& time,
                              bool use_spline_interpolation_) {
    assert(p_p.size() == time.size());
    driver->writeTrajectoryControlMessage(
        urcl::control::TrajectoryControlMessage::TRAJECTORY_START, p_p.size(), RobotReceiveTimeout::off());
    for (size_t i = 0; i < p_p.size() && p_p.size() == time.size() && p_p[i].size() == 6; i++) {
        if (!use_spline_interpolation_) {
            // movej
            driver->writeTrajectoryPoint(p_p[i], false, time[i], 0.0);
        } else {  // use spline interpolation
            if (p_v.size() == time.size() && p_a.size() == time.size() && p_v[i].size() == 6 && p_a[i].size() == 6) {
                // quintic
                driver->writeTrajectorySplinePoint(p_p[i], p_v[i], p_a[i], time[i]);
            } else if (p_v.size() == time.size() && p_v[i].size() == 6) {
                // cubic
                driver->writeTrajectorySplinePoint(p_p[i], p_v[i], time[i]);
            } else {
                // linear
                driver->writeTrajectorySplinePoint(p_p[i], time[i]);
            }
        }
    }
}

// helper function to read a data packet and send a noop message
void UR5eArm::read_and_noop() {
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg) {
        // read current joint positions from robot data
        if (!data_pkg->getData("actual_q", joint_state)) {
            throw std::runtime_error("couldn't get joint positions");
        }

        // send a noop to keep the connection alive
        driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off());
    }
}
