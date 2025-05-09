#include "ur5e_arm.hpp"

// this chunk of code uses the rust FFI to handle the spatialmath calculations to turn a UR vector to a pose
extern "C" void* quaternion_from_axis_angle(double x, double y, double z, double theta);
extern "C" void* orientation_vector_from_quaternion(void* q);
extern "C" double* orientation_vector_get_components(void* ov);
extern "C" void free_orientation_vector_memory(void* ov);
extern "C" void free_quaternion_memory(void* q);

pose ur_vector_to_pose(urcl::vector6d_t vec) {
    double norm = sqrt(vec[3] * vec[3] + vec[4] * vec[4] + vec[5] * vec[5]);
    void* q = quaternion_from_axis_angle(vec[3] / norm, vec[4] / norm, vec[5] / norm, norm);
    void* ov = orientation_vector_from_quaternion(q);
    double* components = orientation_vector_get_components(ov);  // returned as ox, oy, oz, theta
    auto position = coordinates{1000 * vec[0], 1000 * vec[1], 1000 * vec[2]};
    auto orientation = pose_orientation{components[0], components[1], components[2]};
    auto theta = components[3] * 180 / M_PI;
    free_orientation_vector_memory(ov);
    free_quaternion_memory(q);
    delete[] components;
    return pose{position, orientation, theta};
}

// global used to track if a trajectory is in progress
std::atomic<bool> trajectory_running(false);

// define callback function to be called by UR client library when program state changes
void reportRobotProgramState(bool program_running) {
    // Print the text in green so we see it better
    BOOST_LOG_TRIVIAL(info) << "\033[1;32mUR program running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
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
    BOOST_LOG_TRIVIAL(debug) << "\033[1;32mtrajectory report: " << report << "\033[0m\n" << std::endl;
}

UR5eArm::UR5eArm(Dependencies deps, const ResourceConfig& cfg) : Arm(cfg.name()) {
    this->reconfigure(deps, cfg);

    // get the APPDIR environment variable
    path_offset = std::getenv("APPDIR");
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
    // Now the robot is ready to receive a program
    urcl::UrDriverConfiguration ur_cfg = {host,
                                          path_offset + SCRIPT_FILE,
                                          path_offset + OUTPUT_RECIPE,
                                          path_offset + INPUT_RECIPE,
                                          &reportRobotProgramState,
                                          true,  // headless mode
                                          nullptr};
    driver.reset(new UrDriver(ur_cfg));
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
    auto key = cfg.attributes().find(attribute);
    if (key == cfg.attributes().end()) {
        buffer << "required attribute `" << attribute << "` not found in configuration";
        throw std::invalid_argument(buffer.str());
    }
    const auto* const val = key->second.get<T>();
    if (!val) {
        buffer << "required non-empty attribute `" << attribute << " could not be decoded";
        throw std::invalid_argument(buffer.str());
    }
    return *val;
}

void UR5eArm::reconfigure(const Dependencies& deps, const ResourceConfig& cfg) {
    // extract relevant attributes from config
    host = find_config_attribute<std::string>(cfg, "host");
    speed.store(find_config_attribute<double>(cfg, "speed_degs_per_sec") * (M_PI / 180.0));
    acceleration.store(find_config_attribute<double>(cfg, "acceleration_degs_per_sec2") * (M_PI / 180.0));
}

std::vector<double> UR5eArm::get_joint_positions(const ProtoStruct& extra) {
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

void UR5eArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) {
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), positions.size());
    Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
    waypoints.push_back(next_waypoint_rad);
    move(waypoints);
}

void UR5eArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                           const MoveOptions& options,
                                           const viam::sdk::ProtoStruct& extra) {
    // TODO: use options
    if (positions.size() > 0) {
        std::vector<Eigen::VectorXd> waypoints;
        for (auto position : positions) {
            Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(position.data(), position.size());
            Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
            waypoints.push_back(next_waypoint_rad);
        }
        move(waypoints);
    }
    return;
}

pose UR5eArm::get_end_position(const ProtoStruct& extra) {
    mu.lock();
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg && data_pkg->getData("actual_TCP_pose", tcp_state)) {
        pose to_ret = ur_vector_to_pose(tcp_state);
        mu.unlock();
        return to_ret;
    }
    mu.unlock();
    return pose();
}

bool UR5eArm::is_moving() {
    return trajectory_running;
}

UR5eArm::KinematicsData UR5eArm::get_kinematics(const ProtoStruct& extra) {
    // Open the file in binary mode
    std::ifstream file(path_offset + SVA_FILE, std::ios::binary);
    if (!file) {
        throw std::runtime_error("unable to open file");
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

    return KinematicsDataSVA(std::move(urdf_bytes));
}

void UR5eArm::stop(const ProtoStruct& extra) {
    if (trajectory_running) {
        trajectory_running = false;
        driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off());
    }
}

ProtoStruct UR5eArm::do_command(const ProtoStruct& command) {
    ProtoStruct resp = ProtoStruct{};

    for (auto kv : command) {
        if (kv.first == VEL_KEY) {
            const double val = *kv.second.get<double>();
            speed.store(val * (M_PI / 180.0));
            resp.emplace(VEL_KEY, val);
        }
        if (kv.first == ACC_KEY) {
            const double val = *kv.second.get<double>();
            acceleration.store(val * (M_PI / 180.0));
            resp.emplace(ACC_KEY, val);
        }
    }

    return resp;
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
    // log to csv
    std::stringstream buffer;
    for (int i = 0; i < waypoints.size(); i++) {
        for (int j = 0; j < 6; j++) {
            buffer << waypoints[i][j] << ",";
        }
        buffer << std::endl;
    }
    std::ofstream outputFile(path_offset + WAYPOINTS_LOG);
    outputFile << buffer.str();
    outputFile.close();

    // get current joint position and add that as starting pose to waypoints
    std::vector<double> curr_joint_pos = get_joint_positions(ProtoStruct{});
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
        if (abs(dot + 1) < 1e-3) {
            segments.push_back(i - 1);
        }
    }
    segments.push_back(waypoints.size() - 1);

    // set velocity/acceleration constraints
    const double move_speed = speed.load();
    const double move_acceleration = acceleration.load();
    BOOST_LOG_TRIVIAL(debug) << "generating trajectory with max speed: " << move_speed * (180.0 / M_PI) << std::endl;
    Eigen::VectorXd max_acceleration(6);
    Eigen::VectorXd max_velocity(6);
    max_acceleration << move_acceleration, move_acceleration, move_acceleration, move_acceleration, move_acceleration, move_acceleration;
    max_velocity << move_speed, move_speed, move_speed, move_speed, move_speed, move_speed;

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
            double t = 0.0;
            while (t < duration) {
                Eigen::VectorXd position = trajectory.getPosition(t);
                Eigen::VectorXd velocity = trajectory.getVelocity(t);
                p.push_back(vector6d_t{position[0], position[1], position[2], position[3], position[4], position[5]});
                v.push_back(vector6d_t{velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]});
                time.push_back(TIMESTEP);
                t += TIMESTEP;
            }

            Eigen::VectorXd position = trajectory.getPosition(duration);
            Eigen::VectorXd velocity = trajectory.getVelocity(duration);
            p.push_back(vector6d_t{position[0], position[1], position[2], position[3], position[4], position[5]});
            v.push_back(vector6d_t{velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]});
            time.push_back(duration - (t - TIMESTEP));

        } else {
            std::stringstream buffer;
            buffer << "trajectory generation failed for path:";
            for (auto position : positions_subset) {
                buffer << "{";
                for (int j = 0; j < 6; j++) {
                    buffer << position[j] << " ";
                }
                buffer << "}";
            }
            throw std::runtime_error(buffer.str());
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

// Define the destructor
UR5eArm::~UR5eArm() {
    // stop the robot
    stop(ProtoStruct{});
    // disconnect from the dashboard
    if (dashboard) {
        dashboard->disconnect();
    }
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

    // log desired trajectory to a file
    std::stringstream buffer;
    buffer << "t(s), j0, j1, j2, j3, j4, j5, v0, v1, v2, v3, v4, v5, a0, a1, a2, a3, a4, a5" << std::endl;
    for (size_t i = 0; i < p_p.size() && p_p.size() == time.size() && p_p[i].size() == 6; i++) {
        buffer << time[i];
        for (int j = 0; j < 6; j++) {
            buffer << "," << p_p[i][j];
        }
        for (int j = 0; j < 6; j++) {
            buffer << "," << p_v[i][j];
        }
        for (int j = 0; j < 6; j++) {
            buffer << "," << p_v[i][j];
        }
        buffer << std::endl;

        // move
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

    std::ofstream outputFile(path_offset + TRAJECTORY_LOG);
    outputFile << buffer.str();
    outputFile.close();
}

// helper function to read a data packet and send a noop message
void UR5eArm::read_and_noop() {
    // check to see if an estop has occurred.
    std::string status;
    dashboard->commandSafetyStatus(status);

    if (status.find(urcl::safetyStatusString(urcl::SafetyStatus::NORMAL)) == std::string::npos) {
        // the arm is currently estopped. save this state.
        estop.store(true);
    } else {
        // the arm is in a normal state.
        if (estop.load()) {
            // if the arm was previously estopped, attempt to recover from the estop.
            // We should not enter this code without the user interacting with the arm in some way(i.e. resetting the estop)
            try {
                BOOST_LOG_TRIVIAL(info) << "recovering from e-stop" << std::endl;
                driver->resetRTDEClient(path_offset + OUTPUT_RECIPE, path_offset + INPUT_RECIPE);

                if (!dashboard->commandPowerOff()) {
                    BOOST_LOG_TRIVIAL(info) << "yo fail power down: " << status << std::endl;
                    throw std::runtime_error("couldn't power off arm");
                }
                if (!dashboard->commandPowerOn()) {
                    BOOST_LOG_TRIVIAL(info) << "yo fail power up: " << status << std::endl;
                    throw std::runtime_error("couldn't power on arm");
                }
                // Release the brakes
                if (!dashboard->commandBrakeRelease()) {
                    BOOST_LOG_TRIVIAL(info) << "yo failed release: " << status << std::endl;
                    throw std::runtime_error("couldn't release the arm brakes");
                }
                BOOST_LOG_TRIVIAL(info) << "yo break released: " << status << std::endl;

                bool send_prog = driver->sendRobotProgram();
                // don't know what to do if this is false yet
                BOOST_LOG_TRIVIAL(info) << "send robot program successful: " << send_prog << std::endl;

                trajectory_running = false;

                driver->startRTDECommunication();
                BOOST_LOG_TRIVIAL(info) << "restarted communication" << std::endl;
                estop.store(false);
            } catch (...) {
                BOOST_LOG_TRIVIAL(info) << "failed to restart the arm" << std::endl;
            }
            return;
        }
    }

    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg) {
        // read current joint positions from robot data
        if (!data_pkg->getData("actual_q", joint_state)) {
            throw std::runtime_error("couldn't get joint positions");
        }

        // send a noop to keep the connection alive
        driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off());
    } else {
        // we received no data packet, so our comms are down. reset the comms from the driver.
        BOOST_LOG_TRIVIAL(info) << "no packet found, resetting RTDE client connection" << std::endl;
        driver->resetRTDEClient(path_offset + OUTPUT_RECIPE, path_offset + INPUT_RECIPE);
        driver->startRTDECommunication();
    }
}
