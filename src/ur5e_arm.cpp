#include "ur5e_arm.hpp"

#include <cmath>

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

void write_trajectory_to_file(std::string filepath,
                              const std::vector<vector6d_t>& p_p,
                              const std::vector<vector6d_t>& p_v,
                              const std::vector<vector6d_t>& p_a,
                              const std::vector<float>& time) {
    // log desired trajectory to a file
    std::stringstream buffer;
    buffer << "t(s), j0, j1, j2, j3, j4, j5, v0, v1, v2, v3, v4, v5, a0, a1, a2, a3, a4, a5" << std::endl;
    for (size_t i = 0; i < p_p.size() && p_p.size() == time.size() && p_p[i].size() == 6; i++) {
        buffer << time[i];
        for (size_t j = 0; j < 6; j++) {
            buffer << "," << p_p[i][j];
        }
        for (size_t j = 0; j < 6; j++) {
            buffer << "," << p_v[i][j];
        }
        for (size_t j = 0; j < 6; j++) {
            buffer << "," << p_a[i][j];
        }
        buffer << std::endl;
    }

    std::ofstream outputFile(filepath);
    outputFile << buffer.str();
    outputFile.close();
}

void log_waypoints_to_csv(std::string filepath, std::vector<Eigen::VectorXd> waypoints) {
    std::stringstream buffer;
    for (size_t i = 0; i < waypoints.size(); i++) {
        for (size_t j = 0; j < 6; j++) {
            buffer << waypoints[i][j] << ",";
        }
        buffer << std::endl;
    }
    std::ofstream outputFile(filepath);
    outputFile << buffer.str();
    outputFile.close();
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
    BOOST_LOG_TRIVIAL(info) << "\033[1;32mtrajectory report: " << report << "\033[0m\n" << std::endl;
}

UR5eArm::UR5eArm(Dependencies deps, const ResourceConfig& cfg) : Arm(cfg.name()) {
    this->reconfigure(deps, cfg);

    // get the APPDIR environment variable
    auto tmp = std::getenv("APPDIR");
    if (!tmp) {
        throw std::runtime_error("required environment variable APPDIR unset");
    }
    path_offset = std::string(tmp);

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
    if (!read_and_noop()) {
        throw std::runtime_error("couldn't get joint positions");
    }

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
    if (data_pkg == nullptr) {
        mu.unlock();
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_joint_positions got nullptr from driver->getDataPackage()" << std::endl;
        return std::vector<double>();
    }

    if (!data_pkg->getData("actual_q", joint_state)) {
        mu.unlock();
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_joint_positions()->getData(\"actual_q\") returned false" << std::endl;
        return std::vector<double>();
    }

    std::vector<double> to_ret;
    for (double joint_pos_rad : joint_state) {
        double joint_pos_deg = 180.0 / M_PI * joint_pos_rad;
        to_ret.push_back(joint_pos_deg);
    }
    mu.unlock();
    return to_ret;
}
std::chrono::milliseconds unix_now_ms() {
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
    std::chrono::system_clock::duration dtn = tp.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(dtn);
}

void UR5eArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) {
    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), positions.size());
    Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
    waypoints.push_back(next_waypoint_rad);
    std::chrono::milliseconds unix_time_ms = unix_now_ms();
    log_waypoints_to_csv(path_offset + WAYPOINTS_LOG, waypoints);
    if (!move(waypoints, unix_time_ms)) {
        throw std::runtime_error("move failed");
    };
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
        std::chrono::milliseconds unix_time_ms = unix_now_ms();
        log_waypoints_to_csv(path_offset + WAYPOINTS_LOG, waypoints);
        if (!move(waypoints, unix_time_ms)) {
            throw std::runtime_error("move failed");
        };
    }
    return;
}

pose UR5eArm::get_end_position(const ProtoStruct& extra) {
    mu.lock();
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg == nullptr) {
        mu.unlock();
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_end_position got nullptr from driver->getDataPackage()" << std::endl;
        return pose();
    }
    if (!data_pkg->getData("actual_TCP_pose", tcp_state)) {
        mu.unlock();
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_end_position driver->getDataPackage().getData(\"actual_TCP_pos\") returned false"
                                   << std::endl;
        return pose();
    }
    pose to_ret = ur_vector_to_pose(tcp_state);
    mu.unlock();
    return to_ret;
}

bool UR5eArm::is_moving() {
    return trajectory_running.load();
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
    if (trajectory_running.load()) {
        bool ok = driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off());
        if (!ok) {
            BOOST_LOG_TRIVIAL(warning) << "UR5eArm::stop driver->writeTrajectoryControlMessage returned false" << std::endl;
            return;
        }
        trajectory_running.store(false);
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

bool UR5eArm::move(std::vector<Eigen::VectorXd> waypoints, std::chrono::milliseconds unix_time_ms) {
    BOOST_LOG_TRIVIAL(info) << "move: start unix_time_ms " << unix_time_ms.count() << " waypoints size " << waypoints.size() << std::endl;

    // get current joint position and add that as starting pose to waypoints
    BOOST_LOG_TRIVIAL(info) << "move: get_joint_positions star " << unix_time_ms.count() << std::endl;
    std::vector<double> curr_joint_pos = get_joint_positions(ProtoStruct{});
    BOOST_LOG_TRIVIAL(info) << "move: get_joint_positions end" << unix_time_ms.count() << std::endl;

    BOOST_LOG_TRIVIAL(info) << "move: compute_trajectory start " << unix_time_ms.count() << std::endl;
    Eigen::VectorXd curr_waypoint_deg = Eigen::VectorXd::Map(curr_joint_pos.data(), curr_joint_pos.size());
    Eigen::VectorXd curr_waypoint_rad = curr_waypoint_deg * (M_PI / 180.0);
    waypoints.insert(waypoints.begin(), curr_waypoint_rad);

    // calculate dot products and identify any consecutive segments with dot product == -1
    std::vector<size_t> segments;
    segments.push_back(0);
    for (size_t i = 2; i < waypoints.size(); i++) {
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
    std::vector<float> time;

    for (size_t i = 0; i < segments.size() - 1; i++) {
        size_t start = segments[i];
        size_t end = segments[i + 1] + 1;
        std::list<Eigen::VectorXd> positions_subset(waypoints.begin() + start, waypoints.begin() + end);
        Trajectory trajectory(Path(positions_subset, 0.1), max_velocity, max_acceleration);
        trajectory.outputPhasePlaneTrajectory();
        if (!trajectory.isValid()) {
            std::stringstream buffer;
            buffer << "move: unix_time_ms " << unix_time_ms.count() << " trajectory generation failed for path:";
            for (auto position : positions_subset) {
                buffer << "{";
                for (size_t j = 0; j < 6; j++) {
                    buffer << position[j] << " ";
                }
                buffer << "}";
            }
            BOOST_LOG_TRIVIAL(error) << buffer.str() << std::endl;
            return false;
        }

        float duration = static_cast<float>(trajectory.getDuration());
        if (std::isinf(duration)) {
            BOOST_LOG_TRIVIAL(error) << "trajectory.getDuration() was infinite" << std::endl;
            return false;
        }
        float t = 0.0;
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
        float t2 = duration - (t - TIMESTEP);
        if (std::isinf(t2)) {
            BOOST_LOG_TRIVIAL(error) << "duration - (t - TIMESTEP) was infinite" << std::endl;
            return false;
        }
        time.push_back(t2);
    }
    BOOST_LOG_TRIVIAL(info) << "move: compute_trajectory end" << unix_time_ms.count() << " p.count() " << p.size() << " v " << v.size()
                            << " a " << a.size() << " time " << time.size() << std::endl;

    write_trajectory_to_file(path_offset + TRAJECTORY_LOG, p, v, a, time);
    mu.lock();
    if (!send_trajectory(p, v, a, time)) {
        mu.unlock();
        BOOST_LOG_TRIVIAL(error) << "move: " << unix_time_ms.count() << " send_trajectory failed" << std::endl;
        return false;
    };

    bool ok = false;
    do {
        ok = read_and_noop();
    } while (trajectory_running.load() && ok);
    mu.unlock();

    if (!ok) {
        BOOST_LOG_TRIVIAL(error) << "move: unix_time_ms" << unix_time_ms.count() << " read_and_noop failed, couldn't get joint positions"
                                 << std::endl;
        return false;
    }
    BOOST_LOG_TRIVIAL(info) << "move: end unix_time_ms " << unix_time_ms.count() << std::endl;
    return true;
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
// dashboard connect
// dashboard disconnect
// driver connect
// driver getDataPackage
// driver writeTrajectoryControlMessage
// driver writeTrajectoryPoint
// driver writeTrajectorySplinePoint
// driver disconnect

// helper function to send time-indexed position, velocity, acceleration setpoints to the UR driver
bool UR5eArm::send_trajectory(const std::vector<vector6d_t>& p_p,
                              const std::vector<vector6d_t>& p_v,
                              const std::vector<vector6d_t>& p_a,
                              const std::vector<float>& time) {
    BOOST_LOG_TRIVIAL(info) << "UR5eArm::send_trajectory start" << std::endl;
    if (p_p.size() != time.size()) {
        BOOST_LOG_TRIVIAL(error) << "UR5eArm::send_trajectory p_p.size() != time.size() not executing" << std::endl;
        return false;
    };
    // NOTE: (Nick) - why are we sending a control message before we are ready to actually do the work?
    auto point_number = static_cast<int>(p_p.size());
    if (!driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_START, point_number, RobotReceiveTimeout::off())) {
        BOOST_LOG_TRIVIAL(error) << "read_and_noop driver->writeTrajectoryControlMessage returned false" << std::endl;
        return false;
    };

    trajectory_running.store(true);
    size_t quintic = 0;
    size_t cubic = 0;
    size_t linear = 0;
    for (size_t i = 0; i < p_p.size() && p_p.size() == time.size() && p_p[i].size() == 6; i++) {
        if (p_v.size() == time.size() && p_a.size() == time.size() && p_v[i].size() == 6 && p_a[i].size() == 6) {
            quintic++;
            if (!driver->writeTrajectorySplinePoint(p_p[i], p_v[i], p_a[i], time[i])) {
                BOOST_LOG_TRIVIAL(error) << "send_trajectory quintic driver->writeTrajectorySplinePoint returned false" << std::endl;
                return false;
            };
        } else if (p_v.size() == time.size() && p_v[i].size() == 6) {
            cubic++;
            if (!driver->writeTrajectorySplinePoint(p_p[i], p_v[i], time[i])) {
                BOOST_LOG_TRIVIAL(error) << "send_trajectory cubic driver->writeTrajectorySplinePoint returned false" << std::endl;
                return false;
            };
        } else {
            linear++;
            if (!driver->writeTrajectorySplinePoint(p_p[i], time[i])) {
                BOOST_LOG_TRIVIAL(error) << "send_trajectory linear driver->writeTrajectorySplinePoint returned false" << std::endl;
                return false;
            };
        }
    }

    BOOST_LOG_TRIVIAL(info) << "UR5eArm::send_trajectory end: quintic" << quintic << " cubic " << cubic << " linear " << linear
                            << std::endl;
    return true;
}

// helper function to read a data packet and send a noop message
bool UR5eArm::read_and_noop() {
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "read_and_noop driver->getDataPackage() returned nullptr" << std::endl;
        return false;
    }
    // read current joint positions from robot data
    if (!data_pkg->getData("actual_q", joint_state)) {
        BOOST_LOG_TRIVIAL(error) << "read_and_noop driver->getDataPackage()->data_pkg->getData(\"actual_q\") returned false" << std::endl;
        return false;
    }

    // send a noop to keep the connection alive
    if (!driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        BOOST_LOG_TRIVIAL(error) << "read_and_noop driver->writeTrajectoryControlMessage returned false" << std::endl;
        return false;
    };
    return true;
}
