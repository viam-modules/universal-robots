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
                              const std::vector<float>& time) {
    bool valid = p_p.size() == p_v.size() && p_p.size() == time.size();
    if (!valid) {
        BOOST_LOG_TRIVIAL(info) << "write_trajectory_to_file called with invalid parameters";
        return;
    }
    std::ofstream of(filepath);
    of << "t(s),j0,j1,j2,j3,j4,j5,v0,v1,v2,v3,v4,v5\n";
    for (size_t i = 0; i < p_p.size(); i++) {
        of << time[i];
        for (size_t j = 0; j < 6; j++) {
            of << "," << p_p[i][j];
        }
        for (size_t j = 0; j < 6; j++) {
            of << "," << p_v[i][j];
        }
        of << "\n";
    }

    of.close();
}

void write_waypoints_to_csv(std::string filepath, std::vector<Eigen::VectorXd> waypoints) {
    unsigned i;
    std::ofstream of(filepath);
    for (const Eigen::VectorXd& vec : waypoints) {
        i = 0;
        for (const auto& n : vec) {
            i++;
            if (i == vec.size()) {
                of << n;
            } else {
                of << n << ",";
            }
        }
        of << "\n";
    }
    of.close();
}

// global used to track if a trajectory is in progress
std::atomic<bool> trajectory_running(false);

// define callback function to be called by UR client library when program state changes
void reportRobotProgramState(bool program_running) {
    // Print the text in green so we see it better
    BOOST_LOG_TRIVIAL(info) << "\033[1;32mUR program running: " << std::boolalpha << program_running << "\033[0m";
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
    BOOST_LOG_TRIVIAL(info) << "\033[1;32mtrajectory report: " << report << "\033[0m";
}

UR5eArm::UR5eArm(Dependencies deps, const ResourceConfig& cfg) : Arm(cfg.name()) {
    BOOST_LOG_TRIVIAL(info) << "UR5eArm constructor start";
    this->reconfigure(deps, cfg);

    // get the APPDIR environment variable
    auto tmp = std::getenv("APPDIR");
    if (!tmp) {
        throw std::runtime_error("required environment variable APPDIR unset");
    }
    appdir = std::string(tmp);
    BOOST_LOG_TRIVIAL(info) << "appdir" << appdir;

    {
        std::lock_guard<std::mutex> guard{output_csv_dir_path_mu};
        if (output_csv_dir_path.empty()) {
            tmp = std::getenv("VIAM_MODULE_DATA");
            if (!tmp) {
                throw std::runtime_error("required environment variable VIAM_MODULE_DATA unset");
            }
            output_csv_dir_path = std::string(tmp);
            BOOST_LOG_TRIVIAL(info) << "VIAM_MODULE_DATA" << output_csv_dir_path;
        }
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
    urcl::UrDriverConfiguration ur_cfg = {host,
                                          appdir + SCRIPT_FILE,
                                          appdir + OUTPUT_RECIPE,
                                          appdir + INPUT_RECIPE,
                                          &reportRobotProgramState,
                                          true,  // headless mode
                                          nullptr};
    driver.reset(new UrDriver(ur_cfg));
    driver->registerTrajectoryDoneCallback(&reportTrajectoryState);

    // Once RTDE communication is started, we have to make sure to read from the interface buffer,
    // as otherwise we will get pipeline overflows. Therefore, do this directly before starting your
    // main loop
    driver->startRTDECommunication();
    int retry_count = 100;
    while (!read_joint_keep_alive(false)) {
        if (retry_count <= 0) {
            throw std::runtime_error("couldn't get joint positions; unable to establish communication with the arm");
        }
        retry_count--;
        usleep(NOOP_DELAY);
    }

    // start background thread to continuously send no-ops and keep socket connection alive
    BOOST_LOG_TRIVIAL(info) << "starting background_thread";
    keep_alive_thread_alive.store(true);
    std::thread keep_alive_thread(&UR5eArm::keep_alive, this);
    BOOST_LOG_TRIVIAL(info) << "UR5eArm constructor end";
    keep_alive_thread.detach();
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
    try {
        std::lock_guard<std::mutex> guard{output_csv_dir_path_mu};
        output_csv_dir_path = find_config_attribute<std::string>(cfg, "csv_output_path");
    } catch (...) {
    }
}

std::vector<double> UR5eArm::get_joint_positions(const ProtoStruct& extra) {
    std::lock_guard<std::mutex> guard{mu};
    if (!read_joint_keep_alive(true)) {
        return std::vector<double>();
    };
    std::vector<double> to_ret;
    for (double joint_pos_rad : joint_state) {
        double joint_pos_deg = 180.0 / M_PI * joint_pos_rad;
        to_ret.push_back(joint_pos_deg);
    }
    return to_ret;
}
std::chrono::milliseconds unix_now_ms() {
    namespace chrono = std::chrono;
    return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
}

std::string waypoints_filename(std::string path, unsigned long long unix_time_ms) {
    auto fmt = boost::format(path + WAYPOINTS_CSV_NAME_TEMPLATE);
    return (fmt % std::to_string(unix_time_ms)).str();
}

std::string trajectory_filename(std::string path, unsigned long long unix_time_ms) {
    auto fmt = boost::format(path + TRAJECTORY_CSV_NAME_TEMPLATE);
    return (fmt % std::to_string(unix_time_ms)).str();
}

std::string arm_joint_positions_filename(std::string path, unsigned long long unix_time_ms) {
    auto fmt = boost::format(path + ARM_JOINT_POSITIONS_CSV_NAME_TEMPLATE);
    return (fmt % std::to_string(unix_time_ms)).str();
}

std::string UR5eArm::get_output_csv_dir_path() {
    std::string path;
    {
        std::lock_guard<std::mutex> guard{output_csv_dir_path_mu};
        path = output_csv_dir_path;
    }
    return path;
}

void UR5eArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) {
    if (estop.load()) {
        throw std::runtime_error("move_to_joint_positions cancelled -> emergency stop is currently active");
    }

    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), positions.size());
    Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
    waypoints.push_back(next_waypoint_rad);
    std::chrono::milliseconds unix_time_ms = unix_now_ms();
    auto filename = waypoints_filename(get_output_csv_dir_path(), unix_time_ms.count());
    write_waypoints_to_csv(filename, waypoints);

    try {
        move(waypoints, unix_time_ms);
    } catch (const std::exception& ex) {
        BOOST_LOG_TRIVIAL(error) << "move failed. unix_time_ms " << unix_time_ms.count() << " Exception: " << std::string(ex.what());
        throw;
    }
}

void UR5eArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                           const MoveOptions& options,
                                           const viam::sdk::ProtoStruct& extra) {
    if (estop.load()) {
        throw std::runtime_error("move_through_joint_positions cancelled -> emergency stop is currently active");
    }
    // TODO: use options
    if (positions.size() > 0) {
        std::vector<Eigen::VectorXd> waypoints;
        for (auto position : positions) {
            Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(position.data(), position.size());
            Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
            waypoints.push_back(next_waypoint_rad);
        }
        std::chrono::milliseconds unix_time_ms = unix_now_ms();
        auto filename = waypoints_filename(get_output_csv_dir_path(), unix_time_ms.count());
        write_waypoints_to_csv(filename, waypoints);
        try {
            move(waypoints, unix_time_ms);
        } catch (const std::exception& ex) {
            BOOST_LOG_TRIVIAL(error) << "move failed. unix_time_ms " << unix_time_ms.count() << " Exception: " << std::string(ex.what());
            throw;
        }
    }
    return;
}

pose UR5eArm::get_end_position(const ProtoStruct& extra) {
    std::lock_guard<std::mutex> guard{mu};
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg == nullptr) {
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_end_position got nullptr from driver->getDataPackage()";
        return pose();
    }
    if (!data_pkg->getData("actual_TCP_pose", tcp_state)) {
        BOOST_LOG_TRIVIAL(warning) << "UR5eArm::get_end_position driver->getDataPackage().getData(\"actual_TCP_pos\") returned false";
        return pose();
    }
    return ur_vector_to_pose(tcp_state);
}

bool UR5eArm::is_moving() {
    return trajectory_running.load();
}

UR5eArm::KinematicsData UR5eArm::get_kinematics(const ProtoStruct& extra) {
    // Open the file in binary mode
    std::ifstream file(appdir + SVA_FILE, std::ios::binary);
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
            BOOST_LOG_TRIVIAL(warning) << "UR5eArm::stop driver->writeTrajectoryControlMessage returned false";
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
    BOOST_LOG_TRIVIAL(info) << "keep_alive thread started";
    while (true) {
        if (shutdown.load()) {
            break;
        }
        {
            std::lock_guard<std::mutex> guard{mu};
            try {
                read_joint_keep_alive(true);
            } catch (const std::exception& ex) {
                BOOST_LOG_TRIVIAL(error) << "keep_alive failed Exception: " << std::string(ex.what());
            }
        }
        usleep(NOOP_DELAY);
    }
    BOOST_LOG_TRIVIAL(info) << "keep_alive thread terminating";
    keep_alive_thread_alive.store(false);
}

void UR5eArm::move(std::vector<Eigen::VectorXd> waypoints, std::chrono::milliseconds unix_time_ms) {
    BOOST_LOG_TRIVIAL(info) << "move: start unix_time_ms " << unix_time_ms.count() << " waypoints size " << waypoints.size();

    // get current joint position and add that as starting pose to waypoints
    BOOST_LOG_TRIVIAL(info) << "move: get_joint_positions start " << unix_time_ms.count();
    std::vector<double> curr_joint_pos = get_joint_positions(ProtoStruct{});
    BOOST_LOG_TRIVIAL(info) << "move: get_joint_positions end" << unix_time_ms.count();

    BOOST_LOG_TRIVIAL(info) << "move: compute_trajectory start " << unix_time_ms.count();
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
    BOOST_LOG_TRIVIAL(info) << "generating trajectory with max speed: " << move_speed * (180.0 / M_PI);
    Eigen::VectorXd max_acceleration(6);
    Eigen::VectorXd max_velocity(6);
    max_acceleration << move_acceleration, move_acceleration, move_acceleration, move_acceleration, move_acceleration, move_acceleration;
    max_velocity << move_speed, move_speed, move_speed, move_speed, move_speed, move_speed;

    std::vector<vector6d_t> p;
    std::vector<vector6d_t> v;
    std::vector<float> time;

    for (size_t i = 0; i < segments.size() - 1; i++) {
        size_t start = segments[i];
        size_t end = segments[i + 1] + 1;
        std::list<Eigen::VectorXd> positions_subset(waypoints.begin() + start, waypoints.begin() + end);
        Trajectory trajectory(Path(positions_subset, 0.1), max_velocity, max_acceleration);
        trajectory.outputPhasePlaneTrajectory();
        if (!trajectory.isValid()) {
            std::stringstream buffer;
            buffer << "trajectory generation failed for path:";
            for (auto position : positions_subset) {
                buffer << "{";
                for (size_t j = 0; j < 6; j++) {
                    buffer << position[j] << " ";
                }
                buffer << "}";
            }
            throw std::runtime_error(buffer.str());
        }

        float duration = static_cast<float>(trajectory.getDuration());
        if (std::isinf(duration)) {
            throw std::runtime_error("trajectory.getDuration() was infinite");
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
            throw std::runtime_error("duration - (t - TIMESTEP) was infinite");
        }
        time.push_back(t2);
    }
    BOOST_LOG_TRIVIAL(info) << "move: compute_trajectory end " << unix_time_ms.count() << " p.count() " << p.size() << " v " << v.size()
                            << " time " << time.size();

    std::string path = get_output_csv_dir_path();
    write_trajectory_to_file(trajectory_filename(path, unix_time_ms.count()), p, v, time);
    {  // note the open brace which introduces a new variable scope
        // construct a lock_guard: locks the mutex on construction and unlocks on destruction
        std::lock_guard<std::mutex> guard{mu};
        std::stringstream pre_trajectory_state;
        {
            bool ok = false;
            unsigned long long now = 0;
            for (unsigned i = 0; i < 5; i++) {
                now = unix_now_ms().count();
                ok = read_joint_keep_alive(true);
                if (ok) {
                    break;
                }
            }
            if (!ok) {
                throw std::runtime_error("unable to get arm state before send_trajectory");
            }
            write_joint_pos_rad(joint_state, pre_trajectory_state, now, 0);
        }
        if (!send_trajectory(p, v, time)) {
            throw std::runtime_error("send_trajectory failed");
        };

        std::ofstream of(arm_joint_positions_filename(path, unix_time_ms.count()));

        of << "time_ms,read_attempt,joint_0_rad,joint_1_rad,joint_2_rad,joint_3_rad,joint_4_rad,joint_5_rad\n";
        of << pre_trajectory_state.str();
        unsigned attempt = 1;
        unsigned long long now = 0;
        while (trajectory_running.load() && !shutdown.load()) {
            now = unix_now_ms().count();
            read_joint_keep_alive(true);
            write_joint_pos_rad(joint_state, of, now, attempt);
            attempt++;
        };

        if (shutdown.load()) {
            of.close();
            throw std::runtime_error("interrupted by shutdown");
        }
        of.close();
    }

    BOOST_LOG_TRIVIAL(info) << "move: end unix_time_ms " << unix_time_ms.count();
}

// Define the destructor
UR5eArm::~UR5eArm() {
    BOOST_LOG_TRIVIAL(warning) << "UR5eArm destructor called";
    shutdown.store(true);
    // stop the robot
    BOOST_LOG_TRIVIAL(info) << "UR5eArm destructor calling stop";
    stop(ProtoStruct{});
    // disconnect from the dashboard
    if (dashboard) {
        BOOST_LOG_TRIVIAL(info) << "UR5eArm destructor calling dashboard->disconnect()";
        dashboard->disconnect();
    }
    BOOST_LOG_TRIVIAL(info) << "UR5eArm destructor waiting for keep_alive thread to terminate";
    while (keep_alive_thread_alive.load()) {
        BOOST_LOG_TRIVIAL(info) << "UR5eArm destructor still waiting for keep_alive thread to terminate";
        usleep(NOOP_DELAY);
    }
    BOOST_LOG_TRIVIAL(info) << "keep_alive thread terminated";
}

// helper function to send time-indexed position, velocity, acceleration setpoints to the UR driver
bool UR5eArm::send_trajectory(const std::vector<vector6d_t>& p_p, const std::vector<vector6d_t>& p_v, const std::vector<float>& time) {
    BOOST_LOG_TRIVIAL(info) << "UR5eArm::send_trajectory start";
    if (p_p.size() != time.size() || p_v.size() != time.size()) {
        BOOST_LOG_TRIVIAL(error) << "UR5eArm::send_trajectory p_p.size() != time.size() || p_v.size() != time.size(): not executing";
        return false;
    };
    auto point_number = static_cast<int>(p_p.size());
    if (!driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_START, point_number, RobotReceiveTimeout::off())) {
        BOOST_LOG_TRIVIAL(error) << "send_trajectory driver->writeTrajectoryControlMessage returned false";
        return false;
    };

    trajectory_running.store(true);
    BOOST_LOG_TRIVIAL(info) << "UR5eArm::send_trajectory sending " << p_p.size() << " cubic writeTrajectorySplinePoint/3";
    for (size_t i = 0; i < p_p.size(); i++) {
        if (!driver->writeTrajectorySplinePoint(p_p[i], p_v[i], time[i])) {
            BOOST_LOG_TRIVIAL(error) << "send_trajectory cubic driver->writeTrajectorySplinePoint returned false";
            return false;
        };
    }

    BOOST_LOG_TRIVIAL(info) << "UR5eArm::send_trajectory end";
    return true;
}

void write_joint_pos_rad(vector6d_t js, std::ostream& of, unsigned long long unix_now_ms, unsigned attempt) {
    of << unix_now_ms << "," << attempt << ",";
    unsigned i = 0;
    for (double joint_pos_rad : js) {
        i++;
        if (i == js.size()) {
            of << joint_pos_rad;
        } else {
            of << joint_pos_rad << ",";
        }
    }
    of << "\n";
}

// helper function to read a data packet and send a noop message
bool UR5eArm::read_joint_keep_alive(bool log) {
    // check to see if an estop has occurred.
    std::string status;

    if (!dashboard->commandSafetyStatus(status)) {
        // we currently do not attempt to reconnect to the dashboard client. hopefully this error resolves itself.
        BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive dashboard->commandSafetyStatus() returned false when retrieving the status";
        return false;
    }

    if (status.find(urcl::safetyStatusString(urcl::SafetyStatus::NORMAL)) == std::string::npos) {
        // the arm is currently estopped. save this state.
        estop.store(true);

        // clear any currently running trajectory.
        trajectory_running.store(false);

        // sleep longer to prevent buffer error
        // Removing this will cause the RTDE client to move into an unrecoverable state
        usleep(ESTOP_DELAY);

    } else {
        // the arm is in a normal state.
        if (estop.load()) {
            // if the arm was previously estopped, attempt to recover from the estop.
            // We should not enter this code without the user interacting with the arm in some way(i.e. resetting the estop)
            try {
                BOOST_LOG_TRIVIAL(info) << "recovering from e-stop";
                driver->resetRTDEClient(appdir + OUTPUT_RECIPE, appdir + INPUT_RECIPE);

                if (!dashboard->commandPowerOn()) {
                    BOOST_LOG_TRIVIAL(error)
                        << "read_joint_keep_alive dashboard->commandPowerOn() returned false when attempting to restart the arm";
                    return false;
                }
                // Release the brakes
                if (!dashboard->commandBrakeRelease()) {
                    BOOST_LOG_TRIVIAL(error)
                        << "read_joint_keep_alive dashboard->commandBrakeRelease() returned false when attempting to restart the arm";
                    return false;
                }

                // send control script again to complete the restart
                if (!driver->sendRobotProgram()) {
                    BOOST_LOG_TRIVIAL(error)
                        << "read_joint_keep_alive driver->sendRobotProgram() returned false when attempting to restart the arm";
                    return false;
                }

            } catch (const std::exception& ex) {
                BOOST_LOG_TRIVIAL(info) << "failed to restart the arm: : " << std::string(ex.what());
                return false;
            }

            BOOST_LOG_TRIVIAL(info) << "send robot program successful, restarting communication";
            driver->startRTDECommunication();

            estop.store(false);
            BOOST_LOG_TRIVIAL(info) << "arm successfully recovered from estop";
            return true;
        }
    }

    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg == nullptr) {
        // we received no data packet, so our comms are down. reset the comms from the driver.
        if (log) {
            BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver->getDataPackage() returned nullptr. resetting RTDE client connection";
        }
        try {
            driver->resetRTDEClient(appdir + OUTPUT_RECIPE, appdir + INPUT_RECIPE);
        } catch (const std::exception& ex) {
            if (log) {
                BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver RTDEClient failed to restart: " << std::string(ex.what());
            }
            return false;
        }
        driver->startRTDECommunication();
        if (log) {
            BOOST_LOG_TRIVIAL(info) << "RTDE client connection successfully restarted";
        }
        return false;
    }

    // read current joint positions from robot data
    if (!data_pkg->getData("actual_q", joint_state)) {
        if (log) {
            BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver->getDataPackage()->data_pkg->getData(\"actual_q\") returned false";
        }
        return false;
    }

    // send a noop to keep the connection alive
    if (!driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off())) {
        if (log) {
            BOOST_LOG_TRIVIAL(error) << "read_joint_keep_alive driver->writeTrajectoryControlMessage returned false";
        }
        return false;
    }
    return true;
}
