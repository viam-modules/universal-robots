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
        VIAM_SDK_LOG(info) << "write_trajectory_to_file called with invalid parameters";
        return;
    }
    std::ofstream of(filepath);
    of << "t(s),j0,j1,j2,j3,j4,j5,v0,v1,v2,v3,v4,v5\n";
    float time_traj = 0;
    for (size_t i = 0; i < p_p.size(); i++) {
        time_traj += time[i];
        of << time_traj;
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
    VIAM_SDK_LOG(info) << "\033[1;32mUR program running: " << std::boolalpha << program_running << "\033[0m";
}

UR5eArm::UR5eArm(Dependencies deps, const ResourceConfig& cfg) : Arm(cfg.name()) {
    VIAM_SDK_LOG(info) << "UR5eArm constructor start";
    current_state_ = std::make_unique<state_>();
    this->reconfigure(deps, cfg);

    // get the APPDIR environment variable
    auto tmp = std::getenv("APPDIR");
    if (!tmp) {
        throw std::runtime_error("required environment variable APPDIR unset");
    }
    current_state_->appdir = std::string(tmp);
    VIAM_SDK_LOG(info) << "appdir" << current_state_->appdir;

    {
        std::lock_guard<std::mutex> guard{current_state_->output_csv_dir_path_mu};
        if (current_state_->output_csv_dir_path.empty()) {
            tmp = std::getenv("VIAM_MODULE_DATA");
            if (!tmp) {
                throw std::runtime_error("required environment variable VIAM_MODULE_DATA unset");
            }
            current_state_->output_csv_dir_path = std::string(tmp);
            VIAM_SDK_LOG(info) << "VIAM_MODULE_DATA" << current_state_->output_csv_dir_path;
        }
    }

    // connect to the robot dashboard
    current_state_->dashboard.reset(new DashboardClient(current_state_->host));
    if (!current_state_->dashboard->connect()) {
        throw std::runtime_error("couldn't connect to dashboard");
    }

    // stop program, if there is one running
    if (!current_state_->dashboard->commandStop()) {
        throw std::runtime_error("couldn't stop program running on dashboard");
    }

    // if the robot is not powered on and ready
    std::string robotModeRunning("RUNNING");
    while (!current_state_->dashboard->commandRobotMode(robotModeRunning)) {
        // power cycle the arm
        if (!current_state_->dashboard->commandPowerOff()) {
            throw std::runtime_error("couldn't power off arm");
        }
        if (!current_state_->dashboard->commandPowerOn()) {
            throw std::runtime_error("couldn't power on arm");
        }
    }

    // Release the brakes
    if (!current_state_->dashboard->commandBrakeRelease()) {
        throw std::runtime_error("couldn't release the arm brakes");
    }

    // Now the robot is ready to receive a program
    urcl::UrDriverConfiguration ur_cfg = {current_state_->host,
                                          current_state_->appdir + SCRIPT_FILE,
                                          current_state_->appdir + OUTPUT_RECIPE,
                                          current_state_->appdir + INPUT_RECIPE,
                                          &reportRobotProgramState,
                                          true,  // headless mode
                                          nullptr};
    current_state_->driver.reset(new UrDriver(ur_cfg));

    // define callback function to be called by UR client library when trajectory state changes
    current_state_->driver->registerTrajectoryDoneCallback([](const control::TrajectoryResult state) {
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
        VIAM_SDK_LOG(info) << "\033[1;32mtrajectory report: " << report << "\033[0m";
    });

    // Once RTDE communication is started, we have to make sure to read from the interface buffer,
    // as otherwise we will get pipeline overflows. Therefore, do this directly before starting your
    // main loop
    current_state_->driver->startRTDECommunication();
    int retry_count = 100;
    while (read_joint_keep_alive(false) != UrDriverStatus::NORMAL) {
        if (retry_count <= 0) {
            throw std::runtime_error("couldn't get joint positions; unable to establish communication with the arm");
        }
        retry_count--;
        usleep(NOOP_DELAY);
    }

    // start background thread to continuously send no-ops and keep socket connection alive
    VIAM_SDK_LOG(info) << "starting background_thread";
    current_state_->keep_alive_thread_alive.store(true);
    std::thread keep_alive_thread(&UR5eArm::keep_alive, this);
    VIAM_SDK_LOG(info) << "UR5eArm constructor end";
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
    current_state_->host = find_config_attribute<std::string>(cfg, "host");
    current_state_->speed.store(find_config_attribute<double>(cfg, "speed_degs_per_sec") * (M_PI / 180.0));
    current_state_->acceleration.store(find_config_attribute<double>(cfg, "acceleration_degs_per_sec2") * (M_PI / 180.0));
    try {
        std::lock_guard<std::mutex> guard{current_state_->output_csv_dir_path_mu};
        current_state_->output_csv_dir_path = find_config_attribute<std::string>(cfg, "csv_output_path");
    } catch (...) {
    }
}

std::vector<double> UR5eArm::get_joint_positions(const ProtoStruct& extra) {
    std::lock_guard<std::mutex> guard{current_state_->mu};
    if (read_joint_keep_alive(true) == UrDriverStatus::READ_FAILURE) {
        throw std::runtime_error("failed to read from arm");
    };
    std::vector<double> to_ret;
    for (double joint_pos_rad : current_state_->joint_state) {
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
        std::lock_guard<std::mutex> guard{current_state_->output_csv_dir_path_mu};
        path = current_state_->output_csv_dir_path;
    }
    return path;
}

void UR5eArm::move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) {
    if (current_state_->estop.load()) {
        throw std::runtime_error("move_to_joint_positions cancelled -> emergency stop is currently active");
    }

    std::vector<Eigen::VectorXd> waypoints;
    Eigen::VectorXd next_waypoint_deg = Eigen::VectorXd::Map(positions.data(), positions.size());
    Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI / 180.0);  // convert from radians to degrees
    waypoints.push_back(next_waypoint_rad);
    std::chrono::milliseconds unix_time_ms = unix_now_ms();
    auto filename = waypoints_filename(get_output_csv_dir_path(), unix_time_ms.count());
    write_waypoints_to_csv(filename, waypoints);

    // move will throw if an error occurs
    move(waypoints, unix_time_ms);
}

void UR5eArm::move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                           const MoveOptions& options,
                                           const viam::sdk::ProtoStruct& extra) {
    if (current_state_->estop.load()) {
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

        // move will throw if an error occurs
        move(waypoints, unix_time_ms);
    }
    return;
}

pose UR5eArm::get_end_position(const ProtoStruct& extra) {
    std::lock_guard<std::mutex> guard{current_state_->mu};
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = current_state_->driver->getDataPackage();
    if (data_pkg == nullptr) {
        VIAM_SDK_LOG(warn) << "UR5eArm::get_end_position got nullptr from driver->getDataPackage()";
        return pose();
    }
    if (!data_pkg->getData("actual_TCP_pose", current_state_->tcp_state)) {
        VIAM_SDK_LOG(warn) << "UR5eArm::get_end_position driver->getDataPackage().getData(\"actual_TCP_pos\") returned false";
        return pose();
    }
    return ur_vector_to_pose(current_state_->tcp_state);
}

bool UR5eArm::is_moving() {
    return trajectory_running.load();
}

UR5eArm::KinematicsData UR5eArm::get_kinematics(const ProtoStruct& extra) {
    // Open the file in binary mode
    std::ifstream file(current_state_->appdir + SVA_FILE, std::ios::binary);
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
        bool ok = current_state_->driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0, RobotReceiveTimeout::off());
        if (!ok) {
            VIAM_SDK_LOG(warn) << "UR5eArm::stop driver->writeTrajectoryControlMessage returned false";
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
            current_state_->speed.store(val * (M_PI / 180.0));
            resp.emplace(VEL_KEY, val);
        }
        if (kv.first == ACC_KEY) {
            const double val = *kv.second.get<double>();
            current_state_->acceleration.store(val * (M_PI / 180.0));
            resp.emplace(ACC_KEY, val);
        }
    }

    return resp;
}

// Send no-ops and keep socket connection alive
void UR5eArm::keep_alive() {
    VIAM_SDK_LOG(info) << "keep_alive thread started";
    while (true) {
        if (current_state_->shutdown.load()) {
            break;
        }
        {
            std::lock_guard<std::mutex> guard{current_state_->mu};
            try {
                read_joint_keep_alive(true);
            } catch (const std::exception& ex) {
                VIAM_SDK_LOG(error) << "keep_alive failed Exception: " << std::string(ex.what());
            }
        }
        usleep(NOOP_DELAY);
    }
    VIAM_SDK_LOG(info) << "keep_alive thread terminating";
    current_state_->keep_alive_thread_alive.store(false);
}

void UR5eArm::move(std::vector<Eigen::VectorXd> waypoints, std::chrono::milliseconds unix_time_ms) {
    VIAM_SDK_LOG(info) << "move: start unix_time_ms " << unix_time_ms.count() << " waypoints size " << waypoints.size();

    // get current joint position and add that as starting pose to waypoints
    VIAM_SDK_LOG(info) << "move: get_joint_positions start " << unix_time_ms.count();
    std::vector<double> curr_joint_pos = get_joint_positions(ProtoStruct{});
    VIAM_SDK_LOG(info) << "move: get_joint_positions end" << unix_time_ms.count();

    VIAM_SDK_LOG(info) << "move: compute_trajectory start " << unix_time_ms.count();
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
    const double move_speed = current_state_->speed.load();
    const double move_acceleration = current_state_->acceleration.load();
    VIAM_SDK_LOG(info) << "generating trajectory with max speed: " << move_speed * (180.0 / M_PI);
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
    VIAM_SDK_LOG(info) << "move: compute_trajectory end " << unix_time_ms.count() << " p.count() " << p.size() << " v " << v.size()
                       << " time " << time.size();

    std::string path = get_output_csv_dir_path();
    write_trajectory_to_file(trajectory_filename(path, unix_time_ms.count()), p, v, time);
    {  // note the open brace which introduces a new variable scope
        // construct a lock_guard: locks the mutex on construction and unlocks on destruction
        std::lock_guard<std::mutex> guard{current_state_->mu};
        std::stringstream pre_trajectory_state;
        {
            UrDriverStatus status;
            unsigned long long now = 0;
            for (unsigned i = 0; i < 5; i++) {
                now = unix_now_ms().count();
                status = read_joint_keep_alive(true);
                if (status == UrDriverStatus::NORMAL) {
                    break;
                }
            }
            if (status != UrDriverStatus::NORMAL) {
                throw std::runtime_error("unable to get arm state before send_trajectory");
            }
            write_joint_pos_rad(current_state_->joint_state, pre_trajectory_state, now, 0);
        }
        if (!send_trajectory(p, v, time)) {
            throw std::runtime_error("send_trajectory failed");
        };

        std::ofstream of(arm_joint_positions_filename(path, unix_time_ms.count()));

        of << "time_ms,read_attempt,joint_0_rad,joint_1_rad,joint_2_rad,joint_3_rad,joint_4_rad,joint_5_rad\n";
        of << pre_trajectory_state.str();
        unsigned attempt = 1;
        unsigned long long now = 0;
        UrDriverStatus status;
        while (trajectory_running.load() && !current_state_->shutdown.load()) {
            now = unix_now_ms().count();
            status = read_joint_keep_alive(true);
            if (status != UrDriverStatus::NORMAL) {
                trajectory_running.store(false);
                break;
            }
            write_joint_pos_rad(current_state_->joint_state, of, now, attempt);
            attempt++;
        };
        if (current_state_->shutdown.load()) {
            of.close();
            throw std::runtime_error("interrupted by shutdown");
        }

        of.close();
        VIAM_SDK_LOG(info) << "move: end unix_time_ms " << unix_time_ms.count();

        switch (status) {
            case UrDriverStatus::ESTOPPED:
                throw std::runtime_error("arm is estopped");
            case UrDriverStatus::READ_FAILURE:
                throw std::runtime_error("arm failed to retrieve data packet");
            case UrDriverStatus::DASHBOARD_FAILURE:
                throw std::runtime_error("arm dashboard is disconnected");
            case UrDriverStatus::NORMAL:
                return;
        }
    }
}

std::string UR5eArm::status_to_string(UrDriverStatus status) {
    switch (status) {
        case UrDriverStatus::ESTOPPED:
            return "ESTOPPED";
        case UrDriverStatus::READ_FAILURE:
            return "READ_FAILURE";
        case UrDriverStatus::DASHBOARD_FAILURE:
            return "DASHBOARD_FAILURE";
        case UrDriverStatus::NORMAL:
            return "NORMAL";
    }
    return "no status";
}

// Define the destructor
UR5eArm::~UR5eArm() {
    VIAM_SDK_LOG(warn) << "UR5eArm destructor called";
    current_state_->shutdown.store(true);
    // stop the robot
    VIAM_SDK_LOG(info) << "UR5eArm destructor calling stop";
    stop(ProtoStruct{});
    // disconnect from the dashboard
    if (current_state_->dashboard) {
        VIAM_SDK_LOG(info) << "UR5eArm destructor calling dashboard->disconnect()";
        current_state_->dashboard->disconnect();
    }
    VIAM_SDK_LOG(info) << "UR5eArm destructor waiting for keep_alive thread to terminate";
    while (current_state_->keep_alive_thread_alive.load()) {
        VIAM_SDK_LOG(info) << "UR5eArm destructor still waiting for keep_alive thread to terminate";
        usleep(NOOP_DELAY);
    }
    VIAM_SDK_LOG(info) << "keep_alive thread terminated";
}

// helper function to send time-indexed position, velocity, acceleration setpoints to the UR driver
bool UR5eArm::send_trajectory(const std::vector<vector6d_t>& p_p, const std::vector<vector6d_t>& p_v, const std::vector<float>& time) {
    VIAM_SDK_LOG(info) << "UR5eArm::send_trajectory start";
    if (p_p.size() != time.size() || p_v.size() != time.size()) {
        VIAM_SDK_LOG(error) << "UR5eArm::send_trajectory p_p.size() != time.size() || p_v.size() != time.size(): not executing";
        return false;
    };
    auto point_number = static_cast<int>(p_p.size());
    if (!current_state_->driver->writeTrajectoryControlMessage(
            urcl::control::TrajectoryControlMessage::TRAJECTORY_START, point_number, RobotReceiveTimeout::off())) {
        VIAM_SDK_LOG(error) << "send_trajectory driver->writeTrajectoryControlMessage returned false";
        return false;
    };

    trajectory_running.store(true);
    VIAM_SDK_LOG(info) << "UR5eArm::send_trajectory sending " << p_p.size() << " cubic writeTrajectorySplinePoint/3";
    for (size_t i = 0; i < p_p.size(); i++) {
        if (!current_state_->driver->writeTrajectorySplinePoint(p_p[i], p_v[i], time[i])) {
            VIAM_SDK_LOG(error) << "send_trajectory cubic driver->writeTrajectorySplinePoint returned false";
            return false;
        };
    }

    VIAM_SDK_LOG(info) << "UR5eArm::send_trajectory end";
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
UR5eArm::UrDriverStatus UR5eArm::read_joint_keep_alive(bool log) {
    // check to see if an estop has occurred.
    std::string status;

    try {
        if (!current_state_->dashboard->commandSafetyStatus(status)) {
            // we currently do not attempt to reconnect to the dashboard client. hopefully this error resolves itself.
            VIAM_SDK_LOG(error) << "read_joint_keep_alive dashboard->commandSafetyStatus() returned false when retrieving the status";
            return UrDriverStatus::DASHBOARD_FAILURE;
        }
    } catch (const std::exception& ex) {
        VIAM_SDK_LOG(error) << "failed to talk to the arm, is the tablet in local mode? : " << std::string(ex.what());
        return UrDriverStatus::DASHBOARD_FAILURE;
    }

    if (status.find(urcl::safetyStatusString(urcl::SafetyStatus::NORMAL)) == std::string::npos) {
        // the arm is currently estopped. save this state.
        current_state_->estop.store(true);

        // clear any currently running trajectory.
        trajectory_running.store(false);

        // TODO: further investigate the need for this delay
        // sleep longer to prevent buffer error
        // Removing this will cause the RTDE client to move into an unrecoverable state
        usleep(ESTOP_DELAY);

    } else {
        // the arm is in a normal state.
        if (current_state_->estop.load()) {
            // if the arm was previously estopped, attempt to recover from the estop.
            // We should not enter this code without the user interacting with the arm in some way(i.e. resetting the estop)
            try {
                VIAM_SDK_LOG(info) << "recovering from e-stop";
                current_state_->driver->resetRTDEClient(current_state_->appdir + OUTPUT_RECIPE, current_state_->appdir + INPUT_RECIPE);

                VIAM_SDK_LOG(info) << "restarting arm";
                if (!current_state_->dashboard->commandPowerOff()) {
                    std::runtime_error(
                        "read_joint_keep_alive dashboard->commandPowerOff() returned false when attempting to restart the arm");
                }

                if (!current_state_->dashboard->commandPowerOn()) {
                    std::runtime_error(
                        "read_joint_keep_alive dashboard->commandPowerOn() returned false when attempting to restart the arm");
                }
                // Release the brakes
                if (!current_state_->dashboard->commandBrakeRelease()) {
                    std::runtime_error(
                        "read_joint_keep_alive dashboard->commandBrakeRelease() returned false when attempting to restart the arm");
                }

                // send control script again to complete the restart
                if (!current_state_->driver->sendRobotProgram()) {
                    std::runtime_error(
                        "read_joint_keep_alive driver->sendRobotProgram() returned false when attempting to restart the arm");
                }

            } catch (const std::exception& ex) {
                VIAM_SDK_LOG(info) << "failed to restart the arm: : " << std::string(ex.what());
                return UrDriverStatus::ESTOPPED;
            }

            VIAM_SDK_LOG(info) << "send robot program successful, restarting communication";
            current_state_->driver->startRTDECommunication();

            current_state_->estop.store(false);
            VIAM_SDK_LOG(info) << "arm successfully recovered from estop";
            return UrDriverStatus::NORMAL;
        }
    }

    std::unique_ptr<rtde_interface::DataPackage> data_pkg = current_state_->driver->getDataPackage();
    if (data_pkg == nullptr) {
        // we received no data packet, so our comms are down. reset the comms from the driver.
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->getDataPackage() returned nullptr. resetting RTDE client connection";
        }
        try {
            current_state_->driver->resetRTDEClient(current_state_->appdir + OUTPUT_RECIPE, current_state_->appdir + INPUT_RECIPE);
        } catch (const std::exception& ex) {
            if (log) {
                VIAM_SDK_LOG(error) << "read_joint_keep_alive driver RTDEClient failed to restart: " << std::string(ex.what());
            }
            return UrDriverStatus::READ_FAILURE;
        }
        current_state_->driver->startRTDECommunication();
        if (log) {
            VIAM_SDK_LOG(info) << "RTDE client connection successfully restarted";
        }
        // we restarted the RTDE client so the robot should be back to a normal state
        return UrDriverStatus::NORMAL;
    }

    // read current joint positions from robot data
    if (!data_pkg->getData("actual_q", current_state_->joint_state)) {
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->getDataPackage()->data_pkg->getData(\"actual_q\") returned false";
        }
        return UrDriverStatus::READ_FAILURE;
    }

    // send a noop to keep the connection alive
    if (!current_state_->driver->writeTrajectoryControlMessage(
            control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off())) {
        if (log) {
            VIAM_SDK_LOG(error) << "read_joint_keep_alive driver->writeTrajectoryControlMessage returned false";
        }
        return UrDriverStatus::READ_FAILURE;
    }

    // check if we detect an estop. while estopped we could still retrieve data from the arm
    if (current_state_->estop.load()) {
        return UrDriverStatus::ESTOPPED;
    }
    return UrDriverStatus::NORMAL;
}
