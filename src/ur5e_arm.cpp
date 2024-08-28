#include "ur5e_arm.hpp"

UR5eArm::UR5eArm(Dependencies dep, ResourceConfig cfg) : Arm(cfg.name()), calling_trajectory(false) {
    if(!initialize())
        throw std::runtime_error("initialization failed\n");

    // start background thread to continuously send no-ops and keep socket connection alive
    std::thread t(&UR5eArm::keepAlive, this);
    t.detach();
}

std::vector<double> UR5eArm::get_joint_positions(const AttributeMap& extra) {
    mu.lock();
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    if (data_pkg && data_pkg->getData("actual_q", g_joint_positions)){
        std::vector<double> to_ret;
        for(double joint_pos_rad: g_joint_positions){
            double joint_pos_deg = 180.0/M_PI * joint_pos_rad;
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
    Eigen::VectorXd next_waypoint_rad = next_waypoint_deg * (M_PI/180.0); // convert from radians to degrees
    waypoints.push_back(next_waypoint_rad);
    move(waypoints);
}

bool UR5eArm::is_moving() {
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
    vector6d_t joint_velocities;
    if (data_pkg && data_pkg->getData("actual_q", joint_velocities)){
        for(double joint_vel : joint_velocities){
            if (joint_vel > STOP_VELOCITY_THRESHOLD){
                return false;
            }
        }
        return true;
    }
    throw std::runtime_error("couldn't get velocities from arm");
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
    if (!dashboard->commandStop())
        throw std::runtime_error("UNABLE TO STOP!\n");
}

AttributeMap UR5eArm::do_command(const AttributeMap& command) {
    if(!command){
        throw std::runtime_error("command is null\n");
    }
    
    // parse waypoints
    std::vector<Eigen::VectorXd> waypoints;
    if(command->count(POSITIONS_KEY) > 0){
        std::vector<std::shared_ptr<ProtoType>>* vec_protos = (*command)[POSITIONS_KEY]->get<std::vector<std::shared_ptr<ProtoType>>>();
        if (vec_protos){
            for(std::shared_ptr<ProtoType>& vec_proto: *vec_protos) {
                std::vector<std::shared_ptr<ProtoType>>* vec_joint_pos = vec_proto->get<std::vector<std::shared_ptr<ProtoType>>>();
                if (vec_joint_pos){
                    vector6d_t to_add;
                    int counter = 0;
                    for(std::shared_ptr<ProtoType> joint_pos: *vec_joint_pos){
                        double* joint_pos_deg = joint_pos->get<double>();
                        to_add[counter] = *joint_pos_deg;
                        counter += 1;
                    }
                    Eigen::VectorXd waypoint = Eigen::VectorXd::Map(to_add.data(), to_add.size());
                    waypoints.push_back(waypoint);
                }
            }
        }
    }

    move(waypoints);
    return NULL;
}

bool g_trajectory_running(false);
void handleRobotProgramState(bool program_running) {
    // Print the text in green so we see it better
    std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

// Callback function for trajectory execution.
void handleTrajectoryState(control::TrajectoryResult state) {
    // trajectory_state = state;
    g_trajectory_running = false;
    std::string report = "?";
    switch (state)
    {
    case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
        report = "success";
        break;
    case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
        report = "canceled";
        break;
    case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
    default:
        report = "failure";
        break;
    }
    std::cout << "\033[1;32mTrajectory report: " << report << "\033[0m\n" << std::endl;
}

bool UR5eArm::initialize() {
    // Making the robot ready for the program by:
    // Connect to the robot Dashboard
    dashboard.reset(new DashboardClient(DEFAULT_ROBOT_IP));
    if (!dashboard->connect())
        return false;
    
    // Stop program, if there is one running
    if (!dashboard->commandStop())
        return false;

    // if the robot is not powered on and ready
    std::string robotModeRunning("RUNNING");
    while (!dashboard->commandRobotMode(robotModeRunning))
    {
        // Power it off
        if (!dashboard->commandPowerOff())
            return false;

        // Power it on
        if (!dashboard->commandPowerOn())
            return false;
    }

    // Release the brakes
    if (!dashboard->commandBrakeRelease())
        return false;

    // Now the robot is ready to receive a program
    std::unique_ptr<ToolCommSetup> tool_comm_setup;
    const bool HEADLESS = true;
    driver.reset(new UrDriver(DEFAULT_ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, HEADLESS,
                                    std::move(tool_comm_setup), CALIBRATION_CHECKSUM));

    driver->registerTrajectoryDoneCallback(&handleTrajectoryState);
    
    // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
    // otherwise we will get pipeline overflows. Therefore, do this directly before starting your main
    // loop.
    driver->startRTDECommunication();
    std::cout << "started RTDE comms" << std::endl;

    std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();

    if (data_pkg)
    {
        // Read current joint positions from robot data
        if (!data_pkg->getData("actual_q", g_joint_positions))
            return false;
    }

    return true;
}

// Send no-ops and keep socket connection alive
void UR5eArm::keepAlive() {
    while(true){
        mu.lock();
        std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
        if (data_pkg)
        {
            // Read current joint positions from robot data
            if (!data_pkg->getData("actual_q", g_joint_positions))
            {
                // This throwing should never happen unless misconfigured
                std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
                throw std::runtime_error(error_msg);
            }
            driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP);
        }
        mu.unlock();
        usleep(NOOP_DELAY);
    }
}

void UR5eArm::move(std::vector<Eigen::VectorXd> waypoints) {
    // get current joint position and add that as starting pose to waypoints
    std::vector<double> curr_joint_pos = get_joint_positions(NULL);
    Eigen::VectorXd curr_waypoint_deg = Eigen::VectorXd::Map(curr_joint_pos.data(), curr_joint_pos.size());
    Eigen::VectorXd curr_waypoint_rad = curr_waypoint_deg * (M_PI/180.0);
    waypoints.insert(waypoints.begin(), curr_waypoint_rad);

    // calculate dot products and identify any consecutive segments with dot product == -1
    std::vector<int> segments;
    segments.push_back(0);
    for(int i = 2; i < waypoints.size(); i++){
        Eigen::VectorXd segment_AB = (waypoints[i-1] - waypoints[i-2]);
        Eigen::VectorXd segment_BC = (waypoints[i] - waypoints[i-1]);
        segment_AB.normalize();
        segment_BC.normalize();
        double dot = segment_BC.dot(segment_AB);
        if(dot == -1){
            segments.push_back(i-1);
        } 
    }
    segments.push_back(waypoints.size()-1);

    // set velocity/acceleration constraints
    Eigen::VectorXd maxAcceleration(6);
	Eigen::VectorXd maxVelocity(6);
    maxAcceleration << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    maxVelocity << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;

    std::vector<vector6d_t> p;
    std::vector<vector6d_t> v;
    std::vector<vector6d_t> a;
    std::vector<double> time;

    for(int i = 0; i < segments.size()-1; i++){
        int start = segments[i];
        int end = segments[i+1]+1;
        std::list<Eigen::VectorXd> positions_subset(waypoints.begin() + start, waypoints.begin() + end);
        Trajectory trajectory(Path(positions_subset, 0.1), maxVelocity, maxAcceleration);
        trajectory.outputPhasePlaneTrajectory();
        if(trajectory.isValid()) {
            double duration = trajectory.getDuration();
            for(double t = 0.0; t < duration; t += TIMESTEP) {
                Eigen::VectorXd position = trajectory.getPosition(t);
                Eigen::VectorXd velocity = trajectory.getVelocity(t);
                p.push_back(vector6d_t{position[0], position[1], position[2], position[3], position[4], position[5]});
                v.push_back(vector6d_t{velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]});
                time.push_back(TIMESTEP);
            }
        }
        else {
            throw std::runtime_error("trajectory generation failed\n");
        }
    }
    
    mu.lock();
    SendTrajectory(p, v, a, time, true);
    g_trajectory_running = true;
    while (g_trajectory_running)
    {
        std::unique_ptr<rtde_interface::DataPackage> data_pkg = driver->getDataPackage();
        if (data_pkg)
        {
            // Read current joint positions from robot data
            if (!data_pkg->getData("actual_q", g_joint_positions))
            {
                // This throwing should never happen unless misconfigured
                std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
                throw std::runtime_error(error_msg);
            }
            driver->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, 0, RobotReceiveTimeout::off());
        }
    }
    mu.unlock();
}

// function to send trajectories to the UR driver
void UR5eArm::SendTrajectory(
    const std::vector<vector6d_t>& p_p, 
    const std::vector<vector6d_t>& p_v,
    const std::vector<vector6d_t>& p_a, 
    const std::vector<double>& time, 
    bool use_spline_interpolation_
){
  assert(p_p.size() == time.size());
  
  driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, p_p.size(), RobotReceiveTimeout::off());
  for (size_t i = 0; i < p_p.size() && p_p.size() == time.size() && p_p[i].size() == 6; i++)
  {
    // MoveJ
    if (!use_spline_interpolation_)
    {
      driver->writeTrajectoryPoint(p_p[i], false, time[i], 0.0);
    }
    else  // Use spline interpolation
    {
      // QUINTIC
      if (p_v.size() == time.size() && p_a.size() == time.size() && p_v[i].size() == 6 && p_a[i].size() == 6){
        driver->writeTrajectorySplinePoint(p_p[i], p_v[i], p_a[i], time[i]);
      }
      // CUBIC
      else if (p_v.size() == time.size() && p_v[i].size() == 6){
        driver->writeTrajectorySplinePoint(p_p[i], p_v[i], time[i]);
      }
      else{
        driver->writeTrajectorySplinePoint(p_p[i], time[i]);
      }
    }
  }
  std::cout << "finshed sending trajectory" << std::endl;
}


