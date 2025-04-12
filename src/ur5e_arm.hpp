#pragma once

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/types.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>

#include <boost/log/trivial.hpp>
#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/resource/resource.hpp>

#include "../trajectories/Path.h"
#include "../trajectories/Trajectory.h"

using namespace viam::sdk;
using namespace urcl;

// locations of files necessary to build module, specified as relative paths
const std::string SVA_FILE = "/src/kinematics/ur5e.json";
const std::string SCRIPT_FILE = "/src/control/external_control.urscript";
const std::string OUTPUT_RECIPE = "/src/control/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "/src/control/rtde_input_recipe.txt";

// locations of log files that will be written
const std::string TRAJECTORY_LOG = "/logs/trajectory.csv";
const std::string WAYPOINTS_LOG = "/logs/waypoints.csv";

// TODO: using this is deprecated by the URCL, we could find some way around using it
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

// constants for robot operation
const int NOOP_DELAY = 100000;  // 100 milliseconds
const double TIMESTEP = 0.2;    // seconds

class UR5eArm : public Arm, public Reconfigurable {
   public:
    UR5eArm(Dependencies deps, const ResourceConfig& cfg);
    ~UR5eArm() override;    

    void reconfigure(const Dependencies& deps, const ResourceConfig& cfg) override;

    /// @brief Get the joint positions of the arm (in degrees)
    /// @param extra Any additional arguments to the method.
    /// @return a vector of joint positions of the arm in degrees
    std::vector<double> get_joint_positions(const ProtoStruct& extra) override;

    /// @brief Move to the the specified joint positions (in degrees)
    /// @param positions The joint positions in degrees to move to
    /// @param extra Any additional arguments to the method.
    void move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) override;

    /// @brief Move through the specified joint positions (in degrees)
    /// @param positions The joint positions to move through
    /// @param options Optional parameters that should be obeyed during the motion
    /// @param extra Any additional arguments to the method.
    void move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                      const MoveOptions& options,
                                      const viam::sdk::ProtoStruct& extra) override;

    /// @brief Get the cartesian pose of the end effector
    /// @param extra Any additional arguments to the method.
    /// @return Pose of the end effector with respect to the arm base.
    pose get_end_position(const ProtoStruct& extra) override;

    /// @brief Reports if the arm is in motion.
    bool is_moving() override;

    /// @brief Get the kinematics data associated with the arm.
    /// @param extra Any additional arguments to the method.
    /// @return A variant of kinematics data, with bytes field containing the raw bytes of the file
    /// and the object's type indicating the file format.
    KinematicsData get_kinematics(const ProtoStruct& extra) override;

    /// @brief Stops the Arm.
    /// @param extra Extra arguments to pass to the resource's `stop` method.
    void stop(const ProtoStruct& extra) override;

    /// @brief This is being used as a proxy to move_to_joint_positions except with support for
    /// multiple waypoints
    /// @param command Will contain a std::vector<std::vector<double>> called positions that will
    /// contain joint waypoints
    ProtoStruct do_command(const ProtoStruct& command) override;

    // --------------- UNIMPLEMENTED FUNCTIONS ---------------
    void move_to_position(const pose& pose, const ProtoStruct& extra) override {
        throw std::runtime_error("unimplemented");
    }

    // the arm server within RDK will reconstruct the geometries from the kinematics and joint positions if left unimplemented
    std::vector<GeometryConfig> get_geometries(const ProtoStruct& extra) {
        throw std::runtime_error("unimplemented");
    }

   private:
    void keep_alive();
    void move(std::vector<Eigen::VectorXd> waypoints);
    void send_trajectory(const std::vector<vector6d_t>& p_p,
                         const std::vector<vector6d_t>& p_v,
                         const std::vector<vector6d_t>& p_a,
                         const std::vector<double>& time,
                         bool use_spline_interpolation_);
    void read_and_noop();

    // private variables to maintain connection and state
    std::unique_ptr<UrDriver> driver;
    std::unique_ptr<DashboardClient> dashboard;
    vector6d_t joint_state, tcp_state;
    std::mutex mu;

    // specified through APPDIR environment variable
    const char* path_offset;

    // variables specified by ResourceConfig and set through reconfigure
    std::string host;
    double speed;
    double acceleration;
};
