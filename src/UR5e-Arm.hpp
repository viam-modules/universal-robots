#pragma once

#include <string>

#include <boost/log/trivial.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>

#include <viam/api/robot/v1/robot.pb.h>
#include <viam/api/common/v1/common.grpc.pb.h>
#include <viam/api/component/generic/v1/generic.grpc.pb.h>

#include <viam/sdk/spatialmath/geometry.hpp>
#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <Path.h>
#include <Trajectory.h>

#include <math.h>

using namespace viam::sdk;
using namespace urcl;


class UR5eArm : public Arm{
    public:
        UR5eArm(Dependencies dep, ResourceConfig cfg);
        bool initialize();
        void send_noops();

        void SendTrajectory(const std::vector<vector6d_t>& p_p, const std::vector<vector6d_t>& p_v,
                    const std::vector<vector6d_t>& p_a, const std::vector<double>& time, bool use_spline_interpolation_);
    
        // --------------- UNSUPPORTED FUNCTIONS - DEPRECATED FROM API SOON ---------------
        pose get_end_position(const AttributeMap& extra) override{
            throw std::runtime_error("get_end_position unimplemented");
        }
        void move_to_position(const pose& pose, const AttributeMap& extra) override{
            throw std::runtime_error("move_to_position unimplemented");
        }
        void move_to_joint_positions(const std::vector<double>& positions,
                                         const AttributeMap& extra) override{
            throw std::runtime_error("move_to_joint_positions unimplemented");
        }

        // --------------- SUPPORTED FUNCTIONS ---------------
        /// @brief Lists the joint positions in degrees of every joint on a robot arm
        /// @param extra Any additional arguments to the method.
        std::vector<double> get_joint_positions(const AttributeMap& extra) override;

        /// @brief Reports if the arm is in motion.
        bool is_moving() override;

        /// @brief This is being used as a proxy to move_to_joint_positions except with support for multiple waypoints
        /// @param command will contain a std::vector<std::vector<double>> called positions that will contain joint waypoints
        /// @return bool depending on success of calculating and sending trajectory to UR
        AttributeMap do_command(const AttributeMap& command) override;

        /// @brief Get the kinematics data associated with the arm.
        /// @param extra Any additional arguments to the method.
        /// @return A variant of kinematics data, with bytes field containing the raw bytes of the file
        /// and the object's type indicating the file format.
        KinematicsData get_kinematics(const AttributeMap& extra) override;

        /// @brief Returns `GeometryConfig`s associated with the calling arm
        /// @param extra Any additional arguments to the method
        std::vector<GeometryConfig> get_geometries(const AttributeMap& extra) override;

        /// @brief Stops the Arm.
        /// @param extra Extra arguments to pass to the resource's `stop` method.
        void stop(const AttributeMap& extra) override;
        
        std::unique_ptr<UrDriver> driver;
        std::unique_ptr<DashboardClient> dashboard;
        vector6d_t g_joint_positions;
        std::atomic<bool> calling_trajectory;
        std::mutex mu;
        
        const double STOP_VELOCITY_THRESHOLD = 0.005; // rad/s
        const int NOOP_DELAY = 100000; // microseconds
        const double TIMESTEP = 0.2; // seconds

        const std::string DEFAULT_ROBOT_IP = "10.1.10.84";
        const std::string URDF_FILE = "/host/src/ur5e.urdf";
        const std::string SCRIPT_FILE = "/host/Universal_Robots_Client_Library/resources/external_control.urscript";
        const std::string OUTPUT_RECIPE = "/host/Universal_Robots_Client_Library/examples/resources/rtde_output_recipe.txt";
        const std::string INPUT_RECIPE = "/host/Universal_Robots_Client_Library/examples/resources/rtde_input_recipe.txt";
        const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
        const std::string POSITIONS_KEY = "positions";
};
