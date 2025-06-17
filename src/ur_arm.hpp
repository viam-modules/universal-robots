#pragma once

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/types.h>

#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include "../trajectories/Path.h"

using namespace viam::sdk;
using namespace urcl;

void write_trajectory_to_file(const std::string& filepath,
                              const std::vector<vector6d_t>& p_p,
                              const std::vector<vector6d_t>& p_v,
                              const std::vector<float>& time);
void write_waypoints_to_csv(const std::string& filepath, const std::vector<Eigen::VectorXd>& waypoints);
void write_joint_pos_rad(vector6d_t js, std::ostream& of, unsigned long long unix_now_ms, unsigned attempt);
std::string waypoints_filename(const std::string& path, unsigned long long unix_time_ms);
std::string trajectory_filename(const std::string& path, unsigned long long unix_time_ms);
std::string arm_joint_positions_filename(const std::string& path, unsigned long long unix_time_ms);

class URArm final : public Arm, public Reconfigurable {
   public:
    /// @brief Returns the common ModelFamily for all implementations
    static const ModelFamily& model_family();

    /// @brief Returns a Model in the correct family for the given model name.
    static Model model(std::string model_name);

    /// @brief Returns a registration for each model of ARM supported by this class.
    static std::vector<std::shared_ptr<ModelRegistration>> create_model_registrations();

    URArm(Model model, const Dependencies& deps, const ResourceConfig& cfg);
    ~URArm() override;

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
    void move_to_position(const pose&, const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

    std::string get_output_csv_dir_path();

    // the arm server within RDK will reconstruct the geometries from the kinematics and joint positions if left unimplemented
    std::vector<GeometryConfig> get_geometries(const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

   private:
    struct state_;

    enum class UrDriverStatus : int8_t;  // Only available on 3.10/5.4
    static std::string status_to_string(UrDriverStatus status);

    void keep_alive();

    void move(std::vector<Eigen::VectorXd> waypoints, std::chrono::milliseconds unix_time_ms);

    bool send_trajectory(const std::vector<vector6d_t>& p_p, const std::vector<vector6d_t>& p_v, const std::vector<float>& time);

    void trajectory_done_cb(control::TrajectoryResult);

    URArm::UrDriverStatus read_joint_keep_alive(bool log);

    const Model model_;
    std::unique_ptr<state_> current_state_;
};
