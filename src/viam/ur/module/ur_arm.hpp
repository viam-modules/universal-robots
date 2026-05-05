#pragma once

#include <list>
#include <optional>
#include <shared_mutex>
#include <variant>

#include <Eigen/Core>

#include <ur_client_library/primary/robot_state/kinematics_info.h>
#include <ur_client_library/types.h>

#include <viam/sdk/common/mesh.hpp>
#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#include <viam/trajex/totg/tools/legacy.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>

using namespace viam::sdk;
using namespace urcl;

struct trajectory_sample_point_pv {
    vector6d_t p;
    vector6d_t v;
    float timestep;
};

struct trajectory_sample_point_pva {
    vector6d_t p;
    vector6d_t v;
    vector6d_t a;
    float timestep;
};

using trajectory_samples = std::variant<std::vector<trajectory_sample_point_pv>, std::vector<trajectory_sample_point_pva>>;

struct pose_sample {
    vector6d_t p;
};

struct ephemeral_data {
    vector6d_t actual_current;
    vector6d_t actual_joint_voltage;
    vector6d_t actual_tcp_speed;
    vector6d_t joint_control_output;
    vector6d_t joint_positions;
    vector6d_t joint_temperatures;
    vector6d_t joint_velocities;
    std::optional<uint32_t> safety_status;
    double speed_scaling;
    vector6d_t target_current;
    vector6d_t target_joint_accelerations;
    vector6d_t target_joint_positions;
    vector6d_t target_joint_velocities;
    vector6d_t target_moment;
    vector6d_t target_tcp_speed;
    vector6d_t tcp_forces;
    vector6d_t tcp_state;
};

std::string failed_trajectory_filename(const std::string& path, const std::string& resource_name, const std::string& unix_time);
std::string unix_time_iso8601();

class URArm final : public Arm, public Reconfigurable {
   public:
    ///
    /// Default robot control frequency in Hz.
    ///
    static constexpr double k_default_robot_control_freq_hz = 100.0;

    ///
    /// Default maximum trajectory duration in seconds.
    ///
    static constexpr double k_default_max_trajectory_duration_secs = 600.0;

    ///
    /// Default waypoint segmentation threshold.
    ///
    static constexpr double k_default_segmentation_threshold = 5e-3;

    ///
    /// Default trajectory sampling frequency in Hz.
    ///
    static constexpr double k_default_trajectory_sampling_freq_hz = 10.0;

    ///
    /// Default path tolerance delta in radians.
    ///
    static constexpr double k_default_path_tolerance_delta_rads = 0.1;

    ///
    /// Default waypoint deduplication tolerance in radians.
    ///
    static constexpr double k_default_waypoint_deduplication_tolerance_rads = 1e-3;

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

    /// @brief Move the arm to a defined pose.
    /// @param p The pose the arm will move to.
    /// @return An error indicating if we succeeded or not.
    void move_to_position(const pose& p, const ProtoStruct&) override;

    /// @brief Reports if the arm is in motion.
    bool is_moving() override;

    /// @brief Get the kinematics data associated with the arm.
    /// @param extra Any additional arguments to the method.
    /// @return A variant of kinematics data, with bytes field containing the raw bytes of the file
    /// and the object's type indicating the file format.
    viam::sdk::KinematicsData get_kinematics(const ProtoStruct& extra) override;

    /// @brief Get the 3D models associated with the arm.
    /// @param extra Any additional arguments to the method.
    /// @return A map of model names to 3D models.
    std::map<std::string, mesh> get_3d_models(const ProtoStruct& extra) override;

    /// @brief Stops the Arm.
    /// @param extra Extra arguments to pass to the resource's `stop` method.
    void stop(const ProtoStruct& extra) override;

    /// @brief This is being used as a proxy to move_to_joint_positions except with support for
    /// multiple waypoints
    /// @param command Will contain a std::vector<std::vector<double>> called positions that will
    /// contain joint waypoints
    ProtoStruct do_command(const ProtoStruct& command) override;

    // --------------- UNIMPLEMENTED FUNCTIONS --------------
    // the arm server within RDK will reconstruct the geometries from the kinematics and joint positions if left unimplemented
    std::vector<GeometryConfig> get_geometries(const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

   private:
    class state_;

    void configure_(const std::unique_lock<std::shared_mutex>& lock, const Dependencies& deps, const ResourceConfig& cfg);

    template <template <typename> typename lock_type>
    void check_configured_(const lock_type<std::shared_mutex>&);

    void shutdown_(const std::unique_lock<std::shared_mutex>& lock) noexcept;

    vector6d_t get_joint_positions_rad_(const std::shared_lock<std::shared_mutex>&);

    void move_joint_space_(std::shared_lock<std::shared_mutex> config_rlock,
                           const xt::xarray<double>& waypoints,
                           const MoveOptions& options,
                           const std::string& unix_time);

    void move_tool_space_(std::shared_lock<std::shared_mutex> config_rlock, pose p, const std::string& unix_time_ms);

    template <template <typename> typename lock_type>
    void stop_(const lock_type<std::shared_mutex>&);

    const Model model_;

    const struct ports_ {
        ports_();

        uint32_t reverse_port;
        uint32_t script_sender_port;
        uint32_t trajectory_port;
        uint32_t script_command_port;
    } ports_;

    std::shared_mutex config_mutex_;
    std::unique_ptr<state_> current_state_;

    // Empty when the arm uses the static-file kinematics path; non-empty when
    // the DH-form JSON synthesized from the controller's DH parameters should
    // be returned by get_kinematics().
    std::string dh_kinematics_json_;

    // Calibrated DH parameters captured once at configure_, reused by the
    // get_calibrated_dh_params do_command. Calibration does not change at
    // runtime, so caching the value avoids re-walking the state-machine
    // variant and re-acquiring its mutex on every call.
    std::optional<urcl::primary_interface::KinematicsInfo> calibrated_kin_info_;

    std::unordered_map<std::string, std::vector<std::string>> arm_name_to_model_parts_;
};
