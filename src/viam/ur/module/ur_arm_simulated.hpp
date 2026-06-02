#pragma once

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ur_client_library/types.h>

#include <viam/sdk/common/mesh.hpp>
#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/services/motion.hpp>

using namespace viam::sdk;
using namespace urcl;

///
/// Hardware-free simulated Universal Robots arm.
///
/// Registered as `viam:universal-robots:simulated-arm`. Selects which UR variant to
/// emulate via the `arm_model` attribute (one of `ur3e`, `ur5e`, `ur7e`, `ur12e`,
/// `ur20`) and otherwise behaves like the hardware arm without requiring a robot:
///   * joint moves interpolate toward the target over time at a configurable speed,
///   * `get_end_position` is computed via forward kinematics,
///   * `move_to_position` (Cartesian) is delegated to the motion service, and
///   * kinematics / 3D meshes are served from the same data files as the real arm.
///
/// This mirrors rdk's builtin simulated arm and the so-101 simulated arm so configs,
/// motion plans, and the 3D scene viewer can be developed and tested without hardware.
///
class URArmSimulated final : public Arm, public Reconfigurable {
   public:
    /// @brief Default joint interpolation speed in degrees/second.
    static constexpr double k_default_speed_degs_per_sec = 60.0;

    /// @brief Default motion service name used to plan `move_to_position` requests.
    static constexpr char k_default_motion_service[] = "builtin";

    /// @brief Returns a registration for the simulated arm model.
    static std::vector<std::shared_ptr<ModelRegistration>> create_model_registrations();

    URArmSimulated(Model model, const Dependencies& deps, const ResourceConfig& cfg);
    ~URArmSimulated() override;

    void reconfigure(const Dependencies& deps, const ResourceConfig& cfg) override;

    std::vector<double> get_joint_positions(const ProtoStruct& extra) override;

    void move_to_joint_positions(const std::vector<double>& positions, const ProtoStruct& extra) override;

    void move_through_joint_positions(const std::vector<std::vector<double>>& positions,
                                      const MoveOptions& options,
                                      const ProtoStruct& extra) override;

    pose get_end_position(const ProtoStruct& extra) override;

    void move_to_position(const pose& p, const ProtoStruct& extra) override;

    bool is_moving() override;

    viam::sdk::KinematicsData get_kinematics(const ProtoStruct& extra) override;

    std::map<std::string, mesh> get_3d_models(const ProtoStruct& extra) override;

    void stop(const ProtoStruct& extra) override;

    ProtoStruct do_command(const ProtoStruct& command) override;

    // The arm server within RDK reconstructs geometries from the kinematics and joint
    // positions when this is left unimplemented, matching the hardware arm.
    std::vector<GeometryConfig> get_geometries(const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

   private:
    void configure_(const Dependencies& deps, const ResourceConfig& cfg);
    void shutdown_() noexcept;

    // Background loop that advances the simulated joints against the realtime clock.
    void run_simulation_();

    // Advances `joint_positions_rad_` toward `target_positions_rad_` for the elapsed
    // wall-clock time, clearing `moving_` and notifying waiters when the target is
    // reached. Caller must hold `mutex_`.
    void update_for_time_(std::chrono::steady_clock::time_point now);

    const Model model_;

    // Immutable after configure_: which UR variant we are emulating and where the
    // module's data files live.
    std::string arm_model_;
    std::filesystem::path resource_root_;
    std::shared_ptr<Motion> motion_;
    std::unordered_map<std::string, std::vector<std::string>> arm_name_to_model_parts_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    vector6d_t joint_positions_rad_{};
    vector6d_t target_positions_rad_{};
    vector6d_t speed_rad_per_sec_{};
    bool moving_ = false;
    bool simulate_time_ = true;
    std::chrono::steady_clock::time_point last_update_;

    std::thread sim_thread_;
    std::atomic<bool> shutdown_requested_{false};
};

///
/// Supported `arm_model` values for the simulated arm, in registration order.
///
const std::vector<std::string>& simulated_arm_models();

///
/// Advance `current` toward `target` by the given per-joint speeds (rad/s) over
/// `elapsed_secs`, scaling each joint's speed so that all joints reach their targets
/// simultaneously (matching rdk/so-101 interpolation). Sets `*reached` to true when
/// every joint has arrived. Exposed as a free function for unit testing.
///
urcl::vector6d_t simulated_interpolate_step(const urcl::vector6d_t& current,
                                            const urcl::vector6d_t& target,
                                            const urcl::vector6d_t& speed_rad_per_sec,
                                            double elapsed_secs,
                                            bool* reached);
