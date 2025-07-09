#pragma once

#include <shared_mutex>

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/types.h>

#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>

#include <third_party/trajectories/Path.h>

using namespace viam::sdk;
using namespace urcl;

struct trajectory_sample_point {
    vector6d_t p;
    vector6d_t v;
    float timestep;
};

template <typename Func>
void sampling_func(std::vector<trajectory_sample_point>& samples, double duration_sec, double sampling_frequency_hz, const Func& f) {
    if (duration_sec <= 0.0 || sampling_frequency_hz <= 0.0) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz are not both positive");
    }
    static constexpr std::size_t k_max_samples = 1000000;
    const auto putative_samples = duration_sec * sampling_frequency_hz;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument("duration_sec and sampling_frequency_hz exceed the maximum allowable samples");
    }

    // Calculate the number of samples needed. this will always be at least 2.
    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);

    // Calculate the actual step size
    const double step = duration_sec / static_cast<double>((num_samples - 1));

    // Generate samples by evaluating f at each time point
    for (std::size_t i = 1; i < num_samples - 1; ++i) {
        samples.push_back(f(static_cast<double>(i) * step, step));
    }

    // Ensure the last sample uses exactly the duration_sec
    samples.push_back(f(duration_sec, step));
}

void write_trajectory_to_file(const std::string& filepath, const std::vector<trajectory_sample_point>& samples);
void write_waypoints_to_csv(const std::string& filepath, const std::list<Eigen::VectorXd>& waypoints);
std::string waypoints_filename(const std::string& path, std::chrono::milliseconds unix_time_ms);
std::string trajectory_filename(const std::string& path, std::chrono::milliseconds unix_time_ms);
std::string arm_joint_positions_filename(const std::string& path, std::chrono::milliseconds unix_time_ms);

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

    // the arm server within RDK will reconstruct the geometries from the kinematics and joint positions if left unimplemented
    std::vector<GeometryConfig> get_geometries(const ProtoStruct&) override {
        throw std::runtime_error("unimplemented");
    }

   private:
    struct state_;

    enum class UrDriverStatus : int8_t;  // Only available on 3.10/5.4

    static std::string status_to_string_(UrDriverStatus status);

    void configure_(const std::unique_lock<std::shared_mutex>& lock, const Dependencies& deps, const ResourceConfig& cfg);

    template <template <typename> typename lock_type>
    void check_configured_(const lock_type<std::shared_mutex>&);

    void shutdown_(const std::unique_lock<std::shared_mutex>& lock) noexcept;

    void keep_alive_();

    std::vector<double> get_joint_positions_(const std::shared_lock<std::shared_mutex>&);

    void move_(std::shared_lock<std::shared_mutex> config_rlock,
               std::list<Eigen::VectorXd> waypoints,
               std::chrono::milliseconds unix_time_ms);

    bool send_trajectory_(const std::vector<trajectory_sample_point>& samples);

    void trajectory_done_cb_(control::TrajectoryResult);

    URArm::UrDriverStatus read_joint_keep_alive_(bool log);
    URArm::UrDriverStatus read_joint_keep_alive_inner_(bool log);

    template <template <typename> typename lock_type>
    void stop_(const lock_type<std::shared_mutex>&);

    const Model model_;

    std::shared_mutex config_mutex_;
    std::unique_ptr<state_> current_state_;
};
