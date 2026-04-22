#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include <json/json.h>
#include <ur_client_library/types.h>

#include <viam/trajex/totg/waypoint_accumulator.hpp>

#include "ur_arm.hpp"

using namespace urcl;

class RealtimeTrajectoryLogger {
   public:
    RealtimeTrajectoryLogger(const std::filesystem::path& telemetry_path,
                             const std::string& timestamp,
                             const std::string& robot_model,
                             const std::string& resource_name);
    ~RealtimeTrajectoryLogger();

    RealtimeTrajectoryLogger(RealtimeTrajectoryLogger&& other) noexcept;
    RealtimeTrajectoryLogger& operator=(RealtimeTrajectoryLogger&& other) noexcept;

    RealtimeTrajectoryLogger(const RealtimeTrajectoryLogger&) = delete;
    RealtimeTrajectoryLogger& operator=(const RealtimeTrajectoryLogger&) = delete;

    void set_velocity_limits(const vector6d_t& limits);
    void set_acceleration_limits(const vector6d_t& limits);
    void set_waypoints(const viam::trajex::totg::waypoint_accumulator& waypoints);
    void set_planned_trajectory(const trajectory_samples& samples);

    struct ephemeral_data {
        vector6d_t joint_positions;
        vector6d_t joint_velocities;
        vector6d_t tcp_state;
        vector6d_t tcp_forces;
        vector6d_t target_joint_positions;
        vector6d_t target_joint_velocities;
        vector6d_t target_joint_accelerations;
        vector6d_t target_current;
        vector6d_t target_moment;
        vector6d_t target_tcp_speed;
        vector6d_t actual_tcp_speed;
        vector6d_t joint_temperatures;
        vector6d_t joint_control_output;
        std::optional<uint32_t> safety_status;
    };

    void append_realtime_sample(const std::string& timestamp_iso,
                                const ephemeral_data& data,
                                std::optional<uint32_t> robot_status_bits,
                                std::optional<uint32_t> safety_status_bits);

    static std::string realtime_trajectory_filename(const std::string& path,
                                                    const std::string& resource_name,
                                                    const std::string& unix_time);

   private:
    void write_and_flush();

    static Json::Value vector6d_to_json(const vector6d_t& v);

    Json::Value root_;
    std::filesystem::path output_path_;
    bool active_{true};
};
