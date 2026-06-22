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

    RealtimeTrajectoryLogger(RealtimeTrajectoryLogger&&) = delete;
    RealtimeTrajectoryLogger& operator=(RealtimeTrajectoryLogger&&) = delete;
    RealtimeTrajectoryLogger(const RealtimeTrajectoryLogger&) = delete;
    RealtimeTrajectoryLogger& operator=(const RealtimeTrajectoryLogger&) = delete;

    void set_velocity_limits(const vector6d_t& limits);
    void set_acceleration_limits(const vector6d_t& limits);
    void set_waypoints(const viam::trajex::totg::waypoint_accumulator& waypoints);
    void set_planned_trajectory(const trajectory_samples& samples);

    void append_realtime_sample(uint64_t timestamp_us,
                                const ephemeral_data& data,
                                std::optional<uint32_t> robot_status_bits,
                                std::optional<uint32_t> safety_status_bits);

    // Records one spline knot exactly as handed to writeTrajectorySplinePoint: the position/velocity
    // boundary conditions, trajex's intended acceleration, and the segment duration. Lets the raw knot
    // (p, v, dt) be compared against the robot's cubic-derived target_accelerations in realtime_samples.
    void append_streamed_point(uint64_t timestamp_us,
                               const vector6d_t& positions,
                               const vector6d_t& velocities,
                               const vector6d_t& accelerations,
                               double timestep_sec);

    static std::string realtime_trajectory_filename(const std::string& path,
                                                    const std::string& resource_name,
                                                    const std::string& unix_time);

   private:
    void write_and_flush();

    static Json::Value vector6d_to_json(const vector6d_t& v);

    Json::Value root_;
    std::filesystem::path output_path_;
};
