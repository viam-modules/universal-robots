#include "trajectory_logger.hpp"

#include <fstream>
#include <iostream>

#include <boost/format.hpp>

#include <viam/sdk/log/logging.hpp>

RealtimeTrajectoryLogger::RealtimeTrajectoryLogger(const std::filesystem::path& telemetry_path,
                                                   const std::string& timestamp,
                                                   const std::string& robot_model,
                                                   const std::string& resource_name) {
    root_["timestamp"] = timestamp;
    root_["robot_model"] = robot_model;
    root_["resource_name"] = resource_name;
    root_["realtime_samples"] = Json::Value(Json::arrayValue);

    output_path_ = telemetry_path / (timestamp + "_" + resource_name + "_realtime_trajectory.json");
}

RealtimeTrajectoryLogger::~RealtimeTrajectoryLogger() {
    try {
        write_and_flush();
    } catch (const std::exception& e) {
        // Use stderr instead of VIAM_SDK_LOG since the SDK logger may
        // not be alive during destruction (e.g. in tests or shutdown).
        std::cerr << "RealtimeTrajectoryLogger failed to write JSON on destruction: " << e.what() << '\n';
    } catch (...) {
        std::cerr << "RealtimeTrajectoryLogger failed to write JSON on destruction (unknown error)\n";
    }
}

Json::Value RealtimeTrajectoryLogger::vector6d_to_json(const vector6d_t& v) {
    Json::Value arr(Json::arrayValue);
    for (const auto& val : v) {
        arr.append(val);
    }
    return arr;
}

void RealtimeTrajectoryLogger::set_velocity_limits(const vector6d_t& limits) {
    root_["configuration"]["max_velocity_rad_per_sec"] = vector6d_to_json(limits);
}

void RealtimeTrajectoryLogger::set_acceleration_limits(const vector6d_t& limits) {
    root_["configuration"]["max_acceleration_rad_per_sec2"] = vector6d_to_json(limits);
}

void RealtimeTrajectoryLogger::set_waypoints(const viam::trajex::totg::waypoint_accumulator& waypoints) {
    Json::Value arr(Json::arrayValue);
    for (const auto& wp : waypoints) {
        Json::Value wp_arr(Json::arrayValue);
        for (const auto& val : wp) {
            wp_arr.append(val);
        }
        arr.append(wp_arr);
    }
    root_["waypoints_rad"] = arr;
}

void RealtimeTrajectoryLogger::set_planned_trajectory(const trajectory_samples& samples) {
    Json::Value arr(Json::arrayValue);
    std::visit(
        [&arr](const auto& pts) {
            float time_from_start = 0;
            for (const auto& pt : pts) {
                time_from_start += pt.timestep;
                Json::Value sample;
                sample["positions_rad"] = vector6d_to_json(pt.p);
                sample["velocities_rad_per_sec"] = vector6d_to_json(pt.v);
                if constexpr (requires { pt.a; }) {
                    sample["accelerations_rad_per_sec2"] = vector6d_to_json(pt.a);
                }
                sample["time_from_start_sec"] = static_cast<double>(time_from_start);
                arr.append(sample);
            }
        },
        samples);
    root_["planned_trajectory"] = arr;
}

void RealtimeTrajectoryLogger::append_realtime_sample(uint64_t timestamp_us,
                                                      const ephemeral_data_& data,
                                                      std::optional<uint32_t> robot_status_bits,
                                                      std::optional<uint32_t> safety_status_bits) {
    Json::Value sample;
    sample["timestamp_us"] = static_cast<Json::UInt64>(timestamp_us);
    sample["positions_rad"] = vector6d_to_json(data.joint_positions);
    sample["velocities_rad_per_sec"] = vector6d_to_json(data.joint_velocities);
    sample["target_positions_rad"] = vector6d_to_json(data.target_joint_positions);
    sample["target_velocities_rad_per_sec"] = vector6d_to_json(data.target_joint_velocities);
    sample["target_accelerations_rad_per_sec2"] = vector6d_to_json(data.target_joint_accelerations);
    sample["target_current"] = vector6d_to_json(data.target_current);
    sample["target_moment"] = vector6d_to_json(data.target_moment);
    sample["target_tcp_speed"] = vector6d_to_json(data.target_tcp_speed);
    sample["actual_tcp_speed"] = vector6d_to_json(data.actual_tcp_speed);
    sample["tcp_pose"] = vector6d_to_json(data.tcp_state);
    sample["tcp_forces"] = vector6d_to_json(data.tcp_forces);
    sample["joint_temperatures"] = vector6d_to_json(data.joint_temperatures);
    sample["joint_control_output"] = vector6d_to_json(data.joint_control_output);
    if (robot_status_bits) {
        sample["robot_status_bits"] = *robot_status_bits;
    }
    if (safety_status_bits) {
        sample["safety_status_bits"] = *safety_status_bits;
    }
    if (data.safety_status) {
        sample["safety_status"] = *data.safety_status;
    }
    root_["realtime_samples"].append(sample);
}

void RealtimeTrajectoryLogger::write_and_flush() {
    Json::StreamWriterBuilder builder;
    builder["precision"] = 17;
    const auto json_str = Json::writeString(builder, root_);

    std::ofstream out(output_path_);
    if (!out) {
        std::cerr << "RealtimeTrajectoryLogger could not open file: " << output_path_ << '\n';
        return;
    }
    out << json_str;
    out.close();
}

std::string RealtimeTrajectoryLogger::realtime_trajectory_filename(const std::string& path,
                                                                   const std::string& resource_name,
                                                                   const std::string& unix_time) {
    constexpr char kTemplate[] = "/%1%_%2%_realtime_trajectory.json";
    auto fmt = boost::format(path + kTemplate);
    return (fmt % unix_time % resource_name).str();
}
