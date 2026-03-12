#include <viam/trajex/service/trajectory_planner.hpp>

#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <json/json.h>

namespace viam::trajex {

trajectory_planner_base::trajectory_planner_base(struct config cfg) : config_(std::move(cfg)) {}

const struct trajectory_planner_base::config& trajectory_planner_base::get_config() const noexcept {
    return config_;
}

const trajectory_planner_base::timing_stats& trajectory_planner_base::timing() const noexcept {
    return timing_;
}

std::size_t trajectory_planner_base::processed_waypoint_count() const noexcept {
    return processed_waypoint_count_;
}

struct trajectory_planner_base::config& trajectory_planner_base::mutable_config() noexcept {
    return config_;
}

trajectory_planner_base::timing_stats& trajectory_planner_base::mutable_timing() noexcept {
    return timing_;
}

std::size_t& trajectory_planner_base::mutable_processed_waypoint_count() noexcept {
    return processed_waypoint_count_;
}

std::string trajectory_planner_base::serialize_for_replay(const totg::waypoint_accumulator& waypoints,
                                                          std::optional<std::string_view> error_message) const {
    // Build ISO 8601 timestamp with microsecond precision.
    const auto now = std::chrono::system_clock::now();
    const auto seconds_part = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    const auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::time_point{seconds_part});
    const auto delta_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch() - seconds_part);

    struct tm buf;
    if (gmtime_r(&tt, &buf) == nullptr) {
        throw std::runtime_error("failed to convert time to iso8601");
    }
    std::ostringstream ts;
    ts << std::put_time(&buf, "%FT%T") << "." << std::setw(6) << std::setfill('0') << delta_us.count() << "Z";

    // Reconstruct path::options the same way run_trajex_ does, so we can read off the
    // min/max blend curvature defaults that are currently not configurable via get_config().
    auto path_opts = totg::path::options{}.set_max_blend_deviation(get_config().path_blend_tolerance);
    if (get_config().colinearization_ratio) {
        path_opts.set_max_linear_deviation(get_config().path_blend_tolerance * *get_config().colinearization_ratio);
    }

    Json::Value root;
    root["schema_version"] = 1;
    root["timestamp"] = ts.str();
    if (error_message) {
        root["error_message"] = std::string(*error_message);
    }
    root["path_tolerance_delta_rads"] = get_config().path_blend_tolerance;
    if (get_config().colinearization_ratio) {
        root["path_colinearization_ratio"] = *get_config().colinearization_ratio;
    }

    Json::Value vel_array(Json::arrayValue);
    for (const double v : get_config().velocity_limits) {
        vel_array.append(v);
    }
    root["max_velocity_vec_rads_per_sec"] = std::move(vel_array);

    Json::Value acc_array(Json::arrayValue);
    for (const double a : get_config().acceleration_limits) {
        acc_array.append(a);
    }
    root["max_acceleration_vec_rads_per_sec2"] = std::move(acc_array);

    root["min_blend_curvature"] = path_opts.min_blend_curvature();
    root["max_blend_curvature"] = path_opts.max_blend_curvature();

    Json::Value waypoints_array(Json::arrayValue);
    for (const auto& waypoint : waypoints) {
        Json::Value wp(Json::arrayValue);
        for (const double val : waypoint) {
            wp.append(val);
        }
        waypoints_array.append(std::move(wp));
    }
    root["waypoints_rads"] = std::move(waypoints_array);

    Json::StreamWriterBuilder writer;
    writer["precision"] = static_cast<int>(std::numeric_limits<double>::max_digits10);
    writer["indentation"] = " ";
    return Json::writeString(writer, root);
}

}  // namespace viam::trajex
