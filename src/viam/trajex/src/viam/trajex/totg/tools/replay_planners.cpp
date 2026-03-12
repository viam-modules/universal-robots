#include <viam/trajex/totg/tools/replay_planners.hpp>

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <json/json.h>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

#if __has_include(<xtensor/core/xmath.hpp>)
#include <xtensor/core/xmath.hpp>
#else
#include <xtensor/xmath.hpp>
#endif

namespace viam::trajex {

namespace {

// Parse a JSON replay record stream into a trajectory_planner_base::config and a waypoints
// xarray. Throws std::runtime_error on malformed input.
std::pair<trajectory_planner_base::config, xt::xarray<double>> parse_replay_record(std::istream& in) {
    Json::Value root;
    const Json::CharReaderBuilder reader;
    std::string errs;
    if (!Json::parseFromStream(reader, in, &root, &errs)) {
        throw std::runtime_error("failed to parse replay record JSON: " + errs);
    }

    auto require = [&](const char* field) -> const Json::Value& {
        if (!root.isMember(field)) {
            throw std::runtime_error(std::string("replay record missing required field: ") + field);
        }
        return root[field];
    };

    const auto& vel_json = require("max_velocity_vec_rads_per_sec");
    const auto& acc_json = require("max_acceleration_vec_rads_per_sec2");
    const auto& wps_json = require("waypoints_rads");

    if (!vel_json.isArray() || vel_json.empty()) {
        throw std::runtime_error("max_velocity_vec_rads_per_sec must be a non-empty array");
    }
    if (!acc_json.isArray() || acc_json.empty()) {
        throw std::runtime_error("max_acceleration_vec_rads_per_sec2 must be a non-empty array");
    }
    if (!wps_json.isArray() || wps_json.empty()) {
        throw std::runtime_error("waypoints_rads must be a non-empty array");
    }

    const auto dof = static_cast<std::size_t>(vel_json.size());
    const auto num_waypoints = static_cast<std::size_t>(wps_json.size());

    xt::xarray<double> velocity_limits = xt::zeros<double>(std::vector<std::size_t>{dof});
    for (Json::ArrayIndex i = 0; i < vel_json.size(); ++i) {
        velocity_limits(i) = vel_json[i].asDouble();
    }

    xt::xarray<double> acceleration_limits = xt::zeros<double>(std::vector<std::size_t>{static_cast<std::size_t>(acc_json.size())});
    for (Json::ArrayIndex i = 0; i < acc_json.size(); ++i) {
        acceleration_limits(i) = acc_json[i].asDouble();
    }

    xt::xarray<double> waypoints = xt::zeros<double>(std::vector<std::size_t>{num_waypoints, dof});
    for (Json::ArrayIndex i = 0; i < wps_json.size(); ++i) {
        const auto& wp = wps_json[i];
        if (!wp.isArray() || static_cast<std::size_t>(wp.size()) != dof) {
            throw std::runtime_error("waypoint has wrong number of joints");
        }
        for (Json::ArrayIndex j = 0; j < wp.size(); ++j) {
            waypoints(i, static_cast<std::size_t>(j)) = wp[j].asDouble();
        }
    }

    trajectory_planner_base::config cfg;
    cfg.velocity_limits = std::move(velocity_limits);
    cfg.acceleration_limits = std::move(acceleration_limits);
    cfg.path_blend_tolerance = require("path_tolerance_delta_rads").asDouble();
    if (root.isMember("path_colinearization_ratio")) {
        cfg.colinearization_ratio = root["path_colinearization_ratio"].asDouble();
    }

    return {std::move(cfg), std::move(waypoints)};
}

}  // namespace

trajex_replay_planner::trajex_replay_planner(config cfg, std::unique_ptr<totg::trajectory_integration_event_collector> collector)
    : trajectory_planner<trajex_replay_receiver>(std::move(cfg)), collector_(std::move(collector)) {
    mutable_config().observer = collector_.get();
}

trajex_replay_planner trajex_replay_planner::create(std::istream& in) {
    auto [cfg, waypoints] = parse_replay_record(in);

    auto collector = std::make_unique<totg::trajectory_integration_event_collector>();
    trajex_replay_planner planner(std::move(cfg), std::move(collector));

    // Stash the waypoints array and provision it as a single unsegmented waypoint set.
    auto data = planner.stash(std::move(waypoints));
    planner.with_waypoint_provider([data](auto&) { return totg::waypoint_accumulator{*data}; });

    planner.with_trajex(
        [](const auto&, trajex_replay_receiver& recv, const totg::waypoint_accumulator&, const totg::trajectory& traj, auto) {
            recv.traj = traj;
        });

    return planner;
}

trajex_replay_planner trajex_replay_planner::create(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open replay record file: " + path.string());
    }
    return create(in);
}

const totg::trajectory_integration_event_collector& trajex_replay_planner::collector() const noexcept {
    return *collector_;
}

legacy_replay_planner legacy_replay_planner::create(std::istream& in) {
    auto [cfg, waypoints] = parse_replay_record(in);

    legacy_replay_planner planner(std::move(cfg));

    auto data = planner.stash(std::move(waypoints));
    planner.with_waypoint_provider([data](auto&) { return totg::waypoint_accumulator{*data}; });

    planner.with_legacy(
        [](const auto&, legacy_replay_receiver&, const totg::waypoint_accumulator&, const Path&, const Trajectory&, auto) {});

    return planner;
}

legacy_replay_planner legacy_replay_planner::create(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("failed to open replay record file: " + path.string());
    }
    return create(in);
}

}  // namespace viam::trajex
