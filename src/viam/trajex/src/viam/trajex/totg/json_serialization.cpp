#include <viam/trajex/totg/json_serialization.hpp>

#include <cmath>
#include <ostream>
#include <stdexcept>
#include <string>
#include <variant>

#include <json/json.h>

#include <viam/trajex/types/arc_length.hpp>

namespace viam::trajex::totg {

namespace {

// Serialize xarray to JSON array
Json::Value xarray_to_json_array(const xt::xarray<double>& arr) {
    Json::Value result(Json::arrayValue);
    result.resize(static_cast<Json::ArrayIndex>(arr.size()));
    for (size_t i = 0; i < arr.size(); ++i) {
        result[static_cast<Json::ArrayIndex>(i)] = arr.data()[i];
    }
    return result;
}

// Convert switching_point_kind enum to string for JSON
std::string switching_point_kind_to_string(trajectory::switching_point_kind kind) {
    switch (kind) {
        case trajectory::switching_point_kind::k_path_begin:
            return "k_path_begin";
        case trajectory::switching_point_kind::k_discontinuous_curvature:
            return "k_discontinuous_curvature";
        case trajectory::switching_point_kind::k_nondifferentiable_extremum:
            return "k_nondifferentiable_extremum";
        case trajectory::switching_point_kind::k_velocity_escape:
            return "k_velocity_escape";
        case trajectory::switching_point_kind::k_discontinuous_velocity_limit:
            return "k_discontinuous_velocity_limit";
        case trajectory::switching_point_kind::k_path_end:
            return "k_path_end";
    }
    throw std::invalid_argument("Unknown switching_point_kind");
}

// Serialize integration points with joint-space data and limit curves (struct of arrays)
Json::Value serialize_integration_points(const trajectory& traj) {
    const auto& points = traj.get_integration_points();
    const auto& p = traj.path();
    Json::Value result(Json::objectValue);

    // Phase plane data
    Json::Value times(Json::arrayValue);
    Json::Value s_values(Json::arrayValue);
    Json::Value s_dot_values(Json::arrayValue);
    Json::Value s_ddot_values(Json::arrayValue);

    // Limit curves at each point
    Json::Value s_dot_max_acc_values(Json::arrayValue);
    Json::Value s_dot_max_vel_values(Json::arrayValue);

    // Joint-space data (each element is an array)
    Json::Value configurations(Json::arrayValue);
    Json::Value velocities(Json::arrayValue);
    Json::Value accelerations(Json::arrayValue);

    times.resize(static_cast<Json::ArrayIndex>(points.size()));
    s_values.resize(static_cast<Json::ArrayIndex>(points.size()));
    s_dot_values.resize(static_cast<Json::ArrayIndex>(points.size()));
    s_ddot_values.resize(static_cast<Json::ArrayIndex>(points.size()));
    s_dot_max_acc_values.resize(static_cast<Json::ArrayIndex>(points.size()));
    s_dot_max_vel_values.resize(static_cast<Json::ArrayIndex>(points.size()));
    configurations.resize(static_cast<Json::ArrayIndex>(points.size()));
    velocities.resize(static_cast<Json::ArrayIndex>(points.size()));
    accelerations.resize(static_cast<Json::ArrayIndex>(points.size()));

    // Create cursor for efficient sequential access
    auto cursor = p.create_cursor();

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& pt = points[i];
        const auto idx = static_cast<Json::ArrayIndex>(i);

        // Phase plane
        times[idx] = pt.time.count();
        s_values[idx] = static_cast<double>(pt.s);
        s_dot_values[idx] = static_cast<double>(pt.s_dot);
        s_ddot_values[idx] = static_cast<double>(pt.s_ddot);

        // Move cursor to this arc length
        cursor.seek(pt.s);

        // Query joint-space at this point
        const auto q = cursor.configuration();
        const auto q_dot = cursor.tangent();
        const auto q_ddot = cursor.curvature();

        // Get limit curves at this arc length using cursor
        const auto limits = traj.get_velocity_limits(cursor);
        const arc_velocity s_dot_max_acc = limits.s_dot_max_acc;
        const arc_velocity s_dot_max_vel = limits.s_dot_max_vel;

        // Handle infinite limits (serialize as null for JSON)
        if (std::isinf(static_cast<double>(s_dot_max_acc))) {
            s_dot_max_acc_values[idx] = Json::Value::null;
        } else {
            s_dot_max_acc_values[idx] = static_cast<double>(s_dot_max_acc);
        }

        if (std::isinf(static_cast<double>(s_dot_max_vel))) {
            s_dot_max_vel_values[idx] = Json::Value::null;
        } else {
            s_dot_max_vel_values[idx] = static_cast<double>(s_dot_max_vel);
        }

        configurations[idx] = xarray_to_json_array(q);

        // Joint velocity: q̇ = (dq/ds) * (ds/dt) = tangent * s_dot
        velocities[idx] = xarray_to_json_array(q_dot * static_cast<double>(pt.s_dot));

        // Joint acceleration: q̈ = (d²q/ds²) * (ds/dt)² + (dq/ds) * (d²s/dt²)
        //                       = curvature * s_dot² + tangent * s_ddot
        const double s_dot_val = static_cast<double>(pt.s_dot);
        const double s_ddot_val = static_cast<double>(pt.s_ddot);
        accelerations[idx] = xarray_to_json_array(q_ddot * (s_dot_val * s_dot_val) + q_dot * s_ddot_val);
    }

    // Phase plane
    result["time"] = std::move(times);
    result["s"] = std::move(s_values);
    result["s_dot"] = std::move(s_dot_values);
    result["s_ddot"] = std::move(s_ddot_values);

    // Limit curves
    result["s_dot_max_acc"] = std::move(s_dot_max_acc_values);
    result["s_dot_max_vel"] = std::move(s_dot_max_vel_values);

    // Joint-space
    result["configuration"] = std::move(configurations);
    result["velocity"] = std::move(velocities);
    result["acceleration"] = std::move(accelerations);

    return result;
}

// Serialize events by type (struct of arrays)
Json::Value serialize_events(const std::vector<trajectory::integration_event_observer::event>& events) {
    Json::Value result(Json::objectValue);

    Json::Value forward_starts(Json::arrayValue);
    Json::Value limit_hits(Json::arrayValue);
    Json::Value backward_starts(Json::arrayValue);
    Json::Value splices(Json::arrayValue);

    for (const auto& event : events) {
        std::visit(
            [&](auto&& ev) {
                using T = std::decay_t<decltype(ev)>;

                if constexpr (std::is_same_v<T, trajectory::integration_observer::started_forward_event>) {
                    Json::Value obj;
                    obj["s"] = static_cast<double>(ev.start.s);
                    obj["s_dot"] = static_cast<double>(ev.start.s_dot);
                    forward_starts.append(std::move(obj));
                } else if constexpr (std::is_same_v<T, trajectory::integration_observer::limit_hit_event>) {
                    Json::Value obj;
                    obj["s"] = static_cast<double>(ev.breach.s);
                    obj["s_dot"] = static_cast<double>(ev.breach.s_dot);
                    obj["s_dot_max_acc"] = std::isinf(static_cast<double>(ev.s_dot_max_acc))
                                               ? Json::Value::null
                                               : Json::Value{static_cast<double>(ev.s_dot_max_acc)};
                    obj["s_dot_max_vel"] = std::isinf(static_cast<double>(ev.s_dot_max_vel))
                                               ? Json::Value::null
                                               : Json::Value{static_cast<double>(ev.s_dot_max_vel)};
                    limit_hits.append(std::move(obj));
                } else if constexpr (std::is_same_v<T, trajectory::integration_observer::started_backward_event>) {
                    Json::Value obj;
                    obj["s"] = static_cast<double>(ev.start.s);
                    obj["s_dot"] = static_cast<double>(ev.start.s_dot);
                    obj["kind"] = switching_point_kind_to_string(ev.kind);
                    backward_starts.append(std::move(obj));
                } else if constexpr (std::is_same_v<T, trajectory::integration_observer::splice_event>) {
                    Json::Value obj;
                    obj["num_pruned"] = static_cast<Json::Int64>(std::ranges::distance(ev.pruned));

                    // Serialize pruned points (what was replaced by backward integration)
                    Json::Value pruned_points(Json::objectValue);
                    Json::Value pruned_times(Json::arrayValue);
                    Json::Value pruned_s(Json::arrayValue);
                    Json::Value pruned_s_dot(Json::arrayValue);
                    Json::Value pruned_s_ddot(Json::arrayValue);

                    for (const auto& pt : ev.pruned) {
                        pruned_times.append(pt.time.count());
                        pruned_s.append(static_cast<double>(pt.s));
                        pruned_s_dot.append(static_cast<double>(pt.s_dot));
                        pruned_s_ddot.append(static_cast<double>(pt.s_ddot));
                    }

                    pruned_points["time"] = std::move(pruned_times);
                    pruned_points["s"] = std::move(pruned_s);
                    pruned_points["s_dot"] = std::move(pruned_s_dot);
                    pruned_points["s_ddot"] = std::move(pruned_s_ddot);

                    obj["pruned_points"] = std::move(pruned_points);
                    splices.append(std::move(obj));
                }
            },
            event);
    }

    result["forward_starts"] = std::move(forward_starts);
    result["limit_hits"] = std::move(limit_hits);
    result["backward_starts"] = std::move(backward_starts);
    result["splices"] = std::move(splices);

    return result;
}

}  // namespace

std::string serialize_trajectory_to_json(const trajectory_integration_event_collector& collector, const trajectory* traj) {
    // On failure the caller passes nullptr; fall back to whatever the collector captured.
    const trajectory* effective = traj ? traj : collector.invalid_trajectory().get();

    Json::Value root;

    Json::Value metadata;
    if (effective) {
        metadata["dof"] = static_cast<Json::Int64>(effective->path().dof());
        metadata["path_length"] = static_cast<double>(effective->path().length());
        metadata["duration"] = effective->duration().count();
        metadata["num_integration_points"] = static_cast<Json::Int64>(effective->get_integration_points().size());
        metadata["max_velocity"] = xarray_to_json_array(effective->get_options().max_velocity);
        metadata["max_acceleration"] = xarray_to_json_array(effective->get_options().max_acceleration);
    }

    // If there was a failure, annotate metadata with the error message.
    if (const auto exc = collector.invalid_exception()) {
        metadata["failure"] = true;
        try {
            std::rethrow_exception(exc);
        } catch (const std::exception& e) {
            metadata["error"] = e.what();
        } catch (...) {
            metadata["error"] = "unknown exception";
        }
    }

    root["metadata"] = std::move(metadata);

    if (effective) {
        root["integration_points"] = serialize_integration_points(*effective);
    }

    root["events"] = serialize_events(collector.events());

    // When events exist beyond the last integration point (which happens on failure),
    // sample the limit curves densely through the gap so the visualizer can show
    // where the trajectory ran into trouble. On success all events fall within the
    // confirmed integration range and this block is a no-op.
    if (effective && !effective->get_integration_points().empty()) {
        const auto& points = effective->get_integration_points();
        const arc_length last_s = points.back().s;

        // Find the farthest s across all event types - backward starts can be well
        // ahead of limit hits.
        arc_length farthest_s = last_s;
        for (const auto& ev : collector) {
            std::visit(
                [&](auto&& e) {
                    using T = std::decay_t<decltype(e)>;
                    if constexpr (std::is_same_v<T, trajectory::integration_observer::limit_hit_event>) {
                        if (e.breach.s > farthest_s) {
                            farthest_s = e.breach.s;
                        }
                    } else if constexpr (std::is_same_v<T, trajectory::integration_observer::started_backward_event> ||
                                         std::is_same_v<T, trajectory::integration_observer::started_forward_event>) {
                        if (e.start.s > farthest_s) {
                            farthest_s = e.start.s;
                        }
                    }
                },
                ev);
        }

        const arc_length path_end = effective->path().length();
        if (farthest_s > path_end) {
            farthest_s = path_end;
        }

        if (farthest_s > last_s) {
            double delta;
            if (points.size() >= 2) {
                delta =
                    (static_cast<double>(points.back().s) - static_cast<double>(points.front().s)) / static_cast<double>(points.size() - 1);
            }
            if (points.size() < 2 || delta <= 0.0) {
                delta = (static_cast<double>(farthest_s) - static_cast<double>(last_s)) / 20.0;
            }

            Json::Value limit_curve_samples(Json::objectValue);
            Json::Value lcs_s(Json::arrayValue);
            Json::Value lcs_s_dot_max_acc(Json::arrayValue);
            Json::Value lcs_s_dot_max_vel(Json::arrayValue);

            auto cursor = effective->path().create_cursor();
            auto emit = [&](arc_length s) {
                cursor.seek(s);
                const auto limits = effective->get_velocity_limits(cursor);
                lcs_s.append(static_cast<double>(s));
                lcs_s_dot_max_acc.append(std::isinf(static_cast<double>(limits.s_dot_max_acc))
                                             ? Json::Value::null
                                             : Json::Value{static_cast<double>(limits.s_dot_max_acc)});
                lcs_s_dot_max_vel.append(std::isinf(static_cast<double>(limits.s_dot_max_vel))
                                             ? Json::Value::null
                                             : Json::Value{static_cast<double>(limits.s_dot_max_vel)});
            };

            for (arc_length s = last_s; s < farthest_s; s += arc_length{delta}) {
                emit(s);
            }
            emit(farthest_s);

            limit_curve_samples["s"] = std::move(lcs_s);
            limit_curve_samples["s_dot_max_acc"] = std::move(lcs_s_dot_max_acc);
            limit_curve_samples["s_dot_max_vel"] = std::move(lcs_s_dot_max_vel);
            root["limit_curve_samples"] = std::move(limit_curve_samples);
        }
    }

    Json::StreamWriterBuilder writer;
    writer["indentation"] = "  ";
    return Json::writeString(writer, root);
}

void write_trajectory_json(std::ostream& out, const trajectory_integration_event_collector& collector, const trajectory* traj) {
    out << serialize_trajectory_to_json(collector, traj);
}

}  // namespace viam::trajex::totg
