#pragma once

#include <iosfwd>
#include <string>

#include <viam/trajex/totg/observers.hpp>
#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

///
/// Serializes trajectory with collected events to JSON.
///
/// Includes integration points (phase plane + joint-space data + limit curves),
/// events (forward starts, limit hits, backward starts, splices with pruned points),
/// and metadata.
///
/// JSON structure:
/// @code{.json}
/// {
///   "metadata": {
///     "dof": <int>,
///     "path_length": <number>,
///     "duration": <number>,
///     "num_integration_points": <int>,
///     "max_velocity": [<number>, ...],
///     "max_acceleration": [<number>, ...]
///   },
///   "integration_points": {
///     "time": [<number>, ...],
///     "s": [<number>, ...],
///     "s_dot": [<number>, ...],
///     "s_ddot": [<number>, ...],
///     "s_dot_max_acc": [<number|null>, ...],
///     "s_dot_max_vel": [<number|null>, ...],
///     "configuration": [[<number>, ...], ...],
///     "velocity": [[<number>, ...], ...],
///     "acceleration": [[<number>, ...], ...]
///   },
///   "events": {
///     "forward_starts": [{"s": <number>, "s_dot": <number>}, ...],
///     "limit_hits": [{"s": <number>, "s_dot": <number>,
///                     "s_dot_max_acc": <number>, "s_dot_max_vel": <number>}, ...],
///     "backward_starts": [{"s": <number>, "s_dot": <number>, "kind": <string>}, ...],
///     "splices": [{"num_pruned": <int>,
///                  "pruned_points": {"time": [...], "s": [...], "s_dot": [...], "s_ddot": [...]}}, ...]
///   }
/// }
/// @endcode
///
/// @param traj The trajectory to serialize
/// @param collector Event collector containing integration events
/// @return JSON string
///
std::string serialize_trajectory_to_json(const trajectory& traj, const trajectory_integration_event_collector& collector);

///
/// Writes trajectory JSON directly to output stream.
///
/// Convenience wrapper around serialize_trajectory_to_json().
///
/// @param out Output stream to write to
/// @param traj The trajectory to serialize
/// @param collector Event collector containing integration events
///
void write_trajectory_json(std::ostream& out, const trajectory& traj, const trajectory_integration_event_collector& collector);

///
/// Writes trajectory JSON for a failed generation to output stream.
///
/// Serializes partial integration points (accumulated before the failure), events,
/// and an error message. Uses the same JSON schema as write_trajectory_json, with
/// the following additions:
/// - metadata gains "failure": true and "error": "<message>"
/// - top-level "limit_curve_samples" section with limit curves sampled densely across
///   the gap from the last integration point to the farthest event position (limit
///   hits, backward starts, forward starts):
///   @code{.json}
///   "limit_curve_samples": {
///     "s": [<number>, ...],
///     "s_dot_max_acc": [<number|null>, ...],
///     "s_dot_max_vel": [<number|null>, ...]
///   }
///   @endcode
///
/// The sampling delta matches the average spacing between integration points, giving
/// visually continuous limit curve coverage through the gap where integration failed.
///
/// @param out Output stream to write to
/// @param collector Event collector containing failure data from on_failed
///
void write_failed_trajectory_json(std::ostream& out, const trajectory_integration_event_collector& collector);

}  // namespace viam::trajex::totg
