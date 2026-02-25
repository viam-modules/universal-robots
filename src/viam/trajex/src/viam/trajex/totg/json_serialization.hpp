#pragma once

#include <iosfwd>
#include <string>

#include <viam/trajex/totg/observers.hpp>
#include <viam/trajex/totg/trajectory.hpp>

namespace viam::trajex::totg {

///
/// Serializes trajectory with collected events to JSON.
///
/// Handles both successful and failed generations. On success, pass the completed
/// trajectory as @p traj. On failure, pass nullptr; the function uses
/// collector.invalid_trajectory() and collector.invalid_exception() to populate
/// the output.
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
///     "max_acceleration": [<number>, ...],
///     // on failure only:
///     "failure": true,
///     "error": "<message>"
///   },
///   "integration_points": { ... },  // omitted if no trajectory available
///   "events": { "forward_starts": [...], "limit_hits": [...],
///               "backward_starts": [...], "splices": [...] },
///   // present when events exist beyond the last integration point (i.e. on failure):
///   "limit_curve_samples": {
///     "s": [<number>, ...],
///     "s_dot_max_acc": [<number|null>, ...],
///     "s_dot_max_vel": [<number|null>, ...]
///   }
/// }
/// @endcode
///
/// @param collector Event collector containing integration events and optional failure data
/// @param traj Completed trajectory (success path), or nullptr (failure path)
/// @return JSON string
///
std::string serialize_trajectory_to_json(const trajectory_integration_event_collector& collector, const trajectory* traj = nullptr);

///
/// Writes trajectory JSON to output stream.
///
/// Convenience wrapper around serialize_trajectory_to_json(). See that function
/// for full schema documentation and success/failure semantics.
///
/// @param out Output stream to write to
/// @param collector Event collector containing integration events and optional failure data
/// @param traj Completed trajectory (success path), or nullptr (failure path)
///
void write_trajectory_json(std::ostream& out, const trajectory_integration_event_collector& collector, const trajectory* traj = nullptr);

}  // namespace viam::trajex::totg
