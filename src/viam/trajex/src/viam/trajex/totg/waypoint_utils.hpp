#pragma once

#include <vector>

#include <viam/trajex/totg/waypoint_accumulator.hpp>

namespace viam::trajex::totg {

///
/// Removes consecutive duplicate waypoints.
///
/// Compares waypoints using L-infinity (maximum absolute difference) norm.
/// Always keeps the first waypoint. Returns a new waypoint_accumulator viewing
/// deduplicated data (zero-copy if no duplicates found).
///
/// @param waypoints Input waypoints to deduplicate
/// @param tolerance Maximum L-infinity distance for waypoints to be considered duplicates
/// @return New waypoint_accumulator with duplicates removed
///
waypoint_accumulator deduplicate_waypoints(const waypoint_accumulator& waypoints, double tolerance);

///
/// Segments waypoints at direction reversals.
///
/// Detects cusps where the path reverses direction by comparing normalized
/// direction vectors. A reversal occurs when the dot product of consecutive
/// segments is approximately -1 (within threshold tolerance).
///
/// It is recommended to deduplicate waypoints with `deduplicate_waypoints`
/// (or some similar mechanism) before invoking this to avoid numerical
/// issues caused by very short segments.
///
/// @param waypoints Input waypoints to segment
/// @param threshold Tolerance for detecting reversals: |dot + 1.0| < threshold (default: 1e-3)
/// @return Vector of waypoint_accumulators, one per segment
///
std::vector<waypoint_accumulator> segment_at_reversals(waypoint_accumulator waypoints, double threshold = 1e-3);

}  // namespace viam::trajex::totg
