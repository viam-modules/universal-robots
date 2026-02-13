#pragma once

#include <list>

#include <Eigen/Dense>

namespace viam::trajex {

///
/// Check if point is within tolerance cylinder around line segment.
///
/// Returns false if the segment is degenerate (start == end) or if the
/// point projects outside the segment bounds (non-monotonic movement).
/// Tolerance is interpreted as cylinder diameter (radius = tolerance/2).
///
/// @param point Point to test
/// @param line_start Start of line segment
/// @param line_end End of line segment
/// @param tolerance Diameter of tolerance cylinder
/// @return true if point is within tolerance, false otherwise
///
bool within_colinearization_tolerance(const Eigen::VectorXd& point,
                                      const Eigen::VectorXd& line_start,
                                      const Eigen::VectorXd& line_end,
                                      double tolerance);

///
/// Apply colinearization to waypoint list, removing redundant waypoints.
///
/// Implements waypoint coalescing by extending a tolerance cylinder from each
/// anchor waypoint, removing intermediate waypoints that remain within
/// tolerance of the straight-line segment. When extending the cylinder, all
/// previously skipped waypoints are revalidated to prevent drift.
///
/// @param waypoints List of waypoints to coalesce (modified in-place)
/// @param tolerance Diameter of tolerance cylinder (in radians)
///
void apply_colinearization(std::list<Eigen::VectorXd>& waypoints, double tolerance);

}  // namespace viam::trajex
