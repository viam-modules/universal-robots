#include <viam/trajex/service/colinearization.hpp>

namespace viam::trajex {

bool within_colinearization_tolerance(const Eigen::VectorXd& point,
                                      const Eigen::VectorXd& line_start,
                                      const Eigen::VectorXd& line_end,
                                      double tolerance) {
    const auto start_to_end = line_end - line_start;

    // Degenerate segment: cannot form meaningful cylinder
    if ((start_to_end.array() == 0.0).all()) {
        return false;
    }

    const auto start_to_point = point - line_start;
    const double start_to_end_sq = start_to_end.squaredNorm();
    const double projection_dot = start_to_point.dot(start_to_end);

    // Monotonic advancement check: reject if point projects before start or
    // after end. Check bounds BEFORE dividing to avoid division-by-near-zero
    // issues. Since start_to_end_sq > 0 (checked above), we can multiply
    // through: t < 0 becomes dot < 0, t > 1 becomes dot > start_to_end_sq.
    if (projection_dot < 0.0 || projection_dot > start_to_end_sq) {
        return false;
    }

    const double t = projection_dot / start_to_end_sq;
    const auto projected_point = line_start + t * start_to_end;
    const double deviation_sq = (point - projected_point).squaredNorm();

    const double radius = tolerance / 2.0;
    return deviation_sq <= radius * radius;
}

void apply_colinearization(std::list<Eigen::VectorXd>& waypoints, double tolerance) {
    if (waypoints.size() <= 2 || tolerance <= 0.0) {
        return;
    }

    auto anchor = waypoints.begin();

    for (auto locus = std::next(waypoints.begin()); locus != waypoints.end(); ++locus) {
        auto next = std::next(locus);
        const bool at_last = (next == waypoints.end());

        // Try to extend the cylinder to skip locus
        if (!at_last) {
            // Degenerate segment (anchor == next) represents an intentional return
            // to the same position -- must preserve the intermediate waypoint
            if (((*anchor - *next).array() == 0.0).all()) {
                waypoints.erase(std::next(anchor), locus);
                anchor = locus;
                continue;
            }

            if (within_colinearization_tolerance(*locus, *anchor, *next, tolerance)) {
                // Revalidate all waypoints between anchor and locus against
                // the extended cylinder to prevent drift
                bool all_previous_valid = true;
                for (auto it = std::next(anchor); it != locus; ++it) {
                    if (!within_colinearization_tolerance(*it, *anchor, *next, tolerance)) {
                        all_previous_valid = false;
                        break;
                    }
                }

                if (all_previous_valid) {
                    continue;
                }
            }
        }

        // Cannot extend cylinder (or reached last waypoint): emit the segment
        waypoints.erase(std::next(anchor), locus);
        anchor = locus;
    }
}

}  // namespace viam::trajex
