#include <viam/trajex/totg/waypoint_utils.hpp>

#include <cmath>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/core/xmath.hpp>
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xmath.hpp>
#include <xtensor/xnorm.hpp>
#endif

namespace viam::trajex::totg {

waypoint_accumulator deduplicate_waypoints(const waypoint_accumulator& waypoints, double tolerance) {
    if (waypoints.empty()) {
        throw std::invalid_argument("Cannot deduplicate empty waypoints");
    }

    // Always keep first waypoint
    waypoint_accumulator result(waypoints[0]);

    // Keep waypoints that differ from previous by more than tolerance
    for (size_t i = 1; i < waypoints.size(); ++i) {
        if (xt::norm_linf(waypoints[i] - waypoints[i - 1])() > tolerance) {
            result.add_waypoint(waypoints[i]);
        }
    }

    return result;
}

std::vector<waypoint_accumulator> segment_at_reversals(waypoint_accumulator waypoints, double threshold) {
    std::vector<waypoint_accumulator> segments;

    // There can be no cusps unless there are at least three waypoints.
    if (waypoints.size() < 3) {
        segments.push_back(std::move(waypoints));
        return segments;
    }

    waypoint_accumulator current_segment(waypoints[0]);

    // Walk all interior points of the waypoints list, if any. If the point of current interest is the cusp
    // of a direction reversal w.r.t. the points immediately before and after it, then complete the current
    // segment with all waypoints up to and including the cusp point into a new segment, and then begin
    // accumulating a new segment starting at the cusp point. The cusp point is duplicated, forming both the
    // end of one segment and the beginning of the next segment. After exiting the loop, any remaining waypoints
    // form the last (and if no cusps were identified the only) segment. If one or more cusp points were
    // identified, the segments vector will always have at least two entries, since the last waypoint is never
    // examined as a cusp candidate.
    for (auto where = std::next(waypoints.begin()); where != std::prev(waypoints.end()); ++where) {
        const auto segment_ab = *where - *std::prev(where);
        const auto segment_bc = *std::next(where) - *where;

        const auto ab_normalized = segment_ab / xt::norm_l2(segment_ab)();
        const auto bc_normalized = segment_bc / xt::norm_l2(segment_bc)();
        const auto dot = xt::sum(ab_normalized * bc_normalized)();

        if (std::abs(dot + 1.0) < threshold) {
            current_segment.add_waypoint(*where);
            segments.push_back(std::move(current_segment));
            current_segment = waypoint_accumulator(*where);
        } else {
            current_segment.add_waypoint(*where);
        }
    }

    current_segment.add_waypoint(*std::prev(waypoints.end()));
    segments.push_back(std::move(current_segment));

    return segments;
}

}  // namespace viam::trajex::totg
