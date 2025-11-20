#include "utils.hpp"

#include <ur_client_library/log.h>

#include <Eigen/Dense>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/resource/resource.hpp>

using urcl::vector6d_t;

void configure_logger(const viam::sdk::ResourceConfig& cfg) {
    auto level_str = find_config_attribute<std::string>(cfg, "log_level").value_or("warn");

    VIAM_SDK_LOG(debug) << "setting URArm log level to '" << level_str << "'";
    const auto level = [&] {
        if (level_str == "info") {
            return urcl::LogLevel::INFO;
        } else if (level_str == "debug") {
            return urcl::LogLevel::DEBUG;
        } else if (level_str == "warn") {
            return urcl::LogLevel::WARN;
        } else if (level_str == "error") {
            return urcl::LogLevel::ERROR;
        } else if (level_str == "fatal") {
            return urcl::LogLevel::FATAL;
        } else {
            VIAM_SDK_LOG(error) << "invalid log_level: '" << level_str << "' - defaulting to 'warn'";
            return urcl::LogLevel::WARN;
        }
    }();

    urcl::setLogLevel(level);
    urcl::registerLogHandler(std::make_unique<URArmLogHandler>());
}

Eigen::Matrix3d rotation_vector_to_matrix(const vector6d_t& tcp_pose) {
    const Eigen::Vector3d rotation_vector(tcp_pose[3], tcp_pose[4], tcp_pose[5]);
    const double angle = rotation_vector.norm();

    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }

    const Eigen::Vector3d axis = rotation_vector / angle;
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

Eigen::Vector3d transform_vector(const Eigen::Vector3d& vector, const Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix.transpose() * vector;
}

vector6d_t convert_tcp_force_to_tool_frame(const vector6d_t& tcp_pose, const vector6d_t& tcp_force_base_frame) {
    const Eigen::Matrix3d rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    const Eigen::Vector3d force_base(tcp_force_base_frame[0], tcp_force_base_frame[1], tcp_force_base_frame[2]);
    const Eigen::Vector3d torque_base(tcp_force_base_frame[3], tcp_force_base_frame[4], tcp_force_base_frame[5]);

    const Eigen::Vector3d force_tool = transform_vector(force_base, rotation_matrix);
    const Eigen::Vector3d torque_tool = transform_vector(torque_base, rotation_matrix);

    return {force_tool[0], force_tool[1], force_tool[2], torque_tool[0], torque_tool[1], torque_tool[2]};
}

// Check if point is within tolerance cylinder around line segment
// Returns false if segment is degenerate (start == end)
// tolerance parameter is interpreted as DIAMETER (radius = tolerance/2)
bool within_colinearization_tolerance(const Eigen::VectorXd& point,
                                      const Eigen::VectorXd& line_start,
                                      const Eigen::VectorXd& line_end,
                                      double tolerance) {
    const auto start_to_end = line_end - line_start;

    // Check if start and end are exactly the same position (degenerate segment)
    // Use exact zero check - if they're the same point, all components are identically zero
    if ((start_to_end.array() == 0.0).all()) {
        // Degenerate segment - cannot form meaningful cylinder
        return false;
    }

    const auto start_to_point = point - line_start;
    const double start_to_end_sq = start_to_end.squaredNorm();
    const double projection_dot = start_to_point.dot(start_to_end);

    // Monotonic advancement check: reject if point would project before start
    // or after end. This ensures we only coalesce waypoints that maintain forward
    // progress along the segment direction.
    //
    // Check bounds BEFORE dividing to avoid division-by-near-zero issues.
    // Since start_to_end_sq > 0 (checked above), we can multiply through the inequality:
    //   t < 0  becomes  dot < 0
    //   t > 1  becomes  dot > start_to_end_sq
    if (projection_dot < 0.0 || projection_dot > start_to_end_sq) {
        return false;  // Point represents non-monotonic movement, must preserve
    }

    // Now safe to divide: bounds check guarantees 0 <= t <= 1
    const double t = projection_dot / start_to_end_sq;
    const auto projected_point = line_start + t * start_to_end;
    const double deviation_sq = (point - projected_point).squaredNorm();

    // Interpret tolerance as diameter: radius = tolerance / 2
    const double radius = tolerance / 2.0;
    const double radius_sq = radius * radius;

    return deviation_sq <= radius_sq;
}

// Apply colinearization: remove waypoints within tolerance of line segments
//
// This implements waypoint coalescing by extending a "tolerance cylinder" from an anchor
// waypoint forward, skipping intermediate waypoints that remain within tolerance of
// the straight-line segment. Critically, when extending the cylinder to a new endpoint,
// all previously skipped waypoints are revalidated to prevent drift where
// early waypoints might have been within tolerance of a shorter segment but violate
// tolerance when the cylinder is extended further.
void apply_colinearization(std::list<Eigen::VectorXd>& waypoints, double tolerance) {
    if (waypoints.size() <= 2 || tolerance <= 0.0) {
        return;  // Nothing to coalesce
    }

    auto anchor = waypoints.begin();

    for (auto locus = std::next(waypoints.begin()); locus != waypoints.end(); ++locus) {
        auto next = std::next(locus);
        const bool at_last = (next == waypoints.end());

        // Try to extend the cylinder to skip locus
        if (!at_last) {
            // Check for degenerate segment (anchor == next)
            // This represents an intentional return to the same position - must preserve
            if (((*anchor - *next).array() == 0.0).all()) {
                // Degenerate segment: cannot coalesce, emit current cylinder
                waypoints.erase(std::next(anchor), locus);
                anchor = locus;
                continue;
            }

            // Check if locus is within tolerance of the extended cylinder (anchor to next)
            if (within_colinearization_tolerance(*locus, *anchor, *next, tolerance)) {
                // Revalidate all waypoints between anchor and locus against the extended cylinder.
                // This prevents drift: a waypoint that was within tolerance of
                // anchor→candidate might violate tolerance of anchor→endpoint.
                bool all_previous_valid = true;
                for (auto it = std::next(anchor); it != locus; ++it) {
                    if (!within_colinearization_tolerance(*it, *anchor, *next, tolerance)) {
                        all_previous_valid = false;
                        break;
                    }
                }

                if (all_previous_valid) {
                    // Can extend cylinder - continue to next waypoint
                    continue;
                }
            }
        }

        // Cannot extend cylinder (or reached last waypoint): emit the segment from
        // anchor to locus by erasing all waypoints in the range [next(anchor), locus).
        waypoints.erase(std::next(anchor), locus);
        anchor = locus;
    }
}
