#include "utils.hpp"

#include <ranges>

#include <ur_client_library/log.h>

#include <Eigen/Dense>
#include <boost/format.hpp>
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

vector6d_t degrees_to_radians(vector6d_t degrees) {
    for (auto& val : degrees) {
        val = degrees_to_radians(val);
    }
    return degrees;
}

vector6d_t radians_to_degrees(vector6d_t radians) {
    for (auto& val : radians) {
        val = radians_to_degrees(val);
    }
    return radians;
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

void deduplicate_waypoints(std::list<Eigen::VectorXd>& waypoints, double tolerance) {
    const auto close_enough = [tolerance](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> bool {
        return (a - b).lpNorm<Eigen::Infinity>() <= tolerance;
    };
    const auto result = std::ranges::unique(waypoints, close_enough);
    waypoints.erase(result.begin(), result.end());
}

vector6d_t parse_and_validate_joint_limits(const viam::sdk::ProtoValue& value, const std::string& param_name) {
    using boost::format;
    using boost::str;
    using viam::sdk::ProtoValue;

    vector6d_t result;

    if (const auto* scalar = value.get<double>()) {
        const double val = *scalar;
        if (val < 0.0) {
            throw std::invalid_argument(str(format("`%1%` cannot be negative, got: %2%") % param_name % val));
        }
        if (val == 0.0) {
            throw std::invalid_argument(str(format("`%1%` cannot be zero") % param_name));
        }
        result.fill(degrees_to_radians(val));
    } else if (const auto* arr = value.get<std::vector<ProtoValue>>()) {
        if (arr->size() != result.size()) {
            throw std::invalid_argument(
                str(format("`%1%` must be either a scalar or 6-element array, got %2% elements") % param_name % arr->size()));
        }

        bool all_zero = true;
        for (size_t i = 0; i < result.size(); ++i) {
            const auto* elem = (*arr)[i].get<double>();
            if (!elem) {
                throw std::invalid_argument(str(format("`%1%` array element %2% is not a number") % param_name % i));
            }
            const double val = *elem;
            if (val < 0.0) {
                throw std::invalid_argument(str(format("`%1%` element %2% cannot be negative, got: %3%") % param_name % i % val));
            }
            if (val != 0.0) {
                all_zero = false;
            }
            result[i] = degrees_to_radians(val);
        }

        if (all_zero) {
            throw std::invalid_argument(str(format("`%1%` cannot have all elements set to zero") % param_name));
        }
    } else {
        throw std::invalid_argument(str(format("`%1%` must be either a scalar number or 6-element array") % param_name));
    }

    return result;
}

vector6d_t parse_and_validate_joint_limits(const viam::sdk::ResourceConfig& cfg, const std::string& param_name) {
    using boost::format;
    using boost::str;

    const auto& attributes = cfg.attributes();
    auto key = attributes.find(param_name);
    if (key == attributes.end()) {
        throw std::invalid_argument(str(format("`%1%` is required") % param_name));
    }
    return parse_and_validate_joint_limits(key->second, param_name);
}
