#pragma once

#include <filesystem>
#include <list>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <boost/variant.hpp>

#include <Eigen/Dense>

#include <ur_client_library/log.h>
#include <ur_client_library/types.h>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

#include <viam/trajex/types/angles.hpp>

inline constexpr auto k_ur_arm_dof = std::tuple_size_v<urcl::vector6d_t>;

void configure_logger(const viam::sdk::ResourceConfig& cfg);

// helper function to extract an attribute value from its key within a ResourceConfig
template <class T>
std::optional<T> find_config_attribute(const viam::sdk::ResourceConfig& cfg, const std::string& attribute) {
    auto key = cfg.attributes().find(attribute);
    if (key == cfg.attributes().end()) {
        return std::nullopt;
    }
    const auto* const val = key->second.get<T>();
    if (!val) {
        std::ostringstream buffer;
        buffer << "attribute `" << attribute << " could not be converted to the required type";
        throw std::invalid_argument(buffer.str());
    }
    return std::make_optional(*val);
}

class URArmLogHandler : public urcl::LogHandler {
   public:
    URArmLogHandler() = default;
    void log(const char* file, int line, urcl::LogLevel loglevel, const char* log) override {
        std::ostringstream os;
        os << "URCL - " << file << " " << line << ": " << log;
        const std::string logMsg = os.str();

        switch (loglevel) {
            case urcl::LogLevel::INFO:
                VIAM_SDK_LOG(info) << logMsg;
                break;
            case urcl::LogLevel::DEBUG:
                VIAM_SDK_LOG(debug) << logMsg;
                break;
            case urcl::LogLevel::WARN:
                VIAM_SDK_LOG(warn) << logMsg;
                break;
            case urcl::LogLevel::ERROR:
                VIAM_SDK_LOG(error) << logMsg;
                break;
            case urcl::LogLevel::FATAL:
                VIAM_SDK_LOG(error) << logMsg;
                break;
            default:
                break;
        }
    }
};

// Import angle conversion functions from trajex
using viam::trajex::degrees_to_radians;
using viam::trajex::radians_to_degrees;

[[nodiscard]] urcl::vector6d_t degrees_to_radians(urcl::vector6d_t degrees);
[[nodiscard]] urcl::vector6d_t radians_to_degrees(urcl::vector6d_t radians);

Eigen::Matrix3d rotation_vector_to_matrix(const urcl::vector6d_t& tcp_pose);

Eigen::Vector3d transform_vector(const Eigen::Vector3d& vector, const Eigen::Matrix3d& rotation_matrix);

urcl::vector6d_t convert_tcp_force_to_tool_frame(const urcl::vector6d_t& tcp_pose, const urcl::vector6d_t& tcp_force_base_frame);

///
/// Deduplicate waypoints in-place using L-infinity norm.
///
/// Removes consecutive waypoints that are within tolerance of each other using
/// the L-infinity (maximum) norm. Always keeps the first waypoint.
///
/// @param waypoints List of waypoints to deduplicate (modified in-place)
/// @param tolerance Maximum L-infinity distance for waypoints to be considered duplicates (in radians)
///
void deduplicate_waypoints(std::list<Eigen::VectorXd>& waypoints, double tolerance);

///
/// Apply a move-limit override to a 6-DOF limits array.
///
/// Scalar: fills all joints uniformly with the given value (degrees, converted to radians).
/// Vector: sets per-joint limits. Requires exactly k_ur_arm_dof elements.
///
/// @param limits Array to modify in-place (in radians)
/// @param value Scalar (degrees) or per-joint vector (degrees)
/// @throws std::invalid_argument on non-positive scalar, wrong vector size, or negative vector elements
///
void apply_move_limit(urcl::vector6d_t& limits, const boost::variant<double, std::vector<double>>& value);

///
/// Parse and validate velocity or acceleration limits.
///
/// Returns a 6-element vector in radians. Input (in degrees) can be scalar
/// (expands to 6 identical values) or 6-element array. Rejects negative values,
/// scalar zero, or all-zero arrays.
///
/// @param value ProtoValue containing scalar double or 6-element array
/// @param param_name Name of the parameter (for error messages)
/// @return 6-element vector of limits in radians
/// @throws std::invalid_argument if validation fails
///
urcl::vector6d_t parse_and_validate_joint_limits(const viam::sdk::ProtoValue& value, const std::string& param_name);

///
/// Parse and validate velocity or acceleration limits from config.
///
/// Extracts the named parameter from config attributes and validates it.
///
/// @param cfg Configuration structure to read from
/// @param param_name Name of the parameter to read
/// @return 6-element vector of limits in radians
/// @throws std::invalid_argument if validation fails
///
urcl::vector6d_t parse_and_validate_joint_limits(const viam::sdk::ResourceConfig& cfg, const std::string& param_name);

///
/// Parse and validate `stream_limit_tolerance_ratio` from config.
///
/// @param cfg Configuration structure to read from
/// @return The configured ratio, or std::nullopt if the attribute is absent
/// @throws std::invalid_argument if the ratio is less than 1
///
std::optional<double> parse_stream_limit_tolerance_ratio(const viam::sdk::ResourceConfig& cfg);

///
/// Check per-joint values from a streamed trajectory point against per-joint limits.
///
/// Values are compared by magnitude, so negative values are checked too. `values` and `limits`
/// must be in the same units.
///
/// @param kind What the values are, e.g. "velocity" (for error messages)
/// @param values One value per joint
/// @param limits One limit per joint
/// @throws std::invalid_argument if `values` has the wrong number of joints or any value exceeds its limit
///
void check_joint_constraints(std::string_view kind, const std::vector<double>& values, const urcl::vector6d_t& limits);

///
/// Extract trace-id from W3C Trace Context traceparent header.
///
/// @param traceparent Header value, e.g. "00-0af7651916cd43dd8448eb211c80319c-b7ad6b7169203331-01"
/// @return The 32-character hex trace-id, or std::nullopt if parsing fails
///
std::optional<std::string> extract_trace_id_from_traceparent(std::string_view traceparent);

///
/// Expand telemetry path by applying a trace-id template.
///
/// Replaces `{trace_id}` in the template with the actual trace-id and appends
/// the result as a subdirectory of the base path.
///
/// @param base_path The telemetry output base directory
/// @param traceid_template Template string containing `{trace_id}`, e.g. "tag={trace_id}"
/// @param trace_id The actual trace-id to substitute
/// @return The expanded path, e.g. base_path / "tag=0af765..."
///
std::filesystem::path expand_telemetry_path(const std::filesystem::path& base_path,
                                            const std::string& traceid_template,
                                            const std::string& trace_id);

///
/// Per-move options carried in the `extra` map of `move_to_joint_positions` and
/// `move_through_joint_positions`, following the convention shared by the RDK teleop pipeline and
/// the ufactory module: `{"waitAtEnd": false}` returns as soon as the move has been handed to the
/// arm, and `{"interpolate": false}` sends the targets to the arm as servo setpoints instead of
/// planning a trajectory through them. An absent key, or a value that is not boolean `false`,
/// leaves the default.
///
struct move_extras {
    bool wait_at_end = true;
    bool interpolate = true;
};

move_extras parse_move_extras(const viam::sdk::ProtoStruct& extra);

///
/// Chases a moving joint-space target with bounded velocity and acceleration.
///
/// Each `step` advances the commanded position toward the target by one control period. Per
/// joint, the velocity is capped at the velocity limit, at the speed from which the joint can
/// still decelerate to rest exactly at the target, and at the speed that would reach the target
/// within the step; the change in velocity between steps is capped by the acceleration limit. The
/// commanded positions are what we send the controller as servo setpoints, so a target that jumps
/// (teleop retargeting mid-motion) produces a bounded correction rather than a discontinuity: the
/// commanded velocity never changes by more than three acceleration steps between periods, and
/// the per-step change in position stays well inside the speed guard the external control script
/// applies to each setpoint.
///
class servo_chaser {
   public:
    explicit servo_chaser(urcl::vector6d_t start_rad);

    /// Advance one period of `dt` seconds toward `target_rad` and return the new commanded position.
    const urcl::vector6d_t& step(const urcl::vector6d_t& target_rad,
                                 const urcl::vector6d_t& velocity_limits_rad,
                                 const urcl::vector6d_t& acceleration_limits_rad,
                                 double dt);

    const urcl::vector6d_t& position() const;
    const urcl::vector6d_t& velocity() const;

    /// True once the commanded position equals `target_rad` and the commanded velocity is zero.
    bool at(const urcl::vector6d_t& target_rad) const;

    /// True when the last `step` neither moved the commanded position nor left any joint with a
    /// nonzero commanded velocity. Covers `at` as well as a joint that cannot move because its
    /// limits are zero.
    bool settled() const;

   private:
    urcl::vector6d_t position_;
    urcl::vector6d_t velocity_{};
    bool moving_ = false;
};
