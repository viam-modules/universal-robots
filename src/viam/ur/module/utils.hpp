#pragma once

#include <filesystem>
#include <list>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>

#include <Eigen/Dense>

#include <ur_client_library/log.h>
#include <ur_client_library/types.h>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

#include <viam/trajex/service/colinearization.hpp>
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

// Colinearization functions live in trajex service layer. Import them here
// for backward compatibility with existing call sites (test.cpp).
using viam::trajex::apply_colinearization;
using viam::trajex::within_colinearization_tolerance;

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
