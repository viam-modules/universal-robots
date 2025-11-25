#pragma once

#include <list>
#include <numbers>
#include <optional>
#include <sstream>

#include <Eigen/Dense>

#include <ur_client_library/log.h>
#include <ur_client_library/types.h>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

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

template <typename T>
[[nodiscard]] constexpr decltype(auto) degrees_to_radians(T&& degrees) {
    return std::forward<T>(degrees) * (std::numbers::pi / 180.0);
}

template <typename T>
[[nodiscard]] constexpr decltype(auto) radians_to_degrees(T&& radians) {
    return std::forward<T>(radians) * (180.0 / std::numbers::pi);
}

[[nodiscard]] urcl::vector6d_t degrees_to_radians(urcl::vector6d_t degrees);
[[nodiscard]] urcl::vector6d_t radians_to_degrees(urcl::vector6d_t radians);

Eigen::Matrix3d rotation_vector_to_matrix(const urcl::vector6d_t& tcp_pose);

Eigen::Vector3d transform_vector(const Eigen::Vector3d& vector, const Eigen::Matrix3d& rotation_matrix);

urcl::vector6d_t convert_tcp_force_to_tool_frame(const urcl::vector6d_t& tcp_pose, const urcl::vector6d_t& tcp_force_base_frame);

///
/// Check if point is within tolerance cylinder around line segment.
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
