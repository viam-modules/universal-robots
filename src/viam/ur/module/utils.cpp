#include "utils.hpp"

#include <ranges>

#include <ur_client_library/log.h>

#include <Eigen/Dense>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/split.hpp>
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

std::optional<std::string> extract_trace_id_from_traceparent(std::string_view traceparent) {
    std::vector<std::string_view> parts;
    boost::split(parts, traceparent, boost::is_any_of("-"));  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)

    // W3C Trace Context format requires exactly 4 components
    if (parts.size() != 4) {
        return std::nullopt;
    }

    // Validate version field (parts[0]) - should be "00" for current spec
    if (parts[0] != "00") {
        return std::nullopt;
    }

    // Validate trace-id (parts[1]) - must be 32 hex characters
    if (parts[1].size() != 32) {
        return std::nullopt;
    }

    // Validate parent-id (parts[2]) - must be 16 hex characters
    if (parts[2].size() != 16) {
        return std::nullopt;
    }

    // Validate trace-flags (parts[3]) - must be 2 hex characters
    if (parts[3].size() != 2) {
        return std::nullopt;
    }

    return std::string(parts[1]);
}

std::filesystem::path expand_telemetry_path(const std::filesystem::path& base_path,
                                            const std::string& traceid_template,
                                            const std::string& trace_id) {
    auto expanded = boost::replace_all_copy(traceid_template, "{trace_id}", trace_id);
    return base_path / expanded;
}
