#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>

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

std::optional<double> parse_stream_limit_tolerance_ratio(const viam::sdk::ResourceConfig& cfg) {
    auto ratio = find_config_attribute<double>(cfg, "stream_limit_tolerance_ratio");
    if (ratio && *ratio < 1.0) {
        throw std::invalid_argument(
            boost::str(boost::format("attribute `stream_limit_tolerance_ratio` must be >= 1, it is: %1%") % *ratio));
    }
    return ratio;
}

void check_joint_constraints(std::string_view kind, const std::vector<double>& values, const vector6d_t& limits) {
    using boost::format;
    using boost::str;

    if (values.size() != limits.size()) {
        throw std::invalid_argument(str(format("trajectory point has %1% %2% values but %3% DoF") % values.size() % kind % limits.size()));
    }
    for (size_t i = 0; i < values.size(); ++i) {
        if (std::abs(values[i]) > limits[i]) {
            throw std::invalid_argument(str(format("joint %1% %2% %3% exceeds limit %4%") % i % kind % values[i] % limits[i]));
        }
    }
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

void apply_move_limit(vector6d_t& limits, const boost::variant<double, std::vector<double>>& value) {
    struct visitor {
        vector6d_t& limits;

        void operator()(double s) const {
            if (s <= 0) {
                throw std::invalid_argument("move limit scalar must be positive, got: " + std::to_string(s));
            }
            limits.fill(degrees_to_radians(s));
        }

        void operator()(const std::vector<double>& v) const {
            if (v.size() != k_ur_arm_dof) {
                throw std::invalid_argument("move limit vector must have exactly " + std::to_string(k_ur_arm_dof) +
                                            " elements (one per joint), got " + std::to_string(v.size()));
            }
            for (size_t i = 0; i < v.size(); ++i) {
                if (v[i] < 0) {
                    throw std::invalid_argument("move limit element " + std::to_string(i) +
                                                " cannot be negative, got: " + std::to_string(v[i]));
                }
            }
            for (size_t i = 0; i < v.size(); ++i) {
                limits[i] = degrees_to_radians(v[i]);
            }
        }
    };
    boost::apply_visitor(visitor{limits}, value);
}

move_extras parse_move_extras(const viam::sdk::ProtoStruct& extra) {
    const auto is_false = [&](const char* key) {
        const auto it = extra.find(key);
        if (it == extra.end()) {
            return false;
        }
        const auto* const b = it->second.get<bool>();
        return b && !*b;
    };
    return move_extras{.wait_at_end = !is_false("waitAtEnd"), .interpolate = !is_false("interpolate")};
}

servo_chaser::servo_chaser(vector6d_t start_rad) : position_{start_rad} {}

const vector6d_t& servo_chaser::step(const vector6d_t& target_rad,
                                     const vector6d_t& velocity_limits_rad,
                                     const vector6d_t& acceleration_limits_rad,
                                     double dt) {
    if (!(dt > 0.0)) {
        throw std::invalid_argument("servo_chaser::step: dt must be positive");
    }

    moving_ = false;
    for (size_t i = 0; i < position_.size(); ++i) {
        const double error = target_rad[i] - position_[i];
        const double v_max = std::abs(velocity_limits_rad[i]);
        const double a_max = std::abs(acceleration_limits_rad[i]);
        const double a_step = a_max * dt;

        // Cruise speed, the braking envelope, or just reaching the target this step: whichever is
        // slowest. The envelope is the discrete one: shedding `a_step` of velocity per period from
        // `v` covers `v * (v + a_step) / (2 * a_max)` before stopping, so that is what must fit in
        // the remaining error. (The continuous `sqrt(2 * a * error)` cannot be tracked in steps
        // and leaves the chaser arriving too fast.) Then bound the change from the current
        // velocity by the acceleration limit.
        const double v_brake = 0.5 * (-a_step + std::sqrt((a_step * a_step) + (8.0 * a_max * std::abs(error))));
        const double v_reach = std::min({v_max, v_brake, std::abs(error) / dt});
        const double v_desired = std::copysign(v_reach, error);
        const double v_new = std::clamp(v_desired, velocity_[i] - a_step, velocity_[i] + a_step);
        const double p_new = position_[i] + (v_new * dt);

        // Snap onto the target once this step would reach or cross it on the final approach,
        // where the envelope has the velocity down to a couple of acceleration steps and discarding
        // it is no larger a change than any other step. Crossing it any faster than that (the
        // target moved to just ahead of a chaser in full flight) is a genuine overshoot: let it
        // happen, and the next steps decelerate, reverse, and come back under the same limits.
        if (error != 0.0 && (target_rad[i] - p_new) * error <= 0.0 && std::abs(v_new) <= 2.0 * a_step) {
            position_[i] = target_rad[i];
            velocity_[i] = 0.0;
            moving_ = true;
            continue;
        }

        moving_ = moving_ || p_new != position_[i] || v_new != 0.0;
        position_[i] = p_new;
        velocity_[i] = v_new;
    }
    return position_;
}

const vector6d_t& servo_chaser::position() const {
    return position_;
}

const vector6d_t& servo_chaser::velocity() const {
    return velocity_;
}

bool servo_chaser::at(const vector6d_t& target_rad) const {
    return position_ == target_rad && velocity_ == vector6d_t{};
}

bool servo_chaser::settled() const {
    return !moving_;
}
