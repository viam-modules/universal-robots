#include "utils.hpp"

#include <ur_client_library/log.h>

#include <Eigen/Dense>
#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/resource/resource.hpp>

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
