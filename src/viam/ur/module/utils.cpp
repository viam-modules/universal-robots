#include "utils.hpp"

#include <ur_client_library/log.h>

#include <viam/sdk/log/logging.hpp>
#include <viam/sdk/resource/resource.hpp>

void configure_logger(const viam::sdk::ResourceConfig& cfg) {
    std::string level_str{};
    try {
        level_str = find_config_attribute<std::string>(cfg, "log_level");
    } catch (...) {
        level_str = "warn";
    }
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
