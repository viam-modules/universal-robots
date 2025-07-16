#include <ur_client_library/log.h>

#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/log/logging.hpp>

// helper function to extract an attribute value from its key within a ResourceConfig
template <class T>
T find_config_attribute(const ResourceConfig& cfg, const std::string& attribute) {
    std::ostringstream buffer;
    auto key = cfg.attributes().find(attribute);
    if (key == cfg.attributes().end()) {
        buffer << "required attribute `" << attribute << "` not found in configuration";
        throw std::invalid_argument(buffer.str());
    }
    const auto* const val = key->second.get<T>();
    if (!val) {
        buffer << "required non-empty attribute `" << attribute << " could not be decoded";
        throw std::invalid_argument(buffer.str());
    }
    return *val;
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

void configure_logger(const ResourceConfig& cfg) {
    std::string level_str{};
    try {
        level_str = find_config_attribute<std::string>(cfg, "log_level");
    } catch (...) {
        level_str = "debug";
    }
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
            VIAM_SDK_LOG(error) << "invalid log_level: '" << level_str << "' - defaulting to 'debug'";
            return urcl::LogLevel::DEBUG;
        }
    }();
    urcl::setLogLevel(level);
    urcl::registerLogHandler(std::make_unique<URArmLogHandler>());
}
