#include "ur_ft_sensor.hpp"

#include <stdexcept>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/resource/resource_api.hpp>

#include "ur_arm.hpp"
#include "utils.hpp"

namespace {

constexpr char k_arm_attribute[] = "arm";

std::string arm_name(const viam::sdk::ResourceConfig& cfg) {
    auto name = find_config_attribute<std::string>(cfg, k_arm_attribute);
    if (!name || name->empty()) {
        throw std::invalid_argument("attribute `arm` is required");
    }
    return *name;
}

}  // namespace

std::shared_ptr<viam::sdk::ModelRegistration> URFTSensor::create_model_registration() {
    return std::make_shared<viam::sdk::ModelRegistration>(
        viam::sdk::API::get<viam::sdk::Sensor>(),
        URArm::model("ft_sensor"),
        // NOLINTNEXTLINE(performance-unnecessary-value-param): Signature is fixed by ModelRegistration.
        [](auto deps, auto config) { return std::make_unique<URFTSensor>(deps, config); },
        [](auto const& config) { return validate(config); });
}

std::vector<std::string> URFTSensor::validate(const viam::sdk::ResourceConfig& cfg) {
    return {arm_name(cfg)};
}

viam::sdk::ProtoStruct URFTSensor::wrench_from_arm_response(const viam::sdk::ProtoStruct& resp) {
    const auto it = resp.find(URArm::k_tcp_forces_tool_key);
    if (it == resp.end()) {
        throw std::runtime_error(std::string{"arm response is missing `"} + URArm::k_tcp_forces_tool_key + "`");
    }
    const auto* const wrench = it->second.get<viam::sdk::ProtoStruct>();
    if (!wrench) {
        throw std::runtime_error(std::string{"arm response `"} + URArm::k_tcp_forces_tool_key + "` is not a struct");
    }
    return *wrench;
}

URFTSensor::URFTSensor(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg) : Sensor(cfg.name()) {
    const auto name = arm_name(cfg);
    for (const auto& kv : deps) {
        if (kv.first.short_name() == name) {
            arm_ = std::dynamic_pointer_cast<viam::sdk::Arm>(kv.second);
        }
    }
    if (!arm_) {
        throw std::invalid_argument("arm `" + name + "` is not among the sensor's dependencies");
    }
}

viam::sdk::ProtoStruct URFTSensor::get_readings(const viam::sdk::ProtoStruct&) {
    return wrench_from_arm_response(arm_->do_command({{URArm::k_get_tcp_forces_tool_key, true}}));
}

viam::sdk::ProtoStruct URFTSensor::do_command(const viam::sdk::ProtoStruct& command) {
    if (!command.empty()) {
        throw std::runtime_error("unsupported do_command key: " + command.begin()->first);
    }
    return {};
}

viam::sdk::ProtoStruct URFTSensor::get_status() {
    return {};
}

std::vector<viam::sdk::GeometryConfig> URFTSensor::get_geometries(const viam::sdk::ProtoStruct&) {
    return {};
}
