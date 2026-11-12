#pragma once

#include <memory>
#include <string>
#include <vector>

#include <viam/sdk/components/arm.hpp>
#include <viam/sdk/components/sensor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>

class URFTSensor final : public viam::sdk::Sensor {
   public:
    static std::shared_ptr<viam::sdk::ModelRegistration> create_model_registration();

    static std::vector<std::string> validate(const viam::sdk::ResourceConfig& cfg);

    static viam::sdk::ProtoStruct wrench_from_arm_response(const viam::sdk::ProtoStruct& resp);

    URFTSensor(const viam::sdk::Dependencies& deps, const viam::sdk::ResourceConfig& cfg);

    viam::sdk::ProtoStruct get_readings(const viam::sdk::ProtoStruct& extra) override;

    viam::sdk::ProtoStruct do_command(const viam::sdk::ProtoStruct& command) override;

    viam::sdk::ProtoStruct get_status() override;

    std::vector<viam::sdk::GeometryConfig> get_geometries(const viam::sdk::ProtoStruct& extra) override;

   private:
    std::shared_ptr<viam::sdk::Arm> arm_;
};
