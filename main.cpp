#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/rpc/dial.hpp>
#include <viam/sdk/rpc/server.hpp>

#include "src/ur5e_arm.hpp"

using namespace viam::sdk;

int main(int argc, char** argv) {
    const Instance instance;

    const API arm_api = API::get<Arm>();
    const Model ur5e_model("viam", "universal-robots", "ur5e");

    const auto ur5e_mr = std::make_shared<ModelRegistration>(
        arm_api, ur5e_model, [](const Dependencies& dep, const ResourceConfig& cfg) { return std::make_unique<URArm>(dep, cfg); });

    const std::vector<std::shared_ptr<ModelRegistration>> mrs = {ur5e_mr};
    std::make_shared<ModuleService>(argc, argv, mrs)->serve();

    return EXIT_SUCCESS;
};
