#include "src/UR5e-Arm.hpp"
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/module.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/rpc/dial.hpp>
#include <viam/sdk/rpc/server.hpp>

using namespace viam::sdk;

int main(int argc, char** argv) {
    std::cout << "program started\n";
    API arm_api = API::get<Arm>();
    Model ur5e_model("viamrobotics", "arm", "ur5earm");

    std::shared_ptr<ModelRegistration> ur5e_mr = std::make_shared<ModelRegistration>(
        arm_api,
        ur5e_model,
        [](Dependencies dep, ResourceConfig cfg) { return std::make_unique<UR5eArm>(dep, cfg); });

    std::vector<std::shared_ptr<ModelRegistration>> mrs = {ur5e_mr};
    auto my_mod = std::make_shared<ModuleService>(argc, argv, mrs);
    my_mod->serve();

    return EXIT_SUCCESS;
};