#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/module/service.hpp>

#include "src/ur_arm.hpp"

using namespace viam::sdk;

int main(int argc, char** argv) {
    const Instance instance;
    std::make_shared<ModuleService>(argc, argv, URArm::create_model_registrations())->serve();
    return EXIT_SUCCESS;
};
