#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/module/service.hpp>

#include "ur_arm.hpp"
#include "ur_ft_sensor.hpp"

using namespace viam::sdk;

int main(int argc, char** argv) {
    const Instance instance;
    auto registrations = URArm::create_model_registrations();
    registrations.push_back(URFTSensor::create_model_registration());
    std::make_shared<ModuleService>(argc, argv, registrations)->serve();
    return EXIT_SUCCESS;
};
