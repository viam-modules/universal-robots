#include <iterator>
#include <utility>
#include <vector>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/module/service.hpp>

#include "ur_arm.hpp"
#include "ur_arm_simulated.hpp"

using namespace viam::sdk;

namespace {

// Combine the hardware UR arm models (ur3e, ur5e, ur7e, ur20) with the simulated arm
// model (simulated-arm) into a single registration list for the module service.
std::vector<std::shared_ptr<ModelRegistration>> all_model_registrations() {
    auto registrations = URArm::create_model_registrations();
    auto simulated = URArmSimulated::create_model_registrations();
    registrations.insert(registrations.end(), std::make_move_iterator(simulated.begin()), std::make_move_iterator(simulated.end()));
    return registrations;
}

}  // namespace

int main(int argc, char** argv) {
    const Instance instance;
    std::make_shared<ModuleService>(argc, argv, all_model_registrations())->serve();
    return EXIT_SUCCESS;
};
