#include "ur_models.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ur_arm.hpp"

const std::vector<UrModelDescriptor>& UrModelDescriptor::all() {
    static const std::vector<UrModelDescriptor> table = [] {
        std::vector<UrModelDescriptor> t;
        // ur3e/ur7e have no shipped meshes; `get_3d_models` returns an
        // empty map for them.
        t.push_back({"ur3e", "ur3", {}});
        t.push_back(
            {"ur5e", "ur5", {"base_link", "ee_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}});
        t.push_back({"ur7e", "ur5", {}});
        t.push_back({"ur20",
                     "ur20",
                     {"base_link", "wrist_3_link", "shoulder_link", "forearm_link", "upper_arm_link", "wrist_1_link", "wrist_2_link"}});
        return t;
    }();
    return table;
}

const UrModelDescriptor& UrModelDescriptor::for_sdk_name(const std::string& sdk_name) {
    for (const auto& d : all()) {
        if (d.sdk_name == sdk_name) {
            return d;
        }
    }
    throw std::invalid_argument("UrModelDescriptor::for_sdk_name: unknown sdk model name `" + sdk_name + "`");
}

UrArmModel::UrArmModel(viam::sdk::Model sdk_model, const UrModelDescriptor& descriptor)
    : sdk_model_(std::move(sdk_model)), descriptor_(&descriptor) {}

UrArmModel::UrArmModel(viam::sdk::Model sdk_model)
    // sdk_model_ is initialized in declaration order before descriptor_,
    // so reading the moved-into sdk_model_ for the descriptor lookup here
    // is well-defined (a delegating-constructor form risks reading from
    // the moved-from parameter, depending on argument evaluation order).
    : sdk_model_(std::move(sdk_model)), descriptor_(&UrModelDescriptor::for_sdk_name(sdk_model_.model_name())) {}

UrArmModel UrArmModel::from_sdk_name(const std::string& sdk_name) {
    return UrArmModel{URArm::model(sdk_name), UrModelDescriptor::for_sdk_name(sdk_name)};
}
