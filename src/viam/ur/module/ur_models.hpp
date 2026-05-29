#pragma once

#include <string>
#include <vector>

#include <viam/sdk/resource/resource_api.hpp>

struct UrModelDescriptor {
    std::string sdk_name;
    std::string urcl_category;
    std::vector<std::string> mesh_parts;
};

// Returns the descriptor for an SDK model name. Throws
// std::invalid_argument if the name is not registered.
const UrModelDescriptor& ur_model_descriptor(const std::string& sdk_name);

// Returns all registered descriptors, in registration order. Used by
// `URArm::create_model_registrations` so adding a new arm is a single-site
// change here.
const std::vector<UrModelDescriptor>& ur_model_descriptors();

// Value type pairing an SDK `Model` with its `UrModelDescriptor`. The
// canonical "what UR variant am I" carrier inside the module -- replacing
// the std::string model-name that used to flow through `URArm`, `state_`,
// and `cached_kinematics_payload`. Cheap to copy (one Model, one pointer).
//
// Equality is descriptor-pointer identity: two `UrArmModel`s are equal iff
// they describe the same registered UR variant. The contained SDK `Model`
// is not part of the equality check (in practice it is fully determined by
// the descriptor's `sdk_name` plus `URArm::model_family()`).
class UrArmModel {
   public:
    // Build from an existing SDK Model. The model's `model_name()` must
    // identify a registered UR variant.
    explicit UrArmModel(viam::sdk::Model sdk_model);

    // Build from an SDK model name (e.g. "ur5e"). Synthesizes the SDK
    // Model in `URArm::model_family()`. Throws std::invalid_argument if
    // the name is not registered.
    static UrArmModel from_sdk_name(const std::string& sdk_name);

    const viam::sdk::Model& sdk_model() const& {
        return sdk_model_;
    }
    const std::string& sdk_name() const& {
        return descriptor_->sdk_name;
    }
    const std::string& urcl_category() const& {
        return descriptor_->urcl_category;
    }
    const std::vector<std::string>& mesh_parts() const& {
        return descriptor_->mesh_parts;
    }
    const UrModelDescriptor& descriptor() const& {
        return *descriptor_;
    }

    friend bool operator==(const UrArmModel& a, const UrArmModel& b) noexcept {
        return a.descriptor_ == b.descriptor_;
    }
    friend bool operator!=(const UrArmModel& a, const UrArmModel& b) noexcept {
        return !(a == b);
    }

   private:
    UrArmModel(viam::sdk::Model sdk_model, const UrModelDescriptor& descriptor);

    viam::sdk::Model sdk_model_;
    const UrModelDescriptor* descriptor_;
};
