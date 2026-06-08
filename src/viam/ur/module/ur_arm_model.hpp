#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Geometry>

#include <ur_client_library/types.h>

#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/resource/resource_api.hpp>
#include <viam/sdk/spatialmath/geometry.hpp>

struct UrModelDescriptor {
    std::string sdk_name;
    std::string urcl_category;
    std::vector<std::string> mesh_parts;

    // Returns the descriptor for an SDK model name. Throws
    // std::invalid_argument if the name is not registered.
    static const UrModelDescriptor& for_sdk_name(const std::string& sdk_name);

    // Returns all registered descriptors, in registration order. Used by
    // `URArm::create_model_registrations` so adding a new arm is a
    // single-site change here.
    static const std::vector<UrModelDescriptor>& all();
};

// Per-joint DH parameters for a 6-DOF UR arm, as reported by the controller
// via `urcl::primary_interface::KinematicsInfo`. Each array holds one entry
// per joint; units match what urcl reports: meters for `a` and `d`, radians
// for `alpha` and `theta`. The reported `theta` is a fixed per-joint
// calibration offset; it does not change as the joint moves.
struct DHParams {
    urcl::vector6d_t a;
    urcl::vector6d_t d;
    urcl::vector6d_t alpha;
    urcl::vector6d_t theta;
};

// Joint angular limits, in degrees, matching the `min`/`max` fields in
// shipped `kinematics/<model>.json` files.
struct JointLimits {
    double min_deg;
    double max_deg;
};

// A geometry expressed in its link's parent (joint) frame.
// `pose` translation in mm, orientation as an OV unit-vector with
// `theta` in degrees. `shape` is one of viam-cpp-sdk's shape primitives.
struct Geometry {
    viam::sdk::pose pose;
    std::variant<viam::sdk::sphere, viam::sdk::capsule> shape;
};

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

    const UrModelDescriptor& descriptor() const& {
        return *descriptor_;
    }

    // The kinematic representation of this arm model at joints with zero
    // rotation. Defined below; forward-declared here so `load_kinematics`
    // can return it by value.
    class Kinematics;

    // Parse a Viam-shipped SVA-form kinematics JSON describing this arm
    // and return a `Kinematics` anchored to it.
    Kinematics load_kinematics(const std::filesystem::path& sva_json_path) const;

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

// Mirror of an RDK SVA kinematics document, anchored to the `UrArmModel`
// it was parsed for. Constructed via `UrArmModel::load_kinematics(path)`.
class UrArmModel::Kinematics {
   public:
    UrArmModel model;
    std::array<JointLimits, 6> limits;
    std::array<Eigen::Matrix4d, 7> link_locals;
    std::array<std::optional<Geometry>, 7> geometries;
    std::array<std::string, 7> link_names;

    Kinematics apply_calibration(const DHParams& dh) const;

    // Serialize this kinematics to an RDK-compatible SVA kinematics JSON.
    std::string to_sva_json() const;

    // Cumulative world-frame pose of the i-th link's parent at joints with zero rotation.
    Eigen::Matrix4d parent_pose_at(std::size_t i) const;

   private:
    friend class UrArmModel;
    explicit Kinematics(UrArmModel m);
};
