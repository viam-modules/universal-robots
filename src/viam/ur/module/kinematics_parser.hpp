#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>

#include <Eigen/Geometry>

#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/spatialmath/geometry.hpp>

#include "ur_models.hpp"

struct DHParams;

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

// Mirror of an RDK SVA kinematics document.
struct ModelTable {
    UrArmModel model;
    std::array<JointLimits, 6> limits;
    std::array<Eigen::Matrix4d, 7> link_locals;
    std::array<std::optional<Geometry>, 7> geometries;
    std::array<std::string, 7> link_names;

    explicit ModelTable(UrArmModel m);

    ModelTable with_calibrated_dh(const DHParams& dh) const;
};

// Cumulative world-frame pose of the i-th link's parent at joints with zero rotation.
Eigen::Matrix4d parent_pose_at(const ModelTable& tbl, std::size_t i);

// Apply a 4x4 correction matrix to a viam-cpp-sdk pose, returning a new
// pose. Internally round-trips through a quaternion (via rust-utils) to
// compose with the matrix.
viam::sdk::pose apply_correction_to_pose(const viam::sdk::pose& p, const Eigen::Matrix4d& correction);

// Parse a Viam-shipped SVA-form kinematics JSON describing a UR arm at
// joints with zero rotation.
ModelTable parse_kinematics(const std::filesystem::path& sva_json_path, UrArmModel arm_model);

// Serialize a ModelTable to an RDK-compatible SVA kinematics JSON.
std::string to_sva_json(const ModelTable& tbl);
