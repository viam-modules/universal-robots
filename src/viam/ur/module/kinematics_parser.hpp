#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>

#include <Eigen/Geometry>

#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/spatialmath/geometry.hpp>

// Joint angular limits, in degrees, matching the `min`/`max` fields in
// shipped `kinematics/<model>.json` files.
struct JointLimits {
    double min_deg;
    double max_deg;
};

// A geometry stored in world frame at zero joints. `pose` is a viam-cpp-sdk
// pose (translation in mm, orientation as an OV unit-vector with `theta` in
// degrees). `shape` is one of viam-cpp-sdk's shape primitives (`radius` in
// mm; capsule additionally carries `length` in mm).
struct Geometry {
    viam::sdk::pose pose;
    std::variant<viam::sdk::sphere, viam::sdk::capsule> shape;
};

// Apply a 4x4 correction matrix to a viam-cpp-sdk pose, returning a new
// pose. Internally round-trips through a quaternion (via rust-utils) to
// compose with the matrix.
viam::sdk::pose apply_correction_to_pose(const viam::sdk::pose& p, const Eigen::Matrix4d& correction);

// Per-model data extracted from a shipped `kinematics/<model>.json` file:
// joint limits for the six DH joints, the world-frame pose at zero joints of
// each emitted link's geometry (capsule/sphere), and the seven emitted link
// names in chain order. Indices line up: `link_names[i]` and `geometries[i]`
// describe the same link, with index 0 being the static base link (chain
// root) and indices 1..6 being the children of DH joints 0..5.
struct ModelTables {
    std::array<JointLimits, 6> limits;
    std::array<std::optional<Geometry>, 7> geometries;
    std::array<std::string, 7> link_names;
};

// Parse a Viam-shipped SVA-form kinematics JSON describing a UR arm at zero
// joints, producing a per-model table whose geometry poses are pre-multiplied
// by the encoded chain so each geometry sits in world frame.
//
// Throws std::invalid_argument if the file cannot be opened, the JSON cannot
// be parsed, the document is not SVA-form, any of the six standard DH joints
// is missing, a DH joint has zero or more than one child link, a geometry
// definition is malformed, or the parent chain from any DH-child link cannot
// be resolved back to "world".
ModelTables parse_kinematics(const std::filesystem::path& sva_json_path);
