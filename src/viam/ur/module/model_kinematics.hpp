#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>

#include <Eigen/Geometry>

#include <ur_client_library/types.h>

#include <viam/sdk/common/pose.hpp>
#include <viam/sdk/spatialmath/geometry.hpp>

#include "ur_models.hpp"

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

// Mirror of an RDK SVA kinematics document.
struct ModelKinematics {
    UrArmModel model;
    std::array<JointLimits, 6> limits;
    std::array<Eigen::Matrix4d, 7> link_locals;
    std::array<std::optional<Geometry>, 7> geometries;
    std::array<std::string, 7> link_names;

    explicit ModelKinematics(UrArmModel m);

    // Parse a Viam-shipped SVA-form kinematics JSON describing a UR arm at
    // joints with zero rotation.
    static ModelKinematics from_sva_json(const std::filesystem::path& sva_json_path, UrArmModel arm_model);

    ModelKinematics apply_calibrated_dh(const DHParams& dh) const;

    // Serialize this table to an RDK-compatible SVA kinematics JSON.
    std::string to_sva_json() const;

    // Cumulative world-frame pose of the i-th link's parent at joints with zero rotation.
    Eigen::Matrix4d parent_pose_at(std::size_t i) const;
};
