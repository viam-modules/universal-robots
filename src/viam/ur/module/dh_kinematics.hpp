#pragma once

#include <string>

#include <ur_client_library/types.h>

#include "kinematics_parser.hpp"

// Per-joint DH parameters for a 6-DOF UR arm, as reported by the controller via
// `urcl::primary_interface::KinematicsInfo`. Each array holds one entry per joint;
// units match what urcl reports: meters for `a` and `d`, radians for `alpha` and
// `theta`. The reported `theta` is a fixed per-joint calibration offset; it does
// not change as the joint moves.
struct DHParams {
    urcl::vector6d_t a;
    urcl::vector6d_t d;
    urcl::vector6d_t alpha;
    urcl::vector6d_t theta;
};

// Builds an RDK-compatible kinematics JSON document for the given UR model
// from calibrated DH parameters plus a per-model table parsed from the
// shipped `kinematics/<model>.json`.
std::string build_dh_kinematics_json(const std::string& model_name, const DHParams& dh, const ModelTables& tbl);
