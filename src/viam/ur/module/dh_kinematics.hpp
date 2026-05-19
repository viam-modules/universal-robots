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
// shipped `kinematics/<model>.json` (see `parse_kinematics`).
//
// The returned JSON uses SVA form ("kinematic_param_type": "SVA") with
// explicit `joints[]` and `links[]` arrays. The chain layout matches what the
// shipped kinematics JSON describes: a static `base_link` parented to `world`
// elevates the chain by `dh.d[0]`, then six DH joints alternate with six
// named links (`shoulder_link`, `upper_arm_link`, ..., `wrist_3_link` or
// `ee_link`, depending on the model).
//
// Geometries in `tbl` are stored in world frame at zero joints; the per-link
// correction `inv(W_cal_i)` is applied here so the runtime composition with
// the calibrated chain reproduces those world poses.
//
// SVA is used (rather than DH) so the calibrated theta offset can be baked
// into the static link as a fixed Rz rotation -- DHParamConfig has no theta
// field, and standard 4-parameter DH cannot represent UR's parallel-axis
// calibration without resorting to numerically degenerate huge `d` values.
std::string build_dh_kinematics_json(const std::string& model_name, const DHParams& dh, const ModelTables& tbl);
