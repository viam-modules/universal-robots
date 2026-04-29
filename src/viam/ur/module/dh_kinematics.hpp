#pragma once

#include <array>
#include <string>

namespace viam_ur {

// DH parameters as returned by the UR controller's primary client.
// Units match what urcl reports: meters for a/d, radians for alpha and theta.
// `theta` is the per-joint calibration offset (NOT the joint variable).
struct DHParams6 {
    std::array<double, 6> a;
    std::array<double, 6> d;
    std::array<double, 6> alpha;
    std::array<double, 6> theta;
};

// Builds an RDK-compatible kinematics JSON document for the given UR model from
// calibrated DH parameters.
//
// The returned JSON uses SVA form ("kinematic_param_type": "SVA") with explicit
// `joints[]` and `links[]` arrays. Each segment is encoded as:
//   * a revolute joint around Z with the per-model joint limits, and
//   * a static link whose pose is `Rz(theta_i) * T(a_i, 0, d_i) * Rx(alpha_i)`.
//
// SVA is used (rather than DH) so the calibrated theta offset can be baked into
// the static link as a fixed Rz rotation -- DHParamConfig has no theta field,
// and standard 4-parameter DH cannot represent UR's parallel-axis calibration
// without resorting to numerically degenerate huge `d` values.
//
// Throws std::invalid_argument if model_name has no per-model table registered.
std::string build_dh_kinematics_json(const std::string& model_name, const DHParams6& dh);

}  // namespace viam_ur
