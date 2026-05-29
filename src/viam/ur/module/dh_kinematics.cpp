#include "dh_kinematics.hpp"

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>

#include "kinematics_parser.hpp"

namespace {

constexpr double k_m_to_mm = 1000.0;

// Build a 4x4 homogeneous transform for a link's static pose:
//   Rz(theta) * T(a, 0, d) * Rx(alpha)
// All inputs in millimeters/radians. Output is the matrix that takes a child-frame
// point to its parent-frame coordinates.
Eigen::Matrix4d dh_link_pose_matrix(double a_mm, double d_mm, double alpha_rad, double theta_rad) {
    const double ca = std::cos(alpha_rad);
    const double sa = std::sin(alpha_rad);
    const double ct = std::cos(theta_rad);
    const double st = std::sin(theta_rad);

    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    M(0, 0) = ct;
    M(0, 1) = -st * ca;
    M(0, 2) = st * sa;
    M(1, 0) = st;
    M(1, 1) = ct * ca;
    M(1, 2) = -ct * sa;
    M(2, 0) = 0.0;
    M(2, 1) = sa;
    M(2, 2) = ca;
    M(0, 3) = a_mm * ct;
    M(1, 3) = a_mm * st;
    M(2, 3) = d_mm;
    return M;
}

Eigen::Matrix4d dh_local_for_slot(std::size_t i, const DHParams& dh) {
    if (i == 0) {
        return dh_link_pose_matrix(0.0, dh.d[0] * k_m_to_mm, 0.0, 0.0);
    }
    const std::size_t s = i - 1;
    const double d_mm = (i == 1) ? 0.0 : (dh.d[s] * k_m_to_mm);
    return dh_link_pose_matrix(dh.a[s] * k_m_to_mm, d_mm, dh.alpha[s], dh.theta[s]);
}

}  // namespace

ModelTable ModelTable::with_calibrated_dh(const DHParams& dh) const {
    ModelTable out = *this;

    std::array<Eigen::Matrix4d, 7> new_link_locals;
    for (std::size_t i = 0; i < 7; ++i) {
        new_link_locals[i] = dh_local_for_slot(i, dh);
    }

    // For each present geometry, re-express its pose in the new emitted
    // parent frame: G_new = inv(W_new[i]) * W_old[i] * G_old. Walk both
    // cumulative chains in lockstep so the correction at slot i uses the
    // parent poses, i.e. the product of link_locals 0..i-1) on both sides.
    Eigen::Matrix4d cum_old = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d cum_new = Eigen::Matrix4d::Identity();
    for (std::size_t i = 0; i < 7; ++i) {
        if (out.geometries[i].has_value()) {
            const Eigen::Matrix4d correction = cum_new.inverse() * cum_old;
            out.geometries[i] = Geometry{apply_correction_to_pose(out.geometries[i]->pose, correction), out.geometries[i]->shape};
        }
        cum_old = cum_old * link_locals[i];
        cum_new = cum_new * new_link_locals[i];
    }

    out.link_locals = new_link_locals;
    return out;
}
