#include "dh_kinematics.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>

#include <Eigen/Geometry>

#include <json/json.h>

#include "kinematics_parser.hpp"

namespace {

constexpr double k_m_to_mm = 1000.0;

// Build a 4x4 homogeneous transform for a link's static pose:
//   Rz(theta) * T(a, 0, d) * Rx(alpha)
// All inputs in millimeters/radians. Output is the matrix that takes a
// child-frame point to its parent-frame coordinates.
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

struct PoseTQ {
    double tx, ty, tz;
    double qw, qx, qy, qz;
};

// Apply a 4x4 correction on the left of a translation+quaternion pose,
// returning the composed pose. Used to push a world-frame geometry into its
// chain-link-local frame for emission.
PoseTQ apply_correction(const Eigen::Matrix4d& correction, const PoseTQ& g) {
    const Eigen::Quaterniond q_g(g.qw, g.qx, g.qy, g.qz);
    Eigen::Matrix4d G = Eigen::Matrix4d::Identity();
    G.block<3, 3>(0, 0) = q_g.toRotationMatrix();
    G(0, 3) = g.tx;
    G(1, 3) = g.ty;
    G(2, 3) = g.tz;

    const Eigen::Matrix4d Result = correction * G;
    PoseTQ out;
    out.tx = Result(0, 3);
    out.ty = Result(1, 3);
    out.tz = Result(2, 3);
    Eigen::Quaterniond q_out(Result.block<3, 3>(0, 0));
    q_out.normalize();
    out.qw = q_out.w();
    out.qx = q_out.x();
    out.qy = q_out.y();
    out.qz = q_out.z();
    return out;
}

Json::Value capsule_to_json(const CapsuleGeometry& c) {
    Json::Value g(Json::objectValue);
    g["type"] = "capsule";
    g["r"] = c.radius_mm;
    g["l"] = c.length_mm;

    Json::Value translation(Json::objectValue);
    translation["x"] = c.tx_mm;
    translation["y"] = c.ty_mm;
    translation["z"] = c.tz_mm;
    g["translation"] = translation;

    Json::Value orient(Json::objectValue);
    orient["type"] = "quaternion";
    Json::Value qval(Json::objectValue);
    qval["W"] = c.qw;
    qval["X"] = c.qx;
    qval["Y"] = c.qy;
    qval["Z"] = c.qz;
    orient["value"] = qval;
    g["orientation"] = orient;

    return g;
}

Json::Value sphere_to_json(const SphereGeometry& s) {
    Json::Value g(Json::objectValue);
    g["type"] = "sphere";
    g["r"] = s.radius_mm;

    Json::Value translation(Json::objectValue);
    translation["x"] = s.tx_mm;
    translation["y"] = s.ty_mm;
    translation["z"] = s.tz_mm;
    g["translation"] = translation;

    return g;
}

Json::Value geometry_to_json(const Geometry& geom) {
    return std::visit(
        [](const auto& g) -> Json::Value {
            using T = std::decay_t<decltype(g)>;
            if constexpr (std::is_same_v<T, CapsuleGeometry>) {
                return capsule_to_json(g);
            } else {
                return sphere_to_json(g);
            }
        },
        geom);
}

// Returns a copy of `geom` with its translation+orientation transformed by
// `correction`.
Geometry apply_correction_to_geometry(const Geometry& geom, const Eigen::Matrix4d& correction) {
    return std::visit(
        [&correction](const auto& g) -> Geometry {
            using T = std::decay_t<decltype(g)>;
            PoseTQ in;
            in.tx = g.tx_mm;
            in.ty = g.ty_mm;
            in.tz = g.tz_mm;
            if constexpr (std::is_same_v<T, CapsuleGeometry>) {
                in.qw = g.qw;
                in.qx = g.qx;
                in.qy = g.qy;
                in.qz = g.qz;
                const PoseTQ out = apply_correction(correction, in);
                CapsuleGeometry r = g;
                r.tx_mm = out.tx;
                r.ty_mm = out.ty;
                r.tz_mm = out.tz;
                r.qw = out.qw;
                r.qx = out.qx;
                r.qy = out.qy;
                r.qz = out.qz;
                return r;
            } else {
                // Spheres carry no orientation; correction reduces to its
                // translation component on the input point.
                in.qw = 1.0;
                in.qx = 0.0;
                in.qy = 0.0;
                in.qz = 0.0;
                const PoseTQ out = apply_correction(correction, in);
                SphereGeometry r = g;
                r.tx_mm = out.tx;
                r.ty_mm = out.ty;
                r.tz_mm = out.tz;
                return r;
            }
        },
        geom);
}

}  // namespace

std::string build_dh_kinematics_json(const std::string& model_name, const DHParams& dh, const ModelTables& tbl) {
    Json::Value root(Json::objectValue);
    root["name"] = model_name;
    root["kinematic_param_type"] = "SVA";

    // Emitted link names match the chain in `kinematics/<model>.json` so the
    // meshes returned by `URArm::get_3d_models` attach to the right frames.
    // The first DH segment is split across two emitted links: a static
    // `base_link` (parent=world, pure z-elevation by d_0) carries the base
    // mesh, and the DH-child link (parent=<model>_q_0) carries the post-joint
    // Rz(theta_0)*Rx(alpha_0) rotation, a_0 offset, and any shoulder
    // geometry. Putting Rx(alpha_0) on base_link would tilt the shoulder-pan
    // axis off world z.

    // Cumulative pose of the parent joint frame above each chain link at zero
    // joints. W_cal_links[0] is base_link's transform (T(0,0,d_0_cal)). For
    // i>=1 it is the running calibrated product, which equals base_link.T *
    // (the first DH link's full L_cal[0]) -- the d_0 split between base_link
    // and the shoulder link cancels in the product.
    std::array<Eigen::Matrix4d, 6> W_cal_links;
    {
        Eigen::Matrix4d W_cal = Eigen::Matrix4d::Identity();
        W_cal(2, 3) = dh.d[0] * k_m_to_mm;
        for (std::size_t i = 0; i < 6; ++i) {
            W_cal_links[i] = W_cal;
            // The shoulder-side link drops the (0,0,d_0) translation that
            // base_link absorbed; its accumulator contribution therefore uses
            // d=0. Later DH segments contribute their full L_cal[i].
            const double d_cal_chain = (i == 0) ? 0.0 : (dh.d[i] * k_m_to_mm);
            W_cal = W_cal * dh_link_pose_matrix(dh.a[i] * k_m_to_mm, d_cal_chain, dh.alpha[i], dh.theta[i]);
        }
    }

    Json::Value joints(Json::arrayValue);
    Json::Value links(Json::arrayValue);

    // base_link: static z-elevation above world, holds the base mesh.
    {
        Json::Value base_link(Json::objectValue);
        base_link["id"] = tbl.link_names[0];
        base_link["parent"] = "world";

        Json::Value translation(Json::objectValue);
        translation["x"] = 0.0;
        translation["y"] = 0.0;
        translation["z"] = dh.d[0] * k_m_to_mm;
        base_link["translation"] = translation;

        Json::Value orient(Json::objectValue);
        orient["type"] = "quaternion";
        Json::Value qval(Json::objectValue);
        qval["W"] = 1.0;
        qval["X"] = 0.0;
        qval["Y"] = 0.0;
        qval["Z"] = 0.0;
        orient["value"] = qval;
        base_link["orientation"] = orient;

        if (tbl.geometries[0].has_value()) {
            // base_link's frame is the world frame (parent=world per SVA
            // convention), and the parsed geometry is already in world frame,
            // so no correction is needed.
            base_link["geometry"] = geometry_to_json(*tbl.geometries[0]);
        } else {
            // Models with no usable base geometry (ur3e ships a box, which
            // the parser intentionally drops) still need a frame present in
            // RDK's frame system so any base mesh has somewhere to attach.
            Json::Value geom(Json::objectValue);
            geom["type"] = "sphere";
            geom["r"] = 1.0;
            base_link["geometry"] = geom;
        }

        links.append(base_link);
    }

    for (std::size_t i = 0; i < 6; ++i) {
        const std::string joint_id = model_name + "_q_" + std::to_string(i);
        const std::string& link_id = tbl.link_names[i + 1];
        const std::string& joint_parent = tbl.link_names[i];

        Json::Value joint(Json::objectValue);
        joint["id"] = joint_id;
        joint["type"] = "revolute";
        joint["parent"] = joint_parent;
        Json::Value axis(Json::objectValue);
        axis["x"] = 0.0;
        axis["y"] = 0.0;
        axis["z"] = 1.0;
        joint["axis"] = axis;
        joint["min"] = tbl.limits[i].min_deg;
        joint["max"] = tbl.limits[i].max_deg;
        joints.append(joint);

        // Static link pose = Rz(theta_i) * T(a_i, 0, d_i) * Rx(alpha_i).
        // Composed:
        //   translation = Rz(theta) * (a, 0, d) = (a*cos(theta), a*sin(theta), d)
        //   rotation    = Rz(theta) * Rx(alpha)
        // i=0 drops the d_0 z-translation; base_link absorbed it.
        const double theta_i = dh.theta[i];
        const double alpha_i = dh.alpha[i];
        const double a_mm = dh.a[i] * k_m_to_mm;
        const double d_mm = (i == 0) ? 0.0 : (dh.d[i] * k_m_to_mm);

        const double ct = std::cos(theta_i);
        const double st = std::sin(theta_i);
        Json::Value translation(Json::objectValue);
        translation["x"] = a_mm * ct;
        translation["y"] = a_mm * st;
        translation["z"] = d_mm;

        // Quaternion for Rz(theta) * Rx(alpha):
        //   qZ = (cos(theta/2), 0, 0, sin(theta/2))
        //   qX = (cos(alpha/2), sin(alpha/2), 0, 0)
        //   qZ * qX = (cz*cx, cz*sx, sz*sx, sz*cx)
        const double cz = std::cos(theta_i / 2.0);
        const double sz = std::sin(theta_i / 2.0);
        const double cx = std::cos(alpha_i / 2.0);
        const double sx = std::sin(alpha_i / 2.0);
        Json::Value orient(Json::objectValue);
        orient["type"] = "quaternion";
        Json::Value qval(Json::objectValue);
        qval["W"] = cz * cx;
        qval["X"] = cz * sx;
        qval["Y"] = sz * sx;
        qval["Z"] = sz * cx;
        orient["value"] = qval;

        Json::Value link(Json::objectValue);
        link["id"] = link_id;
        link["parent"] = joint_id;
        link["translation"] = translation;
        link["orientation"] = orient;
        if (tbl.geometries[i + 1].has_value()) {
            // tbl.geometries[i+1] is in world frame at zero joints; pulling
            // it back through inv(W_cal_i) yields the chain-link-local pose
            // that RDK will re-compose with the calibrated chain at runtime.
            const Eigen::Matrix4d correction = W_cal_links[i].inverse();
            const Geometry corrected = apply_correction_to_geometry(*tbl.geometries[i + 1], correction);
            link["geometry"] = geometry_to_json(corrected);
        }
        links.append(link);
    }

    root["joints"] = joints;
    root["links"] = links;

    Json::StreamWriterBuilder writer;
    writer["indentation"] = "    ";
    return Json::writeString(writer, root);
}
