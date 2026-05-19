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

Json::Value geometry_to_json(const Geometry& geom) {
    Json::Value json(Json::objectValue);

    Json::Value translation(Json::objectValue);
    translation["x"] = geom.pose.coordinates.x;
    translation["y"] = geom.pose.coordinates.y;
    translation["z"] = geom.pose.coordinates.z;
    json["translation"] = translation;

    std::visit(
        [&](const auto& shape) {
            using S = std::decay_t<decltype(shape)>;
            json["r"] = shape.radius;
            if constexpr (std::is_same_v<S, viam::sdk::capsule>) {
                json["type"] = "capsule";
                json["l"] = shape.length;
                Json::Value orient(Json::objectValue);
                orient["type"] = "ov_degrees";
                Json::Value val(Json::objectValue);
                val["x"] = geom.pose.orientation.o_x;
                val["y"] = geom.pose.orientation.o_y;
                val["z"] = geom.pose.orientation.o_z;
                val["th"] = geom.pose.theta;
                orient["value"] = val;
                json["orientation"] = orient;
            } else {
                // Spheres carry no orientation; emit dimensions only.
                json["type"] = "sphere";
            }
        },
        geom.shape);

    return json;
}

// Returns a copy of `geom` with its pose transformed by `correction`. Shape
// dimensions pass through unchanged.
Geometry apply_correction_to_geometry(const Geometry& geom, const Eigen::Matrix4d& correction) {
    return Geometry{apply_correction_to_pose(geom.pose, correction), geom.shape};
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
    // mesh, and the next link (parent=<model>_q_0) carries the post-joint
    // Rz(theta_0)*Rx(alpha_0) rotation, a_0 offset, and any shoulder
    // geometry. Putting Rx(alpha_0) on base_link would tilt the shoulder-pan
    // axis off world z.

    // Static transform contributed by emitted link i, in DH form:
    //   i = 0 (base_link):   T(0, 0, d_0) -- pure z-elevation, identity rotation.
    //   i = 1 (shoulder):    Rz(theta_0) * T(a_0, 0, 0) * Rx(alpha_0)
    //                        -- d_0 was hoisted onto base_link, so this drops it.
    //   i = 2..6:            Rz(theta_{i-1}) * T(a_{i-1}, 0, d_{i-1}) * Rx(alpha_{i-1}).
    const auto link_local_pose = [&](std::size_t i) -> Eigen::Matrix4d {
        if (i == 0) {
            return dh_link_pose_matrix(0.0, dh.d[0] * k_m_to_mm, 0.0, 0.0);
        }
        const std::size_t s = i - 1;
        const double d_mm = (i == 1) ? 0.0 : (dh.d[s] * k_m_to_mm);
        return dh_link_pose_matrix(dh.a[s] * k_m_to_mm, d_mm, dh.alpha[s], dh.theta[s]);
    };

    // Cumulative pose of each emitted link's parent frame at zero joints.
    // Index 0 is the world frame (base_link's parent); for i >= 1 it is the
    // running calibrated product, which equals L_cal[0] * ... * L_cal[i-2]
    // (the d_0 split between base_link and shoulder_link cancels in the
    // product).
    std::array<Eigen::Matrix4d, 7> parent_pose;
    parent_pose[0] = Eigen::Matrix4d::Identity();
    for (std::size_t i = 1; i < 7; ++i) {
        parent_pose[i] = parent_pose[i - 1] * link_local_pose(i - 1);
    }

    Json::Value joints(Json::arrayValue);
    Json::Value links(Json::arrayValue);

    for (std::size_t i = 0; i < 7; ++i) {
        // Joint above this link: base_link has none (parent=world); links
        // 1..6 are children of ur*_q_{i-1}.
        if (i > 0) {
            Json::Value joint(Json::objectValue);
            joint["id"] = model_name + "_q_" + std::to_string(i - 1);
            joint["type"] = "revolute";
            joint["parent"] = tbl.link_names[i - 1];
            Json::Value axis(Json::objectValue);
            axis["x"] = 0.0;
            axis["y"] = 0.0;
            axis["z"] = 1.0;
            joint["axis"] = axis;
            joint["min"] = tbl.limits[i - 1].min_deg;
            joint["max"] = tbl.limits[i - 1].max_deg;
            joints.append(joint);
        }

        const Eigen::Matrix4d local_pose = link_local_pose(i);
        const Eigen::Vector3d t = local_pose.block<3, 1>(0, 3);
        const Eigen::Quaterniond q(local_pose.block<3, 3>(0, 0));

        Json::Value translation(Json::objectValue);
        translation["x"] = t.x();
        translation["y"] = t.y();
        translation["z"] = t.z();

        Json::Value orient(Json::objectValue);
        orient["type"] = "quaternion";
        Json::Value qval(Json::objectValue);
        qval["W"] = q.w();
        qval["X"] = q.x();
        qval["Y"] = q.y();
        qval["Z"] = q.z();
        orient["value"] = qval;

        Json::Value link(Json::objectValue);
        link["id"] = tbl.link_names[i];
        link["parent"] = (i == 0) ? std::string{"world"} : (model_name + "_q_" + std::to_string(i - 1));
        link["translation"] = translation;
        link["orientation"] = orient;
        if (tbl.geometries[i].has_value()) {
            // tbl.geometries[i] is in world frame at zero joints; pulling it
            // back through inv(parent_pose[i]) yields the chain-link-local
            // pose that RDK will re-compose with the calibrated chain at
            // runtime. For i=0 the correction is identity (parent is world).
            const Eigen::Matrix4d correction = parent_pose[i].inverse();
            const Geometry corrected = apply_correction_to_geometry(*tbl.geometries[i], correction);
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
