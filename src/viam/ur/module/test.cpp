#define BOOST_TEST_MODULE test module test_ur5e

#include "ur_arm.hpp"
#include "utils.hpp"

#include <boost/numeric/conversion/cast.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <boost/test/included/unit_test.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Dense>

#include <json/json.h>

#include <third_party/trajectories/Path.h>
#include <third_party/trajectories/Trajectory.h>

#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>

namespace {

Eigen::VectorXd makeVector(std::vector<double> data) {
    return Eigen::Map<Eigen::VectorXd>(data.data(), boost::numeric_cast<Eigen::Index>(data.size()));
}

}  // namespace

BOOST_AUTO_TEST_SUITE(test_1)

BOOST_AUTO_TEST_CASE(test_sampling_func) {
    // test a random set of samples
    {
        std::vector<trajectory_sample_point> test_samples = {};
        const double test_duration_sec = 5;
        const double test_freq_hz = 5;
        // we are not testing the behavior of the trajectory generation, so we create a simple lambda function.
        // this allows us to get a good idea of what sampling_func is doing
        sampling_func(test_samples, test_duration_sec, test_freq_hz, [](const double t, const double step) {
            return trajectory_sample_point{{t, 0, 0, 0, 0, 0}, {t * step, 0, 0, 0, 0, 0}, boost::numeric_cast<float>(step)};
        });
        BOOST_CHECK_EQUAL(test_samples.size(), static_cast<std::size_t>(std::ceil(test_duration_sec * test_freq_hz)));
    }
    // check for durations smaller than the sampling frequency
    {
        std::vector<trajectory_sample_point> test_samples = {};
        const double test_duration_sec = 1;
        const double test_freq_hz = 0.5;  // 1 sample every 2 seconds

        sampling_func(test_samples, test_duration_sec, test_freq_hz, [](const double t, const double step) {
            return trajectory_sample_point{{t, 0, 0, 0, 0, 0}, {t * step, 0, 0, 0, 0, 0}, boost::numeric_cast<float>(step)};
        });
        BOOST_CHECK_EQUAL(test_samples.size(), 1);
        BOOST_CHECK_EQUAL(test_samples[0].p[0], test_duration_sec);
        BOOST_CHECK_EQUAL(test_samples[0].v[0], test_duration_sec * test_duration_sec);
        BOOST_CHECK_EQUAL(test_samples[0].timestep, boost::numeric_cast<float>(test_duration_sec));
    }
    // test invalid input
    {
        std::vector<trajectory_sample_point> test_samples = {};
        const double test_duration_sec = 0;
        const double test_freq_hz = 0.5;  // 1 sample every 2 seconds

        BOOST_CHECK_THROW(sampling_func(test_samples,
                                        test_duration_sec,
                                        test_freq_hz,
                                        [](const double t, const double step) {
                                            BOOST_FAIL("we should never reach this");
                                            return trajectory_sample_point{
                                                {t, 0, 0, 0, 0, 0}, {t * step, 0, 0, 0, 0, 0}, boost::numeric_cast<float>(step)};
                                        }),
                          std::invalid_argument);
    }
}

BOOST_AUTO_TEST_CASE(test_write_waypoints_to_csv) {
    const std::list<Eigen::VectorXd> waypoints = {
        makeVector({10.1, 0, 0, 0, 0, 0}),
        makeVector({20.2, 0, 0, 0, 0, 0}),
        makeVector({30.3, 0, 0, 0, 0, 0}),
        makeVector({40.4, 0, 0, 0, 0, 0}),
        makeVector({50.5, 0, 0, 0, 0, 0}),
        makeVector({60.6, 0, 0, 0, 0, 0}),
        makeVector({70.7, 0, 0, 0, 0, 0}),
        makeVector({80.8, 0, 0, 0, 0, 0}),
    };

    const auto* const expected =
        "10.1,0,0,0,0,0\n"
        "20.2,0,0,0,0,0\n"
        "30.3,0,0,0,0,0\n"
        "40.4,0,0,0,0,0\n"
        "50.5,0,0,0,0,0\n"
        "60.6,0,0,0,0,0\n"
        "70.7,0,0,0,0,0\n"
        "80.8,0,0,0,0,0\n";

    write_waypoints_to_csv("./write_waypoints_to_csv_test.csv", waypoints);
    const std::ifstream csv("./write_waypoints_to_csv_test.csv");
    std::stringstream buf;
    buf << csv.rdbuf();
    BOOST_CHECK_EQUAL(std::remove("./write_waypoints_to_csv_test.csv"), 0);
    BOOST_CHECK_EQUAL(buf.str(), expected);
}

BOOST_AUTO_TEST_CASE(test_write_trajectory_to_file) {
    const std::vector<trajectory_sample_point> samples = {{{1.1, 2, 3, 4, 5, 6}, {1.2, 2, 3, 4, 5, 6}, 1.2F},
                                                          {{3.1, 2, 3, 4, 5, 6}, {4.2, 2, 3, 4, 5, 6}, 0.8F},
                                                          {{6.1, 2, 3, 4, 5, 6}, {7.1, 2, 3, 4, 5, 6}, 1},
                                                          {{9.1, 2, 3, 4, 5, 6}, {10.1, 2, 3, 4, 5, 6}, 1}};

    const auto* const expected =
        "t(s),j0,j1,j2,j3,j4,j5,v0,v1,v2,v3,v4,v5\n"
        "1.2,1.1,2,3,4,5,6,1.2,2,3,4,5,6\n"
        "2,3.1,2,3,4,5,6,4.2,2,3,4,5,6\n"
        "3,6.1,2,3,4,5,6,7.1,2,3,4,5,6\n"
        "4,9.1,2,3,4,5,6,10.1,2,3,4,5,6\n";

    write_trajectory_to_file("./write_trajectory_to_file_test.csv", samples);
    const std::ifstream csv("./write_trajectory_to_file_test.csv");
    std::stringstream buf;
    buf << csv.rdbuf();
    BOOST_CHECK_EQUAL(std::remove("./write_trajectory_to_file_test.csv"), 0);
    BOOST_CHECK_EQUAL(buf.str(), expected);
}

using namespace std::chrono_literals;

BOOST_AUTO_TEST_CASE(test_waypoints_filename) {
    const std::string k_path = "/home/user";
    const std::string k_resource_name = "test_arm";
    const auto timestamp = unix_time_iso8601();
    const auto path = k_path + "/" + timestamp + "_" + k_resource_name + "_waypoints.csv";

    auto x = waypoints_filename(k_path, k_resource_name, timestamp);
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_CASE(test_trajectory_filename) {
    const std::string k_path = "/home/user";
    const std::string k_resource_name = "test_arm";
    const auto timestamp = unix_time_iso8601();
    const auto path = k_path + "/" + timestamp + "_" + k_resource_name + "_trajectory.csv";

    auto x = trajectory_filename(k_path, k_resource_name, timestamp);
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_CASE(test_arm_joint_positions_filename) {
    const std::string k_path = "/home/user";
    const std::string k_resource_name = "test_arm";
    const auto timestamp = unix_time_iso8601();
    const auto path = k_path + "/" + timestamp + "_" + k_resource_name + "_arm_joint_positions.csv";

    auto x = arm_joint_positions_filename(k_path, k_resource_name, timestamp);
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(tcp_force_transformation_tests)

// Test rotation_vector_to_matrix function with various edge cases and standard rotations

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_identity) {
    // Test: Zero rotation vector should produce identity matrix
    // Why: This is the fundamental edge case - when there's no rotation,
    // the transformation should be identity (no change to coordinates)
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    BOOST_CHECK(rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_zero_rotation) {
    // Test: Zero rotation with non-zero position should still produce identity matrix
    // Why: The position components of the TCP pose shouldn't affect rotation calculation
    const vector6d_t tcp_pose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    BOOST_CHECK(rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_90_degrees_x) {
    // Test: 90-degree rotation around X-axis
    // Why: Standard orthogonal rotation - Y becomes Z, Z becomes -Y
    // This verifies correct implementation of Rodrigues' rotation formula
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, std::numbers::pi / 2.0, 0.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    Eigen::Matrix3d expected;
    expected << 1, 0, 0, 0, 0, -1, 0, 1, 0;

    BOOST_CHECK(rotation_matrix.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_90_degrees_y) {
    // Test: 90-degree rotation around Y-axis
    // Why: Standard orthogonal rotation - X becomes -Z, Z becomes X
    // Verifies correct axis ordering in rotation matrix construction
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, std::numbers::pi / 2.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    Eigen::Matrix3d expected;
    expected << 0, 0, 1, 0, 1, 0, -1, 0, 0;

    BOOST_CHECK(rotation_matrix.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_90_degrees_z) {
    // Test: 90-degree rotation around Z-axis
    // Why: Standard orthogonal rotation - X becomes Y, Y becomes -X
    // Most common rotation in robotics applications
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, std::numbers::pi / 2.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    Eigen::Matrix3d expected;
    expected << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    BOOST_CHECK(rotation_matrix.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_arbitrary_angle) {
    // Test: Arbitrary rotation around diagonal axis
    // Why: Tests general case with non-orthogonal axis and non-standard angle
    // Validates against Eigen's reference implementation
    const double angle = 0.5;
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, angle / std::sqrt(3), angle / std::sqrt(3), angle / std::sqrt(3)};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    const Eigen::Vector3d axis(1.0 / std::sqrt(3), 1.0 / std::sqrt(3), 1.0 / std::sqrt(3));
    const Eigen::Matrix3d expected = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

    BOOST_CHECK(rotation_matrix.isApprox(expected, 1e-8));

    // Test: Verify rotation matrix properties (orthogonal with unit determinant)
    // Why: All valid rotation matrices must satisfy R * R^T = I and det(R) = 1
    BOOST_CHECK(std::abs(rotation_matrix.determinant() - 1.0) < 1e-8);
    BOOST_CHECK((rotation_matrix * rotation_matrix.transpose()).isApprox(Eigen::Matrix3d::Identity(), 1e-8));
}

// Test transform_vector function with various rotation scenarios

BOOST_AUTO_TEST_CASE(test_transform_vector_identity_rotation) {
    // Test: Identity transformation should leave vector unchanged
    // Why: Baseline test - when there's no rotation, the vector should remain identical
    const Eigen::Vector3d input_vector(1.0, 2.0, 3.0);
    const Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    const auto result = transform_vector(input_vector, rotation_matrix);

    BOOST_CHECK(result.isApprox(input_vector, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_transform_vector_90_degrees_x) {
    // Test: 90-degree rotation around X-axis applied to vector transformation
    // Why: Verifies that Y component becomes Z and Z becomes -Y (inverse transformation)
    // Uses transpose of rotation matrix for base-to-tool transformation
    const Eigen::Vector3d input_vector(1.0, 2.0, 3.0);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1, 0, 0, 0, 0, -1, 0, 1, 0;

    const auto result = transform_vector(input_vector, rotation_matrix);

    const Eigen::Vector3d expected(1.0, 3.0, -2.0);
    BOOST_CHECK(result.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_transform_vector_90_degrees_z) {
    // Test: 90-degree rotation around Z-axis applied to vector transformation
    // Why: Verifies that X component becomes Y and Y becomes -X (inverse transformation)
    // Critical for tool frame transformations in typical robot orientations
    const Eigen::Vector3d input_vector(1.0, 2.0, 3.0);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    const auto result = transform_vector(input_vector, rotation_matrix);

    const Eigen::Vector3d expected(2.0, -1.0, 3.0);
    BOOST_CHECK(result.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_transform_vector_arbitrary_rotation) {
    // Test: 45-degree rotation around Z-axis with unit X vector
    // Why: Tests trigonometric accuracy and validates the inverse transformation
    // Expected result should be at 45-degree angle in XY plane
    const Eigen::Vector3d input_vector(1.0, 0.0, 0.0);
    const Eigen::Vector3d axis(0.0, 0.0, 1.0);
    const double angle = std::numbers::pi / 4.0;
    const Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

    const auto result = transform_vector(input_vector, rotation_matrix);

    // transform_vector uses rotation_matrix.transpose() * input_vector
    // For inverse transformation, the expected result is transpose * input
    const Eigen::Vector3d expected = rotation_matrix.transpose() * input_vector;
    BOOST_CHECK(result.isApprox(expected, 1e-8));
}

// Test complete TCP force coordinate frame conversion

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_identity) {
    // Test: Zero rotation should leave forces unchanged
    // Why: Baseline test - when tool and base frames are aligned,
    // force/torque values should be identical in both frames
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 1.0, 1e-8);
    BOOST_CHECK_CLOSE(result[1], 2.0, 1e-8);
    BOOST_CHECK_CLOSE(result[2], 3.0, 1e-8);
    BOOST_CHECK_CLOSE(result[3], 4.0, 1e-8);
    BOOST_CHECK_CLOSE(result[4], 5.0, 1e-8);
    BOOST_CHECK_CLOSE(result[5], 6.0, 1e-8);
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_90_degrees_x) {
    // Test: 90-degree X rotation transforms Y→Z, Z→-Y for both force and torque
    // Why: Simulates tool rotated 90° around its X-axis (common in assembly tasks)
    // Force in base Y becomes force in tool Z, etc.
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, std::numbers::pi / 2.0, 0.0, 0.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 1.0, 1e-6);   // Fx unchanged
    BOOST_CHECK_CLOSE(result[1], 3.0, 1e-6);   // Fy = Fz_base
    BOOST_CHECK_CLOSE(result[2], -2.0, 1e-6);  // Fz = -Fy_base
    BOOST_CHECK_CLOSE(result[3], 4.0, 1e-6);   // TRx unchanged
    BOOST_CHECK_CLOSE(result[4], 6.0, 1e-6);   // TRy = TRz_base
    BOOST_CHECK_CLOSE(result[5], -5.0, 1e-6);  // TRz = -TRy_base
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_90_degrees_z) {
    // Test: 90-degree Z rotation transforms X→Y, Y→-X for both force and torque
    // Why: Most common robot tool rotation - tool frame Z-axis aligned with approach direction
    // Critical for correct force feedback in contact tasks
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, std::numbers::pi / 2.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 2.0, 1e-6);   // Fx = Fy_base
    BOOST_CHECK_CLOSE(result[1], -1.0, 1e-6);  // Fy = -Fx_base
    BOOST_CHECK_CLOSE(result[2], 3.0, 1e-6);   // Fz unchanged
    BOOST_CHECK_CLOSE(result[3], 5.0, 1e-6);   // TRx = TRy_base
    BOOST_CHECK_CLOSE(result[4], -4.0, 1e-6);  // TRy = -TRx_base
    BOOST_CHECK_CLOSE(result[5], 6.0, 1e-6);   // TRz unchanged
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_combined_rotation) {
    // Test: Combined X and Y axis rotations (complex 3D orientation)
    // Why: Tests composition of rotations and validates against reference calculation
    // Represents realistic tool orientations in 6-DOF manipulation tasks
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, std::numbers::pi / 4.0, std::numbers::pi / 4.0, 0.0};
    const vector6d_t tcp_force_base = {10.0, 0.0, 0.0, 0.0, 10.0, 0.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    // Verify against independent calculation using rotation matrix
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);
    const Eigen::Vector3d force_base(10.0, 0.0, 0.0);
    const Eigen::Vector3d torque_base(0.0, 10.0, 0.0);

    const Eigen::Vector3d expected_force = rotation_matrix.transpose() * force_base;
    const Eigen::Vector3d expected_torque = rotation_matrix.transpose() * torque_base;

    BOOST_CHECK_CLOSE(result[0], expected_force[0], 1e-6);
    BOOST_CHECK_CLOSE(result[1], expected_force[1], 1e-6);
    BOOST_CHECK_CLOSE(result[2], expected_force[2], 1e-6);
    BOOST_CHECK_CLOSE(result[3], expected_torque[0], 1e-6);
    BOOST_CHECK_CLOSE(result[4], expected_torque[1], 1e-6);
    BOOST_CHECK_CLOSE(result[5], expected_torque[2], 1e-6);
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_zero_force) {
    // Test: Zero forces with arbitrary rotation should remain zero
    // Why: Tests numerical stability - zero vector should transform to zero vector
    // Important for sensor noise filtering and null force detection
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0};
    const vector6d_t tcp_force_base = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[1], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[2], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[3], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[4], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[5], 0.0, 1e-8);
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_force_magnitude_preservation) {
    // Test: Rotation preserves vector magnitudes (orthogonal transformation property)
    // Why: Force and torque magnitudes must be conserved during coordinate transformation
    // Critical for maintaining physical meaning of sensor readings
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.5, 1.0, 1.5};
    const vector6d_t tcp_force_base = {3.0, 4.0, 5.0, 1.0, 2.0, 3.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    const Eigen::Vector3d force_base(3.0, 4.0, 5.0);
    const Eigen::Vector3d torque_base(1.0, 2.0, 3.0);
    const Eigen::Vector3d force_tool(result[0], result[1], result[2]);
    const Eigen::Vector3d torque_tool(result[3], result[4], result[5]);

    BOOST_CHECK_CLOSE(force_base.norm(), force_tool.norm(), 1e-6);
    BOOST_CHECK_CLOSE(torque_base.norm(), torque_tool.norm(), 1e-6);
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_small_rotation_angle) {
    // Test: Very small rotation angles (near numerical precision limits)
    // Why: Tests numerical stability for small-angle approximations
    // Important for high-precision applications and avoiding singularities
    const double small_angle = 1e-6;
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, small_angle, 0.0, 0.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    // Small rotations produce small but measurable changes due to the rotation calculation
    // Use looser tolerances to account for small-angle numerical effects
    BOOST_CHECK_CLOSE(result[0], 1.0, 1e-3);  // X component (unchanged for X rotation)
    BOOST_CHECK_CLOSE(result[1], 2.0, 1e-3);  // Y component (small change due to rotation)
    BOOST_CHECK_CLOSE(result[2], 3.0, 1e-3);  // Z component (small change due to rotation)
    BOOST_CHECK_CLOSE(result[3], 4.0, 1e-3);  // TRx component (unchanged for X rotation)
    BOOST_CHECK_CLOSE(result[4], 5.0, 1e-3);  // TRy component (small change due to rotation)
    BOOST_CHECK_CLOSE(result[5], 6.0, 1e-3);  // TRz component (small change due to rotation)
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_large_rotation_angle) {
    // Test: 180-degree rotation (maximum rotation angle)
    // Why: Tests behavior at rotation boundaries and sign changes
    // 180° rotation around X should flip Y and Z components
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, std::numbers::pi, 0.0, 0.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 1.0, 1e-6);   // X unchanged
    BOOST_CHECK_CLOSE(result[1], -2.0, 1e-6);  // Y flipped
    BOOST_CHECK_CLOSE(result[2], -3.0, 1e-6);  // Z flipped
    BOOST_CHECK_CLOSE(result[3], 4.0, 1e-6);   // TRx unchanged
    BOOST_CHECK_CLOSE(result[4], -5.0, 1e-6);  // TRy flipped
    BOOST_CHECK_CLOSE(result[5], -6.0, 1e-6);  // TRz flipped
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_multiple_axes) {
    // Test: Complex rotation involving all three axes simultaneously
    // Why: Tests real-world scenarios where tool has arbitrary 3D orientation
    // Validates composition of rotations for complex manipulator poses
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, std::numbers::pi / 6.0, std::numbers::pi / 4.0, std::numbers::pi / 3.0};
    const vector6d_t tcp_force_base = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    // Validate against independent calculation using rotation matrix
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);
    const Eigen::Vector3d force_base(1.0, 1.0, 1.0);
    const Eigen::Vector3d torque_base(1.0, 1.0, 1.0);

    const Eigen::Vector3d expected_force = rotation_matrix.transpose() * force_base;
    const Eigen::Vector3d expected_torque = rotation_matrix.transpose() * torque_base;

    BOOST_CHECK_CLOSE(result[0], expected_force[0], 1e-6);
    BOOST_CHECK_CLOSE(result[1], expected_force[1], 1e-6);
    BOOST_CHECK_CLOSE(result[2], expected_force[2], 1e-6);
    BOOST_CHECK_CLOSE(result[3], expected_torque[0], 1e-6);
    BOOST_CHECK_CLOSE(result[4], expected_torque[1], 1e-6);
    BOOST_CHECK_CLOSE(result[5], expected_torque[2], 1e-6);
}

// Additional edge case tests for comprehensive coverage

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_very_small_angle) {
    // Test: Extremely small rotation angle near floating-point precision limits
    // Why: Tests the epsilon threshold (1e-8) in the implementation
    // Verifies smooth transition between identity and computed rotation
    const double tiny_angle = 1e-9;
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, tiny_angle, 0.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    // Should return identity matrix for angles below threshold
    BOOST_CHECK(rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_threshold_boundary) {
    // Test: Rotation angle exactly at the epsilon threshold
    // Why: Tests boundary condition in the implementation (angle == 1e-8)
    const double threshold_angle = 1e-8;
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, threshold_angle, 0.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    // At threshold, should still return identity
    BOOST_CHECK(rotation_matrix.isApprox(Eigen::Matrix3d::Identity(), 1e-8));
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_negative_angles) {
    // Test: Negative rotation angles (opposite direction rotations)
    // Why: Tests sign handling in rotation vector calculations
    // Validates bidirectional rotation capability
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, -std::numbers::pi / 2.0, 0.0, 0.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    // -90° X rotation: Y→-Z, Z→Y
    BOOST_CHECK_CLOSE(result[0], 1.0, 1e-6);   // Fx unchanged
    BOOST_CHECK_CLOSE(result[1], -3.0, 1e-6);  // Fy = -Fz_base
    BOOST_CHECK_CLOSE(result[2], 2.0, 1e-6);   // Fz = Fy_base
    BOOST_CHECK_CLOSE(result[3], 4.0, 1e-6);   // TRx unchanged
    BOOST_CHECK_CLOSE(result[4], -6.0, 1e-6);  // TRy = -TRz_base
    BOOST_CHECK_CLOSE(result[5], 5.0, 1e-6);   // TRz = TRy_base
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_pure_torque) {
    // Test: Only torque components (zero forces)
    // Why: Tests decoupled force/torque transformation
    // Important for torque-only applications (pure rotation tasks)
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0};
    const vector6d_t tcp_force_base = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 0.0, 1e-8);  // Forces remain zero
    BOOST_CHECK_CLOSE(result[1], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[2], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[3], 2.0, 1e-6);   // TRx = TRy_base
    BOOST_CHECK_CLOSE(result[4], -1.0, 1e-6);  // TRy = -TRx_base
    BOOST_CHECK_CLOSE(result[5], 3.0, 1e-6);   // TRz unchanged
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_pure_force) {
    // Test: Only force components (zero torques)
    // Why: Tests decoupled force/torque transformation
    // Important for force-only applications (linear contact tasks)
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};

    const auto result = convert_tcp_force_to_tool_frame(tcp_pose, tcp_force_base);

    BOOST_CHECK_CLOSE(result[0], 2.0, 1e-6);   // Fx = Fy_base
    BOOST_CHECK_CLOSE(result[1], -1.0, 1e-6);  // Fy = -Fx_base
    BOOST_CHECK_CLOSE(result[2], 3.0, 1e-6);   // Fz unchanged
    BOOST_CHECK_CLOSE(result[3], 0.0, 1e-8);   // Torques remain zero
    BOOST_CHECK_CLOSE(result[4], 0.0, 1e-8);
    BOOST_CHECK_CLOSE(result[5], 0.0, 1e-8);
}

BOOST_AUTO_TEST_CASE(test_convert_tcp_force_to_tool_frame_position_independence) {
    // Test: TCP position should not affect force transformation (only rotation matters)
    // Why: Validates that only the rotation part of TCP pose is used
    // Position translation doesn't affect force/torque coordinate transformation
    const vector6d_t tcp_pose_at_origin = {0.0, 0.0, 0.0, M_PI / 4.0, 0.0, 0.0};
    const vector6d_t tcp_pose_displaced = {100.0, 200.0, 300.0, M_PI / 4.0, 0.0, 0.0};
    const vector6d_t tcp_force_base = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

    const auto result_origin = convert_tcp_force_to_tool_frame(tcp_pose_at_origin, tcp_force_base);
    const auto result_displaced = convert_tcp_force_to_tool_frame(tcp_pose_displaced, tcp_force_base);

    // Results should be identical regardless of TCP position
    for (std::size_t i = 0; i < 6; ++i) {
        BOOST_CHECK_CLOSE(result_origin[i], result_displaced[i], 1e-8);
    }
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(colinearization_tests)

// Test within_colinearization_tolerance function

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_on_segment) {
    // Point exactly on the line segment should be within tolerance
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;  // diameter (radius = 0.01)

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_within_tube) {
    // Point slightly off the line but within tolerance
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.005, 0.0, 0, 0, 0});  // 0.005 away
    const double tolerance = 0.02;                                         // diameter (radius = 0.01)

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_outside_tube) {
    // Point outside tolerance tube
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.02, 0.0, 0, 0, 0});  // 0.02 away, exceeds tolerance
    const double tolerance = 0.02;                                        // diameter (radius = 0.01)

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_before_segment) {
    // Point projects before segment start (monotonic advancement check)
    const Eigen::VectorXd line_start = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({0.5, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;  // diameter (radius = 0.01)

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_after_segment) {
    // Point projects after segment end (monotonic advancement check)
    const Eigen::VectorXd line_start = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({2.5, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;  // diameter (radius = 0.01)

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_degenerate_segment) {
    // Degenerate segment (start == end) should return false
    const Eigen::VectorXd line_start = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;  // diameter (radius = 0.01)

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_zero_tolerance) {
    // Zero tolerance - only exact points on line pass
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_on = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_off = makeVector({1.0, 0.0001, 0.0, 0, 0, 0});
    const double tolerance = 0.0;

    BOOST_CHECK(within_colinearization_tolerance(point_on, line_start, line_end, tolerance));
    BOOST_CHECK(!within_colinearization_tolerance(point_off, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_multi_dof) {
    // Test with multi-DOF configuration where deviation is in different joint
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({0.0, 0.0, 2.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({0.0, 0.005, 1.0, 0, 0, 0});  // Deviation in joint 1
    const double tolerance = 0.02;                                         // diameter (radius = 0.01)

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_at_boundary) {
    // Point exactly at tolerance boundary
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.01, 0.0, 0, 0, 0});  // Exactly at tolerance
    const double tolerance = 0.02;                                        // diameter (radius = 0.01)

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

// Test apply_colinearization function

BOOST_AUTO_TEST_CASE(test_apply_colinearization_empty_list) {
    // Empty list should remain unchanged
    std::list<Eigen::VectorXd> waypoints;
    apply_colinearization(waypoints, 0.01);
    BOOST_CHECK_EQUAL(waypoints.size(), 0);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_single_waypoint) {
    // Single waypoint should remain unchanged
    std::list<Eigen::VectorXd> waypoints = {makeVector({1.0, 0, 0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 1);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_two_waypoints) {
    // Two waypoints should remain unchanged (nothing to coalesce)
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0, 0, 0, 0, 0}), makeVector({1.0, 0, 0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_three_points_middle_coalesced) {
    // Three points where middle is within tolerance - should be coalesced
    // With diameter=0.02 (radius=0.01), point at 0.005 perpendicular distance is within
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),  // Slightly off line, within tolerance
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);  // diameter (radius = 0.01)
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
    BOOST_CHECK(waypoints.front().isApprox(makeVector({0.0, 0.0, 0.0, 0, 0, 0})));
    BOOST_CHECK(waypoints.back().isApprox(makeVector({2.0, 0.0, 0.0, 0, 0, 0})));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_three_points_middle_not_coalesced) {
    // Three points where middle is outside tolerance - should not be coalesced
    // With diameter=0.02 (radius=0.01), point at 0.02 perpendicular distance is outside
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.02, 0.0, 0, 0, 0}),  // Outside tolerance
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_multiple_points_all_coalesced) {
    // Multiple points all within tolerance - all middle points coalesced
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({3.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({4.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);  // diameter (radius = 0.01)
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
    BOOST_CHECK(waypoints.front().isApprox(makeVector({0.0, 0.0, 0.0, 0, 0, 0})));
    BOOST_CHECK(waypoints.back().isApprox(makeVector({4.0, 0.0, 0.0, 0, 0, 0})));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_mixed_coalescing) {
    // Some points coalesced, some not
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),  // Coalesced
                                            makeVector({2.0, 0.02, 0.0, 0, 0, 0}),   // Not coalesced (outside tolerance)
                                            makeVector({3.0, 0.005, 0.0, 0, 0, 0}),  // Coalesced
                                            makeVector({4.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);  // diameter (radius = 0.01)
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_degenerate_segment) {
    // Degenerate segment: returning to the same position creates anchor == next
    // This represents intentional "go there and come back" movement
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}),   // A
        makeVector({1.0, 0.0, 0.0, 0, 0, 0}),   // B
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}),   // C - back to A (degenerate: A→C has zero length conceptually)
        makeVector({2.0, 0.0, 0.0, 0, 0, 0})};  // D
    apply_colinearization(waypoints, 0.02);
    // B cannot be coalesced because it projects outside A→C segment (C goes backward)
    BOOST_CHECK_EQUAL(waypoints.size(), 4);  // All preserved due to degenerate segment
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_drift_prevented) {
    // Test that revalidation prevents drift
    // This is a regression test to ensure all skipped waypoints are revalidated
    // when extending the cylinder. The implementation should prevent drift.
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),    // A
                                            makeVector({1.0, 0.009, 0.0, 0, 0, 0}),  // B - within 0.01 of A→C
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0}),    // C
                                            makeVector({3.0, 0.0, 0.0, 0, 0, 0})};   // D
    apply_colinearization(waypoints, 0.02);                                          // diameter (radius = 0.01)
    // With the revalidation logic, B remains within tolerance even when extending to D
    BOOST_CHECK_EQUAL(waypoints.size(), 2);  // B and C coalesced
    BOOST_CHECK(waypoints.front().isApprox(makeVector({0.0, 0.0, 0.0, 0, 0, 0})));
    BOOST_CHECK(waypoints.back().isApprox(makeVector({3.0, 0.0, 0.0, 0, 0, 0})));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_zero_tolerance) {
    // Zero tolerance - no coalescing should occur
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({1.0, 0.0, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.0);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_negative_tolerance) {
    // Negative tolerance - no coalescing should occur
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({1.0, 0.0, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, -0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_diameter_interpretation) {
    // Verify diameter interpretation: tolerance=0.02 means radius=0.01
    // Point at perpendicular distance 0.01 should be exactly on boundary (within tolerance)
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_inside = makeVector({1.0, 0.009, 0.0, 0, 0, 0});   // < radius
    const Eigen::VectorXd point_outside = makeVector({1.0, 0.011, 0.0, 0, 0, 0});  // > radius

    BOOST_CHECK(within_colinearization_tolerance(point_inside, line_start, line_end, 0.02));
    BOOST_CHECK(!within_colinearization_tolerance(point_outside, line_start, line_end, 0.02));
}

BOOST_AUTO_TEST_CASE(test_colinearization_reversal_inside_bounds) {
    // Test reversals that stay within tolerance cylinder
    // Waypoint reverses direction but remains within diameter bounds
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),  // Forward, within tolerance
                                            makeVector({0.5, 0.005, 0.0, 0, 0, 0}),  // Reversal, but within tolerance of 0→2 line
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);  // diameter=0.02 (radius=0.01)
    // The reversal at waypoint 2 projects outside [0,1] range on the 0→3 segment
    // so it should be preserved by the monotonic advancement check
    BOOST_CHECK_EQUAL(waypoints.size(), 4);
}

BOOST_AUTO_TEST_CASE(test_colinearization_reversal_outside_bounds) {
    // Test reversals that extend outside tolerance
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.02, 0.0, 0, 0, 0}),  // Outside tolerance
                                            makeVector({0.5, 0.0, 0.0, 0, 0, 0}),   // Reversal
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    // All waypoints preserved - middle point outside tolerance, reversal detected
    BOOST_CHECK_EQUAL(waypoints.size(), 4);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_preserves_monotonicity) {
    // Verify monotonic advancement check (projection bounds checking)
    // Points that project before start or after end of segment must be preserved
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({0.5, 0.005, 0.0, 0, 0, 0}),  // Projects in [0,1] range
                                            makeVector({1.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    // Middle point should be coalesced (within tolerance and projects in range)
    BOOST_CHECK_EQUAL(waypoints.size(), 2);

    // Now test point that projects outside range
    waypoints = {makeVector({1.0, 0.0, 0.0, 0, 0, 0}),
                 makeVector({0.5, 0.005, 0.0, 0, 0, 0}),  // Projects before start (backward movement)
                 makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    // Middle point preserved due to projection < 0 (monotonicity violation)
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(failed_trajectory_tests)

BOOST_AUTO_TEST_CASE(test_failed_trajectory_low_tolerance) {
    namespace json = Json;
    const std::list<Eigen::VectorXd> waypoints = {
        makeVector({-4.69128, -2.91963, -2.26317, 0.489444, 1.56107, 1.59114}),
        makeVector({-4.69117, -2.91964, -2.26309, 0.489552, 1.56106, 1.59116}),
        makeVector({-4.70382, -2.91788, -2.26929, 0.482124, 1.56016, 1.5789}),
        makeVector({-4.70963, -2.91635, -2.27192, 0.475846, 1.5592, 1.5733}),
        makeVector({-4.72595, -2.91965, -2.28115, 0.48728, 1.55764, 1.55699}),
        makeVector({-4.73848, -2.91581, -2.28515, 0.472227, 1.55893, 1.54486}),
        makeVector({-4.82513, -2.90866, -2.31311, 0.440307, 1.56338, 1.45867}),
        makeVector({-4.88347, -2.90632, -2.32819, 0.425642, 1.55701, 1.40029}),
        makeVector({-4.92608, -2.90817, -2.33889, 0.432907, 1.55442, 1.35774}),
        makeVector({-5.01661, -2.91314, -2.35718, 0.457662, 1.55075, 1.26755}),
        makeVector({-5.064311111111, -2.914144444, -2.3635444442, 1.2345678901234567, 1.54814444446, 1.22006}),
    };

    const double k_tolerance = 5e-18;
    const Path path(waypoints, k_tolerance);

    // Create trajectory with normal velocity/acceleration constraints
    const auto max_velocity_vec = Eigen::VectorXd::Constant(6, 1.0);
    const auto max_acceleration_vec = Eigen::VectorXd::Constant(6, 1.0);

    const Trajectory trajectory(path, max_velocity_vec, max_acceleration_vec);

    BOOST_REQUIRE(!trajectory.isValid());
    const std::string k_test_path = std::filesystem::temp_directory_path();
    const std::string k_resource_name = "test_arm";
    const std::string k_timestamp = unix_time_iso8601();
    const std::string k_filename = failed_trajectory_filename(k_test_path, k_resource_name, k_timestamp);

    const std::string json_content = serialize_failed_trajectory_to_json(waypoints, max_velocity_vec, max_acceleration_vec, k_tolerance);

    // Write the failed trajectory JSON
    std::ofstream json_file(k_filename);
    json_file << json_content;
    json_file.close();

    std::ifstream readback(k_filename);
    BOOST_CHECK(readback.good());

    std::stringstream buffer;
    buffer << readback.rdbuf();
    readback.close();

    json::Value readback_parsed;
    const json::CharReaderBuilder reader;
    BOOST_REQUIRE(json::parseFromStream(reader, buffer, &readback_parsed, NULL));

    BOOST_REQUIRE(readback_parsed.isMember("timestamp"));
    BOOST_REQUIRE(readback_parsed.isMember("path_tolerance_delta_rads"));
    BOOST_REQUIRE(readback_parsed.isMember("max_velocity_vec_rads_per_sec"));
    BOOST_REQUIRE(readback_parsed.isMember("max_acceleration_vec_rads_per_sec2"));
    BOOST_REQUIRE(readback_parsed.isMember("waypoints_rads"));

    BOOST_REQUIRE(readback_parsed["timestamp"].isString());
    BOOST_REQUIRE(readback_parsed["path_tolerance_delta_rads"].isDouble());
    BOOST_REQUIRE(readback_parsed["max_velocity_vec_rads_per_sec"].isArray());
    BOOST_REQUIRE(readback_parsed["max_acceleration_vec_rads_per_sec2"].isArray());
    BOOST_REQUIRE(readback_parsed["waypoints_rads"].isArray());

    BOOST_REQUIRE_EQUAL(readback_parsed["max_velocity_vec_rads_per_sec"].size(), 6);
    BOOST_REQUIRE_EQUAL(readback_parsed["max_acceleration_vec_rads_per_sec2"].size(), 6);
    BOOST_REQUIRE_EQUAL(readback_parsed["waypoints_rads"].size(), waypoints.size());

    BOOST_REQUIRE_CLOSE(readback_parsed["path_tolerance_delta_rads"].asDouble(), k_tolerance, 1e-10);
}

BOOST_AUTO_TEST_CASE(test_state_estimation_mismatch_nearly_identical_waypoints) {
    // Regression test for state estimation mismatch creating nearly-identical waypoints.
    // This case has two waypoints that are 0.000275 rad apart (L∞ norm),
    // which is below the current dedup threshold of 1e-4 rad.
    // Both legacy and new generators fail on this input.
    //
    // Root cause: Motion planner believes arm is at position A, actual position is A',
    // they're close but outside L∞ dedup threshold. With dt=0.001s integration,
    // the distance between waypoints (~0.0004656 rad) is smaller than typical
    // integration step movement, causing trajectory generation to fail.
    //
    // Expected behavior: This test should PASS by confirming that legacy generator
    // FAILS on this input (trajectory.isValid() returns false).

    const std::list<Eigen::VectorXd> waypoints = {
        makeVector(
            {0.86162841320037842, -2.912748476068014, -1.4315807819366455, -0.43625719965014653, 1.4747567176818848, 0.86391860246658325}),
        makeVector(
            {0.86190300345456516, -2.9129917760199686, -1.4314953314444034, -0.43621047815164599, 1.4744832474373546, 0.86391310388875919}),
        makeVector(
            {0.76428190417993802, -2.9066158721334885, -1.4104209952962836, -0.45583191856938904, 1.4657484272628158, 0.76620493188004013}),
        makeVector(
            {0.70063350624593246, -2.9047065501727753, -1.3849914692788434, -0.47677585518868032, 1.453744431644983, 0.70233985427631329})};

    // Original parameters from failed trajectory
    const double path_tolerance = 0.1;  // 0.1 rad

    const Eigen::VectorXd max_velocity_vec = makeVector(
        {2.0943951023931953, 2.0943951023931953, 2.0943951023931953, 2.0943951023931953, 2.0943951023931953, 2.0943951023931953});

    const Eigen::VectorXd max_acceleration_vec = makeVector(
        {5.2359877559829897, 5.2359877559829897, 5.2359877559829897, 5.2359877559829897, 5.2359877559829897, 5.2359877559829897});

    // L∞ distance between first two waypoints (should be ~0.000275 rad)
    const double linf_dist = (waypoints.front() - *std::next(waypoints.begin())).cwiseAbs().maxCoeff();
    BOOST_TEST_MESSAGE("L∞ distance between first two waypoints: " << linf_dist);
    BOOST_CHECK_LT(linf_dist, 1e-3);  // Verify they're very close

    // Create path and trajectory using legacy generator
    const Path path(waypoints, path_tolerance);
    const Trajectory trajectory(path, max_velocity_vec, max_acceleration_vec);

    // Legacy generator should FAIL on this input
    BOOST_CHECK(!trajectory.isValid());

    BOOST_TEST_MESSAGE("Legacy trajectory generator correctly failed on nearly-identical waypoints");
}

BOOST_AUTO_TEST_SUITE_END()
