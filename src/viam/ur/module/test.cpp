#define BOOST_TEST_MODULE test module test_ur5e

#include "ur_arm.hpp"
#include "utils.hpp"

#include <boost/numeric/conversion/cast.hpp>
#include <boost/test/included/unit_test.hpp>

#include <Eigen/Dense>

BOOST_AUTO_TEST_SUITE(test_1)

namespace {

Eigen::VectorXd makeVector(std::vector<double> data) {
    return Eigen::Map<Eigen::VectorXd>(data.data(), boost::numeric_cast<Eigen::Index>(data.size()));
}

}  // namespace

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
    const auto timestamp = unix_time_iso8601();
    const auto path = k_path + "/" + timestamp + "_waypoints.csv";

    auto x = waypoints_filename(k_path, timestamp);
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_CASE(test_trajectory_filename) {
    const std::string k_path = "/home/user";
    const auto timestamp = unix_time_iso8601();
    const auto path = k_path + "/" + timestamp + "_trajectory.csv";

    auto x = trajectory_filename(k_path, timestamp);
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_CASE(test_arm_joint_positions_filename) {
    const std::string k_path = "/home/user";
    const auto timestamp = unix_time_iso8601();
    const auto path = k_path + "/" + timestamp + "_arm_joint_positions.csv";

    auto x = arm_joint_positions_filename(k_path, timestamp);
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, M_PI / 2.0, 0.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    Eigen::Matrix3d expected;
    expected << 1, 0, 0, 0, 0, -1, 0, 1, 0;

    BOOST_CHECK(rotation_matrix.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_90_degrees_y) {
    // Test: 90-degree rotation around Y-axis
    // Why: Standard orthogonal rotation - X becomes -Z, Z becomes X
    // Verifies correct axis ordering in rotation matrix construction
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, M_PI / 2.0, 0.0};
    const auto rotation_matrix = rotation_vector_to_matrix(tcp_pose);

    Eigen::Matrix3d expected;
    expected << 0, 0, 1, 0, 1, 0, -1, 0, 0;

    BOOST_CHECK(rotation_matrix.isApprox(expected, 1e-8));
}

BOOST_AUTO_TEST_CASE(test_rotation_vector_to_matrix_90_degrees_z) {
    // Test: 90-degree rotation around Z-axis
    // Why: Standard orthogonal rotation - X becomes Y, Y becomes -X
    // Most common rotation in robotics applications
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0};
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
    const double angle = M_PI / 4.0;
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, M_PI / 2.0, 0.0, 0.0};
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2.0};
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, M_PI / 4.0, M_PI / 4.0, 0.0};
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, M_PI, 0.0, 0.0};
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, M_PI / 6.0, M_PI / 4.0, M_PI / 3.0};
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
    const vector6d_t tcp_pose = {0.0, 0.0, 0.0, -M_PI / 2.0, 0.0, 0.0};
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
