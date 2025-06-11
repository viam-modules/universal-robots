#define BOOST_TEST_MODULE test module test_ur5e

#include "src/ur5e_arm.hpp"

#include <boost/numeric/conversion/cast.hpp>
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(test_1)

namespace {

Eigen::VectorXd makeVector(std::vector<double> data) {
    return Eigen::Map<Eigen::VectorXd>(data.data(), boost::numeric_cast<Eigen::Index>(data.size()));
}

}  // namespace

BOOST_AUTO_TEST_CASE(test_write_waypoints_to_csv) {
    const std::vector<Eigen::VectorXd> waypoints = {
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
    const std::vector<vector6d_t> p_p = {
        {1.1, 2, 3, 4, 5, 6},
        {3.1, 2, 3, 4, 5, 6},
        {6.1, 2, 3, 4, 5, 6},
        {9.1, 2, 3, 4, 5, 6},
    };
    const std::vector<vector6d_t> p_v = {
        {1.2, 2, 3, 4, 5, 6},
        {4.2, 2, 3, 4, 5, 6},
        {7.1, 2, 3, 4, 5, 6},
        {10.1, 2, 3, 4, 5, 6},
    };
    const std::vector<float> time = {1.2, 2, 3, 4};

    const auto* const expected =
        "t(s),j0,j1,j2,j3,j4,j5,v0,v1,v2,v3,v4,v5\n"
        "1.2,1.1,2,3,4,5,6,1.2,2,3,4,5,6\n"
        "2,3.1,2,3,4,5,6,4.2,2,3,4,5,6\n"
        "3,6.1,2,3,4,5,6,7.1,2,3,4,5,6\n"
        "4,9.1,2,3,4,5,6,10.1,2,3,4,5,6\n";

    write_trajectory_to_file("./write_trajectory_to_file_test.csv", p_p, p_v, time);
    const std::ifstream csv("./write_trajectory_to_file_test.csv");
    std::stringstream buf;
    buf << csv.rdbuf();
    BOOST_CHECK_EQUAL(std::remove("./write_trajectory_to_file_test.csv"), 0);
    BOOST_CHECK_EQUAL(buf.str(), expected);
}

BOOST_AUTO_TEST_CASE(test_waypoints_filename) {
    auto x = waypoints_filename("/home/user", 1747161493357);
    BOOST_CHECK_EQUAL(x, "/home/user/1747161493357_waypoints.csv");
}

BOOST_AUTO_TEST_CASE(test_trajectory_filename) {
    auto x = trajectory_filename("/home/user", 1747161493357);
    BOOST_CHECK_EQUAL(x, "/home/user/1747161493357_trajectory.csv");
}

BOOST_AUTO_TEST_CASE(test_arm_joint_positions_filename) {
    auto x = arm_joint_positions_filename("/home/user", 1747161493357);
    BOOST_CHECK_EQUAL(x, "/home/user/1747161493357_arm_joint_positions.csv");
}

BOOST_AUTO_TEST_SUITE_END()
