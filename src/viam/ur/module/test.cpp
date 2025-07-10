#define BOOST_TEST_MODULE test module test_ur5e

#include "ur_arm.hpp"

#include <boost/numeric/conversion/cast.hpp>
#include <boost/test/included/unit_test.hpp>

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

const std::string k_path = "/home/user";

BOOST_AUTO_TEST_CASE(test_waypoints_filename) {
    auto timestamp = unix_time_iso8601();
    auto x = waypoints_filename(k_path, timestamp);
    auto path = k_path + "/" + timestamp + "_waypoints.csv";
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_CASE(test_trajectory_filename) {
    auto timestamp = unix_time_iso8601();
    auto x = trajectory_filename(k_path, timestamp);
    auto path = k_path + "/" + timestamp + "_trajectory.csv";
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_CASE(test_arm_joint_positions_filename) {
    auto timestamp = unix_time_iso8601();
    auto x = arm_joint_positions_filename(k_path, timestamp);
    auto path = k_path + "/" + timestamp + "_arm_joint_positions.csv";
    BOOST_CHECK_EQUAL(x, path);
}

BOOST_AUTO_TEST_SUITE_END()
