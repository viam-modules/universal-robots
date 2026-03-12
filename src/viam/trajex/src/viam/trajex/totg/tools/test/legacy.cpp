#include <viam/trajex/totg/tools/legacy.hpp>

#include <list>

#include <boost/test/unit_test.hpp>

#include <Eigen/Dense>

#include <boost/numeric/conversion/cast.hpp>

namespace {

using namespace viam::trajex::totg::legacy;

Eigen::VectorXd makeVector(std::vector<double> data) {
    return Eigen::Map<Eigen::VectorXd>(data.data(), boost::numeric_cast<Eigen::Index>(data.size()));
}

}  // namespace

BOOST_AUTO_TEST_SUITE(legacy_for_each_sample_tests)

BOOST_AUTO_TEST_CASE(test_for_each_sample_count) {
    std::vector<std::pair<double, double>> samples;
    for_each_sample(5.0, 5.0, [&](double t, double step) { samples.emplace_back(t, step); });
    BOOST_CHECK_EQUAL(samples.size(), static_cast<std::size_t>(std::ceil(5.0 * 5.0)));
}

BOOST_AUTO_TEST_CASE(test_for_each_sample_duration_shorter_than_period) {
    std::vector<std::pair<double, double>> samples;
    for_each_sample(1.0, 0.5, [&](double t, double step) { samples.emplace_back(t, step); });
    BOOST_REQUIRE_EQUAL(samples.size(), 1U);
    // Last sample uses exactly the duration value
    BOOST_CHECK_EQUAL(samples[0].first, 1.0);
    BOOST_CHECK_EQUAL(samples[0].second, 1.0);
}

BOOST_AUTO_TEST_CASE(test_for_each_sample_invalid_duration) {
    BOOST_CHECK_THROW(for_each_sample(0.0, 0.5, [](double, double) { BOOST_FAIL("should not be called"); }), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(test_for_each_sample_invalid_frequency) {
    BOOST_CHECK_THROW(for_each_sample(1.0, 0.0, [](double, double) { BOOST_FAIL("should not be called"); }), std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()  // legacy_for_each_sample_tests

BOOST_AUTO_TEST_SUITE(legacy_colinearization_tests)

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_on_segment) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_within_tube) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.005, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_outside_tube) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.02, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_before_segment) {
    const Eigen::VectorXd line_start = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({0.5, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_point_after_segment) {
    const Eigen::VectorXd line_start = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({2.5, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_degenerate_segment) {
    const Eigen::VectorXd line_start = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(!within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_zero_tolerance) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_on = makeVector({1.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_off = makeVector({1.0, 0.0001, 0.0, 0, 0, 0});
    const double tolerance = 0.0;

    BOOST_CHECK(within_colinearization_tolerance(point_on, line_start, line_end, tolerance));
    BOOST_CHECK(!within_colinearization_tolerance(point_off, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_multi_dof) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({0.0, 0.0, 2.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({0.0, 0.005, 1.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_at_boundary) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point = makeVector({1.0, 0.01, 0.0, 0, 0, 0});
    const double tolerance = 0.02;

    BOOST_CHECK(within_colinearization_tolerance(point, line_start, line_end, tolerance));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_empty_list) {
    std::list<Eigen::VectorXd> waypoints;
    apply_colinearization(waypoints, 0.01);
    BOOST_CHECK_EQUAL(waypoints.size(), 0);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_single_waypoint) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({1.0, 0, 0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 1);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_two_waypoints) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0, 0, 0, 0, 0}), makeVector({1.0, 0, 0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_three_points_middle_coalesced) {
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({1.0, 0.005, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
    BOOST_CHECK(waypoints.front().isApprox(makeVector({0.0, 0.0, 0.0, 0, 0, 0})));
    BOOST_CHECK(waypoints.back().isApprox(makeVector({2.0, 0.0, 0.0, 0, 0, 0})));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_three_points_middle_not_coalesced) {
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({1.0, 0.02, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_multiple_points_all_coalesced) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({3.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({4.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
    BOOST_CHECK(waypoints.front().isApprox(makeVector({0.0, 0.0, 0.0, 0, 0, 0})));
    BOOST_CHECK(waypoints.back().isApprox(makeVector({4.0, 0.0, 0.0, 0, 0, 0})));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_mixed_coalescing) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.02, 0.0, 0, 0, 0}),
                                            makeVector({3.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({4.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_degenerate_segment) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 4);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_drift_prevented) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.009, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({3.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 2);
    BOOST_CHECK(waypoints.front().isApprox(makeVector({0.0, 0.0, 0.0, 0, 0, 0})));
    BOOST_CHECK(waypoints.back().isApprox(makeVector({3.0, 0.0, 0.0, 0, 0, 0})));
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_zero_tolerance) {
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({1.0, 0.0, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.0);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_negative_tolerance) {
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({1.0, 0.0, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, -0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_CASE(test_within_colinearization_tolerance_diameter_interpretation) {
    const Eigen::VectorXd line_start = makeVector({0.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd line_end = makeVector({2.0, 0.0, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_inside = makeVector({1.0, 0.009, 0.0, 0, 0, 0});
    const Eigen::VectorXd point_outside = makeVector({1.0, 0.011, 0.0, 0, 0, 0});

    BOOST_CHECK(within_colinearization_tolerance(point_inside, line_start, line_end, 0.02));
    BOOST_CHECK(!within_colinearization_tolerance(point_outside, line_start, line_end, 0.02));
}

BOOST_AUTO_TEST_CASE(test_colinearization_reversal_inside_bounds) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({0.5, 0.005, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 4);
}

BOOST_AUTO_TEST_CASE(test_colinearization_reversal_outside_bounds) {
    std::list<Eigen::VectorXd> waypoints = {makeVector({0.0, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({1.0, 0.02, 0.0, 0, 0, 0}),
                                            makeVector({0.5, 0.0, 0.0, 0, 0, 0}),
                                            makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 4);
}

BOOST_AUTO_TEST_CASE(test_apply_colinearization_preserves_monotonicity) {
    std::list<Eigen::VectorXd> waypoints = {
        makeVector({0.0, 0.0, 0.0, 0, 0, 0}), makeVector({0.5, 0.005, 0.0, 0, 0, 0}), makeVector({1.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 2);

    waypoints = {makeVector({1.0, 0.0, 0.0, 0, 0, 0}), makeVector({0.5, 0.005, 0.0, 0, 0, 0}), makeVector({2.0, 0.0, 0.0, 0, 0, 0})};
    apply_colinearization(waypoints, 0.02);
    BOOST_CHECK_EQUAL(waypoints.size(), 3);
}

BOOST_AUTO_TEST_SUITE_END()
