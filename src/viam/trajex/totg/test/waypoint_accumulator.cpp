// Waypoint accumulator tests
// Extracted from test.cpp lines 18-148

#include <viam/trajex/totg/waypoint_accumulator.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(waypoint_accumulator_tests)

BOOST_AUTO_TEST_CASE(construct_with_single_waypoint) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}};
    BOOST_CHECK_NO_THROW(waypoint_accumulator{waypoints});
}

BOOST_AUTO_TEST_CASE(construct_with_multiple_waypoints) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    BOOST_CHECK_NO_THROW(waypoint_accumulator{waypoints});
}

BOOST_AUTO_TEST_CASE(add_waypoints) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints1 = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints1};
    BOOST_CHECK_EQUAL(acc.size(), 1);
    BOOST_CHECK_EQUAL(acc.dof(), 3);

    const xt::xarray<double> waypoints2 = {{4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    BOOST_CHECK_NO_THROW(acc.add_waypoints(waypoints2));
    BOOST_CHECK_EQUAL(acc.size(), 3);
}

BOOST_AUTO_TEST_CASE(validates_dimension) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints_1d = {1.0, 2.0, 3.0};
    BOOST_CHECK_THROW(waypoint_accumulator{waypoints_1d}, std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(validates_dof_consistency) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints};

    const xt::xarray<double> waypoints_wrong_dof = {{4.0, 5.0}};
    BOOST_CHECK_THROW(acc.add_waypoints(waypoints_wrong_dof), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(iterator_interface) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    waypoint_accumulator acc{waypoints};

    // Member function calls
    BOOST_CHECK_NO_THROW(acc.begin());
    BOOST_CHECK_NO_THROW(acc.end());
    BOOST_CHECK_NO_THROW(acc.cbegin());
    BOOST_CHECK_NO_THROW(acc.cend());

    // Verify they return the right type and work
    auto it1 = acc.begin();
    auto it2 = acc.end();
    BOOST_CHECK(it1 != it2);

    // ADL calls (unqualified)
    using std::begin;
    using std::end;
    BOOST_CHECK_NO_THROW(static_cast<void>(begin(acc)));
    BOOST_CHECK_NO_THROW(static_cast<void>(end(acc)));

    // std::cbegin/cend (qualified)
    BOOST_CHECK_NO_THROW(static_cast<void>(std::cbegin(acc)));
    BOOST_CHECK_NO_THROW(static_cast<void>(std::cend(acc)));

    // Verify range-based for works
    int count = 0;
    for (const auto& wp : acc) {
        (void)wp;  // Suppress unused warning
        ++count;
    }
    BOOST_CHECK_EQUAL(count, 2);
}

BOOST_AUTO_TEST_CASE(bounds_checked_access) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const waypoint_accumulator acc{waypoints};

    // Valid access should work
    BOOST_CHECK_NO_THROW(acc.at(0));
    BOOST_CHECK_NO_THROW(acc.at(1));

    // Out of bounds should throw
    BOOST_CHECK_THROW(acc.at(2), std::out_of_range);
    BOOST_CHECK_THROW(acc.at(100), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(unchecked_access) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const waypoint_accumulator acc{waypoints};

    // Unchecked access should work
    BOOST_CHECK_NO_THROW(acc[0]);
    BOOST_CHECK_NO_THROW(acc[1]);
}

BOOST_AUTO_TEST_CASE(empty_check) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}};
    const waypoint_accumulator acc{waypoints};

    BOOST_CHECK(!acc.empty());
    BOOST_CHECK_EQUAL(acc.size(), 1);
}

BOOST_AUTO_TEST_CASE(dof_consistency_after_add) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints1 = {{1.0, 2.0, 3.0}};
    waypoint_accumulator acc{waypoints1};
    BOOST_CHECK_EQUAL(acc.dof(), 3);

    const xt::xarray<double> waypoints2 = {{4.0, 5.0, 6.0}};
    acc.add_waypoints(waypoints2);
    BOOST_CHECK_EQUAL(acc.dof(), 3);  // DOF should remain consistent
}

BOOST_AUTO_TEST_SUITE_END()
