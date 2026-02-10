// Waypoint accumulator tests
// Extracted from test.cpp lines 18-148

#include <viam/trajex/totg/waypoint_accumulator.hpp>
#include <viam/trajex/totg/waypoint_utils.hpp>

#include <boost/test/unit_test.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/core/xmath.hpp>
#else
#include <xtensor/xmath.hpp>
#endif

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

BOOST_AUTO_TEST_CASE(construct_from_single_view) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
    const waypoint_accumulator source{waypoints};

    // Construct new accumulator from a view
    const waypoint_accumulator from_view{source[0]};
    BOOST_CHECK_EQUAL(from_view.size(), 1);
    BOOST_CHECK_EQUAL(from_view.dof(), 3);

    // Verify the waypoint data
    BOOST_CHECK_CLOSE(from_view[0](0), 1.0, 1e-10);
    BOOST_CHECK_CLOSE(from_view[0](1), 2.0, 1e-10);
    BOOST_CHECK_CLOSE(from_view[0](2), 3.0, 1e-10);
}

BOOST_AUTO_TEST_CASE(add_waypoint_single_view) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
    const waypoint_accumulator source{waypoints};

    // Build accumulator by adding views one at a time
    waypoint_accumulator result{source[0]};
    BOOST_CHECK_EQUAL(result.size(), 1);

    result.add_waypoint(source[2]);  // Skip middle waypoint
    BOOST_CHECK_EQUAL(result.size(), 2);

    // Verify we have first and third waypoints
    BOOST_CHECK_CLOSE(result[0](0), 1.0, 1e-10);
    BOOST_CHECK_CLOSE(result[1](0), 7.0, 1e-10);
}

BOOST_AUTO_TEST_CASE(add_waypoint_validates_dof) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints_3dof = {{1.0, 2.0, 3.0}};
    const xt::xarray<double> waypoints_2dof = {{4.0, 5.0}};

    waypoint_accumulator acc_3dof{waypoints_3dof};
    const waypoint_accumulator acc_2dof{waypoints_2dof};

    // Adding waypoint with wrong DOF should throw
    BOOST_CHECK_THROW(acc_3dof.add_waypoint(acc_2dof[0]), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(deduplicate_no_duplicates) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {2.0, 2.0, 2.0}};
    const waypoint_accumulator source{waypoints};

    const auto result = deduplicate_waypoints(source, 0.1);

    // All waypoints are sufficiently different, should keep all
    BOOST_CHECK_EQUAL(result.size(), 3);
    BOOST_CHECK_EQUAL(result.dof(), 3);
}

BOOST_AUTO_TEST_CASE(deduplicate_with_duplicates) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0},
        {0.05, 0.05, 0.05},  // Duplicate of first (within 0.1 tolerance)
        {1.0, 1.0, 1.0},     // Different
        {1.02, 1.02, 1.02},  // Duplicate of previous (within 0.1 tolerance)
        {2.0, 2.0, 2.0}      // Different
    };
    const waypoint_accumulator source{waypoints};

    const auto result = deduplicate_waypoints(source, 0.1);

    // Should keep waypoints 0, 2, 4 (indices in source)
    BOOST_CHECK_EQUAL(result.size(), 3);

    // Verify kept waypoints
    BOOST_CHECK_CLOSE(result[0](0), 0.0, 1e-10);
    BOOST_CHECK_CLOSE(result[1](0), 1.0, 1e-10);
    BOOST_CHECK_CLOSE(result[2](0), 2.0, 1e-10);
}

BOOST_AUTO_TEST_CASE(deduplicate_always_keeps_first) {
    using namespace viam::trajex::totg;

    const xt::xarray<double> waypoints = {{1.0, 1.0, 1.0}, {1.01, 1.01, 1.01}, {1.02, 1.02, 1.02}};
    const waypoint_accumulator source{waypoints};

    const auto result = deduplicate_waypoints(source, 0.1);

    // First waypoint always kept, others are duplicates
    BOOST_CHECK_EQUAL(result.size(), 1);
    BOOST_CHECK_CLOSE(result[0](0), 1.0, 1e-10);
}

BOOST_AUTO_TEST_CASE(segment_no_reversals) {
    using namespace viam::trajex::totg;

    // Simple straight line - no reversals
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}, {2.0, 2.0, 2.0}, {3.0, 3.0, 3.0}};
    waypoint_accumulator source{waypoints};

    auto segments = segment_at_reversals(std::move(source));

    // Should be single segment containing all waypoints
    BOOST_CHECK_EQUAL(segments.size(), 1);
    BOOST_CHECK_EQUAL(segments[0].size(), 4);
}

BOOST_AUTO_TEST_CASE(segment_with_single_reversal) {
    using namespace viam::trajex::totg;

    // Forward, then reverse
    const xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0},  // Forward
        {2.0, 2.0, 2.0},  // Cusp - reversal here
        {1.5, 1.5, 1.5},  // Reverse
        {1.0, 1.0, 1.0}   // Continue reverse
    };
    waypoint_accumulator source{waypoints};

    const auto segments = segment_at_reversals(std::move(source));

    // Should have 2 segments
    BOOST_CHECK_EQUAL(segments.size(), 2);

    // First segment: waypoints 0, 1, 2 (up to and including cusp)
    BOOST_CHECK_EQUAL(segments[0].size(), 3);
    BOOST_CHECK_CLOSE(segments[0][0](0), 0.0, 1e-10);
    BOOST_CHECK_CLOSE(segments[0][2](0), 2.0, 1e-10);  // cusp

    // Second segment: waypoints 2, 3, 4 (from cusp onwards)
    BOOST_CHECK_EQUAL(segments[1].size(), 3);
    BOOST_CHECK_CLOSE(segments[1][0](0), 2.0, 1e-10);  // cusp duplicated
    BOOST_CHECK_CLOSE(segments[1][2](0), 1.0, 1e-10);
}

BOOST_AUTO_TEST_CASE(segment_with_multiple_reversals) {
    using namespace viam::trajex::totg;

    // Multiple direction changes
    const xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},  // Forward
        {0.5, 0.0, 0.0},  // Reverse (cusp at waypoint 1)
        {1.0, 0.0, 0.0},  // Forward again (cusp at waypoint 2)
        {0.0, 0.0, 0.0}   // Reverse again
    };
    waypoint_accumulator source{waypoints};

    const auto segments = segment_at_reversals(std::move(source));

    // Should have 4 segments (3 reversals create 4 segments)
    BOOST_CHECK_EQUAL(segments.size(), 4);
}

BOOST_AUTO_TEST_SUITE_END()
