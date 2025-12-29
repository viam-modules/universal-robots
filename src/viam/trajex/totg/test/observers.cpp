// Observer tests: event collection and handling

#include <viam/trajex/totg/observers.hpp>
#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>

#include <boost/test/unit_test.hpp>

namespace {

using namespace viam::trajex::totg;

}  // namespace

BOOST_AUTO_TEST_SUITE(trajectory_observer_tests)

BOOST_AUTO_TEST_CASE(event_collector_empty_on_construction) {
    trajectory_integration_event_collector collector;

    BOOST_CHECK(collector.events().empty());
    BOOST_CHECK_EQUAL(collector.events().size(), 0u);
    BOOST_CHECK(collector.begin() == collector.end());
    BOOST_CHECK(collector.cbegin() == collector.cend());
}

BOOST_AUTO_TEST_CASE(event_collector_stores_events_in_order) {
    trajectory_integration_event_collector collector;

    // Create a simple trajectory to generate some events
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    const path p = path::create(waypoints);

    const trajectory::options opts{
        .max_velocity = xt::ones<double>({3}),
        .max_acceleration = xt::ones<double>({3}),
        .observer = &collector
    };

    const trajectory traj = trajectory::create(p, opts);

    // Should have collected some events
    BOOST_CHECK(!collector.events().empty());
    BOOST_CHECK_GT(collector.events().size(), 0u);

    // Iterator range should match size
    BOOST_CHECK_EQUAL(std::distance(collector.begin(), collector.end()),
                      static_cast<std::ptrdiff_t>(collector.events().size()));
}

BOOST_AUTO_TEST_CASE(event_collector_range_iteration) {
    trajectory_integration_event_collector collector;

    // Create a simple trajectory
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const path p = path::create(waypoints);

    const trajectory::options opts{
        .max_velocity = xt::ones<double>({3}),
        .max_acceleration = xt::ones<double>({3}),
        .observer = &collector
    };

    const trajectory traj = trajectory::create(p, opts);

    // Should be able to iterate with range-for
    size_t count = 0;
    for (const auto& event : collector) {
        // Each event should be one of the variant alternatives
        BOOST_CHECK(std::holds_alternative<trajectory::integration_observer::started_forward_event>(event) ||
                    std::holds_alternative<trajectory::integration_observer::limit_hit_event>(event) ||
                    std::holds_alternative<trajectory::integration_observer::started_backward_event>(event) ||
                    std::holds_alternative<trajectory::integration_observer::splice_event>(event));
        ++count;
    }

    BOOST_CHECK_EQUAL(count, collector.events().size());
}

BOOST_AUTO_TEST_CASE(event_collector_contains_expected_event_types) {
    trajectory_integration_event_collector collector;

    // Create trajectory with reversal to generate diverse events
    const xt::xarray<double> waypoints = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.5, 0.0, 0.0}  // Reversal
    };
    const path p = path::create(waypoints);

    const trajectory::options opts{
        .max_velocity = xt::ones<double>({3}) * 0.5,
        .max_acceleration = xt::ones<double>({3}) * 0.3,
        .observer = &collector
    };

    const trajectory traj = trajectory::create(p, opts);

    // Count event types
    size_t forward_starts = 0;
    size_t limit_hits = 0;
    size_t backward_starts = 0;
    size_t splices = 0;

    for (const auto& event : collector.events()) {
        if (std::holds_alternative<trajectory::integration_observer::started_forward_event>(event)) {
            ++forward_starts;
        } else if (std::holds_alternative<trajectory::integration_observer::limit_hit_event>(event)) {
            ++limit_hits;
        } else if (std::holds_alternative<trajectory::integration_observer::started_backward_event>(event)) {
            ++backward_starts;
        } else if (std::holds_alternative<trajectory::integration_observer::splice_event>(event)) {
            ++splices;
        }
    }

    // Should have at least some forward starts (always happens)
    BOOST_CHECK_GT(forward_starts, 0u);

    // Total events should match
    BOOST_CHECK_EQUAL(forward_starts + limit_hits + backward_starts + splices,
                      collector.events().size());
}

BOOST_AUTO_TEST_SUITE_END()
