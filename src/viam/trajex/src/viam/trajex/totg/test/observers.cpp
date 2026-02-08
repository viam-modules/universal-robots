#include <viam/trajex/totg/observers.hpp>
#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/arc_velocity.hpp>

#include <boost/test/unit_test.hpp>

namespace {

using namespace viam::trajex::totg;
using viam::trajex::arc_length;
using viam::trajex::arc_velocity;

// Test observer that counts method calls
class counting_observer : public trajectory::integration_observer {
   public:
    int forward_starts = 0;
    int limit_hits = 0;
    int backward_starts = 0;
    int splices = 0;

    void on_started_forward_integration(const trajectory&, started_forward_event) override {
        ++forward_starts;
    }

    void on_hit_limit_curve(const trajectory&, limit_hit_event) override {
        ++limit_hits;
    }

    void on_started_backward_integration(const trajectory&, started_backward_event) override {
        ++backward_starts;
    }

    void on_trajectory_extended(const trajectory&, splice_event) override {
        ++splices;
    }
};

}  // namespace

BOOST_AUTO_TEST_SUITE(trajectory_observer_tests)

BOOST_AUTO_TEST_CASE(composite_observer_forwards_to_single_observer) {
    composite_integration_observer composite;
    auto counter = std::make_shared<counting_observer>();
    composite.add_observer(counter);

    // Create dummy trajectory for event context (skip integration)
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const path p = path::create(waypoints);
    const trajectory::options opts{.max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3})};
    const trajectory traj = trajectory::create(p, opts, {});

    // Directly call composite methods
    composite.on_started_forward_integration(traj, {.start = {arc_length{0.0}, arc_velocity{0.0}}});
    composite.on_hit_limit_curve(
        traj, {.breach = {arc_length{0.5}, arc_velocity{1.0}}, .s_dot_max_acc = arc_velocity{1.0}, .s_dot_max_vel = arc_velocity{1.0}});
    composite.on_started_backward_integration(
        traj, {.start = {arc_length{1.0}, arc_velocity{0.0}}, .kind = trajectory::switching_point_kind::k_path_end});

    // Verify forwarding
    BOOST_CHECK_EQUAL(counter->forward_starts, 1);
    BOOST_CHECK_EQUAL(counter->limit_hits, 1);
    BOOST_CHECK_EQUAL(counter->backward_starts, 1);
}

BOOST_AUTO_TEST_CASE(composite_observer_forwards_to_multiple_observers) {
    composite_integration_observer composite;
    auto counter1 = std::make_shared<counting_observer>();
    auto counter2 = std::make_shared<counting_observer>();

    composite.add_observer(counter1);
    composite.add_observer(counter2);

    // Create dummy trajectory (skip integration)
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const path p = path::create(waypoints);
    const trajectory::options opts{.max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3})};
    const trajectory traj = trajectory::create(p, opts, {});

    // Call each method once
    composite.on_started_forward_integration(traj, {.start = {arc_length{0.0}, arc_velocity{0.0}}});
    composite.on_hit_limit_curve(
        traj, {.breach = {arc_length{0.5}, arc_velocity{1.0}}, .s_dot_max_acc = arc_velocity{1.0}, .s_dot_max_vel = arc_velocity{1.0}});
    composite.on_started_backward_integration(
        traj, {.start = {arc_length{1.0}, arc_velocity{0.0}}, .kind = trajectory::switching_point_kind::k_path_end});

    // Both observers should have received all events
    BOOST_CHECK_EQUAL(counter1->forward_starts, 1);
    BOOST_CHECK_EQUAL(counter1->limit_hits, 1);
    BOOST_CHECK_EQUAL(counter1->backward_starts, 1);

    BOOST_CHECK_EQUAL(counter2->forward_starts, 1);
    BOOST_CHECK_EQUAL(counter2->limit_hits, 1);
    BOOST_CHECK_EQUAL(counter2->backward_starts, 1);
}

BOOST_AUTO_TEST_CASE(composite_observer_throws_on_null_observer) {
    composite_integration_observer composite;
    BOOST_CHECK_THROW(composite.add_observer(std::shared_ptr<trajectory::integration_observer>()), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(composite_observer_add_returns_observer) {
    composite_integration_observer composite;
    auto counter = std::make_shared<counting_observer>();
    auto returned = composite.add_observer(counter);

    BOOST_CHECK_EQUAL(returned.get(), counter.get());
}

BOOST_AUTO_TEST_CASE(composite_observer_prevents_reentrancy_via_add) {
    composite_integration_observer composite;

    // Create observer that tries to add another observer during callback
    class reentrant_adder : public trajectory::integration_observer {
       public:
        explicit reentrant_adder(composite_integration_observer& comp) : composite_(comp) {}

        void on_started_forward_integration(const trajectory&, started_forward_event) override {
            // Try to add observer during dispatch - should throw
            composite_.add_observer(std::make_shared<counting_observer>());
        }

        void on_hit_limit_curve(const trajectory&, limit_hit_event) override {}
        void on_started_backward_integration(const trajectory&, started_backward_event) override {}
        void on_trajectory_extended(const trajectory&, splice_event) override {}

       private:
        composite_integration_observer& composite_;
    };

    auto reentrant = std::make_shared<reentrant_adder>(composite);
    composite.add_observer(reentrant);

    // Create dummy trajectory
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const path p = path::create(waypoints);
    const trajectory::options opts{.max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3})};
    const trajectory traj = trajectory::create(p, opts, {});

    // Trigger dispatch - should throw due to re-entrant add_observer call
    BOOST_CHECK_THROW(composite.on_started_forward_integration(traj, {.start = {arc_length{0.0}, arc_velocity{0.0}}}), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(composite_observer_prevents_reentrancy_via_dispatch) {
    composite_integration_observer composite;

    // Create observer that tries to dispatch during callback
    class reentrant_dispatcher : public trajectory::integration_observer {
       public:
        explicit reentrant_dispatcher(composite_integration_observer& comp, const trajectory& traj) : composite_(comp), traj_(traj) {}

        void on_started_forward_integration(const trajectory&, started_forward_event) override {
            // Try to dispatch during dispatch - should throw
            composite_.on_hit_limit_curve(
                traj_,
                {.breach = {arc_length{0.5}, arc_velocity{1.0}}, .s_dot_max_acc = arc_velocity{1.0}, .s_dot_max_vel = arc_velocity{1.0}});
        }

        void on_hit_limit_curve(const trajectory&, limit_hit_event) override {}
        void on_started_backward_integration(const trajectory&, started_backward_event) override {}
        void on_trajectory_extended(const trajectory&, splice_event) override {}

       private:
        composite_integration_observer& composite_;
        const trajectory& traj_;
    };

    // Create dummy trajectory
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const path p = path::create(waypoints);
    const trajectory::options opts{.max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3})};
    const trajectory traj = trajectory::create(p, opts, {});

    auto reentrant = std::make_shared<reentrant_dispatcher>(composite, traj);
    composite.add_observer(reentrant);

    // Trigger dispatch - should throw due to re-entrant dispatch call
    BOOST_CHECK_THROW(composite.on_started_forward_integration(traj, {.start = {arc_length{0.0}, arc_velocity{0.0}}}), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(event_collector_empty_on_construction) {
    const trajectory_integration_event_collector collector;

    BOOST_CHECK(collector.events().empty());
    BOOST_CHECK_EQUAL(collector.events().size(), 0U);
    BOOST_CHECK(collector.begin() == collector.end());
    BOOST_CHECK(collector.cbegin() == collector.cend());
}

BOOST_AUTO_TEST_CASE(event_collector_stores_events_in_order) {
    trajectory_integration_event_collector collector;

    // Create a simple trajectory to generate some events
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    const path p = path::create(waypoints);

    const trajectory::options opts{
        .max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3}), .observer = &collector};

    const trajectory traj = trajectory::create(p, opts);

    // Should have collected some events
    BOOST_CHECK(!collector.events().empty());
    BOOST_CHECK_GT(collector.events().size(), 0U);

    // Iterator range should match size
    BOOST_CHECK_EQUAL(std::distance(collector.begin(), collector.end()), static_cast<std::ptrdiff_t>(collector.events().size()));
}

BOOST_AUTO_TEST_CASE(event_collector_range_iteration) {
    trajectory_integration_event_collector collector;

    // Create a simple trajectory
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    const path p = path::create(waypoints);

    const trajectory::options opts{
        .max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3}), .observer = &collector};

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
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.5, 0.0, 0.0}  // Reversal
    };
    const path p = path::create(waypoints);

    const trajectory::options opts{
        .max_velocity = xt::ones<double>({3}) * 0.5, .max_acceleration = xt::ones<double>({3}) * 0.3, .observer = &collector};

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
    BOOST_CHECK_GT(forward_starts, 0);

    // Total events should match
    BOOST_CHECK_EQUAL(forward_starts + limit_hits + backward_starts + splices, collector.events().size());
}

BOOST_AUTO_TEST_SUITE_END()
