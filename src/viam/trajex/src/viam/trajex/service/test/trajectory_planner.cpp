#include <viam/trajex/service/trajectory_planner.hpp>

#include <boost/test/unit_test.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

namespace {

using namespace viam::trajex;
using namespace viam::trajex::totg;

struct test_receiver {
    int segment_count = 0;
    double total_duration = 0.0;
    std::chrono::microseconds total_gen_time{};
};

trajectory_planner<test_receiver>::config simple_config() {
    return {
        .velocity_limits = xt::xarray<double>{1.0, 1.0, 1.0},
        .acceleration_limits = xt::xarray<double>{1.0, 1.0, 1.0},
        .path_blend_tolerance = 0.001,
        .colinearization_ratio = std::nullopt,
    };
}

// waypoint_accumulator views data, doesn't own it, so the xarray must
// outlive the accumulator. Stash is the mechanism for that.
waypoint_accumulator stash_waypoints(trajectory_planner<test_receiver>& planner, xt::xarray<double> wp) {
    auto data = planner.stash(std::move(wp));
    return waypoint_accumulator{*data};
}

}  // namespace

BOOST_AUTO_TEST_SUITE(trajectory_planner_tests)

BOOST_AUTO_TEST_CASE(missing_waypoint_provider_throws) {
    auto planner = trajectory_planner<test_receiver>(simple_config());

    BOOST_CHECK_THROW(planner.execute([](const auto&, const auto&, const auto&) -> std::optional<test_receiver> { return std::nullopt; }),
                      std::logic_error);
}

BOOST_AUTO_TEST_CASE(fewer_than_two_waypoints_skips_algorithms) {
    bool decider_called = false;

    auto result = trajectory_planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{1.0, 2.0, 3.0}}); })
                      .with_trajex([](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) {})
                      .execute([&](const auto& planner, const auto& trajex, const auto& legacy) -> std::optional<test_receiver> {
                          decider_called = true;
                          BOOST_CHECK_EQUAL(planner.processed_waypoint_count(), 1U);
                          BOOST_CHECK(!trajex.receiver);
                          BOOST_CHECK(!trajex.error);
                          BOOST_CHECK(!legacy.receiver);
                          BOOST_CHECK(!legacy.error);
                          return std::nullopt;
                      });

    BOOST_CHECK(decider_called);
    BOOST_CHECK(!result);
}

BOOST_AUTO_TEST_CASE(trajex_only_success) {
    bool success_called = false;

    auto result = trajectory_planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                      .with_trajex([&](test_receiver& acc, const waypoint_accumulator&, const totg::trajectory& traj, auto elapsed) {
                          success_called = true;
                          acc.segment_count++;
                          acc.total_duration += traj.duration().count();
                          acc.total_gen_time += elapsed;
                      })
                      .execute([](const auto&, const auto& trajex, const auto& legacy) -> std::optional<test_receiver> {
                          BOOST_CHECK(trajex.receiver.has_value());
                          BOOST_CHECK(!legacy.receiver);
                          BOOST_CHECK(!legacy.error);
                          return std::move(trajex.receiver);
                      });

    BOOST_CHECK(success_called);
    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(result->segment_count, 1);
    BOOST_CHECK_GT(result->total_duration, 0.0);
}

BOOST_AUTO_TEST_CASE(legacy_only_success) {
    bool success_called = false;

    auto result = trajectory_planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                      .with_legacy([&](test_receiver& acc, const waypoint_accumulator&, const Path&, const Trajectory& traj, auto elapsed) {
                          success_called = true;
                          acc.segment_count++;
                          acc.total_duration += traj.getDuration();
                          acc.total_gen_time += elapsed;
                      })
                      .execute([](const auto&, const auto& trajex, const auto& legacy) -> std::optional<test_receiver> {
                          BOOST_CHECK(!trajex.receiver);
                          BOOST_CHECK(legacy.receiver.has_value());
                          return std::move(legacy.receiver);
                      });

    BOOST_CHECK(success_called);
    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(result->segment_count, 1);
    BOOST_CHECK_GT(result->total_duration, 0.0);
}

BOOST_AUTO_TEST_CASE(both_algorithms_success) {
    bool trajex_called = false;
    bool legacy_called = false;

    auto result = trajectory_planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                      .with_trajex([&](test_receiver& acc, const waypoint_accumulator&, const totg::trajectory& traj, auto) {
                          trajex_called = true;
                          acc.segment_count++;
                          acc.total_duration += traj.duration().count();
                      })
                      .with_legacy([&](test_receiver& acc, const waypoint_accumulator&, const Path&, const Trajectory& traj, auto) {
                          legacy_called = true;
                          acc.segment_count++;
                          acc.total_duration += traj.getDuration();
                      })
                      .execute([](const auto&, const auto& trajex, const auto& legacy) -> std::optional<test_receiver> {
                          BOOST_CHECK(trajex.receiver.has_value());
                          BOOST_CHECK(legacy.receiver.has_value());
                          return std::move(trajex.receiver);
                      });

    BOOST_CHECK(trajex_called);
    BOOST_CHECK(legacy_called);
    BOOST_REQUIRE(result);
}

BOOST_AUTO_TEST_CASE(preprocessor_runs_before_algorithms) {
    auto result = trajectory_planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                      .with_waypoint_preprocessor(
                          [](auto&, waypoint_accumulator& accumulator) { accumulator = deduplicate_waypoints(accumulator, 1000.0); })
                      .with_trajex([](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) {
                          BOOST_FAIL("trajex should not run on < 2 waypoints");
                      })
                      .execute([](const auto& planner, const auto& trajex, const auto&) -> std::optional<test_receiver> {
                          BOOST_CHECK_EQUAL(planner.processed_waypoint_count(), 1U);
                          BOOST_CHECK(!trajex.receiver);
                          return std::nullopt;
                      });

    BOOST_CHECK(!result);
}

BOOST_AUTO_TEST_CASE(validator_can_reject_move) {
    BOOST_CHECK_THROW(trajectory_planner<test_receiver>(simple_config())
                          .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                          .with_move_validator([](auto&, const waypoint_accumulator&) { throw std::runtime_error("move rejected"); })
                          .with_trajex([](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) {})
                          .execute([](const auto&, const auto&, const auto&) -> std::optional<test_receiver> {
                              BOOST_FAIL("decider should not be called after validation failure");
                              return std::nullopt;
                          }),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(segmenter_produces_multiple_segments) {
    int trajex_segment_count = 0;

    auto result =
        trajectory_planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}); })
            .with_segmenter([](auto&, waypoint_accumulator accumulator) { return segment_at_reversals(std::move(accumulator)); })
            .with_trajex([&](test_receiver& acc, const waypoint_accumulator&, const totg::trajectory&, auto) {
                trajex_segment_count++;
                acc.segment_count++;
            })
            .execute(
                [](const auto&, const auto& trajex, const auto&) -> std::optional<test_receiver> { return std::move(trajex.receiver); });

    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(trajex_segment_count, 2);
    BOOST_CHECK_EQUAL(result->segment_count, 2);
}

BOOST_AUTO_TEST_CASE(failure_disengages_receiver_for_remaining_segments) {
    int success_count = 0;
    bool failure_called = false;

    auto result =
        trajectory_planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}); })
            .with_segmenter([](auto&, waypoint_accumulator accumulator) { return segment_at_reversals(std::move(accumulator)); })
            .with_trajex(
                [&](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) {
                    success_count++;
                    throw std::runtime_error("synthetic failure");
                },
                [&](const test_receiver&, const waypoint_accumulator&, const std::exception& e) {
                    failure_called = true;
                    BOOST_CHECK_EQUAL(std::string(e.what()), "synthetic failure");
                })
            .execute([](const auto&, const auto& trajex, const auto&) -> std::optional<test_receiver> {
                BOOST_CHECK(!trajex.receiver);
                BOOST_CHECK(trajex.error != nullptr);
                return std::nullopt;
            });

    BOOST_CHECK_EQUAL(success_count, 1);
    BOOST_CHECK(failure_called);
    BOOST_CHECK(!result);
}

BOOST_AUTO_TEST_CASE(stash_extends_data_lifetime) {
    auto result =
        trajectory_planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& planner) {
                auto data = planner.stash(xt::xarray<double>{{0.0, 0.0, 0.0}});
                waypoint_accumulator accumulator{*data};
                auto more = planner.stash(xt::xarray<double>{{1.0, 0.0, 0.0}});
                accumulator.add_waypoints(*more);
                return accumulator;
            })
            .with_trajex([](test_receiver& acc, const waypoint_accumulator&, const totg::trajectory&, auto) { acc.segment_count++; })
            .execute(
                [](const auto&, const auto& trajex, const auto&) -> std::optional<test_receiver> { return std::move(trajex.receiver); });

    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(result->segment_count, 1);
}

BOOST_AUTO_TEST_CASE(segment_trajex_false_passes_unsegmented_to_trajex) {
    int trajex_segments_seen = 0;
    int legacy_segments_seen = 0;

    auto cfg = simple_config();
    cfg.segment_trajex = false;

    trajectory_planner<test_receiver>(std::move(cfg))
        .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}); })
        .with_segmenter([](auto&, waypoint_accumulator accumulator) { return segment_at_reversals(std::move(accumulator)); })
        .with_trajex([&](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) { trajex_segments_seen++; })
        .with_legacy([&](test_receiver&, const waypoint_accumulator&, const Path&, const Trajectory&, auto) { legacy_segments_seen++; })
        .execute([](const auto&, const auto&, const auto&) -> std::optional<test_receiver> { return std::nullopt; });

    BOOST_CHECK_EQUAL(trajex_segments_seen, 1);
    BOOST_CHECK_EQUAL(legacy_segments_seen, 2);
}

BOOST_AUTO_TEST_CASE(processed_waypoint_count_reflects_preprocessing) {
    trajectory_planner<test_receiver>(simple_config())
        .with_waypoint_provider(
            [](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}); })
        .with_waypoint_preprocessor(
            [](auto&, waypoint_accumulator& accumulator) { accumulator = deduplicate_waypoints(accumulator, 1e-6); })
        .with_trajex([](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) {})
        .execute([](const auto& planner, const auto&, const auto&) -> std::optional<test_receiver> {
            BOOST_CHECK_EQUAL(planner.processed_waypoint_count(), 3U);
            return std::nullopt;
        });
}

BOOST_AUTO_TEST_CASE(decider_return_type_is_flexible) {
    const int result = trajectory_planner<test_receiver>(simple_config())
                           .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                           .with_trajex([](test_receiver&, const waypoint_accumulator&, const totg::trajectory&, auto) {})
                           .execute([](const auto&, const auto& trajex, const auto&) -> int { return trajex.receiver ? 42 : -1; });

    BOOST_CHECK_EQUAL(result, 42);
}

BOOST_AUTO_TEST_SUITE_END()
