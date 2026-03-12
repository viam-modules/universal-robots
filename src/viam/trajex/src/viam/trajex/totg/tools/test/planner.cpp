#include <viam/trajex/totg/tools/planner.hpp>

#include <boost/test/unit_test.hpp>

#if __has_include(<xtensor/containers/xarray.hpp>)
#include <xtensor/containers/xarray.hpp>
#else
#include <xtensor/xarray.hpp>
#endif

namespace {

using namespace viam::trajex::totg;

struct test_receiver {
    int segment_count = 0;
    double total_duration = 0.0;
    std::chrono::microseconds total_gen_time{};
};

planner<test_receiver>::config simple_config() {
    return {
        .velocity_limits = xt::xarray<double>{1.0, 1.0, 1.0},
        .acceleration_limits = xt::xarray<double>{1.0, 1.0, 1.0},
        .path_blend_tolerance = 0.001,
        .colinearization_ratio = std::nullopt,
    };
}

// waypoint_accumulator views data, doesn't own it, so the xarray must
// outlive the accumulator. Stash is the mechanism for that.
waypoint_accumulator stash_waypoints(planner<test_receiver>& p, xt::xarray<double> wp) {
    auto data = p.stash(std::move(wp));
    return waypoint_accumulator{*data};
}

}  // namespace

BOOST_AUTO_TEST_SUITE(planner_tests)

BOOST_AUTO_TEST_CASE(missing_waypoint_provider_throws) {
    auto p = planner<test_receiver>(simple_config());

    BOOST_CHECK_THROW(p.execute([](const auto&, const auto&, const auto&) -> std::optional<test_receiver> { return std::nullopt; }),
                      std::logic_error);
}

BOOST_AUTO_TEST_CASE(fewer_than_two_waypoints_skips_algorithms) {
    bool decider_called = false;

    auto result = planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{1.0, 2.0, 3.0}}); })
                      .with_totg([](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) {})
                      .execute([&](const auto& p, const auto& totg, const auto& legacy) -> std::optional<test_receiver> {
                          decider_called = true;
                          BOOST_CHECK_EQUAL(p.processed_waypoint_count(), 1U);
                          BOOST_CHECK(!totg.receiver);
                          BOOST_CHECK(!totg.error);
                          BOOST_CHECK(!legacy.receiver);
                          BOOST_CHECK(!legacy.error);
                          return std::nullopt;
                      });

    BOOST_CHECK(decider_called);
    BOOST_CHECK(!result);
}

BOOST_AUTO_TEST_CASE(totg_only_success) {
    bool success_called = false;

    auto result = planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                      .with_totg([&](const auto&, test_receiver& acc, const waypoint_accumulator&, const trajectory& traj, auto elapsed) {
                          success_called = true;
                          acc.segment_count++;
                          acc.total_duration += traj.duration().count();
                          acc.total_gen_time += elapsed;
                      })
                      .execute([](const auto&, const auto& totg, const auto& legacy) -> std::optional<test_receiver> {
                          BOOST_CHECK(totg.receiver.has_value());
                          BOOST_CHECK(!legacy.receiver);
                          BOOST_CHECK(!legacy.error);
                          return std::move(totg.receiver);
                      });

    BOOST_CHECK(success_called);
    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(result->segment_count, 1);
    BOOST_CHECK_GT(result->total_duration, 0.0);
}

BOOST_AUTO_TEST_CASE(legacy_only_success) {
    bool success_called = false;

    auto result =
        planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
            .with_legacy(
                [&](const auto&, test_receiver& acc, const waypoint_accumulator&, const Path&, const Trajectory& traj, auto elapsed) {
                    success_called = true;
                    acc.segment_count++;
                    acc.total_duration += traj.getDuration();
                    acc.total_gen_time += elapsed;
                })
            .execute([](const auto&, const auto& totg, const auto& legacy) -> std::optional<test_receiver> {
                BOOST_CHECK(!totg.receiver);
                BOOST_CHECK(legacy.receiver.has_value());
                return std::move(legacy.receiver);
            });

    BOOST_CHECK(success_called);
    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(result->segment_count, 1);
    BOOST_CHECK_GT(result->total_duration, 0.0);
}

BOOST_AUTO_TEST_CASE(both_algorithms_success) {
    bool totg_called = false;
    bool legacy_called = false;

    auto result =
        planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
            .with_totg([&](const auto&, test_receiver& acc, const waypoint_accumulator&, const trajectory& traj, auto) {
                totg_called = true;
                acc.segment_count++;
                acc.total_duration += traj.duration().count();
            })
            .with_legacy([&](const auto&, test_receiver& acc, const waypoint_accumulator&, const Path&, const Trajectory& traj, auto) {
                legacy_called = true;
                acc.segment_count++;
                acc.total_duration += traj.getDuration();
            })
            .execute([](const auto&, const auto& totg, const auto& legacy) -> std::optional<test_receiver> {
                BOOST_CHECK(totg.receiver.has_value());
                BOOST_CHECK(legacy.receiver.has_value());
                return std::move(totg.receiver);
            });

    BOOST_CHECK(totg_called);
    BOOST_CHECK(legacy_called);
    BOOST_REQUIRE(result);
}

BOOST_AUTO_TEST_CASE(preprocessor_runs_before_algorithms) {
    auto result = planner<test_receiver>(simple_config())
                      .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                      .with_waypoint_preprocessor(
                          [](auto&, waypoint_accumulator& accumulator) { accumulator = deduplicate_waypoints(accumulator, 1000.0); })
                      .with_totg([](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) {
                          BOOST_FAIL("totg should not run on < 2 waypoints");
                      })
                      .execute([](const auto& p, const auto& totg, const auto&) -> std::optional<test_receiver> {
                          BOOST_CHECK_EQUAL(p.processed_waypoint_count(), 1U);
                          BOOST_CHECK(!totg.receiver);
                          return std::nullopt;
                      });

    BOOST_CHECK(!result);
}

BOOST_AUTO_TEST_CASE(validator_can_reject_move) {
    BOOST_CHECK_THROW(planner<test_receiver>(simple_config())
                          .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                          .with_move_validator([](auto&, const waypoint_accumulator&) { throw std::runtime_error("move rejected"); })
                          .with_totg([](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) {})
                          .execute([](const auto&, const auto&, const auto&) -> std::optional<test_receiver> {
                              BOOST_FAIL("decider should not be called after validation failure");
                              return std::nullopt;
                          }),
                      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(segmenter_produces_multiple_segments) {
    int totg_segment_count = 0;

    auto result =
        planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}); })
            .with_segmenter([](auto&, waypoint_accumulator accumulator) { return segment_at_reversals(std::move(accumulator)); })
            .with_totg([&](const auto&, test_receiver& acc, const waypoint_accumulator&, const trajectory&, auto) {
                totg_segment_count++;
                acc.segment_count++;
            })
            .execute([](const auto&, const auto& totg, const auto&) -> std::optional<test_receiver> { return std::move(totg.receiver); });

    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(totg_segment_count, 2);
    BOOST_CHECK_EQUAL(result->segment_count, 2);
}

BOOST_AUTO_TEST_CASE(failure_disengages_receiver_for_remaining_segments) {
    int success_count = 0;
    bool failure_called = false;

    auto result =
        planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}); })
            .with_segmenter([](auto&, waypoint_accumulator accumulator) { return segment_at_reversals(std::move(accumulator)); })
            .with_totg(
                [&](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) {
                    success_count++;
                    throw std::runtime_error("synthetic failure");
                },
                [&](const auto&, const test_receiver&, const waypoint_accumulator&, const std::exception& e) {
                    failure_called = true;
                    BOOST_CHECK_EQUAL(std::string(e.what()), "synthetic failure");
                })
            .execute([](const auto&, const auto& totg, const auto&) -> std::optional<test_receiver> {
                BOOST_CHECK(!totg.receiver);
                BOOST_CHECK(totg.error != nullptr);
                return std::nullopt;
            });

    BOOST_CHECK_EQUAL(success_count, 1);
    BOOST_CHECK(failure_called);
    BOOST_CHECK(!result);
}

BOOST_AUTO_TEST_CASE(stash_extends_data_lifetime) {
    auto result =
        planner<test_receiver>(simple_config())
            .with_waypoint_provider([](auto& p) {
                auto data = p.stash(xt::xarray<double>{{0.0, 0.0, 0.0}});
                waypoint_accumulator accumulator{*data};
                auto more = p.stash(xt::xarray<double>{{1.0, 0.0, 0.0}});
                accumulator.add_waypoints(*more);
                return accumulator;
            })
            .with_totg([](const auto&, test_receiver& acc, const waypoint_accumulator&, const trajectory&, auto) { acc.segment_count++; })
            .execute([](const auto&, const auto& totg, const auto&) -> std::optional<test_receiver> { return std::move(totg.receiver); });

    BOOST_REQUIRE(result);
    BOOST_CHECK_EQUAL(result->segment_count, 1);
}

BOOST_AUTO_TEST_CASE(segment_totg_false_passes_unsegmented_to_totg) {
    int totg_segments_seen = 0;
    int legacy_segments_seen = 0;

    auto cfg = simple_config();
    cfg.segment_totg = false;

    planner<test_receiver>(std::move(cfg))
        .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}); })
        .with_segmenter([](auto&, waypoint_accumulator accumulator) { return segment_at_reversals(std::move(accumulator)); })
        .with_totg([&](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) { totg_segments_seen++; })
        .with_legacy(
            [&](const auto&, test_receiver&, const waypoint_accumulator&, const Path&, const Trajectory&, auto) { legacy_segments_seen++; })
        .execute([](const auto&, const auto&, const auto&) -> std::optional<test_receiver> { return std::nullopt; });

    BOOST_CHECK_EQUAL(totg_segments_seen, 1);
    BOOST_CHECK_EQUAL(legacy_segments_seen, 2);
}

BOOST_AUTO_TEST_CASE(processed_waypoint_count_reflects_preprocessing) {
    planner<test_receiver>(simple_config())
        .with_waypoint_provider(
            [](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}); })
        .with_waypoint_preprocessor(
            [](auto&, waypoint_accumulator& accumulator) { accumulator = deduplicate_waypoints(accumulator, 1e-6); })
        .with_totg([](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) {})
        .execute([](const auto& p, const auto&, const auto&) -> std::optional<test_receiver> {
            BOOST_CHECK_EQUAL(p.processed_waypoint_count(), 3U);
            return std::nullopt;
        });
}

BOOST_AUTO_TEST_CASE(decider_return_type_is_flexible) {
    const int result = planner<test_receiver>(simple_config())
                           .with_waypoint_provider([](auto& p) { return stash_waypoints(p, {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}); })
                           .with_totg([](const auto&, test_receiver&, const waypoint_accumulator&, const trajectory&, auto) {})
                           .execute([](const auto&, const auto& totg, const auto&) -> int { return totg.receiver ? 42 : -1; });

    BOOST_CHECK_EQUAL(result, 42);
}

BOOST_AUTO_TEST_SUITE_END()
