// Uniform sampler tests
// Extracted from test.cpp lines 2062-2141

#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>
#include <viam/trajex/types/arc_length.hpp>
#include <viam/trajex/types/arc_velocity.hpp>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(uniform_sampler_tests)

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_basic) {
    using namespace viam::trajex::totg;

    // 1.0 second @ 100 Hz should give 101 samples
    // dt = 1.0 / (101 - 1) = 1.0 / 100 = 0.01
    const double dt = uniform_sampler::calculate_quantized_dt(1.0, 100.0);
    BOOST_CHECK_CLOSE(dt, 0.01, 0.001);  // Within 0.001% tolerance
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_ensures_endpoint_hit) {
    using namespace viam::trajex::totg;

    // 1.01 seconds @ 100 Hz
    // putative_samples = 1.01 * 100 = 101.0
    // num_samples = ceil(101.0) + 1 = 102
    // dt = 1.01 / (102 - 1) = 1.01 / 101 ≈ 0.01
    const double dt = uniform_sampler::calculate_quantized_dt(1.01, 100.0);

    // Verify we can reach exactly 1.01 with 101 steps
    const double endpoint = 101 * dt;
    BOOST_CHECK_CLOSE(endpoint, 1.01, 0.001);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_oversamples) {
    using namespace viam::trajex::totg;

    // 0.99 seconds @ 100 Hz
    // putative_samples = 0.99 * 100 = 99.0
    // num_samples = ceil(99.0) + 1 = 100
    // dt = 0.99 / (100 - 1) = 0.99 / 99 = 0.01
    const double dt = uniform_sampler::calculate_quantized_dt(0.99, 100.0);

    // Should slightly oversample (100 samples instead of 99)
    const int num_samples = static_cast<int>(std::ceil(0.99 / dt)) + 1;
    BOOST_CHECK_EQUAL(num_samples, 100);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_invalid_duration) {
    using namespace viam::trajex::totg;

    // Zero duration
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(0.0, 100.0), std::invalid_argument);

    // Negative duration
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(-1.0, 100.0), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_invalid_frequency) {
    using namespace viam::trajex::totg;

    // Zero frequency
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(1.0, 0.0), std::invalid_argument);

    // Negative frequency
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(1.0, -100.0), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_exceeds_max_samples) {
    using namespace viam::trajex::totg;

    // Duration * frequency > 1000000
    BOOST_CHECK_THROW(uniform_sampler::calculate_quantized_dt(10000.0, 1000.0), std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(calculate_quantized_dt_at_least_two_samples) {
    using namespace viam::trajex::totg;

    // Very small duration and frequency
    const double dt = uniform_sampler::calculate_quantized_dt(0.001, 1.0);

    // Should still get at least 2 samples (start and end)
    // putative = 0.001 * 1 = 0.001
    // num_samples = ceil(0.001) + 1 = 2
    // dt = 0.001 / (2 - 1) = 0.001
    BOOST_CHECK_CLOSE(dt, 0.001, 0.001);
}

BOOST_AUTO_TEST_CASE(no_duplicate_timestamps_at_end) {
    using namespace viam::trajex::totg;
    using namespace viam::trajex::types;

    using viam::trajex::arc_acceleration;
    using viam::trajex::arc_length;
    using viam::trajex::arc_velocity;

    // Create a simple path
    const xt::xarray<double> waypoints = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
    path p = path::create(waypoints);

    // Create trajectory with explicit integration points to have precise control over duration.
    // We choose duration = 1.0 seconds sampled at 10Hz to trigger floating-point accumulation error.
    // With 10Hz over 1.0s, we get 11 samples with dt = 1.0/10 = 0.1
    // The value 0.1 cannot be represented exactly in binary floating point (infinite binary expansion).
    // After accumulating 10 steps of 0.1 (binary approximation), we fall SLIGHTLY SHORT of 1.0,
    // causing the sampler to snap to exactly 1.0, then attempt to sample again at 1.0 with a
    // tiny delta (duplicate timestamp bug).
    const trajectory::seconds target_duration{1.0};

    std::vector<trajectory::integration_point> points = {
        {.time = trajectory::seconds{0.0}, .s = arc_length{0.0}, .s_dot = arc_velocity{0.0}, .s_ddot = arc_acceleration{0.5}},
        {.time = target_duration, .s = p.length(), .s_dot = arc_velocity{0.0}, .s_ddot = arc_acceleration{0.0}}};

    const trajectory::options opts{.max_velocity = xt::ones<double>({3}), .max_acceleration = xt::ones<double>({3})};

    const trajectory traj = trajectory::create(std::move(p), opts, std::move(points));

    // Verify we got the expected duration
    BOOST_CHECK_CLOSE(traj.duration().count(), target_duration.count(), 0.001);

    // Create quantized sampler at exactly 10Hz (produces 11 samples, dt = 0.1)
    // This is the classic case where 0.1 * 10 != 1.0 in floating point
    const auto sample_freq = hertz{10.0};
    auto sampler = uniform_sampler::quantized_for_trajectory(traj, sample_freq);
    auto cursor = traj.create_cursor();

    // Collect all samples and their time deltas
    std::vector<trajectory::seconds> timestamps;
    std::vector<double> deltas;

    std::optional<trajectory::seconds> prev_time;
    while (auto sample = sampler.next(cursor)) {
        timestamps.push_back(sample->time);
        if (prev_time) {
            deltas.push_back((sample->time - *prev_time).count());
        }
        prev_time = sample->time;
    }

    // 1. Basic sanity: got at least 2 samples
    BOOST_REQUIRE_GE(timestamps.size(), 2);

    // 2. Check we got EXACTLY the expected number of samples (no duplicates)
    const size_t expected_samples = static_cast<size_t>(std::ceil(traj.duration().count() * sample_freq.value)) + 1;
    BOOST_CHECK_MESSAGE(
        timestamps.size() == expected_samples,
        "Got " << timestamps.size() << " samples but expected " << expected_samples << " (extra samples indicate duplicate timestamps)");

    // 3. Verify first sample is EXACTLY at t=0
    BOOST_CHECK_EQUAL(timestamps.front().count(), 0.0);

    // 4. Verify last sample is EXACTLY at trajectory duration
    BOOST_CHECK_EQUAL(timestamps.back().count(), traj.duration().count());

    // 5. Verify all timestamps are strictly monotonically increasing (no duplicates)
    for (size_t i = 1; i < timestamps.size(); ++i) {
        BOOST_CHECK_MESSAGE(timestamps[i] > timestamps[i - 1],
                            "Duplicate or non-monotonic timestamp at index " << i << ": t[" << i - 1 << "]=" << timestamps[i - 1].count()
                                                                             << "s, t[" << i << "]=" << timestamps[i].count() << "s");
    }

    // 6. Verify all deltas are close to the expected uniform delta
    // This will FAIL if we get duplicate timestamps (delta ~= 0) or uneven spacing
    const double expected_delta = uniform_sampler::calculate_quantized_dt(traj.duration().count(), sample_freq.value);
    for (size_t i = 0; i < deltas.size(); ++i) {
        BOOST_CHECK_CLOSE(deltas[i], expected_delta, 0.01);  // Within 0.01% tolerance
    }
}

BOOST_AUTO_TEST_SUITE_END()
