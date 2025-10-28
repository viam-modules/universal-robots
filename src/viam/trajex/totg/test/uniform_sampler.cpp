// Uniform sampler tests
// Extracted from test.cpp lines 2062-2141

#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/uniform_sampler.hpp>

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

BOOST_AUTO_TEST_SUITE_END()
