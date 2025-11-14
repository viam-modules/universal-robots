#define BOOST_TEST_MODULE trajex_types_test

#include <viam/trajex/types/arc_length.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#include <boost/test/included/unit_test.hpp>
#pragma GCC diagnostic pop

BOOST_AUTO_TEST_SUITE(arc_length_tests)

BOOST_AUTO_TEST_CASE(arc_length_arithmetic) {
    using namespace viam::trajex;

    const arc_length a{5.0};
    const arc_length b{3.0};

    // Addition
    const arc_length sum = a + b;
    BOOST_CHECK_EQUAL(sum.value, 8.0);

    // Subtraction
    const arc_length diff = a - b;
    BOOST_CHECK_EQUAL(diff.value, 2.0);

    // Scalar multiplication
    const arc_length scaled1 = a * 2.0;
    BOOST_CHECK_EQUAL(scaled1.value, 10.0);

    const arc_length scaled2 = 2.0 * a;
    BOOST_CHECK_EQUAL(scaled2.value, 10.0);

    // Scalar division
    const arc_length divided = a / 2.0;
    BOOST_CHECK_EQUAL(divided.value, 2.5);

    // Compound operations
    arc_length c{10.0};
    c += b;
    BOOST_CHECK_EQUAL(c.value, 13.0);

    c -= arc_length{3.0};
    BOOST_CHECK_EQUAL(c.value, 10.0);

    c *= 2.0;
    BOOST_CHECK_EQUAL(c.value, 20.0);

    c /= 4.0;
    BOOST_CHECK_EQUAL(c.value, 5.0);
}

BOOST_AUTO_TEST_CASE(arc_length_comparison) {
    using namespace viam::trajex;

    const arc_length a{5.0};
    const arc_length b{3.0};
    const arc_length c{5.0};

    BOOST_CHECK(a > b);
    BOOST_CHECK(b < a);
    BOOST_CHECK(a >= c);
    BOOST_CHECK(a <= c);
    BOOST_CHECK(a == c);
    BOOST_CHECK(a != b);
}

BOOST_AUTO_TEST_CASE(arc_length_explicit_conversion) {
    using namespace viam::trajex;

    const arc_length len{5.5};

    // Explicit conversion should work
    const double value = static_cast<double>(len);
    BOOST_CHECK_EQUAL(value, 5.5);

    // Direct initialization should work
    const double value2{len};
    BOOST_CHECK_EQUAL(value2, 5.5);
}

BOOST_AUTO_TEST_SUITE_END()
