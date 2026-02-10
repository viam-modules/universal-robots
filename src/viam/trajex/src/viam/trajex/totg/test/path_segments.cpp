// Path segment tests: linear and circular segment types
// Extracted from test.cpp lines 1755-2060

#include <viam/trajex/totg/path.hpp>
#include <viam/trajex/types/angles.hpp>
#include <viam/trajex/types/arc_length.hpp>

#if __has_include(<xtensor/reducers/xnorm.hpp>)
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xnorm.hpp>
#endif

#include <numbers>

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(segment_tests)

using viam::trajex::degrees_to_radians;
BOOST_AUTO_TEST_CASE(linear_constructor) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Should be able to create linear segments
    const xt::xarray<double> start = {1.0, 2.0, 3.0};
    const xt::xarray<double> end = {4.0, 5.0, 6.0};
    BOOST_CHECK_NO_THROW((path::segment::linear{start, end}));
}

BOOST_AUTO_TEST_CASE(circular_constructor) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Should be able to create circular segments
    const xt::xarray<double> center = {0.0, 0.0, 0.0};
    const xt::xarray<double> x = {1.0, 0.0, 0.0};
    const xt::xarray<double> y = {0.0, 1.0, 0.0};
    BOOST_CHECK_NO_THROW((path::segment::circular{center, x, y, 1.0, 1.57}));
}

BOOST_AUTO_TEST_CASE(segment_view_type_check) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create linear segment
    const path::segment::linear linear_data{xt::xarray<double>{1.0, 2.0, 3.0}, xt::xarray<double>{4.0, 5.0, 6.0}};
    const path::segment linear_seg{linear_data};
    const path::segment::view linear_view{linear_seg, arc_length{0.0}, linear_data.length};

    BOOST_CHECK(linear_view.is<path::segment::linear>());
    BOOST_CHECK(!linear_view.is<path::segment::circular>());

    // Create circular segment
    const path::segment::circular circular_data{
        xt::xarray<double>{0.0, 0.0, 0.0}, xt::xarray<double>{1.0, 0.0, 0.0}, xt::xarray<double>{0.0, 1.0, 0.0}, 1.0, 1.57};
    const path::segment circular_seg{circular_data};
    const path::segment::view circular_view{circular_seg, arc_length{0.0}, arc_length{1.57}};

    BOOST_CHECK(circular_view.is<path::segment::circular>());
    BOOST_CHECK(!circular_view.is<path::segment::linear>());
}

BOOST_AUTO_TEST_CASE(segment_view_visit) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create linear segment
    const path::segment::linear linear_data{xt::xarray<double>{1.0, 2.0, 3.0}, xt::xarray<double>{4.0, 5.0, 6.0}};
    const path::segment linear_seg{linear_data};
    const path::segment::view linear_view{linear_seg, arc_length{0.0}, linear_data.length};

    // Visit linear segment - direction (3,3,3) normalizes to (1/√3, 1/√3, 1/√3)
    bool visited_linear = false;
    linear_view.visit([&visited_linear](const auto& seg_data) {
        using T = std::decay_t<decltype(seg_data)>;
        if constexpr (std::is_same_v<T, path::segment::linear>) {
            visited_linear = true;
            BOOST_CHECK_CLOSE(seg_data.unit_direction(0), 1.0 / std::sqrt(3.0), 1e-6);
            BOOST_CHECK_CLOSE(seg_data.unit_direction(1), 1.0 / std::sqrt(3.0), 1e-6);
            BOOST_CHECK_CLOSE(seg_data.unit_direction(2), 1.0 / std::sqrt(3.0), 1e-6);
        }
    });
    BOOST_CHECK(visited_linear);

    // Create circular segment
    const path::segment::circular circular_data{
        xt::xarray<double>{0.0, 0.0, 0.0}, xt::xarray<double>{1.0, 0.0, 0.0}, xt::xarray<double>{0.0, 1.0, 0.0}, 2.5, 1.57};
    const path::segment circular_seg{circular_data};
    const path::segment::view circular_view{circular_seg, arc_length{0.0}, arc_length{1.57}};

    // Visit circular segment
    bool visited_circular = false;
    circular_view.visit([&visited_circular](const auto& seg_data) {
        using T = std::decay_t<decltype(seg_data)>;
        if constexpr (std::is_same_v<T, path::segment::circular>) {
            visited_circular = true;
            BOOST_CHECK_CLOSE(seg_data.radius, 2.5, 1e-6);
        }
    });
    BOOST_CHECK(visited_circular);
}

BOOST_AUTO_TEST_CASE(linear_segment_configuration) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Linear segment from (0,0) to (3,4) - length = 5, direction = (0.6, 0.8)
    const path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, data.length};

    // At start (s=0): should be (0,0)
    auto config_start = view.configuration(arc_length{0.0});
    BOOST_CHECK_CLOSE(config_start(0), 0.0, 1e-6);
    BOOST_CHECK_CLOSE(config_start(1), 0.0, 1e-6);

    // At middle (s=2.5): should be (1.5, 2.0)
    auto config_mid = view.configuration(arc_length{2.5});
    BOOST_CHECK_CLOSE(config_mid(0), 1.5, 1e-6);
    BOOST_CHECK_CLOSE(config_mid(1), 2.0, 1e-6);

    // At end (s=5): should be (3,4)
    auto config_end = view.configuration(arc_length{5.0});
    BOOST_CHECK_CLOSE(config_end(0), 3.0, 1e-6);
    BOOST_CHECK_CLOSE(config_end(1), 4.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(linear_segment_tangent) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Linear segment from (0,0) to (3,4) - length = 5
    // Unit tangent should be (0.6, 0.8)
    const path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, data.length};

    // Tangent should be constant along the segment
    auto tangent_start = view.tangent(arc_length{0.0});
    BOOST_CHECK_CLOSE(tangent_start(0), 0.6, 1e-6);
    BOOST_CHECK_CLOSE(tangent_start(1), 0.8, 1e-6);

    auto tangent_mid = view.tangent(arc_length{2.5});
    BOOST_CHECK_CLOSE(tangent_mid(0), 0.6, 1e-6);
    BOOST_CHECK_CLOSE(tangent_mid(1), 0.8, 1e-6);

    auto tangent_end = view.tangent(arc_length{5.0});
    BOOST_CHECK_CLOSE(tangent_end(0), 0.6, 1e-6);
    BOOST_CHECK_CLOSE(tangent_end(1), 0.8, 1e-6);
}

BOOST_AUTO_TEST_CASE(linear_segment_curvature) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Linear segments have zero curvature
    const path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, data.length};

    auto curvature = view.curvature(arc_length{2.5});
    BOOST_CHECK_SMALL(curvature(0), 1e-10);
    BOOST_CHECK_SMALL(curvature(1), 1e-10);
}

BOOST_AUTO_TEST_CASE(circular_segment_configuration) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Quarter circle in xy plane, radius 1, centered at origin
    // x = (1, 0), y = (0, 1), arc from (1,0) to (0,1)
    const path::segment::circular data{
        xt::xarray<double>{0.0, 0.0},  // center
        xt::xarray<double>{1.0, 0.0},  // x
        xt::xarray<double>{0.0, 1.0},  // y
        1.0,                           // radius
        degrees_to_radians(90.0)       // angle_rads (90 degrees)
    };
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, arc_length{degrees_to_radians(90.0)}};

    // At start (angle=0): should be (1, 0)
    auto config_start = view.configuration(arc_length{0.0});
    BOOST_CHECK_CLOSE(config_start(0), 1.0, 1e-6);
    BOOST_CHECK_CLOSE(config_start(1), 0.0, 1e-6);

    // At middle (angle=π/4): should be (√2/2, √2/2)
    auto config_mid = view.configuration(arc_length{degrees_to_radians(45.0)});
    BOOST_CHECK_CLOSE(config_mid(0), std::sqrt(2.0) / 2.0, 1e-6);
    BOOST_CHECK_CLOSE(config_mid(1), std::sqrt(2.0) / 2.0, 1e-6);

    // At end (angle=π/2): should be (0, 1)
    auto config_end = view.configuration(arc_length{degrees_to_radians(90.0)});
    BOOST_CHECK_SMALL(config_end(0), 1e-10);  // Near zero, use SMALL not CLOSE
    BOOST_CHECK_CLOSE(config_end(1), 1.0, 1e-6);
}

BOOST_AUTO_TEST_CASE(circular_segment_tangent) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Quarter circle, tangent should be perpendicular to radius
    const path::segment::circular data{
        xt::xarray<double>{0.0, 0.0},  // center
        xt::xarray<double>{1.0, 0.0},  // x
        xt::xarray<double>{0.0, 1.0},  // y
        1.0,                           // radius
        degrees_to_radians(90.0)       // angle_rads
    };
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, arc_length{degrees_to_radians(90.0)}};

    // At start (angle=0): tangent should be (0, 1)
    auto tangent_start = view.tangent(arc_length{0.0});
    BOOST_CHECK_SMALL(tangent_start(0), 1e-10);  // Near zero
    BOOST_CHECK_CLOSE(tangent_start(1), 1.0, 1e-6);

    // At end (angle=π/2): tangent should be (-1, 0)
    auto tangent_end = view.tangent(arc_length{degrees_to_radians(90.0)});
    BOOST_CHECK_CLOSE(tangent_end(0), -1.0, 1e-6);
    BOOST_CHECK_SMALL(tangent_end(1), 1e-10);  // Near zero
}

BOOST_AUTO_TEST_CASE(circular_segment_curvature) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Curvature vector points toward center with magnitude 1/radius
    const path::segment::circular data{
        xt::xarray<double>{0.0, 0.0},  // center
        xt::xarray<double>{1.0, 0.0},  // x
        xt::xarray<double>{0.0, 1.0},  // y
        2.0,                           // radius = 2
        degrees_to_radians(90.0)       // angle_rads
    };
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, arc_length{degrees_to_radians(180.0)}};

    // At start (angle=0): curvature should point from (2,0) to (0,0), magnitude 1/2
    auto curvature_start = view.curvature(arc_length{0.0});
    BOOST_CHECK_CLOSE(curvature_start(0), -0.5, 1e-6);  // -1/radius
    BOOST_CHECK_CLOSE(curvature_start(1), 0.0, 1e-6);

    // Magnitude should be 1/radius = 0.5
    const double mag = xt::norm_l2(curvature_start)();
    BOOST_CHECK_CLOSE(mag, 0.5, 1e-6);
}

BOOST_AUTO_TEST_CASE(segment_view_bounds_checking) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    const path::segment::linear data{xt::xarray<double>{0.0, 0.0}, xt::xarray<double>{3.0, 4.0}};
    const path::segment seg{data};
    const path::segment::view view{seg, arc_length{0.0}, data.length};

    // Negative arc_length should throw
    BOOST_CHECK_THROW(view.configuration(arc_length{-1.0}), std::out_of_range);
    BOOST_CHECK_THROW(view.tangent(arc_length{-0.5}), std::out_of_range);
    BOOST_CHECK_THROW(view.curvature(arc_length{-0.1}), std::out_of_range);

    // Arc_length beyond segment end should throw
    BOOST_CHECK_THROW(view.configuration(arc_length{5.1}), std::out_of_range);
    BOOST_CHECK_THROW(view.tangent(arc_length{6.0}), std::out_of_range);
    BOOST_CHECK_THROW(view.curvature(arc_length{10.0}), std::out_of_range);
}

BOOST_AUTO_TEST_CASE(path_container_semantics) {
    using namespace viam::trajex::totg;
    using viam::trajex::arc_length;

    // Create a path with 3 waypoints -> 2 linear segments
    const xt::xarray<double> waypoints = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
    auto p = path::create(waypoints);

    // Test size and empty
    BOOST_CHECK_EQUAL(p.size(), 2U);
    BOOST_CHECK(!p.empty());

    // Test iteration - iterator yields segment::view
    size_t count = 0;
    for (const auto& view : p) {
        // Each iteration yields a segment::view
        BOOST_CHECK(view.is<path::segment::linear>());
        BOOST_CHECK(view.start() >= arc_length{0.0});
        BOOST_CHECK(view.length() > arc_length{0.0});
        ++count;
    }
    BOOST_CHECK_EQUAL(count, 2U);

    // Test iterator accessors
    auto it = p.begin();
    BOOST_CHECK(it != p.end());

    auto view1 = *it;
    BOOST_CHECK(view1.is<path::segment::linear>());
    BOOST_CHECK_EQUAL(static_cast<double>(view1.start()), 0.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view1.end()), 1.0);

    // Can query the view immediately
    auto config = view1.configuration(arc_length{0.5});
    BOOST_CHECK_EQUAL(config.shape(0), 2U);

    ++it;
    BOOST_CHECK(it != p.end());

    auto view2 = *it;
    BOOST_CHECK_EQUAL(static_cast<double>(view2.start()), 1.0);
    BOOST_CHECK_EQUAL(static_cast<double>(view2.end()), 2.0);  // second segment length is 1.0

    ++it;
    BOOST_CHECK(it == p.end());

    // Test const_iterator
    auto cit = p.cbegin();
    BOOST_CHECK(cit != p.cend());
    ++cit;
    BOOST_CHECK(cit != p.cend());
    ++cit;
    BOOST_CHECK(cit == p.cend());
}

BOOST_AUTO_TEST_SUITE_END()
