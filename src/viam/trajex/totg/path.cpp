#include <viam/trajex/totg/path.hpp>

#include <algorithm>
#include <cmath>
#include <ranges>
#include <stdexcept>

#if __has_include(<xtensor/reducers/xnorm.hpp>)
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xnorm.hpp>
#endif

#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>

namespace viam::trajex::totg {

path::segment::linear::linear(xt::xarray<double> start, xt::xarray<double> end) : start{std::move(start)}, length{0.0} {
    const auto diff = end - this->start;
    const double norm = xt::norm_l2(diff)();

    // TODO(acm): Use scale-relative tolerance instead of hardcoded absolute value
    if (norm < 1e-10) {
        throw std::invalid_argument{"Linear segment: start and end must be different"};
    }

    this->unit_direction = diff / norm;
    this->length = arc_length{norm};
}

path::segment::circular::circular(xt::xarray<double> center, xt::xarray<double> x, xt::xarray<double> y, double radius, double angle_rads)
    : center{std::move(center)}, x{std::move(x)}, y{std::move(y)}, radius{radius}, angle_rads{angle_rads} {
    // Validate x and y are unit vectors
    const double x_norm = xt::norm_l2(this->x)();
    const double y_norm = xt::norm_l2(this->y)();

    // TODO(acm): Use machine epsilon based tolerance for unit vector check
    if (std::abs(x_norm - 1.0) > 1e-6 || std::abs(y_norm - 1.0) > 1e-6) {
        throw std::invalid_argument{"Circular segment: x and y must be unit vectors"};
    }

    // Validate x and y are perpendicular
    const double dot_product = xt::sum(this->x * this->y)();
    // TODO(acm): Consider tighter tolerance for orthogonality
    if (std::abs(dot_product) > 1e-6) {
        throw std::invalid_argument{"Circular segment: x and y must be perpendicular"};
    }
}

path::segment::segment(linear data) : data_{std::move(data)} {}

path::segment::segment(circular data) : data_{std::move(data)} {}

path::segment::view::view(const class segment& seg, arc_length start, arc_length end) noexcept : seg_{seg}, start_{start}, end_{end} {}

const path::segment& path::segment::view::segment() const noexcept {
    return seg_.get();
}

arc_length path::segment::view::start() const noexcept {
    return start_;
}

arc_length path::segment::view::end() const noexcept {
    return end_;
}

arc_length path::segment::view::length() const noexcept {
    return end_ - start_;
}

xt::xarray<double> path::segment::view::configuration(arc_length s) const {
    const arc_length local_s = s - start_;

    // Validate bounds
    if (local_s < arc_length{0.0} || local_s > length()) [[unlikely]] {
        throw std::out_of_range{"Arc length outside segment bounds"};
    }

    return std::visit(
        [local_s](const auto& seg_data) -> xt::xarray<double> {
            using T = std::decay_t<decltype(seg_data)>;

            if constexpr (std::is_same_v<T, segment::linear>) {
                // Linear interpolation: config = start + local_s * unit_direction
                return seg_data.start + (static_cast<double>(local_s) * seg_data.unit_direction);
            } else if constexpr (std::is_same_v<T, segment::circular>) {
                // Circular arc configuration (Equation 7 from Kunz & Stilman):
                // config = center + radius * (x * cos(angle) + y * sin(angle))
                // angle = arc_length / radius (radians)
                const double angle = static_cast<double>(local_s) / seg_data.radius;
                return seg_data.center + (seg_data.radius * (seg_data.x * std::cos(angle) + seg_data.y * std::sin(angle)));
            }
        },
        seg_.get().data_);
}

xt::xarray<double> path::segment::view::tangent(arc_length s) const {
    const arc_length local_s = s - start_;

    // Validate bounds
    if (local_s < arc_length{0.0} || local_s > length()) [[unlikely]] {
        throw std::out_of_range{"Arc length outside segment bounds"};
    }

    return std::visit(
        [local_s](const auto& seg_data) -> xt::xarray<double> {
            using T = std::decay_t<decltype(seg_data)>;

            if constexpr (std::is_same_v<T, segment::linear>) {
                // Linear segment: constant unit tangent
                return seg_data.unit_direction;
            } else if constexpr (std::is_same_v<T, segment::circular>) {
                // Circular arc unit tangent (Equation 8 from Kunz & Stilman):
                // tangent = -x * sin(angle) + y * cos(angle)
                const double angle = static_cast<double>(local_s) / seg_data.radius;
                return (-seg_data.x * std::sin(angle)) + (seg_data.y * std::cos(angle));
            }
        },
        seg_.get().data_);
}

xt::xarray<double> path::segment::view::curvature(arc_length s) const {
    const arc_length local_s = s - start_;

    // Validate bounds
    if (local_s < arc_length{0.0} || local_s > length()) [[unlikely]] {
        throw std::out_of_range{"Arc length outside segment bounds"};
    }

    return std::visit(
        [local_s](const auto& seg_data) -> xt::xarray<double> {
            using T = std::decay_t<decltype(seg_data)>;

            if constexpr (std::is_same_v<T, segment::linear>) {
                // Linear segment: zero curvature vector
                return xt::zeros<double>({seg_data.start.shape(0)});
            } else if constexpr (std::is_same_v<T, segment::circular>) {
                // Circular arc curvature vector (Equation 9 from Kunz & Stilman):
                // curvature = -(1/radius) * (x * cos(angle) + y * sin(angle))
                const double angle = static_cast<double>(local_s) / seg_data.radius;
                return (-(1.0 / seg_data.radius)) * (seg_data.x * std::cos(angle) + seg_data.y * std::sin(angle));
            }
        },
        seg_.get().data_);
}

path::path(std::vector<positioned_segment> segments, size_t dof, arc_length length)
    : segments_{std::move(segments)}, dof_{dof}, length_{length} {}

path path::create(const waypoint_accumulator& waypoints, double max_deviation) {
    if (waypoints.size() < 2) {
        throw std::invalid_argument{"Path requires at least 2 waypoints"};
    }
    if (max_deviation < 0.0) {
        throw std::invalid_argument{"Max deviation must be non-negative"};
    }

    if (max_deviation > 0.0) {
        // TODO(acm): Implement tube-based coalescing algorithm with circular blends
        // - Iterate through waypoints
        // - Maintain cylinder around current linear segment
        // - Extend segment while waypoints stay within tube
        // - Create circular blend when waypoint exits tube
        // - Track cumulative arc length
        throw std::runtime_error{"Circular blends not yet implemented"};
    }

    // Linear-only path: create linear segment between each consecutive waypoint pair
    std::vector<positioned_segment> segments;
    arc_length cumulative_length{0.0};

    // Iterate pairwise: track iterator to current, range-for over next elements
    // (C++23 would have std::views::adjacent<2> or std::views::zip)
    auto start_it = waypoints.begin();
    for (const auto& end : waypoints | std::views::drop(1)) {
        const auto& start = *start_it;

        // Create linear segment (constructor computes unit_direction and length)
        segment::linear linear_data{start, end};

        // Store segment with its starting position
        segments.push_back({.seg = segment{std::move(linear_data)}, .start = cumulative_length});

        cumulative_length = cumulative_length + linear_data.length;
        ++start_it;
    }

    return path{std::move(segments), waypoints.dof(), cumulative_length};
}

path path::create(const xt::xarray<double>& waypoints, double max_deviation) {
    return create(waypoint_accumulator{waypoints}, max_deviation);
}

arc_length path::length() const noexcept {
    return length_;
}

size_t path::size() const noexcept {
    return segments_.size();
}

bool path::empty() const noexcept {
    return segments_.empty();
}

path::const_iterator path::begin() const noexcept {
    return const_iterator{this, segments_.begin()};
}

path::const_iterator path::end() const noexcept {
    return const_iterator{this, segments_.end()};
}

path::const_iterator path::cbegin() const noexcept {
    return begin();
}

path::const_iterator path::cend() const noexcept {
    return end();
}

path::const_iterator::const_iterator() noexcept : path_{nullptr} {}

path::segment::view path::const_iterator::operator*() const {
    // Compute end: next segment's start, or path length
    auto next = std::next(it_);
    arc_length end = (next != path_->segments_.end()) ? next->start : path_->length_;

    return {it_->seg, it_->start, end};
}

path::const_iterator& path::const_iterator::operator++() noexcept {
    ++it_;
    return *this;
}

path::const_iterator path::const_iterator::operator++(int) noexcept {
    auto tmp = *this;
    ++(*this);
    return tmp;
}

bool path::const_iterator::operator==(const const_iterator& other) const noexcept {
    return it_ == other.it_;
}

size_t path::dof() const noexcept {
    return dof_;
}

path::segment::view path::operator()(arc_length s) const {
    if (segments_.empty()) [[unlikely]] {
        throw std::runtime_error{"Path has no segments"};
    }

    if (s > length_) [[unlikely]] {
        throw std::out_of_range{"Arc length exceeds path length"};
    }

    // Find first segment that starts AFTER s using upper_bound
    // upper_bound returns iterator to first element where entry.start > s
    auto it = std::upper_bound(
        segments_.begin(), segments_.end(), s, [](arc_length value, const positioned_segment& entry) { return value < entry.start; });

    // The segment containing s is the one before the iterator returned by upper_bound
    if (it == segments_.begin()) [[unlikely]] {
        // s is before the first segment's start, which indicates a malformed path
        // (first segment should start at arc_length 0)
        throw std::runtime_error{"Arc length before path start"};
    }

    --it;

    // Calculate end position: either the start of next segment, or path length
    const arc_length segment_start = it->start;
    const arc_length segment_end = (it + 1 != segments_.end()) ? (it + 1)->start : length_;

    return {it->seg, segment_start, segment_end};
}

xt::xarray<double> path::configuration(arc_length s) const {
    return (*this)(s).configuration(s);
}

xt::xarray<double> path::tangent(arc_length s) const {
    return (*this)(s).tangent(s);
}

xt::xarray<double> path::curvature(arc_length s) const {
    return (*this)(s).curvature(s);
}

}  // namespace viam::trajex::totg
