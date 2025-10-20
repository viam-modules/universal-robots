#include <viam/trajex/totg/path.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>

#if __has_include(<xtensor/reducers/xnorm.hpp>)
#include <xtensor/reducers/xnorm.hpp>
#else
#include <xtensor/xnorm.hpp>
#endif

#include <viam/trajex/totg/trajectory.hpp>
#include <viam/trajex/totg/waypoint_accumulator.hpp>

namespace viam::trajex::totg {

// path::options implementation
path::options::options() : max_blend_deviation_(k_default_max_deviation), max_linear_deviation_(k_default_max_deviation) {}

path::options& path::options::set_max_deviation(double deviation) {
    return set_max_blend_deviation(deviation).set_max_linear_deviation(deviation);
}

path::options& path::options::set_max_blend_deviation(double deviation) {
    max_blend_deviation_ = deviation;
    return *this;
}

path::options& path::options::set_max_linear_deviation(double deviation) {
    max_linear_deviation_ = deviation;
    return *this;
}

double path::options::max_blend_deviation() const noexcept {
    return max_blend_deviation_;
}

double path::options::max_linear_deviation() const noexcept {
    return max_linear_deviation_;
}

// path::segment::linear implementation
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

path path::create(const waypoint_accumulator& waypoints, const options& opts) {
    if (waypoints.size() < 2) {
        throw std::invalid_argument{"Path requires at least 2 waypoints"};
    }
    if (opts.max_blend_deviation() < 0.0) {
        throw std::invalid_argument{"Max blend deviation must be non-negative"};
    }
    if (opts.max_linear_deviation() < 0.0) {
        throw std::invalid_argument{"Max linear deviation must be non-negative"};
    }

    // TODO(acm): Revisit this algorithm, and write a more graceful one.

    // Tube coalescing with direct segment construction
    // Strategy: Scan through waypoints with locus at each potential corner.
    // At each locus, look backward to segment start and forward to see if we can continue coalescing.

    // Lambda to test if a waypoint can be coalesced (skipped)
    // Returns true if 'locus' is within max_linear_deviation of the line from 'start' to 'next'
    // and advances monotonically along the tube direction
    const auto can_coalesce = [&opts](const auto& start, const auto& locus, const auto& next) -> bool {
        const double max_deviation = opts.max_linear_deviation();

        // With zero tolerance, nothing coalesces (every waypoint is a hard constraint)
        if (max_deviation == 0.0) {
            return false;
        }

        const auto start_to_next = next - start;
        const auto start_to_locus = locus - start;

        // Compute projection parameter t: locus = start + t * (next - start)
        const double start_to_next_sq = xt::sum(start_to_next * start_to_next)();

        // Handle exact duplicate: next == start
        if (start_to_next_sq == 0.0) {
            return true;  // Next is duplicate, so locus is trivially coalescable
        }

        const double start_to_locus_dot_direction = xt::sum(start_to_locus * start_to_next)();
        const double t = start_to_locus_dot_direction / start_to_next_sq;

        // Check monotonic advancement: must be between start and next
        if (t < 0.0 || t > 1.0) {
            return false;
        }

        // Compute perpendicular distance from locus to line
        const auto projected_point = start + (t * start_to_next);
        const auto deviation_vector = locus - projected_point;
        const double deviation = xt::norm_l2(deviation_vector)();

        return deviation <= max_deviation;
    };

    // Helper struct to return blend geometry
    struct blend_geometry {
        segment::circular circular_seg;
        double trim_distance;  // Distance trimmed from both segments
    };

    // Lambda to attempt circular blend creation at a corner
    // Returns std::nullopt if blend cannot be created
    const auto try_create_blend = [&opts](const xt::xarray<double>& current_pos,
                                          const xt::xarray<double>& corner,
                                          const xt::xarray<double>& next_waypoint) -> std::optional<blend_geometry> {
        if (opts.max_blend_deviation() <= 0.0) {
            return std::nullopt;
        }

        const auto incoming = corner - current_pos;
        const auto outgoing = next_waypoint - corner;

        const double incoming_norm = xt::norm_l2(incoming)();
        const double outgoing_norm = xt::norm_l2(outgoing)();

        // Need non-zero segments to compute angle
        if (incoming_norm <= 0.0 || outgoing_norm <= 0.0) {
            return std::nullopt;
        }

        const auto incoming_unit = incoming / incoming_norm;
        const auto outgoing_unit = outgoing / outgoing_norm;

        // Compute angle between incoming and outgoing directions
        const double dot = xt::sum(incoming_unit * outgoing_unit)();
        const double dot_clamped = std::clamp(dot, -1.0, 1.0);
        const double angle = std::acos(dot_clamped);
        const double half_angle = angle / 2.0;

        // Check for degenerate cases that would cause mathematical issues
        // tan(pi/2) is undefined (collinear waypoints going backward)
        // Small angles would create very large radii with tiny blends
        // TODO(acm): Explicit epsilon, scaling, etc.
        constexpr double epsilon = 1e-6;
        if (half_angle < epsilon || half_angle > (std::numbers::pi / 2.0 - epsilon)) {
            return std::nullopt;
        }

        // Calculate trim distance and radius following Kunz & Stilman eq. 3-4
        const double max_trim_from_deviation = opts.max_blend_deviation() * std::sin(half_angle) / (1.0 - std::cos(half_angle));

        const double trim_distance = std::min({incoming_norm / 2.0, outgoing_norm / 2.0, max_trim_from_deviation});

        // Radius from trim distance (eq. 4)
        const double radius = trim_distance / std::tan(half_angle);

        // Create circular blend segment geometry following Kunz & Stilman
        // Equation 5: ci = qi + (ŷi+1 - ŷi)/||ŷi+1 - ŷi|| · ri/cos(αi/2)
        const auto bisector = outgoing_unit - incoming_unit;
        const double bisector_norm = xt::norm_l2(bisector)();
        const auto bisector_unit = bisector / bisector_norm;

        const double center_offset = radius / std::cos(half_angle);
        const auto center = corner + (center_offset * bisector_unit);

        // x̂ points from center to blend start (where circle touches incoming segment)
        const auto blend_start = corner - (trim_distance * incoming_unit);
        const auto x_vec = blend_start - center;
        const auto x_unit = x_vec / xt::norm_l2(x_vec)();

        // ŷ is the incoming direction (perpendicular to x by construction)
        const auto& y_unit = incoming_unit;

        return blend_geometry{.circular_seg = segment::circular{center, x_unit, y_unit, radius, angle}, .trim_distance = trim_distance};
    };

    // Build segments directly via locus-based scan
    std::vector<positioned_segment> segments;
    arc_length cumulative_length{0.0};

    // Track where the current segment started (iterator for coalescing logic)
    auto segment_start = waypoints.begin();

    // Track actual current configuration
    // IMPORTANT: This must be a configuration copy, not an iterator, because after creating
    // a circular blend, current_position is at the blend exit point - a computed position
    // that doesn't correspond to any waypoint in the accumulator
    xt::xarray<double> current_position = *segment_start;

    // Start locus at second waypoint (first potential corner)
    for (auto locus = std::next(waypoints.begin()); locus != waypoints.end(); ++locus) {
        auto next = std::next(locus);

        // If we're at the last waypoint, we must emit the final segment
        const bool at_last = (next == waypoints.end());

        // Check if locus can be coalesced (skipped)
        const bool can_skip = !at_last && can_coalesce(*segment_start, *locus, *next);

        if (can_skip) {
            // Locus can be skipped, continue scanning
            continue;
        }

        // Locus is a corner (or last waypoint) - emit segment and handle corner

        // Try to create circular blend at this corner
        const auto blend_opt = !at_last ? try_create_blend(current_position, *locus, *next) : std::nullopt;

        if (blend_opt) {
            // Blend created successfully
            const auto& blend = *blend_opt;
            const auto incoming = *locus - current_position;
            const auto incoming_unit = incoming / xt::norm_l2(incoming)();
            const auto outgoing = *next - *locus;
            const auto outgoing_unit = outgoing / xt::norm_l2(outgoing)();

            // 1. Create trimmed incoming segment
            const auto trimmed_end = *locus - (blend.trim_distance * incoming_unit);
            segment::linear incoming_segment{current_position, trimmed_end};
            segments.push_back({.seg = segment{std::move(incoming_segment)}, .start = cumulative_length});
            cumulative_length = cumulative_length + incoming_segment.length;

            // 2. Create circular blend segment
            segments.push_back({.seg = segment{blend.circular_seg}, .start = cumulative_length});
            cumulative_length = cumulative_length + arc_length{blend.circular_seg.radius * blend.circular_seg.angle_rads};

            // Update current_position to blend exit point
            current_position = *locus + (blend.trim_distance * outgoing_unit);

            // Update segment_start iterator to locus for coalescing logic
            segment_start = locus;
        } else {
            // No blend created - emit linear segment from current_position to locus
            const auto start_to_locus = *locus - current_position;
            const double dist_sq = xt::sum(start_to_locus * start_to_locus)();

            if (dist_sq > 0.0) {
                segment::linear linear_data{current_position, *locus};
                segments.push_back({.seg = segment{std::move(linear_data)}, .start = cumulative_length});
                cumulative_length = cumulative_length + linear_data.length;
            }

            // Update both current_position and segment_start
            current_position = *locus;
            segment_start = locus;
        }
    }

    // Validate that we created at least one segment
    if (segments.empty()) {
        throw std::invalid_argument{"Path requires at least one non-zero length segment"};
    }

    return path{std::move(segments), waypoints.dof(), cumulative_length};
}

path path::create(const xt::xarray<double>& waypoints, const options& opts) {
    return create(waypoint_accumulator{waypoints}, opts);
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

path::const_iterator& path::const_iterator::operator--() noexcept {
    --it_;
    return *this;
}

path::const_iterator path::const_iterator::operator--(int) noexcept {
    auto tmp = *this;
    --(*this);
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

// path::cursor implementation

path::cursor::cursor(const class path* p, arc_length s)
    : path_{p}, position_{std::clamp(s, arc_length{0.0}, p->length())}, hint_{path_->begin()} {
    if (path_->empty()) {
        throw std::invalid_argument{"Cannot create cursor for empty path"};
    }
    update_hint_();
}

const path& path::cursor::path() const noexcept {
    return *path_;
}

arc_length path::cursor::position() const noexcept {
    return position_;
}

path::segment::view path::cursor::operator*() const {
    return *hint_;
}

void path::cursor::seek(arc_length s) noexcept {
    // Overflow to ±infinity sentinels outside valid range
    if (s < arc_length{0.0}) {
        position_ = arc_length{-std::numeric_limits<double>::infinity()};
    } else if (s > path_->length()) {
        position_ = arc_length{std::numeric_limits<double>::infinity()};
    } else {
        position_ = s;
    }

    // Only update hint if not at sentinel
    if (*this != end()) {
        update_hint_();
    }
}

void path::cursor::seek_by(arc_length delta) noexcept {
    seek(position_ + delta);
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static): Must be non-static for consistent cursor API
std::default_sentinel_t path::cursor::end() const noexcept {
    return std::default_sentinel;
}

bool operator==(const path::cursor& c, std::default_sentinel_t) noexcept {
    return !std::isfinite(static_cast<double>(c.position_));
}

bool operator==(std::default_sentinel_t, const path::cursor& c) noexcept {
    return !std::isfinite(static_cast<double>(c.position_));
}

xt::xarray<double> path::cursor::configuration() const {
    // Validate cursor is not at sentinel
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot query cursor at sentinel position"};
    }

    // Use hint to get segment view, delegate to view
    auto view = *hint_;
    return view.configuration(position_);
}

xt::xarray<double> path::cursor::tangent() const {
    // Validate cursor is not at sentinel
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot query cursor at sentinel position"};
    }

    auto view = *hint_;
    return view.tangent(position_);
}

xt::xarray<double> path::cursor::curvature() const {
    // Validate cursor is not at sentinel
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot query cursor at sentinel position"};
    }

    auto view = *hint_;
    return view.curvature(position_);
}

void path::cursor::update_hint_() noexcept {
    // Dereference hint to get view for comparisons
    auto current_view = *hint_;

    // Fast path: Check if current hint is still valid
    // This handles the common case of small sequential steps within a segment
    if (position_ >= current_view.start() && position_ < current_view.end()) {
        return;  // Hint still valid, O(1)
    }

    // Special case: Exactly at segment end boundary
    // This can happen frequently during integration
    if (position_ == current_view.end()) {
        // Try to advance hint to next segment
        auto next_hint = hint_;
        ++next_hint;
        if (next_hint != path_->end()) {
            auto next_view = *next_hint;
            if (position_ >= next_view.start() && position_ < next_view.end()) {
                hint_ = next_hint;
                return;  // Advanced to next segment, O(1)
            }
        }
        // Position is exactly at path end, keep current hint
        return;
    }

    // Check previous segment (for backward integration)
    // Efficient thanks to bidirectional iterator!
    if (hint_ != path_->begin()) {
        auto prev_hint = hint_;
        --prev_hint;
        auto prev_view = *prev_hint;
        if (position_ >= prev_view.start() && position_ < prev_view.end()) {
            hint_ = prev_hint;
            return;  // Moved to previous segment, O(1)
        }
    }

    // Check next segment (for forward integration)
    if (hint_ != path_->end()) {
        auto next_hint = hint_;
        ++next_hint;
        if (next_hint != path_->end()) {
            auto next_view = *next_hint;
            if (position_ >= next_view.start() && position_ < next_view.end()) {
                hint_ = next_hint;
                return;  // Moved to next segment, O(1)
            }
        }
    }

    // Large jump - use binary search directly on segments_
    // Find first segment that starts AFTER position (upper_bound)
    // The segment containing position is the one before it
    // This is O(log n) but rare in sequential traversal
    auto seg_it = std::upper_bound(
        path_->segments_.begin(), path_->segments_.end(), position_, [](arc_length value, const positioned_segment& entry) {
            return value < entry.start;
        });

    // The segment containing position is before the upper_bound result
    if (seg_it == path_->segments_.begin()) {
        // Position is at or before first segment start (should be position == 0)
        hint_ = path_->begin();
    } else {
        --seg_it;
        // Convert internal iterator to const_iterator
        hint_ = const_iterator{path_, seg_it};
    }
}

path::cursor path::create_cursor(arc_length s) const {
    if (empty()) {
        throw std::invalid_argument{"Cannot create cursor for empty path"};
    }
    return cursor{this, s};
}

}  // namespace viam::trajex::totg
