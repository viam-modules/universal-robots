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
#include <viam/trajex/types/angles.hpp>

namespace viam::trajex::totg {

using viam::trajex::degrees_to_radians;

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

path::segment::linear::linear(xt::xarray<double> start, const xt::xarray<double>& end) : start{std::move(start)}, length{0.0} {
    const auto diff = end - this->start;
    const double norm = xt::norm_l2(diff)();

    // TODO(RSDK-12761): Use scale-relative tolerance instead of hardcoded absolute value
    if (norm < 1e-10) {
        throw std::invalid_argument{"Linear segment: start and end must be different"};
    }

    this->unit_direction = diff / norm;
    this->length = arc_length{norm};
}

path::segment::circular::circular(xt::xarray<double> center, xt::xarray<double> x, xt::xarray<double> y, double radius, double angle_rads)
    : center{std::move(center)}, x{std::move(x)}, y{std::move(y)}, radius{radius}, angle_rads{angle_rads} {
    const double x_norm = xt::norm_l2(this->x)();
    const double y_norm = xt::norm_l2(this->y)();

    // TODO(RSDK-12761): Use machine epsilon based tolerance for unit vector check
    if (std::abs(x_norm - 1.0) > 1e-6 || std::abs(y_norm - 1.0) > 1e-6) {
        throw std::invalid_argument{"Circular segment: x and y must be unit vectors"};
    }

    const double dot_product = xt::sum(this->x * this->y)();
    // TODO(RSDK-12761): Consider tighter tolerance for orthogonality
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
    // `Correction 1`: Need to offset before Kunz & Stilman equation 7.
    const arc_length local_s = s - start_;

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
                // Circular arc configuration - Kunz & Stilman equation 7:
                const double angle = static_cast<double>(local_s) / seg_data.radius;
                return seg_data.center + (seg_data.radius * (seg_data.x * std::cos(angle) + seg_data.y * std::sin(angle)));
            }
        },
        seg_.get().data_);
}

xt::xarray<double> path::segment::view::tangent(arc_length s) const {
    // `Correction 1`: Need to offset before Kunz & Stilman equation 8.
    const arc_length local_s = s - start_;

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
                // Circular arc unit tangent - Kunz & Stilman equation 8:
                const double angle = static_cast<double>(local_s) / seg_data.radius;
                return (-seg_data.x * std::sin(angle)) + (seg_data.y * std::cos(angle));
            }
        },
        seg_.get().data_);
}

xt::xarray<double> path::segment::view::curvature(arc_length s) const {
    // `Correction 1`: Need to offset before Kunz & Stilman equation 9.
    const arc_length local_s = s - start_;

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
                // Circular arc curvature vector - Kunz & Stilman equation 9:
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

    // TODO(RSDK-12771): Revisit this algorithm, and write a more graceful one.

    // Build a path from waypoints using two techniques: tube coalescing to skip waypoints
    // that lie within a tolerance tube around linear segments, and circular blends at corners
    // to smooth transitions between non-colinear segments. Both techniques keep the path
    // within specified deviation tolerances of the original waypoints.
    //
    // The algorithm scans through waypoints with a "locus" at each potential corner. At each
    // locus, we look backward to the segment start and forward to the next waypoint to decide
    // if the locus can be skipped (coalesced) or if it represents an actual corner that needs
    // either a hard stop or a circular blend.

    const auto can_coalesce = [&opts](const auto& start, const auto& locus, const auto& next) -> bool {
        const double max_deviation = opts.max_linear_deviation();

        // With zero tolerance, nothing coalesces (every waypoint is a hard constraint)
        if (max_deviation == 0.0) {
            return false;
        }

        const double radius = max_deviation / 2.0;

        const auto start_to_next = next - start;

        // Check if start and next are exactly the same position (all components identically zero)
        if (xt::all(xt::equal(start_to_next, 0.0))) {
            // When start == next (returning to same position), the locus represents an
            // intentional intermediate goal that must be preserved. Cannot coalesce.
            return false;
        }

        const auto start_to_locus = locus - start;
        const double start_to_next_sq = xt::sum(start_to_next * start_to_next)();
        const double start_to_locus_dot_direction = xt::sum(start_to_locus * start_to_next)();

        // Monotonic advancement check: reject if locus would require going backward from
        // start or forward past next. This ensures we only skip waypoints that maintain
        // forward progress along the segment direction.
        //
        // We check the bounds BEFORE dividing to avoid division-by-near-zero issues.
        // Since start_to_next_sq > 0, we can multiply through the inequality:
        //   t < 0  becomes  dot < 0
        //   t > 1  becomes  dot > start_to_next_sq
        if (start_to_locus_dot_direction < 0.0 || start_to_locus_dot_direction > start_to_next_sq) {
            return false;
        }

        // Now safe to divide: bounds check guarantees 0 <= t <= 1
        const double t = start_to_locus_dot_direction / start_to_next_sq;
        const auto projected_point = start + (t * start_to_next);
        const auto deviation_vector = locus - projected_point;
        const double deviation = xt::norm_l2(deviation_vector)();

        return deviation <= radius;
    };

    struct blend_geometry {
        segment::circular circular_seg;
        double trim_distance;  // Distance trimmed from both segments
    };

    // Try to create a circular blend arc at a corner following the algorithm from
    // Kunz & Stilman Section IV. The blend arc is tangent to both the incoming and
    // outgoing segments, trimming equal distances from each side. The blend keeps
    // the path within max_blend_deviation of the original corner waypoint.
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

        // Compute angle between incoming and outgoing directions; Kunz & Stilman equation 2.
        const double dot = xt::sum(incoming_unit * outgoing_unit)();
        const double dot_clamped = std::clamp(dot, -1.0, 1.0);
        const double angle = std::acos(dot_clamped);
        const double half_angle = angle / 2.0;

        // Reject degenerate cases: very small angles would create enormous blend radii
        // for minimal benefit, and angles near pi/2 cause tan(half_angle) to blow up.
        // These checks prevent numerical issues and avoid creating blends that would
        // violate the deviation constraint.
        //
        // TODO(RSDK-12761): Explicit epsilon, scaling, etc. Also, it
        // isn't clear that it is OK to omit the blend, because we end
        // up with a discontinuity.
        constexpr double epsilon = 1e-6;
        if (half_angle < epsilon || half_angle > (degrees_to_radians(90.0) - epsilon)) {
            return std::nullopt;
        }

        // At this point angle is strictly in (0, pi). trajectory.cpp relies on this guarantee
        // when searching for Case 2 switching points within circular segments.

        // Calculate trim distance and radius following Kunz & Stilman equations 3-4.
        // The trim distance is constrained by three limits: can't trim more than half
        // of either segment, and can't exceed what's allowed by the deviation tolerance.
        const double max_trim_from_deviation = opts.max_blend_deviation() * std::sin(half_angle) / (1.0 - std::cos(half_angle));
        const double trim_distance = std::min({incoming_norm / 2.0, outgoing_norm / 2.0, max_trim_from_deviation});
        const double radius = trim_distance / std::tan(half_angle);

        // Construct the circular arc geometry following Kunz & Stilman equation 5-6.
        // The center lies along the angle bisector, and the x/y basis vectors define
        // the plane of the circular arc with x pointing toward the blend start point.
        const auto bisector = outgoing_unit - incoming_unit;
        const double bisector_norm = xt::norm_l2(bisector)();
        const auto bisector_unit = bisector / bisector_norm;

        const double center_offset = radius / std::cos(half_angle);
        const auto center = corner + (center_offset * bisector_unit);

        // x_hat points from center to blend start (where circle touches incoming segment)
        const auto blend_start = corner - (trim_distance * incoming_unit);
        const auto x_vec = blend_start - center;
        const auto x_unit = x_vec / xt::norm_l2(x_vec)();

        // y_hat is the incoming direction (perpendicular to x by construction)
        const auto& y_unit = incoming_unit;

        return blend_geometry{.circular_seg = segment::circular{center, x_unit, y_unit, radius, angle}, .trim_distance = trim_distance};
    };

    std::vector<positioned_segment> segments;
    arc_length cumulative_length{0.0};

    auto segment_start = waypoints.begin();

    // Track the current position on the path we're building. This must be stored as an actual
    // configuration copy (not just an iterator) because after creating a circular blend, the
    // current position becomes the blend exit point, which is a computed position between
    // waypoints rather than one of the original waypoints in the accumulator.
    xt::xarray<double> current_position = *segment_start;

    // Track waypoints we've skipped since the last segment emission. When extending the tube
    // to a new endpoint, we must revalidate that all previously skipped waypoints remain
    // within tolerance of the extended segment to prevent "tube drift".
    std::vector<decltype(waypoints.begin())> skipped_since_anchor;

    for (auto locus = std::next(waypoints.begin()); locus != waypoints.end(); ++locus) {
        auto next = std::next(locus);

        // If we're at the last waypoint, we must emit the final segment
        const bool at_last = (next == waypoints.end());

        // Check if we can skip this waypoint: it must pass the standard coalesce check
        // AND all previously skipped waypoints must remain within tolerance when the
        // tube is extended to the new endpoint (revalidation prevents drift).
        if (!at_last && can_coalesce(*segment_start, *locus, *next)) {
            // Revalidate all previously skipped waypoints against the extended segment
            const double radius = opts.max_linear_deviation() / 2.0;
            bool all_previous_valid = true;
            for (auto prev_skipped : skipped_since_anchor) {
                const auto start_to_next = *next - *segment_start;
                const auto start_to_prev = *prev_skipped - *segment_start;

                const double start_to_next_sq = xt::sum(start_to_next * start_to_next)();
                const double start_to_prev_dot = xt::sum(start_to_prev * start_to_next)();

                const double t = start_to_prev_dot / start_to_next_sq;
                const auto projected = *segment_start + (t * start_to_next);
                const auto deviation_vec = *prev_skipped - projected;
                const double deviation = xt::norm_l2(deviation_vec)();

                if (deviation > radius) {
                    all_previous_valid = false;
                    break;
                }
            }

            if (all_previous_valid) {
                skipped_since_anchor.push_back(locus);
                continue;
            }
        }

        // Locus represents a corner (or the final waypoint). We need to emit the segment
        // from current_position to this locus, and if it's a corner, decide whether to
        // create a circular blend or stop at the hard waypoint.

        const auto blend_opt = !at_last ? try_create_blend(current_position, *locus, *next) : std::nullopt;

        if (blend_opt) {
            const auto& blend = *blend_opt;
            const auto incoming = *locus - current_position;
            const auto incoming_unit = incoming / xt::norm_l2(incoming)();
            const auto outgoing = *next - *locus;
            const auto outgoing_unit = outgoing / xt::norm_l2(outgoing)();

            const auto trimmed_end = *locus - (blend.trim_distance * incoming_unit);
            segment::linear incoming_segment{current_position, trimmed_end};
            const auto linear_length = incoming_segment.length;
            segments.push_back({.seg = segment{std::move(incoming_segment)}, .start = cumulative_length});
            cumulative_length += linear_length;

            segments.push_back({.seg = segment{blend.circular_seg}, .start = cumulative_length});
            cumulative_length += arc_length{blend.circular_seg.radius * blend.circular_seg.angle_rads};

            // After the blend, we're positioned at the blend exit point on the outgoing segment.
            // This position doesn't correspond to any waypoint in the accumulator, which is why
            // current_position must be a configuration copy rather than an iterator.
            current_position = *locus + (blend.trim_distance * outgoing_unit);

            // Move segment_start forward for coalescing calculations. Even though current_position
            // is between locus and next, we use locus as the reference point for determining if
            // future waypoints can be coalesced.
            segment_start = locus;
            skipped_since_anchor.clear();
        } else {
            // No blend created - emit linear segment from current_position to locus
            const auto start_to_locus = *locus - current_position;
            const double dist_sq = xt::sum(start_to_locus * start_to_locus)();

            if (dist_sq > 0.0) {
                segment::linear linear_data{current_position, *locus};
                const auto linear_data_length = linear_data.length;
                segments.push_back({.seg = segment{std::move(linear_data)}, .start = cumulative_length});
                cumulative_length += linear_data_length;
            }

            current_position = *locus;
            segment_start = locus;
            skipped_since_anchor.clear();
        }
    }

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
    const arc_length end = (next != path_->segments_.end()) ? next->start : path_->length_;

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

    // Use binary search to find the segment containing s. We use upper_bound to find the
    // first segment that starts AFTER s, then step back one position to get the segment
    // that actually contains s. This gives O(log n) lookup for arbitrary queries.
    auto it = std::upper_bound(
        segments_.begin(), segments_.end(), s, [](arc_length value, const positioned_segment& entry) { return value < entry.start; });

    if (it == segments_.begin()) [[unlikely]] {
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

// TOOD: This clamping is sort of weird. I think it should either throw, or decay to the singular end cursor.
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

path::cursor& path::cursor::seek(arc_length s) noexcept {
    // Use +/-infinity sentinels to signal out-of-bounds positions, allowing the cursor to
    // continue operating without throwing exceptions. This is important for algorithms that
    // might overstep slightly and need to detect saturation. The sentinels are detectable
    // via comparison with end().
    if (s < arc_length{0.0}) {
        position_ = arc_length{-std::numeric_limits<double>::infinity()};
    } else if (s > path_->length()) {
        position_ = arc_length{std::numeric_limits<double>::infinity()};
    } else {
        position_ = s;
    }

    if (*this != end()) {
        update_hint_();
    }
    return *this;
}

path::cursor& path::cursor::seek_by(arc_length delta) noexcept {
    return seek(position_ + delta);
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
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot query cursor at sentinel position"};
    }

    auto view = *hint_;
    return view.configuration(position_);
}

xt::xarray<double> path::cursor::tangent() const {
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot query cursor at sentinel position"};
    }

    auto view = *hint_;
    return view.tangent(position_);
}

xt::xarray<double> path::cursor::curvature() const {
    if (*this == end()) [[unlikely]] {
        throw std::out_of_range{"Cannot query cursor at sentinel position"};
    }

    auto view = *hint_;
    return view.curvature(position_);
}

void path::cursor::update_hint_() noexcept {
    // Maintain a hint iterator pointing to the segment containing the current position.
    // This optimization provides O(1) amortized performance for sequential traversal
    // (the common case during trajectory integration) by checking nearby segments before
    // falling back to binary search for large jumps.
    //
    // TODO(RSDK-12770): Optimize hint update by detecting seek direction (forward vs backward from previous
    // position). We can skip checks that won't work based on direction (e.g., don't check prev
    // segment when moving forward), simplifying logic and improving performance for directional
    // sequential access patterns.

    auto current_view = *hint_;

    // Most common case: position is still within the current hint segment.
    if (position_ >= current_view.start() && position_ < current_view.end()) {
        return;
    }

    // Check if we're exactly at a segment boundary, which happens frequently during
    // integration when stepping lands precisely on a segment transition.
    if (position_ == current_view.end()) {
        auto next_hint = hint_;
        ++next_hint;
        if (next_hint != path_->end()) {
            auto next_view = *next_hint;
            if (position_ >= next_view.start() && position_ < next_view.end()) {
                hint_ = next_hint;
                return;
            }
        }
        return;
    }

    // Check adjacent segments (both directions) before resorting to binary search.
    // This handles small steps backward (rare but possible during some algorithms)
    // and forward steps that skip a segment (possible with larger integration steps).

    if (hint_ != path_->begin()) {
        auto prev_hint = hint_;
        --prev_hint;
        auto prev_view = *prev_hint;
        if (position_ >= prev_view.start() && position_ < prev_view.end()) {
            hint_ = prev_hint;
            return;
        }
    }

    if (hint_ != path_->end()) {
        auto next_hint = hint_;
        ++next_hint;
        if (next_hint != path_->end()) {
            auto next_view = *next_hint;
            if (position_ >= next_view.start() && position_ < next_view.end()) {
                hint_ = next_hint;
                return;
            }
        }
    }

    // Position is far from the hint (large jump). Use binary search to find the correct
    // segment. This is O(log n) but should be rare during normal sequential traversal.
    auto seg_it = std::upper_bound(
        path_->segments_.begin(), path_->segments_.end(), position_, [](arc_length value, const positioned_segment& entry) {
            return value < entry.start;
        });

    if (seg_it == path_->segments_.begin()) {
        hint_ = path_->begin();
    } else {
        --seg_it;
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
