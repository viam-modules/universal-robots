#include <viam/trajex/totg/waypoint_accumulator.hpp>

#include <stdexcept>

namespace viam::trajex::totg {

waypoint_accumulator::waypoint_accumulator(const xt::xarray<double>& waypoints) {
    if (waypoints.dimension() != 2) {
        throw std::invalid_argument{"Waypoints must be 2-dimensional"};
    }
    if (waypoints.shape()[0] == 0) {
        throw std::invalid_argument{"Waypoints cannot be empty"};
    }

    dof_ = waypoints.shape()[1];

    // Create views into the waypoint array rather than copying. This zero-copy approach
    // is efficient for path creation, which needs to iterate through waypoints but doesn't
    // need to modify them. The caller must ensure the waypoint array outlives this accumulator.
    add_waypoints(waypoints);
}

waypoint_accumulator::waypoint_accumulator(const waypoint_view_t& first_waypoint) {
    dof_ = first_waypoint.shape()[0];
    waypoints_.push_back(first_waypoint);
}

waypoint_accumulator::waypoint_accumulator(const waypoint_accumulator&) = default;
waypoint_accumulator::waypoint_accumulator(waypoint_accumulator&&) noexcept = default;

// Copy assignment via copy-and-swap. Default copy assignment fails because
// std::vector::operator= tries to assign through existing xview elements,
// and xtensor interprets xview assignment as data copy -- which fails when
// the views reference const data. Copy construction works correctly (creates
// new views of the same data), so we lean on that.
waypoint_accumulator& waypoint_accumulator::operator=(const waypoint_accumulator& other) {
    if (this != &other) {
        auto copy = other;
        *this = std::move(copy);
    }
    return *this;
}

waypoint_accumulator& waypoint_accumulator::operator=(waypoint_accumulator&&) noexcept = default;

waypoint_accumulator& waypoint_accumulator::add_waypoints(const xt::xarray<double>& waypoints) {
    if (waypoints.dimension() != 2) {
        throw std::invalid_argument{"Waypoints must be 2-dimensional"};
    }

    if (waypoints.shape()[1] != dof_) {
        throw std::invalid_argument{"Waypoints dimensions must match existing DOF"};
    }

    const size_t num_waypoints = waypoints.shape()[0];
    for (size_t i = 0; i < num_waypoints; ++i) {
        waypoints_.push_back(xt::view(waypoints, i, xt::all()));
    }

    return *this;
}

waypoint_accumulator& waypoint_accumulator::add_waypoint(const waypoint_view_t& waypoint) {
    if (waypoint.shape()[0] != dof_) {
        throw std::invalid_argument{"Waypoint DOF must match existing DOF"};
    }
    waypoints_.push_back(waypoint);
    return *this;
}

size_t waypoint_accumulator::dof() const noexcept {
    return dof_;
}

size_t waypoint_accumulator::size() const noexcept {
    return waypoints_.size();
}

bool waypoint_accumulator::empty() const noexcept {
    return waypoints_.empty();
}

waypoint_accumulator::const_iterator waypoint_accumulator::begin() const noexcept {
    return waypoints_.cbegin();
}

waypoint_accumulator::const_iterator waypoint_accumulator::end() const noexcept {
    return waypoints_.cend();
}

waypoint_accumulator::const_iterator waypoint_accumulator::cbegin() const noexcept {
    return waypoints_.cbegin();
}

waypoint_accumulator::const_iterator waypoint_accumulator::cend() const noexcept {
    return waypoints_.cend();
}

const waypoint_accumulator::waypoint_view_t& waypoint_accumulator::operator[](size_t i) const {
    return waypoints_[i];
}

const waypoint_accumulator::waypoint_view_t& waypoint_accumulator::at(size_t i) const {
    if (i >= waypoints_.size()) [[unlikely]] {
        throw std::out_of_range{"waypoint_accumulator::at: index out of range"};
    }
    return waypoints_[i];
}

}  // namespace viam::trajex::totg
