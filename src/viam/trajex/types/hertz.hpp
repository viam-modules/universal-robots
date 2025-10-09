#pragma once

namespace viam::trajex::types {

/// Strong type for frequency in Hertz
///
/// Prevents confusion between frequency and time duration values.
struct hertz {
    double value;  ///< Frequency value in Hz

    /// Construct from Hz value
    explicit hertz(double hz) : value{hz} {}
};

}  // namespace viam::trajex::types
