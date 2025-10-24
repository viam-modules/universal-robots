#pragma once

namespace viam::trajex::types {

///
/// Strong type for frequency in Hertz.
///
/// Prevents confusion between frequency and time duration values.
///
struct hertz {
    ///
    /// Frequency value in Hz.
    ///
    double value;

    ///
    /// Constructs from Hz value.
    ///
    /// @param hz Frequency in Hz
    ///
    explicit hertz(double hz) : value{hz} {}
};

}  // namespace viam::trajex::types
