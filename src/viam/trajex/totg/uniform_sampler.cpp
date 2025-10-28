#include <viam/trajex/totg/uniform_sampler.hpp>

#include <cmath>
#include <stdexcept>

namespace viam::trajex::totg {

uniform_sampler::uniform_sampler(trajectory::seconds dt) : dt_{dt} {}

double uniform_sampler::calculate_quantized_dt(double duration_sec, double frequency_hz) {
    if (duration_sec <= 0.0 || frequency_hz <= 0.0) {
        throw std::invalid_argument{"duration and frequency must both be positive"};
    }

    const double putative_samples = duration_sec * frequency_hz;

    static constexpr std::size_t k_max_samples = 1000000;
    if (!std::isfinite(putative_samples) || putative_samples > k_max_samples) {
        throw std::invalid_argument{"duration and frequency exceed maximum allowable samples"};
    }

    // Round up to ensure we slightly oversample, then add 1 to get at least 2 samples
    // (start and end). This guarantees we'll hit the endpoint exactly after adjusting dt.
    const auto num_samples = static_cast<std::size_t>(std::ceil(putative_samples) + 1);

    // Recalculate dt to divide the duration evenly. This adjusted timestep ensures that
    // (num_samples - 1) * dt equals duration exactly, so the final sample lands precisely
    // on the trajectory endpoint without floating point error.
    return duration_sec / static_cast<double>(num_samples - 1);
}

uniform_sampler uniform_sampler::quantized_for_duration(trajectory::seconds duration, types::hertz frequency) {
    const double adjusted_dt_sec = calculate_quantized_dt(duration.count(), frequency.value);
    return uniform_sampler{trajectory::seconds{adjusted_dt_sec}};
}

uniform_sampler uniform_sampler::quantized_for_trajectory(const trajectory& traj, types::hertz frequency) {
    return quantized_for_duration(traj.duration(), frequency);
}

std::optional<struct trajectory::sample> uniform_sampler::next(trajectory::cursor& cursor) {
    if (cursor == cursor.end()) {
        return std::nullopt;
    }

    // Sample before advancing (not after) so that the first call returns t=0 and the final
    // call returns t=duration before advancing past the end. This ordering is important for
    // C++20 range semantics where dereferencing gives the current value and ++ advances.
    auto sample = cursor.sample();

    // Advance for next iteration. If this takes us past the trajectory duration, the cursor
    // moves to the end sentinel, and the next call to next() will return nullopt. We need to special
    // case things near the end to ensure that we hit it exactly, but don't get stuck there.
    const auto now = cursor.time();
    const auto next = now + dt_;
    if ((now != cursor.trajectory().duration()) && (next > cursor.trajectory().duration())) {
        cursor.seek(cursor.trajectory().duration());
    } else {
        cursor.seek(next);
    }

    return sample;
}

}  // namespace viam::trajex::totg
