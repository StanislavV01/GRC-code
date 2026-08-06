#ifndef BACKTRACK_RANDOM_HPP
#define BACKTRACK_RANDOM_HPP

// Thin, seedable Gaussian source. Wraps the standard RNG so every stochastic
// component draws from one deterministic, replayable stream (CP determinism).

#include <random>

namespace backtrack {

class GaussianSource {
public:
    explicit GaussianSource(unsigned long seed) : engine_(seed) {}

    // Sample from N(mean, stddev). stddev == 0 returns mean exactly.
    double gauss(double mean, double stddev) {
        if (stddev <= 0.0) return mean;
        std::normal_distribution<double> dist(mean, stddev);
        return dist(engine_);
    }

private:
    std::mt19937_64 engine_;
};

}  // namespace backtrack

#endif  // BACKTRACK_RANDOM_HPP
