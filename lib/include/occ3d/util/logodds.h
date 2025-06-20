#pragma once

#include <cmath>

namespace occ3d {
    namespace util {
        inline constexpr double prob_to_log_odds(const double prob) {
            return std::log(prob / (prob * -1.0 + 1.0));
        }
        
        inline constexpr double log_odds_to_prob(const double log_odds) {
            return 1.0 - 1.0 / (1.0 + std::exp(log_odds));
        }
    }
}