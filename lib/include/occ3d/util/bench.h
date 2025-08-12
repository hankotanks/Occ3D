#pragma once

#include <chrono>
#include <iostream>

namespace occ3d {
    namespace util {
        class Bench {
        private:
            using Clock = std::chrono::high_resolution_clock;
            using TimePoint = std::chrono::time_point<Clock>;
            using Duration = std::chrono::duration<double>;
            TimePoint start_;
        public:
            Bench() { start_ = Clock::now(); }
            Bench(const Bench& other) = delete;
            Bench(Bench&& other) = delete;
            void operator=(const Bench& other) = delete;
            Bench& operator=(Bench&& other) = default;
            void reset() { *this = Bench(); }
            double duration() { 
                Duration dur = Clock::now() - this->start_;
                return static_cast<double>(dur.count()); 
            }
        };
    } // namespace util
} // namespace occ3d
