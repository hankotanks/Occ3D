#pragma once

#include <chrono>
#include <iostream>

namespace occ3d {
    namespace util {
        class ScopeTimer {
        public:
            static ScopeTimer __scope_timer_init_internal__(const char*, long int);
            ~ScopeTimer();
        private:
            using Clock = std::chrono::high_resolution_clock;
            using TimePoint = std::chrono::time_point<Clock>;
            using Duration = std::chrono::duration<double>;
            const char* file_;
            long int line_;
            TimePoint start_;
        };

        #ifdef SCOPE_TIMER_IMPLEMENTATION
        ScopeTimer ScopeTimer::__scope_timer_init_internal__(const char* file, long int line) {
            ScopeTimer timer;
            timer.file_ = file;
            timer.line_ = line;
            timer.start_ = Clock::now();
            return timer;
        }

        ScopeTimer::~ScopeTimer() {
            Duration dur = Clock::now() - this->start_;
            std::cout << "Exited scope at [" << this->file_ << ":" << this->line_ << "] after " << dur.count() << std::endl;
        }
        #endif // SCOPE_TIMER_IMPLEMENTATION
    }
}

#ifdef SCOPE_TIMER_IMPLEMENTATION
#define SCOPE_TIMER_BEGIN() occ3d::util::ScopeTimer __scope__timer_instance_internal__ = occ3d::util::ScopeTimer::__scope_timer_init_internal__(__FILE__, __LINE__)
#endif