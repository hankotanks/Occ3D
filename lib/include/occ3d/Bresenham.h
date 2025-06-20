#pragma once

#include <Eigen/Core>

namespace occ3d {
    class Bresenham {
    public:
        Bresenham(const Eigen::Vector3i& start, const Eigen::Vector3i& end) : 
            start_(start), end_(end) { /* STUB */ }
        class Iterator {
            friend class Bresenham;
        public:
            Iterator(const Eigen::Vector3i& start, const Eigen::Vector3i& end);
            Eigen::Vector3i operator*();
            Iterator& operator++();
            bool operator!=(const Iterator& other) const;
        private:
            Eigen::Vector3i istart_, iend_, d_, s_;
            bool x_, y_, z_;
            int fst_, snd_;
        }; // class Iterator
        Iterator begin() { return Iterator(start_, end_); }
        Iterator end() {
            Iterator it(start_, end_); 
            it.istart_ = it.iend_;  
            return it;
        }
    private:
        Eigen::Vector3i start_, end_;
    }; // class Bresenham
} // namespace occ3d