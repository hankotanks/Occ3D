#pragma once

#include <unordered_map>
#include <open3d/Open3D.h>
#include "Dataset.h"

template <>
struct std::hash<Eigen::Vector3i> {
    std::size_t operator()(const Eigen::Vector3i &point) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t*>(point.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

namespace occ3d {
    class GridMap {
    public:
        GridMap(
            const double edge_size,
            const double prob_prior = 0.5,
            const double prob_free = 0.4, 
            const double prob_occupied = 0.85
        ) : edge_size_(edge_size), 
            prob_prior_(GridMap::prob_to_log_odds(prob_prior)),
            prob_free_(GridMap::prob_to_log_odds(prob_free)), 
            prob_occupied_(GridMap::prob_to_log_odds(prob_occupied)) { /* STUB */ }
        void process(Dataset& data);
        void visualize();
    private:
        void process_frame(Eigen::Matrix4d pose, const Cloud& points);
        void visualize_update();
        static constexpr double prob_to_log_odds(const double prob) {
            return std::log(prob / (prob * -1.0 + 1.0));
        }
        static constexpr double log_odds_to_prob(const double log_odds) {
            return 1.0 - 1.0 / (1.0 + std::exp(log_odds));
        }
        inline Eigen::Vector3i point_to_cell(const Eigen::Vector3d& point) {
            return Eigen::Vector3i(
                static_cast<int>(std::floor(point.x() / edge_size_)),
                static_cast<int>(std::floor(point.y() / edge_size_)),
                static_cast<int>(std::floor(point.z() / edge_size_))
            );
        }
        inline Eigen::Vector3d cell_to_point(const Eigen::Vector3i& cell) {
            return Eigen::Vector3d(
                static_cast<double>(cell.x()) / edge_size_,
                static_cast<double>(cell.y()) / edge_size_,
                static_cast<double>(cell.z()) / edge_size_
            );
        }
    private:
        double edge_size_;
        double prob_prior_, prob_free_, prob_occupied_;
        std::unordered_map<Eigen::Vector3i, double> occ_;
        std::shared_ptr<open3d::geometry::PointCloud> vis_ = NULL;
    };
} // namespace occ3d