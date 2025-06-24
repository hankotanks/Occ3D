#pragma once

#include <unordered_map>
#include <open3d/Open3D.h>
#include "occ3d/Dataset.h"
#include "occ3d/util/logodds.h"

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
        ) : voxel_size_(edge_size), 
            log_odds_prior_(util::prob_to_log_odds(prob_prior)),
            log_odds_free_(util::prob_to_log_odds(prob_free)), 
            log_odds_occupied_(util::prob_to_log_odds(prob_occupied)) { /* STUB */ }
        void process(Dataset& data);
        auto begin() const { return occ_.cbegin(); }
        auto end() const { return occ_.cend(); }
    private:
        void process_cell(const Eigen::Vector3i& cell, const Eigen::Vector3i& cell_pose);
        void process_frame(Eigen::Matrix4d pose, const Cloud& points);
        inline Eigen::Vector3i point_to_cell(const Eigen::Vector3d& point) {
            return Eigen::Vector3i(
                static_cast<int>(std::floor(point.x() / voxel_size_)),
                static_cast<int>(std::floor(point.y() / voxel_size_)),
                static_cast<int>(std::floor(point.z() / voxel_size_))
            );
        }
        inline Eigen::Vector3d cell_to_point(const Eigen::Vector3i& cell) {
            return Eigen::Vector3d(
                static_cast<double>(cell.x()) * voxel_size_,
                static_cast<double>(cell.y()) * voxel_size_,
                static_cast<double>(cell.z()) * voxel_size_
            );
        }
    private:
        double voxel_size_;
        double log_odds_prior_, log_odds_free_, log_odds_occupied_;
        std::unordered_map<Eigen::Vector3i, double> occ_;
    };
} // namespace occ3d