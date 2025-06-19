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
        GridMap(const double voxel_size) : voxel_size_(voxel_size) { /* STUB */ }
        void process(Dataset& data);
        void visualize();
    private:
        void process_frame(Eigen::Matrix4d pose, const Cloud& points);
    private:
        double voxel_size_;
        std::unordered_map<Eigen::Vector3i, double> occ_;
        std::shared_ptr<open3d::geometry::PointCloud> vis_ = NULL;
    };
} // namespace occ3d