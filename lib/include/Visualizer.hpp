#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include "Dataset.hpp"

#include "open3d/Open3D.h"

inline void visualize(const occ3d::Cloud& pc) {
    open3d::visualization::DrawGeometries({
        std::make_shared<open3d::geometry::PointCloud>(pc)
    });
}