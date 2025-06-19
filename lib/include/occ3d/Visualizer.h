#pragma once

#include <memory>
#include <vector>
#include <open3d/Open3D.h>
#include "Dataset.h"

namespace occ3d {
    inline void visualize(const occ3d::Cloud& pc) {
        open3d::visualization::DrawGeometries({
            std::make_shared<open3d::geometry::PointCloud>(pc)
        });
    }
}
