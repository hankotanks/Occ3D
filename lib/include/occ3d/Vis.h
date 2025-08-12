#pragma once

#include <open3d/geometry/PointCloud.h>

namespace occ3d {
    class Vis {
    public:
        virtual void prepare(const std::shared_ptr<open3d::geometry::PointCloud>) = 0;
        virtual void show() const = 0;
    };
} // namespace occ3d