#pragma once

#include "occ3d/vis/CloudVis.h"

namespace occ3d {
    namespace vis {
        class VoxelVis : public CloudVis {
        public:
            VoxelVis() : CloudVis::CloudVis() { /* STUB */ }
            ~VoxelVis() { CloudVis::~CloudVis(); }
            void prepare(const std::shared_ptr<open3d::geometry::PointCloud> cloud);
            void show() const;
        private:
            std::shared_ptr<open3d::geometry::VoxelGrid> voxels_ = nullptr;
        };
    }
}