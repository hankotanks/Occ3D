#pragma once

#include "occ3d/GridMap.h"
#include "occ3d/vis/CloudVis.h"

namespace occ3d {
    namespace vis {
        class VoxelVis : CloudVis {
        public:
            VoxelVis(const double prob_threshold = 0.5) : 
                CloudVis(prob_threshold) { /* STUB */ };
            void prepare(const occ3d::GridMap& grid_map);
            void show() const;
        private:
            std::shared_ptr<open3d::geometry::VoxelGrid> voxels_ = nullptr;
        };
    }
}