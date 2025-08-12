#include "occ3d/vis/VoxelVis.h"

#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/geometry/VoxelGrid.h>

#include "occ3d/vis/CloudVis.h"

namespace occ3d {
    namespace vis {
        void VoxelVis::prepare(const std::shared_ptr<open3d::geometry::PointCloud> cloud) {
            auto voxels = open3d::geometry::VoxelGrid::CreateFromPointCloud(*cloud, 1.0);
            if(voxels_ == nullptr) {
                voxels_ = voxels;
                win_->AddGeometry(voxels_);
            } else {
                *voxels_ = *voxels;
                win_->UpdateGeometry(voxels_);
            }
        }
        
        void VoxelVis::show() const {
            CloudVis::show();
        }
    }
}