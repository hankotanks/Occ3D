#include "occ3d/vis/VoxelVis.h"
#include "occ3d/vis/CloudVis.h"

namespace occ3d {
    namespace vis {
        void VoxelVis::prepare(const occ3d::GridMap& grid_map) {
            if(voxels_) {
                std::cout << "[INFO] VoxelVis already initialized. Ignoring." << std::endl;
                return;
            }
            
            CloudVis::prepare(grid_map);
            voxels_ = open3d::geometry::VoxelGrid::CreateFromPointCloud(*vis_, 1.0);
        }
        
        void VoxelVis::show() const {
            open3d::visualization::DrawGeometries({voxels_});
        }
    }
}