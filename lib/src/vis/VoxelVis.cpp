#include "occ3d/vis/VoxelVis.h"
#include "occ3d/vis/CloudVis.h"

namespace occ3d {
    namespace vis {
        void VoxelVis::prepare(const occ3d::GridMap& grid_map) {
            if(voxels_) {
                std::cout << "[INFO] VoxelGridVis already initialized. Ignoring." << std::endl;
                return;
            }
            
            CloudVis::prepare(grid_map);

            voxels_ = open3d::geometry::VoxelGrid::CreateFromPointCloud(*vis_, 1.0);
#if 0
            int temp;
            temp = std::numeric_limits<int>::max();
            Eigen::Vector3i minima(temp, temp, temp);
            temp = std::numeric_limits<int>::min();
            Eigen::Vector3i maxima(temp, temp, temp);
            for(const auto& [cell, log_odds] : grid_map) {
                minima = minima.cwiseMin(cell);
                maxima = maxima.cwiseMin(cell);
            }

            Eigen::Vector3i extent(maxima - minima + Eigen::Vector3i(1, 1, 1));
            std::vector<float> field(extent.x() * extent.y() * extent.z(), 0.f);

            for(const auto& [cell, log_odds] : grid_map) {
                Eigen::Vector3i coord = cell - minima;
                field[coord.x() + coord.y() + extent.x() + coord.z() * extent.x() * extent.y()] = 1.f;
            }

            const int resolution = extent.x() * extent.y() * extent.z();
#endif
        }
        
        void VoxelVis::show() const {
            open3d::visualization::DrawGeometries({voxels_});
        }
    }
}