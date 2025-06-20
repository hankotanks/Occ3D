#include "occ3d/vis/CloudVis.h"
#include "occ3d/util/logodds.h"

namespace occ3d {
    namespace vis {
        void CloudVis::prepare(const GridMap& grid_map) {
            if(vis_) {
                std::cout << "[INFO] VoxelCloudVis already initialized. Ignoring." << std::endl;
                return;
            }
            
            std::cout << "[INFO] Preparing occupancy visualization." << std::endl;

            vis_ = std::make_shared<open3d::geometry::PointCloud>();
            for(const auto& [cell, log_odds] : grid_map) {
                if(log_odds < log_odds_threshold_) continue;
                vis_->points_.emplace_back(cell.cast<double>());
                const double color = 1.0 - util::log_odds_to_prob(log_odds);
                vis_->colors_.emplace_back(color, color, color);
            }

            std::cout << "[INFO] Visualization finished with ";
            std::cout << (vis_->points_).size() << " points." << std::endl;
        }

        void CloudVis::show() const {
            open3d::visualization::DrawGeometries({vis_});
        }
    }
}