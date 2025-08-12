#include "occ3d/vis/CloudVis.h"

#include <open3d/visualization/utility/DrawGeometry.h>

namespace occ3d {
    namespace vis {
        CloudVis::CloudVis() {
            win_ = std::make_shared<open3d::visualization::Visualizer>();
            win_->CreateVisualizerWindow("Vis", 800, 600);
        }

        CloudVis::~CloudVis() {
            win_->DestroyVisualizerWindow();
        }

        void CloudVis::prepare(const std::shared_ptr<open3d::geometry::PointCloud> cloud) {
            if(vis_ == nullptr) {
                vis_ = cloud;
                win_->AddGeometry(vis_);
            } else {
                *vis_ = *cloud;
                win_->UpdateGeometry(vis_);
            }
        }

        void CloudVis::show() const {
            if(!win_->PollEvents()) CloudVis::~CloudVis();
            win_->UpdateRender();
        }
    }
}