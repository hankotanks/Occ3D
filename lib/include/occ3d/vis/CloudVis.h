#pragma once

#include <open3d/visualization/visualizer/Visualizer.h>

#include "occ3d/Vis.h"

namespace occ3d {
    namespace vis {
        class CloudVis : public occ3d::Vis {
        public:
            CloudVis();
            ~CloudVis();
            virtual void prepare(const std::shared_ptr<open3d::geometry::PointCloud> cloud);
            virtual void show() const;
        protected:
            std::shared_ptr<open3d::visualization::Visualizer> win_;
            std::shared_ptr<open3d::geometry::PointCloud> vis_ = nullptr;
        };
    }
}