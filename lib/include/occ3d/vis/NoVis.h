#pragma once

#include "occ3d/Vis.h"

namespace occ3d {
    namespace vis {
        class NoVis : public occ3d::Vis {
        public:
            static NoVis& get() { 
                static NoVis instance;
                return instance;
            }
            virtual void prepare(const std::shared_ptr<open3d::geometry::PointCloud> cloud) { (void) cloud; };
            virtual void show() const { /* STUB */ };
        };
    }
}