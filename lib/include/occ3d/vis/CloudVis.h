#pragma once

#include "occ3d/GridMap.h"
#include "occ3d/Vis.h"
#include "occ3d/util/logodds.h"

namespace occ3d {
    namespace vis {
        class CloudVis : occ3d::Vis {
        public:
            CloudVis(const double prob_threshold = 0.5) : 
                log_odds_threshold_(util::prob_to_log_odds(prob_threshold)) { /* STUB */ };
            virtual void prepare(const occ3d::GridMap& grid_map);
            virtual void show() const;
        protected:
            double log_odds_threshold_;
            std::shared_ptr<open3d::geometry::PointCloud> vis_ = nullptr;
        };
    }
}