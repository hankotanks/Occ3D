#include "occ3d/GridMap.h"
#include <memory>
#include <unordered_map>
#include <Eigen/Core>
#include <open3d/Open3D.h>
#include "occ3d/Bresenham.h"

namespace {
    Eigen::Vector3d transform_point(
        const Eigen::Vector3d& pt, 
        const Eigen::Matrix4d& pose
    ) {
        Eigen::Vector4d pt_homo(pt.x(), pt.y(), pt.z(), 1.0);
        pt_homo = pose * pt_homo;
        Eigen::Vector3d pt_transformed(pt_homo.x(), pt_homo.y(), pt_homo.z());
        return pt_transformed;
    }
}

namespace occ3d {
    void GridMap::process(Dataset& data) {
        // setting this to null indicates that the point cloud must be
        // reconverted to open3d representation before visualization
        vis_ = NULL;

        const std::size_t data_frame_count = data.size();
        if(data_frame_count == 0) return;
        const std::size_t data_frame_count_digits = \
            static_cast<std::size_t>(std::log10(data_frame_count)) + 1;
            
        for(std::size_t i = 0, j = 0; i < data_frame_count; ++i) {
            // we can't actually decompose this pair directly,
            // because that gives us references
            // we have to use std::get to assign each tuple value to a variable
            // otherwise, we can't pass it to the lambda
            const auto data_frame = data[i];

            Eigen::Matrix4d pose = std::get<0>(data_frame);
            Cloud points = std::get<1>(data_frame);
            
            GridMap::process_frame(pose, points);
            
            const float completion_f = static_cast<float>(i) / \
                static_cast<float>(data_frame_count) * 100.f;
            const std::size_t completion = static_cast<std::size_t>(completion_f);
            if(completion != j) {
                j = completion;
                std::cout << '[';
                std::cout << std::setw(data_frame_count_digits);
                std::cout << std::setfill('0') << i << '/' << data_frame_count;
                std::cout << "] frames processed." << std::endl;
            }
        }
    }

    void GridMap::process_frame(Eigen::Matrix4d pose, const Cloud& points) {
        Cloud points_transformed(points.size());
        std::transform(points.begin(), points.end(), points_transformed.begin(),
            [&](const Eigen::Vector3d& pt) { return transform_point(pt, pose); });
        
        const Eigen::Vector3i cell_pose = GridMap::point_to_cell(pose.block<3, 1>(0, 3));
        for(const Eigen::Vector3d& point : points_transformed) {
            const Eigen::Vector3i cell = GridMap::point_to_cell(point);
            for(const Eigen::Vector3i cell_path : Bresenham(cell_pose, cell)) {
                if(occ_.count(cell_path) == 0)
                    occ_.insert(std::make_pair(cell_path, prob_prior_));

                occ_[cell_path] += prob_free_;
                occ_[cell_path] -= prob_prior_;
            }

            if(occ_.count(cell) == 0)
                occ_.insert(std::make_pair(cell, prob_prior_));

            occ_[cell] += prob_occupied_;
            occ_[cell] -= prob_prior_;
        }
    }

    void GridMap::visualize_update() {
        if(vis_ == NULL) {
            std::cout << "[INFO] Preparing occupancy visualization." << std::endl;

            vis_ = std::make_shared<open3d::geometry::PointCloud>();
            for(const auto& [cell, log_odds] : occ_) {
                const double prob = GridMap::log_odds_to_prob(log_odds);
                if(prob < 0.4) continue;
                vis_->points_.emplace_back(GridMap::cell_to_point(cell));
                const double color = 1.0 - prob;
                vis_->colors_.emplace_back(color, color, color);
            }
#if 0
            // keeping this here as proof that I understand how to
            // use std::transform.
            // unfortunately, <algorithm> does not have a 
            // function that allows me to populate both colors_ and points_
            // from a single input iterator
            std::transform(occ.begin(), occ.end(), points.begin(),
                [](const std::pair<const Eigen::Vector3i, double>& voxel) {
                    const double log_odds = std::get<1>(voxel);
                    return cell_to_point(std::get<0>(voxel));
                });
#endif
            std::cout << "[INFO] Visualization finished with ";
            std::cout << (vis_->points_).size() << " points." << std::endl;
        } else std::cout << "[INFO] Occupancy grid is unchanged. Re-rendering." << std::endl;
    }

    void GridMap::visualize() {
        GridMap::visualize_update();
        open3d::visualization::DrawGeometries({vis_});
    }
}
