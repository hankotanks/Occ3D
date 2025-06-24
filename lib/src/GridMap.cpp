#include "occ3d/GridMap.h"
#include <memory>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <Eigen/Core>
#include <open3d/Open3D.h>
#include "occ3d/Bresenham.h"
#define SCOPE_TIMER_IMPLEMENTATION
#include "occ3d/util/timer.h"

namespace {
    Eigen::Vector3d transform_point(
        const Eigen::Vector3d& point, 
        const Eigen::Matrix4d& pose
    ) {
        Eigen::Vector4d point_homo(point.x(), point.y(), point.z(), 1.0);
        point_homo = pose * point_homo;
        return Eigen::Vector3d(point_homo.x(), point_homo.y(), point_homo.z());
    }
}

namespace occ3d {
    void GridMap::process(Dataset& data) {
        SCOPE_TIMER_BEGIN();
        const std::size_t data_frame_count = data.size();
        if(data_frame_count == 0) return;
        const std::size_t data_frame_count_digits = \
            static_cast<std::size_t>(std::log10(data_frame_count)) + 1;
        
        // keeping this as a standard for loop specifically
        // so that I can print progress
        for(std::size_t i = 0, j = 0; i < data_frame_count; ++i) {
            const auto [pose, points] = data[i];
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

    void GridMap::process_cell(
        const Eigen::Vector3i& cell, 
        const Eigen::Vector3i& cell_pose
    ) {
        Bresenham path(cell_pose, cell);
        std::for_each(path.begin(), path.end(), 
            [&](const Eigen::Vector3i cell_path) {
                occ_.try_emplace(cell_path, log_odds_prior_);
                occ_[cell_path] += log_odds_free_;
                occ_[cell_path] -= log_odds_prior_;
            });
        
        occ_.try_emplace(cell, log_odds_prior_);
        occ_[cell] += log_odds_occupied_;
        occ_[cell] -= log_odds_prior_;
    }

    void GridMap::process_frame(Eigen::Matrix4d pose, const Cloud& points) {
        const Eigen::Vector3i cell_pose = GridMap::point_to_cell(pose.block<3, 1>(0, 3));
        std::for_each(points.cbegin(), points.cend(), 
            [&](const Eigen::Vector3d& point) {
                const Eigen::Vector3i cell = GridMap::point_to_cell(transform_point(point, pose));
                GridMap::process_cell(cell, cell_pose);
            });
    }
}
