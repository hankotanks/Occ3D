#include "occ3d/GridMap.h"
#include <memory>
#include <unordered_map>
#include <limits>
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

    std::vector<float> voxels(std::unordered_map<Eigen::Vector3i, double> occ) {
        // TODO: Replace with <algorithm>
        int temp;
        temp = std::numeric_limits<int>::max();
        Eigen::Vector3i minima(temp, temp, temp);
        temp = std::numeric_limits<int>::min();
        Eigen::Vector3i maxima(temp, temp, temp);
        for(const auto& [cell, log_odds] : occ) {
            minima = minima.cwiseMin(cell);
            maxima = maxima.cwiseMin(cell);
        }

        Eigen::Vector3i extent(maxima - minima + Eigen::Vector3i(1, 1, 1));
        std::vector<float> field(extent.x() * extent.y() * extent.z(), 0.f);

        for(const auto& [cell, log_odds] : occ) {
            Eigen::Vector3i coord = cell - minima;
            field[coord.x() + coord.y() + extent.x() + coord.z() * extent.x() * extent.y()] = 1.f;
        }

        return field;
    }
}

namespace occ3d {
    void GridMap::process(Dataset& data) {
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
                    occ_.insert(std::make_pair(cell_path, log_odds_prior_));

                occ_[cell_path] += log_odds_free_;
                occ_[cell_path] -= log_odds_prior_;
            }

            if(occ_.count(cell) == 0)
                occ_.insert(std::make_pair(cell, log_odds_prior_));

            occ_[cell] += log_odds_occupied_;
            occ_[cell] -= log_odds_prior_;
        }
    }
}
