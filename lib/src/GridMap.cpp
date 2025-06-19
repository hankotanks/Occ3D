#include "occ3d/GridMap.h"
#include <memory>
#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <unordered_map>

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

    occ3d::Cloud occupancy_to_cloud(const std::unordered_map<Eigen::Vector3i, double> occ) {
        occ3d::Cloud points(occ.size());
        std::transform(occ.begin(), occ.end(), points.begin(),
            [](const std::pair<const Eigen::Vector3i, double>& voxel) {
                Eigen::Vector3i point = std::get<0>(voxel);
                // TODO: handle opacity
                // double alpha = std::get<1>(voxel);
                return Eigen::Vector3d(
                    static_cast<double>(point.x()),
                    static_cast<double>(point.y()),
                    static_cast<double>(point.z())
                );
            });
        
        return points;
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
        Cloud points_posed(points.size());
        std::transform(points.begin(), points.end(), points_posed.begin(),
            [&](const Eigen::Vector3d& pt) { return transform_point(pt, pose); });
        
        for(const Eigen::Vector3d& point : points_posed) {
            Eigen::Vector3d point_norm = point / voxel_size_;
            Eigen::Vector3i point_key(
                std::floor(point_norm.x()), 
                std::floor(point_norm.y()), 
                std::floor(point_norm.z())
            );
            occ_.insert(std::make_pair(point_key, 1.0));
        }
    }

    void GridMap::visualize() {
        if(vis_ == NULL) {
            std::cout << "[INFO] Preparing occupancy visualization." << std::endl;
            vis_ = std::make_shared<open3d::geometry::PointCloud>();
            vis_->points_ = occupancy_to_cloud(occ_);
            std::cout << "[INFO] Visualization finished with ";
            std::cout << (vis_->points_).size() << " points." << std::endl;
        }
        open3d::visualization::DrawGeometries({vis_});
    }
}
