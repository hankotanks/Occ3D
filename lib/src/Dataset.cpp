#include "Dataset.hpp"
#include <fstream>
#include <string>
#include <utility>
#include <filesystem>
#include <vector>
#include <Eigen/Core>
#include <open3d/Open3D.h>

namespace fs = std::filesystem;
namespace {
    std::vector<std::string> read_point_cloud_files(const fs::path& pointcloud_path) {
        std::vector<std::string> filenames;
        for(const auto& file : fs::directory_iterator(pointcloud_path)) {
            if(file.path().extension() == ".ply") {
                filenames.emplace_back(file.path().string());
            }
        }
        if(filenames.empty()) {
            std::cerr << pointcloud_path << "contains no files with .ply extension"
                    << std::endl;
            exit(1);
        }
        std::sort(filenames.begin(), filenames.end());

        return filenames;
    }

    occ3d::Cloud extract_point_cloud(const std::string& filename) {
        open3d::geometry::PointCloud pointcloud;
        open3d::io::ReadPointCloudFromPLY(filename, pointcloud, open3d::io::ReadPointCloudOption());
        return pointcloud.points_;
    }

    std::vector<Eigen::Matrix4d> read_poses(const std::string& filename) {
        std::ifstream file(filename);
        std::vector<Eigen::Matrix4d> poses;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        while(file >> pose(0, 0) >> pose(0, 1) >> pose(0, 2) >> pose(0, 3) >>
            pose(1, 0) >> pose(1, 1) >> pose(1, 2) >> pose(1, 3) >> pose(2, 0) >>
            pose(2, 1) >> pose(2, 2) >> pose(2, 3)) {
            poses.emplace_back(pose);
        }
        return poses;
    }
} // scoped

namespace occ3d {
    Dataset::Dataset(const std::string& data_dir) {
        files_ = read_point_cloud_files(fs::path(data_dir) / "PLY/");

        poses_.reserve(files_.size());
        poses_ = read_poses(fs::path(data_dir) / "gt_poses.txt");
    }

    std::pair<Eigen::Matrix4d, occ3d::Cloud> Dataset::operator[](const size_t idx) const {
        return std::make_pair(poses_[idx], extract_point_cloud(files_[idx]));
    }
} // namespace Dataset
