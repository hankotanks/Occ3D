#pragma once

#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>

namespace occ3d {
    using Cloud = std::vector<Eigen::Vector3d>;

    class Dataset {
    public:
        Dataset(const std::string& path_data);
        Dataset(const std::string& path_data, const std::size_t frames_to_process);
        std::size_t size() const { return size_; };
        std::pair<Eigen::Matrix4d, Cloud> operator[](const std::size_t idx) const;
    private:
        std::size_t size_;
        std::vector<std::string> files_;
        std::vector<Eigen::Matrix4d> poses_;
    };
} // namespace occ3d

