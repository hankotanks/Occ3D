#pragma once

#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>

namespace occ3d {
    using Cloud = std::vector<Eigen::Vector3d>;

    class Dataset {
    public:
        Dataset(const std::string& data_dir);
        std::size_t size() const { return files_.size(); }
        std::pair<Eigen::Matrix4d, Cloud> operator[](const size_t idx) const;
    private:
        std::vector<std::string> files_;
        std::vector<Eigen::Matrix4d> poses_;
    };
} // namespace occ3d
