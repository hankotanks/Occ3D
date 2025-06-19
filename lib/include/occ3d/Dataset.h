#pragma once

#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>

namespace occ3d {
    using Cloud = std::vector<Eigen::Vector3d>;

    class Dataset {
    public:
        virtual std::size_t size() const = 0;
        virtual std::pair<Eigen::Matrix4d, Cloud> operator[](const std::size_t idx) const = 0;
    };
} // namespace occ3d

