#include "occ3d/Bresenham.h"
#include <cassert>
#include <Eigen/Core>

namespace occ3d {
    Bresenham::Iterator::Iterator(const Eigen::Vector3i& start, const Eigen::Vector3i& end) {
        d_ = (end - start).cwiseAbs();
        x_ = d_.x() >= d_.y() && d_.x() >= d_.z();
        y_ = d_.y() >= d_.x() && d_.y() >= d_.z();
        z_ = d_.z() >= d_.x() && d_.z() >= d_.y();
        if(x_) {
            istart_ = start;
            iend_ = end;
        } else if(y_) {
            istart_ = Eigen::Vector3i(start.y(), start.x(), start.z());
            iend_ = Eigen::Vector3i(end.y(), end.x(), end.z());
        } else if(z_) {
            istart_ = Eigen::Vector3i(start.z(), start.y(), start.x());
            iend_ = Eigen::Vector3i(end.z(), end.y(), end.x());
        }
        d_ = (iend_ - istart_).cwiseAbs();
        s_ = (iend_ - istart_).cwiseSign();
        fst_ = d_.y() * 2 - d_.x();
        snd_ = d_.z() * 2 - d_.x();
    }

    Eigen::Vector3i Bresenham::Iterator::operator*() {
        if(x_) return istart_;
        else if(y_) return Eigen::Vector3i(istart_.y(), istart_.x(), istart_.z());
        else if(z_) return Eigen::Vector3i(istart_.z(), istart_.y(), istart_.x());
        // not reachable
        else assert(false);
        return Eigen::Vector3i::Zero();
    }

    Bresenham::Iterator& Bresenham::Iterator::operator++() {
        istart_.x() += s_.x();
        if(fst_ >= 0) {
            istart_.y() += s_.y();
            fst_ -= d_.x() * 2;
        }
        if(snd_ >= 0) {
            istart_.z() += s_.z();
            snd_ -= d_.x() * 2;
        }
        fst_ += d_.y() * 2;
        snd_ += d_.z() * 2;
        return *this;
    }

    bool Bresenham::Iterator::operator!=(const Bresenham::Iterator& other) const {
        return !(istart_ == other.istart_ && iend_ == other.iend_);
    }
} // namespace occ3d