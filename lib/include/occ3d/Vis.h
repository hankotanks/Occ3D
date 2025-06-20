#pragma once

#include "occ3d/GridMap.h"

namespace occ3d {
    class Vis {
    public:
        virtual void prepare(const occ3d::GridMap& grid_map) = 0;
        virtual void show() const = 0;
    };
} // namespace occ3d