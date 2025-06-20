#include <iostream>
#include <sstream>
#include <filesystem>
#include <Eigen/Core>
#include "occ3d/Dataset.h"
#include "occ3d/GridMap.h"

int main(int argc, char* argv[]) {
    if(argc != 3) {
        std::cout << "[ERROR] Must provide data path and voxel size." << std::endl;
        return 1;
    }

    if(!std::filesystem::exists(argv[1])) {
        std::cout << "[ERROR] Provided data path does not exist." << std::endl;
        return 1;
    }

    std::stringstream parser(argv[2]);
    double edge_size;
    parser >> edge_size;
    if(parser.fail()) {
        std::cout << "[ERROR] Voxel size could not be parsed." << std::endl;
        return 1;
    }

#if 0
    occ3d::Bresenham b(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(-10, -27, 7));
    for(const Eigen::Vector3i pt : b) std::cout << pt.transpose() << std::endl;
#endif

    std::string path_data(argv[1]);
    occ3d::Dataset data(path_data);

    occ3d::GridMap occ(edge_size);
    occ.process(data);
    occ.visualize();

    return 0;
}