#include <iostream>
#include <sstream>
#include <filesystem>

#include <Eigen/Core>

#include "occ3d/Dataset.h"
#include "occ3d/GridMap.h"
#include "occ3d/vis/VoxelVis.h"

int main(int argc, char* argv[]) {
    if(argc != 4 && argc != 5) {
        std::cout << "[ERROR] Must provide data path, voxel size, and cutoff threshold." << std::endl;
        return 1;
    }

    if(!std::filesystem::exists(argv[1])) {
        std::cout << "[ERROR] Provided data path does not exist." << std::endl;
        return 1;
    }

    std::stringstream voxel_size_parser(argv[2]);
    double voxel_size;
    voxel_size_parser >> voxel_size;
    if(voxel_size_parser.fail()) {
        std::cout << "[ERROR] Voxel size could not be parsed (" << argv[2] << ")." << std::endl;
        return 1;
    }

    std::stringstream cutoff_threshold_parser(argv[3]);
    double cutoff_threshold;
    cutoff_threshold_parser >> cutoff_threshold;
    if(cutoff_threshold_parser.fail()) {
        std::cout << "[ERROR] Cutoff threshold could not be parsed (" << argv[3] << ")." << std::endl;
        return 1;
    }

    size_t frame_count = 0;
    if(argc == 5) {
        std::stringstream frame_count_parser(argv[4]);
        frame_count_parser >> frame_count;
        if(frame_count_parser.fail()) {
            std::cout <<"[ERROR] Unable to parse frame count (" << argv[4] << ")." << std::endl;
            return 1;
        }
    }

    std::string path_data(argv[1]);
    occ3d::Dataset data = frame_count ? occ3d::Dataset(path_data, frame_count) : occ3d::Dataset(path_data);

    occ3d::vis::VoxelVis vis;
    occ3d::GridMap occ(voxel_size, vis, cutoff_threshold);
    occ.process(data);

    return 0;
}