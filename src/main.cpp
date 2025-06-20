#include <iostream>
#include <sstream>
#include <filesystem>
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
    double voxel_size;
    parser >> voxel_size;
    if(parser.fail()) {
        std::cout << "[ERROR] Voxel size could not be parsed." << std::endl;
        return 1;
    }

    std::string data_path(argv[1]);
    occ3d::Dataset data(data_path);

    occ3d::GridMap occ(voxel_size);
    occ.process(data);
    occ.visualize();

    return 0;
}