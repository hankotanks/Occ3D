#include <iostream>
#include "occ3d/Dataset.h"
#include "occ3d/Visualizer.h"

int main(int argc, char* argv[]) {
    if(argc != 2) {
        std::cout << "Must provide path to point cloud data." << std::endl;
        return 1;
    }

    std::string data_path(argv[1]);
    occ3d::Dataset data(data_path);

    auto [_, pc] = data[0];
    occ3d::visualize(pc);

    return 0;
}