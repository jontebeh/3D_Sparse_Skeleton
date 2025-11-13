#include <libcore/skeleton_finder.hpp>
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "Starting SkeletonFinder..." << std::endl;

    // Step 2: Create SkeletonFinder instance
    libcore::SkeletonFinder finder;

    // Step 4: Initialize (starts skeleton generation)
    finder.init("area_6.ini");

    std::cout << "SkeletonFinder completed." << std::endl;

    return 0;
}
