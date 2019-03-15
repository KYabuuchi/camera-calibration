#include "calibration/loader.hpp"

int main(int argc, char* argv[])
{
    std::string file_path = "../data/kinectv2_00/config.yaml";
    if (argc == 2)
        file_path = argv[1];

    Calibration::Loader loader;
    if (not loader.load(file_path)) {
        std::cout << "cannot open " << file_path << std::endl;
        return -1;
    }
    std::cout << "open " << file_path << std::endl;

    // clang-format off
    std::cout << "\n===[Monocular]===" << std::endl;
    std::cout << "Resolution:"     << loader.monocular().resolution << std::endl;
    std::cout << "\nIntrinsic:\n"  << loader.monocular().intrinsic  << std::endl;
    std::cout << "\nDistortion:\n" << loader.monocular().distortion << std::endl;

    std::cout << "\n===[RGB]===" << std::endl;
    std::cout << "Resolution:"     << loader.rgb().resolution << std::endl;
    std::cout << "\nIntrinsic:\n"  << loader.rgb().intrinsic  << std::endl;
    std::cout << "\nDistortion:\n" << loader.rgb().distortion << std::endl;

    std::cout << "\n===[Depth]===" << std::endl;
    std::cout << "Resolution:"     << loader.depth().resolution << std::endl;
    std::cout << "\nIntrinsic:\n"  << loader.depth().intrinsic  << std::endl;
    std::cout << "\nDistortion:\n" << loader.depth().distortion << std::endl;

    std::cout << "\n===[Extrinsic]===" << std::endl;
    std::cout << "t:"     << loader.t() << std::endl;
    std::cout << "\nR:\n" << loader.R() << std::endl;
    std::cout << "\nT:\n" << loader.T() << std::endl;
    // clang-format on

    return 0;
}