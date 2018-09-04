#include "calibration/calibration.hpp"
#include <iostream>

int main(/*int argc, char* argv[]*/)
{
    Calibration::CameraCalibration camera;

    int mode = 0;
    std::string images_dir, xml_dir, device_dir;

    while (1) {
        std::cout << "\n"
                  << "0: quit\n"
                  << "1: photo -> calc parameters -> output to XML\n"
                  << "2: calc parameters from pictures -> output to XML\n"
                  << "3: open XML -> rectify video\n"
                  << "4: open XML -> show parameters\n"
                  << " Press number of mode" << std::endl;

        std::cin >> mode;

        switch (mode) {
        case 0:
            std::cout << "quit." << std::endl;
            return 0;
            break;
        case 1:
            std::cout << "input 'images_dir', 'xml_dir.xml', 'device_dir'" << std::endl;
            std::cin >> images_dir >> xml_dir >> device_dir;
            camera.calcParametersWithPhoto(images_dir, xml_dir, device_dir);
            break;
        case 2:
            std::cout << "input 'images_dir', 'xml_dir.xml'" << std::endl;
            std::cin >> images_dir >> device_dir;
            camera.calcParameters(images_dir, xml_dir);
            break;
        case 3:
            std::cout << "input 'xml_dir.xml', 'device_dir'" << std::endl;
            std::cin >> xml_dir >> device_dir;
            camera.adaptParameters(xml_dir, device_dir);
            break;
        case 4:
            std::cout << "input 'xml_dir.xml'" << std::endl;
            std::cin >> xml_dir;
            camera.readParameters(xml_dir);
            break;
        default:
            std::cerr << mode << "is not supported." << std::endl;
            return -1;
            break;
        }
    }

    return 0;
}
