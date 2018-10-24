#include "calibration/calibration.hpp"
#include <iostream>

int main(/*int argc, char* argv[]*/)
{
    Calibration::CameraCalibration camera;

    int mode = 0;
    std::string images_dir, xml_dir, device_dir;

    while (1) {
        std::cout << "\n"
                  << " Input number of option\n"
                  << "0: quit\n"
                  << "1: photo -> calc parameters -> output to XML\n"
                  << "2: calc parameters from pictures -> output to XML\n"
                  << "3: open XML -> rectify video\n"
                  << "4: open XML -> show parameters\n"
                  << std::endl;

        std::cin >> mode;

        switch (mode) {
        case 0:
            std::cout << "quit." << std::endl;
            return 0;
            break;
        case 1:
            std::cout << "Input 'output_images_dir', 'xml_dir.xml', 'device_dir'" << std::endl;
            std::cout << " e.g.) ./tmp/ ./out.xml /dev/video0" << std::endl;
            std::cin >> images_dir >> xml_dir >> device_dir;
            camera.calcParametersWithPhoto(images_dir, xml_dir, device_dir);
            break;
        case 2:
            std::cout << "Input 'input_images_dir', 'xml_dir.xml'" << std::endl;
            std::cout << " e.g.) ../reference/pseye/ ./out.xml" << std::endl;
            std::cin >> images_dir >> xml_dir;
            camera.calcParameters(images_dir, xml_dir);
            break;
        case 3:
            std::cout << "Input 'xml_dir.xml', 'device_dir'" << std::endl;
            std::cout << " e.g.)  ./out.xml /dev/video0" << std::endl;
            std::cin >> xml_dir >> device_dir;
            camera.adaptParameters(xml_dir, device_dir);
            break;
        case 4:
            std::cout << "Input 'xml_dir.xml'" << std::endl;
            std::cout << " e.g.) ../reference/pseye/camera_pseye.xml" << std::endl;
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
