#include "calibration/calibration.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc == 1) {
        std::cout << "Please Set some arguments\n"
                  << "arg1:mode, arg2:input_dir, arg3:output_dir,arg4:device\n"
                  << "for example './calibrate 2 hoge.xml ../ref/pseye.xml /dev/video0'\n"
                  << "0: stream video & calibrate & output as XML & show collected video\n"
                  << "1: calibrate & output as XML\n"
                  << "2: read XML & show collected video\n"
                  << "other: not supoorted \n"
                  << std::endl;
        return -1;
    }

    int mode = std::atoi(argv[1]);
    CameraCalibration camera(argv[2], argv[3], argv[4]);

    if (mode == 0) {
        camera.calcParameterWithPhoto();
    } else if (mode == 1) {
        camera.calcParameter();
    } else if (mode == 2) {
        camera.adaptParameter();
    } else {
        std::cerr << "not supported." << std::endl;
        return -1;
    }

    return 0;
}
