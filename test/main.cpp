#include "calibration/calibration.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(
        argc,
        argv,
        "{i input|../reference/pseye_000/files.txt| path to paths file for input images.}"
        "{o output|./|path to directory for captured images.}"
        "{c calib calibration xml x|calibration.xml| path to calibration file.}"
        "{d device|| path to camera device.}"
        "{m mode|calib|'calib' or 'photocalib' or 'rectify' or 'read'}"
        "{r row |10|row(行) size of chess board}"
        "{c col |7|col(列) size of chess board}"
        "{s size|19.5|size of each square side on chess board [mm]}"
        "{h help ?|false|Use like below.\n"
        "$./sample -m=calib -i=../reference/pseye/files.txt -w=10 -c=7 -s=19.5 #calibration by using exsinting image\n"
        "$./sample -m=photocalib -d=/dev/video0 -o=./ -c=calibration.xml -w=10 -c=7 -s=19.5 #shot some pictures & calibration by using them\n"
        "$./sample -m=rectify -v=/dev/video0 -c=calibration.xml #rectify video stream by calibration data\n"
        "$./sample -m=read -c=calibration.xml #read & print calibration data}");

    if (parser.get<bool>("help")) {
        parser.printMessage();
        return 0;
    }
    Camera::Calibration camera(5, 7, 19.5f);

    std::string mode = parser.get<std::string>("mode");

    if (mode == "calib") {
        std::string paths_file_path = parser.get<std::string>("input");
        std::string calibration_path = parser.get<std::string>("calibration");
        camera.calcParameters(paths_file_path, calibration_path);
    } else if (mode == "photocalib") {
        std::string device_path = parser.get<std::string>("device");
        std::string calibration_path = parser.get<std::string>("calibration");
        std::string output_dir = parser.get<std::string>("output");
        camera.calcParametersWithPhoto(output_dir, calibration_path, device_path);
    } else if (mode == "rectify") {
        std::string device_path = parser.get<std::string>("device");
        std::string calibration_path = parser.get<std::string>("calibration");
        camera.readParameters(calibration_path);
        cv::VideoCapture video(device_path);
        if (not video.isOpened()) {
            std::cout << "can not open " << device_path << std::endl;
            return -1;
        }
        cv::namedWindow("window", cv::WINDOW_NORMAL);
        while (true) {
            cv::Mat src, dst, show;
            video >> src;
            dst = camera.rectify(src);
            cv::hconcat(src, dst, show);
            cv::imshow("window", show);
            cv::waitKey(10);
        }

    } else if (mode == "load") {
        std::string calibration_path = parser.get<std::string>("calibration");
        camera.readParameters(calibration_path);
        camera.showParameters();
    } else {
        parser.printMessage();
        std::cout << "invlid arguments" << std::endl;
    }

    return 0;
}
