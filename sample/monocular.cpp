#include "calibration/calibration.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(
        argc,
        argv,
        "{i input|../data/pseye_00/info.txt| path to paths file for input images.}"
        "{y yaml|config.yaml| path to config file.}"
        "{d device v video|/dev/video0| path to camera device.}"
        "{m mode|calib|'calib' or 'rectify' or 'read'}"
        "{r row |10|row(行) size of chess board}"
        "{c col |7|col(列) size of chess board}"
        "{s size|0.0195|size of each square side on chess board [mm]}"
        "{h help ?|false|Use like below.\n"
        "$./monocular -m=calib -i=../data/pseye_00/info.txt -y=config.yaml -w=10 -c=7 -s=0.0195 #calibration by using exsinting image\n"
        "$./monocular -m=rectify -y=../data/pseye_00/config.yaml -d=/dev/video0 #stream rectifyed images \n"
        "$./monocular -m=read -y=../data/pseye_00/config.yaml #read & print calibration data}");
    if (parser.get<bool>("help")) {
        parser.printMessage();
        return 0;
    }

    int row = parser.get<int>("row");
    int col = parser.get<int>("col");
    float size = parser.get<float>("size");

    Calibration::MonocularCalibration calibration(row, col, size);

    std::string mode = parser.get<std::string>("mode");
    if (mode == "calib") {
        std::string paths_file_path = parser.get<std::string>("input");
        std::string yaml_path = parser.get<std::string>("yaml");
        calibration.calcParameters(paths_file_path, yaml_path);
    } else if (mode == "rectify") {
        std::string device_path = parser.get<std::string>("device");
        std::string yaml_path = parser.get<std::string>("yaml");
        calibration.readConfig(yaml_path);

        cv::VideoCapture video(device_path);
        if (not video.isOpened()) {
            std::cout << "can not open " << device_path << std::endl;
            return -1;
        }
        cv::namedWindow("window", cv::WINDOW_NORMAL);
        while (true) {
            cv::Mat src, dst, show;
            video >> src;
            calibration.rectify(src, dst);
            cv::hconcat(src, dst, show);
            cv::imshow("window", show);
            if (cv::waitKey(10) == 'q')
                break;
        }
    } else if (mode == "read") {
        std::string yaml_path = parser.get<std::string>("yaml");
        calibration.readConfig(yaml_path);
    } else {
        parser.printMessage();
        std::cout << "invlid arguments" << std::endl;
    }

    return 0;
}
