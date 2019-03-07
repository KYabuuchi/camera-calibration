#include "calibration/calibration.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    cv::CommandLineParser parser(
        argc,
        argv,
        "{i input|../data/kinectv2_00/info.txt| path to paths file for input images.}"
        "{y yaml|./calibration.yaml| path to calibration file.}"
        "{m mode|calib|'calib' or 'rectify' or 'read'}"
        "{r row |10|row(行) size of chess board}"
        "{c col |7|col(列) size of chess board}"
        "{s size|19.5|size of each square side on chess board [mm]}"
        "{h help ?|false|Use like below.\n"
        "$./stereo -m=calib -i=../data/kinectv2_00/info.txt -y=./calibration.yaml -w=10 -c=7 -s=19.5 #calibration by using exsinting image\n"
        "$./stereo -m=rectify -y=../data/kinectv2_00/calibration.yaml #stream rectifyed images \n"
        "$./stereo -m=read -y=../data/kinectv2_00/calibration.yaml #read & print calibration data}");
    if (parser.get<bool>("help")) {
        parser.printMessage();
        return 0;
    }

    int row = parser.get<int>("row");
    int col = parser.get<int>("col");
    float size = parser.get<float>("size");

    Calibration::StereoCalibration calibration(row, col, size);

    std::string mode = parser.get<std::string>("mode");
    if (mode == "calib") {
        std::string paths_file_path = parser.get<std::string>("input");
        std::string yaml_path = parser.get<std::string>("yaml");
        calibration.calcParameters(paths_file_path, yaml_path);
    } else if (mode == "rectify") {
        std::string yaml_path = parser.get<std::string>("yaml");
        calibration.readYAML(yaml_path);

        cv::namedWindow("window1", cv::WINDOW_NORMAL);
        cv::namedWindow("window2", cv::WINDOW_NORMAL);
        cv::Mat src1 = cv::imread("../data/kinectv2_00/ir02.png");
        cv::Mat src2 = cv::imread("../data/kinectv2_00/rgb02.png");
        cv::Mat dst1, dst2, show1, show2;
        calibration.rectify(src1, src2, dst1, dst2);
        cv::hconcat(src1, dst1, show1);
        cv::hconcat(src2, dst2, show2);
        cv::imshow("window1", show1);
        cv::imshow("window2", show2);
        while (cv::waitKey(10) != 'q')
            ;
    } else if (mode == "read") {
        std::string yaml_path = parser.get<std::string>("yaml");
        calibration.readYAML(yaml_path);
    } else {
        parser.printMessage();
        std::cout << "invlid arguments" << std::endl;
    }

    return 0;
}
