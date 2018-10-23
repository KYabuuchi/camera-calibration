#pragma once
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>


namespace Calibration
{
constexpr int IMAGE_NUM_MAX = 25;
constexpr int PAT_ROW = 7;
constexpr int PAT_COL = 10;
constexpr int PAT_SIZE = PAT_ROW * PAT_COL;
constexpr float CHESS_SIZE = 19.5f;

struct CameraParameters {
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat rotation = cv::Mat(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat(1, 3, CV_32FC1);
    cv::Mat distortion = cv::Mat(1, 4, CV_32FC1);
    double RMS = 0;
};

class CameraCalibration
{
    using vv_point2f = std::vector<std::vector<cv::Point2f>>;
    using vv_point3f = std::vector<std::vector<cv::Point3f>>;

public:
    CameraCalibration() = default;

    int calcParameters(std::string images_dir, std::string xml_dir);

    int calcParametersWithPhoto(std::string images_dir, std::string xml_dir, std::string device_dir);

    int adaptParameters(std::string xml_dir, std::string device_dir);

    CameraParameters readParameters(std::string xml_dir);

    void showParameters();

private:
    int valid_image_num;
    std::vector<cv::Mat> src_imgs;
    vv_point2f corners;
    vv_point3f object_points;

    CameraParameters params;

    void compareCorrection(std::string device_dir);
    void calibrate();
    bool foundCorners(cv::Mat img, std::string window_name);
    void readImage(std::string images_dir);
    void outputXML(std::string xml_dir);
    bool readXML(std::string xml_dir);
    void init();
};

}  // namespace Calibration
