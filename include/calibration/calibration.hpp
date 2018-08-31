#pragma once
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#define IMAGE_NUM_MAX (25) /* $B2hA|?t(B */
#define PAT_ROW (7)        /* $B%Q%?!<%s$N9T?t(B */
#define PAT_COL (10)       /* $B%Q%?!<%s$NNs?t(B */
#define PAT_SIZE (PAT_ROW * PAT_COL)
#define CHESS_SIZE (24.0f) /* $B%Q%?!<%s(B1$B%^%9$N(B1$BJU%5%$%:(B[mm] */


class CameraCalibration
{
    using vv_point2f = std::vector<std::vector<cv::Point2f>>;
    using vv_point3f = std::vector<std::vector<cv::Point3f>>;

public:
    CameraCalibration(std::string input_dir = "./", std::string output_dir = "./camera.xml", std::string device = "/dev/video0")
        : input_dir(input_dir), output_dir(output_dir), device(device)
    {
        init();
    }

    int calcParameter();
    int calcParameterWithPhoto();
    int adaptParameter();

private:
    std::string input_dir, output_dir, device;
    int valid_image_num;
    std::vector<cv::Mat> src_img;
    vv_point2f corners;
    vv_point3f object_points;

    // $BFbIt%Q%i%a!<%?(B
    // $B30It%Q%i%a!<%?(B
    // $BOD%Q%i%a!<%?(B
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat rotation = cv::Mat(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat(1, 3, CV_32FC1);
    cv::Mat distortion = cv::Mat(1, 4, CV_32FC1);

    void compareCorrection();
    void calcCalibration();
    bool foundCorners(cv::Mat img, std::string window_name);
    void readImage();
    void outputXML();
    void readXML();
    void init();
};
