#pragma once
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>


namespace Calibration
{
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
    CameraCalibration(int PAT_ROW = 7, int PAT_COL = 10, float CHESS_SIZE = 19.5f)
        : PAT_ROW(PAT_ROW), PAT_COL(PAT_COL), PAT_SIZE(PAT_ROW * PAT_COL), CHESS_SIZE(CHESS_SIZE) {}

    // 既存の画像群からカメラパラメータを計算し，XMLファイルに保存する
    int calcParameters(std::string images_dir, std::string xml_dir);

    // カメラで写真をとり，画像群からパラメータを計算し，XMLファイルに保存する
    int calcParametersWithPhoto(std::string images_dir, std::string xml_dir, std::string device_dir);

    // パラメータを適応して，カメラからの出力を歪ませる
    int adaptParameters(std::string xml_dir, std::string device_dir);

    // XMLファイルを読み込み，カメラパラメータを取得する
    CameraParameters readParameters(std::string xml_dir);

    // カメラパラメータを出力する
    void showParameters();

private:
    int PAT_ROW = 7;                   // コーナーの行数
    int PAT_COL = 10;                  // コーナーの列数
    int PAT_SIZE = PAT_ROW * PAT_COL;  // コーナーの数
    float CHESS_SIZE = 19.5f;          // 1マスのサイズ(mm)

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
