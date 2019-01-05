#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace Camera
{
struct Parameters {
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat rotation = cv::Mat(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat(1, 3, CV_32FC1);
    cv::Mat distortion = cv::Mat(1, 4, CV_32FC1);
    double RMS = -1;
};

class Calibration
{
    using vv_point2f = std::vector<std::vector<cv::Point2f>>;
    using vv_point3f = std::vector<std::vector<cv::Point3f>>;

public:
    // 行数,列数,寸法[mm]
    Calibration(int row, int col, float size)
        : ROW(row), COL(col), NUM(row * col), SIZE(size) {}

    // 既存の画像群からカメラパラメータを計算し，XMLファイルに保存する
    int calcParameters(const std::string paths_file_path, const std::string xml_path);

    // カメラで写真をとり，画像群からパラメータを計算し，XMLファイルに保存する
    int calcParametersWithPhoto(std::string output_dir, const std::string xml_path, const std::string device_path);

    // XMLファイルを読み込み，パラメータを取得する
    Parameters readParameters(std::string xml_path);

    // パラメータを取得して，XMLファイルを書き込む
    std::string writeParameters(const Parameters parameters);

    // パラメータを取得する
    Parameters getParameters() const;

    // パラメータを標準出力に流す
    void showParameters() const;

    // 入力画像を校正して返す
    cv::Mat rectify(const cv::Mat source_image) const;

private:
    const int ROW;     // コーナーの行数
    const int COL;     // コーナーの列数
    const int NUM;     // コーナーの数
    const float SIZE;  // 1マスのサイズ(mm)

    std::vector<cv::Mat> m_src_imgs;
    vv_point2f m_corners;
    vv_point3f m_object_points;
    Parameters m_parameters;

    void compareCorrection(std::string device_path);
    void calibrate();
    bool foundCorners(cv::Mat img, std::string window_name);
    void readImage(std::string paths_file_path);
    void outputXML(std::string xml_path);
    bool readXML(std::string xml_path);
    void init();
};

}  // namespace Camera
