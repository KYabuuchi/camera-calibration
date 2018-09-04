#pragma once
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>


namespace Calibration
{
constexpr int IMAGE_NUM_MAX = 25;  // 画像数
constexpr int PAT_ROW = 7;         // パターンの行数
constexpr int PAT_COL = 10;        // パターンの列数
constexpr int PAT_SIZE = PAT_ROW * PAT_COL;
constexpr float CHESS_SIZE = 19.5f;  // パターン1マスの1辺サイズ[mm]

struct CameraParameters {
    // 内部パラメータ
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    // 外部パラメータ
    cv::Mat rotation = cv::Mat(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat(1, 3, CV_32FC1);
    // 歪パラメータ
    cv::Mat distortion = cv::Mat(1, 4, CV_32FC1);
    // 投影誤差の Root Mean Square
    double RMS = 0;
};

class CameraCalibration
{
    using vv_point2f = std::vector<std::vector<cv::Point2f>>;
    using vv_point3f = std::vector<std::vector<cv::Point3f>>;

public:
    CameraCalibration() = default;

    // ディレクトリ内にある画像を利用し，パラメータを推定して，XMLへ出力する．
    int calcParameters(std::string images_dir, std::string xml_dir);

    // 写真を撮影して，ディレクトリ内に画像を保存，そのあとパラメータを推定してXMLへ出力する．さらに歪み補正済み画像を比較する．
    int calcParametersWithPhoto(std::string images_dir, std::string xml_dir, std::string device_dir);

    // XMLを読み込み，カメラからの映像を校正してウィンドウに表示する
    int adaptParameters(std::string xml_dir, std::string device_dir);

    // XMLを読み込み，パラメータを返す
    CameraParameters readParameters(std::string xml_dir);

    // 今もっているパラメータを表示する．
    void showParameters();

private:
    // 有効な画像の枚数
    int valid_image_num;
    std::vector<cv::Mat> src_imgs;
    vv_point2f corners;
    vv_point3f object_points;

    // 我々が欲しいもの
    CameraParameters params;

    // 補正した画像と生画像を比較する
    void compareCorrection(std::string device_dir);
    // キャリブレーションを司る
    void calibrate();
    // 画像からコーナーを検出して描画する
    bool foundCorners(cv::Mat img, std::string window_name);
    // 与えられたディレクトリから画像を探す
    void readImage(std::string images_dir);
    // パラメータをXMLへ出力する
    void outputXML(std::string xml_dir);
    // XMLからパラメータを得る
    bool readXML(std::string xml_dir);
    // 変数の初期化
    void init();
};

}  // namespace Calibration
