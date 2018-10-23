#include "calibration/calibration.hpp"

namespace Calibration
{

int CameraCalibration::adaptParameters(const std::string xml_dir, const std::string device_dir)
{
    // 初期化
    init();

    // XMLファイルの読み込み
    readXML(xml_dir);

    // 歪動画と補正動画の比較
    compareCorrection(device_dir);

    return 0;
}

int CameraCalibration::calcParameters(const std::string images_dir, const std::string xml_dir)
{
    // 初期化
    init();

    // 画像の読み込み
    readImage(images_dir);

    // コーナー検出
    const std::string WINDOW_NAME = "Calibration";
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    int tmp_image_num = valid_image_num;

    for (int i = 0; i < tmp_image_num; i++) {
        foundCorners(src_imgs.at(i), WINDOW_NAME);
    }
    cv::destroyWindow(WINDOW_NAME);

    // 実世界での座標の登録
    for (int i = 0; i < valid_image_num; i++) {
        std::vector<cv::Point3f> objects_tmp;

        for (int j = 0; j < PAT_ROW; j++) {
            for (int k = 0; k < PAT_COL; k++) {
                objects_tmp.push_back(cv::Point3f(static_cast<float>(j) * CHESS_SIZE, static_cast<float>(k) * CHESS_SIZE, 0.0f));
            }
        }
        object_points.push_back(objects_tmp);
    }

    // パラメータの推定
    calibrate();

    // XMLファイルへ出力
    outputXML(xml_dir);

    std::cout << "\n[CameraCalibration::calcParameters return]" << std::endl;
    return 0;
}

int CameraCalibration::calcParametersWithPhoto(const std::string images_dir, const std::string xml_dir, const std::string device_dir)
{
    // 初期化
    init();

    // 動画の準備
    cv::VideoCapture video(device_dir);
    const std::string WINDOW_NAME = "Calibration";
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    // キー入力のたびに画像を保存
    valid_image_num = IMAGE_NUM_MAX;
    for (int i = 0; i < valid_image_num;) {
        std::cout << "PRESS KEY when the camera gazes tha pattern. "
                  << (valid_image_num - i) << " left" << std::endl;

        cv::Mat src;
        while (cv::waitKey(10) == -1) {
            video >> src;
            std::stringstream ss;
            ss << images_dir + "/calib_img" << std::setw(2) << std::setfill('0') << i << ".png";
            cv::imwrite(ss.str(), src);
            cv::imshow(WINDOW_NAME, src);
        }

        if (foundCorners(src, WINDOW_NAME)) {
            i++;
            src_imgs.push_back(src);
        } else {
            std::cout << "NOT found the pattern completely" << std::endl;
        }
    }
    video.release();
    cv::destroyWindow(WINDOW_NAME);

    // 実世界での座標の登録
    for (int i = 0; i < valid_image_num; i++) {
        std::vector<cv::Point3f> objects_tmp;

        for (int j = 0; j < PAT_ROW; j++) {
            for (int k = 0; k < PAT_COL; k++) {
                objects_tmp.push_back(cv::Point3f(static_cast<float>(j) * CHESS_SIZE, static_cast<float>(k) * CHESS_SIZE, 0.0f));
            }
        }
        object_points.push_back(objects_tmp);
    }

    // パラメータの推定
    calibrate();

    // XMLファイルへ出力
    outputXML(xml_dir);

    // 歪動画と補正動画の比較
    compareCorrection(device_dir);

    std::cout << "[CameraCalibration::calcParametersWithPhoto return]" << std::endl;
    return 0;
}

// カメラパラメータを計算する
void CameraCalibration::calibrate()
{
    std::cout << "[CALIBRATION START]" << std::endl;
    params.RMS = cv::calibrateCamera(
        object_points,
        corners,
        src_imgs.at(0).size(),
        params.intrinsic,
        params.distortion,
        params.rotation,
        params.translation);
    std::cout << "[CALIBRATION FINISH]" << std::endl;

    showParameters();
}

// コーナーをsubPixel精度で検出する
bool CameraCalibration::foundCorners(cv::Mat img, const std::string WINDOW_NAME)
{
    std::vector<cv::Point2f> corners_tmp;
    std::cout << "...";

    if (cv::findChessboardCorners(img, {PAT_COL, PAT_ROW}, corners_tmp)) {
        std::cout << "ok" << std::endl;
    } else {
        std::cerr << "fail" << std::endl;
        valid_image_num--;
        return false;
    }

    cv::Mat src_gray = cv::Mat(img.size(), CV_8UC1, 1);
    cv::cvtColor(img, src_gray, cv::COLOR_BGR2GRAY);

    // subPixel精度で計算する
    cv::cornerSubPix(
        src_gray,
        corners_tmp,
        cv::Size(3, 3),
        cv::Size(-1, -1),
        cv::TermCriteria((cv::TermCriteria::COUNT | cv::TermCriteria::EPS), 20, 0.03));

    // コーナーを描画する
    cv::drawChessboardCorners(img, {PAT_COL, PAT_ROW}, corners_tmp, true);
    cv::imshow(WINDOW_NAME, img);

    cv::waitKey(200);
    corners.push_back(corners_tmp);

    return true;
}

// 指定のディレクトリにある画像の読み込み
void CameraCalibration::readImage(const std::string images_dir)
{
    std::cout << "[ reading stored images at " << images_dir << " ]" << std::endl;

    src_imgs.clear();

    int image_num = 0;
    for (int i = 0; i < IMAGE_NUM_MAX; i++) {
        std::stringstream ss;
        ss << images_dir << "/calib_img" << std::setw(2) << std::setfill('0') << i << ".png";
        cv::Mat tmp_img = cv::imread(ss.str(), cv::IMREAD_COLOR);
        if (tmp_img.data == NULL) {
            std::cerr << "cannot open : " << ss.str() << std::endl;
        } else {
            std::cerr << "have opened: " << ss.str() << std::endl;
            src_imgs.push_back(tmp_img);
            image_num++;
        }
    }
    valid_image_num = image_num;
    std::cout << "[ found " << valid_image_num << " images ]" << std::endl;
}

// XMLファイルを読み込み
bool CameraCalibration::readXML(const std::string xml_dir)
{
    std::cout << "[ open " << xml_dir << " ] " << std::endl;
    cv::FileStorage fs(xml_dir, cv::FileStorage::READ);

    if (not fs.isOpened()) {
        std::cout << "can not open " << xml_dir << std::endl;
        return false;
    }
    fs["intrinsic"] >> params.intrinsic;
    fs["distortion"] >> params.distortion;
    fs["rotation"] >> params.rotation;
    fs["translation"] >> params.translation;
    fs["RMS"] >> params.RMS;

    showParameters();
    fs.release();

    return true;
}

// 歪動画と補正動画の比較
void CameraCalibration::compareCorrection(const std::string device_dir)
{
    std::cout << "[compare corrected image]" << std::endl;
    cv::VideoCapture video(device_dir);
    const std::string show_window_name = "show image";
    cv::namedWindow(show_window_name, cv::WINDOW_AUTOSIZE);
    std::cout << params.intrinsic << std::endl;

    while (cv::waitKey(10) == -1) {
        cv::Mat src, dst, merge;
        video.read(src);
        cv::undistort(src, dst, params.intrinsic, params.distortion);
        cv::putText(src, "src", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 200), 2, cv::LINE_AA);
        cv::putText(dst, "dst", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 200), 2, cv::LINE_AA);
        cv::hconcat(src, dst, merge);
        cv::imshow(show_window_name, merge);
    }
    cv::destroyWindow(show_window_name);
}

// XMLファイルの出力
void CameraCalibration::outputXML(const std::string xml_dir)
{
    cv::FileStorage fs(xml_dir, cv::FileStorage::WRITE);
    cv::write(fs, "intrinsic", params.intrinsic);
    cv::write(fs, "distortion", params.distortion);
    cv::write(fs, "rotation", params.rotation);
    cv::write(fs, "translation", params.translation);
    cv::write(fs, "RMS", params.RMS);
    fs.release();
    std::cout << "['" << xml_dir << "' has outputed]" << std::endl;
}

// 変数の初期化(いうていらない)
void CameraCalibration::init()
{
    src_imgs.clear();
    corners.clear();
    object_points.clear();
    std::cout << "[CameraCalibration::init return]" << std::endl;
}

// XMLファイルを読み込んで，表示する
CameraParameters CameraCalibration::readParameters(const std::string xml_dir)
{
    if (readXML(xml_dir)) {
        showParameters();
        return params;
    } else {
        std::cout << "return initial struct" << std::endl;
        return CameraParameters();
    }
}

// パラメータの表示
void CameraCalibration::showParameters()
{
    std::cout
        << "\n[intrinsic] " << params.intrinsic
        << "\n[distortion] " << params.distortion
        << "\n[rotation] " << params.rotation
        << "\n[translation] " << params.translation
        << "\n[RMS] " << params.RMS << std::endl;
}

}  // namespace Calibration
