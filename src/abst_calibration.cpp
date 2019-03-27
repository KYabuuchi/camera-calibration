#include "calibration/calibration.hpp"

namespace Calibration
{
bool AbstCalibration::foundCorners(const cv::Mat& img, std::vector<cv::Point2f>& corners)
{
    std::cout << "...";
    if (cv::findChessboardCorners(img, {COL, ROW}, corners)) {
        std::cout << "ok" << std::endl;
    } else {
        std::cerr << "fail" << std::endl;
        cv::imshow(WINDOW_NAME, img);
        cv::waitKey(200);
        return false;
    }

    cv::Mat src_gray = cv::Mat(img.size(), CV_8UC1);
    cv::cvtColor(img, src_gray, cv::COLOR_BGR2GRAY);

    // subPixel精度で計算する
    cv::cornerSubPix(
        src_gray,
        corners,
        cv::Size(3, 3),
        cv::Size(-1, -1),
        cv::TermCriteria((cv::TermCriteria::COUNT | cv::TermCriteria::EPS), 20, 0.03));

    // コーナーを描画する
    cv::drawChessboardCorners(img, {COL, ROW}, corners, true);
    cv::imshow(WINDOW_NAME, img);
    cv::waitKey(200);

    return true;
}

void AbstCalibration::readImages(const std::vector<std::string>& file_paths, std::vector<cv::Mat>& src_images)
{
    src_images.clear();
    for (const std::string& file_path : file_paths) {
        cv::Mat src_img = cv::imread(file_path, cv::IMREAD_COLOR);
        if (src_img.empty()) {
            std::cout << "[ERROR] cannot open : " << file_path << std::endl;
        } else {
            std::cout << "completely opened: " << file_path << std::endl;
            src_images.push_back(src_img);
        }
    }
    std::cout << " found " << src_images.size() << " images " << std::endl;
}

std::pair<cv::Mat, cv::Mat> AbstCalibration::calibrate(IntrinsicParams& int_params, const vv_point2f& corners, const vv_point3f& points, const cv::Size& size)
{
    cv::Mat tmp_intrinsic, tmp_distortion;
    cv::Mat tmp_rotation, tmp_translation;

    std::cout << "\nCALIBRATION START" << std::endl;
    int_params.RMS = cv::calibrateCamera(
        points,
        corners,
        size,
        tmp_intrinsic,
        tmp_distortion,
        tmp_rotation,
        tmp_translation);
    std::cout << "CALIBRATION FINISH\n"
              << std::endl;

    tmp_intrinsic.convertTo(int_params.intrinsic, CV_32FC1);
    tmp_distortion.convertTo(int_params.distortion, CV_32FC1);
    tmp_rotation.convertTo(tmp_rotation, CV_32FC3);
    tmp_translation.convertTo(tmp_translation, CV_32FC3);
    return {tmp_rotation, tmp_translation};
}

std::string AbstCalibration::directorize(std::string file_path)
{
    while (true) {
        if (file_path.empty() or *(file_path.end() - 1) == '/')
            break;
        file_path.erase(file_path.end() - 1);
    }
    return file_path;
}


}  // namespace Calibration