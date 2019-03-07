#include "calibration/calibration.hpp"

namespace Calibration
{
bool AbstCalibration::foundCorners(const cv::Mat& img, vv_point2f& corners)
{
    std::vector<cv::Point2f> corners_tmp;
    std::cout << "...";
    if (cv::findChessboardCorners(img, {COL, ROW}, corners_tmp)) {
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
        corners_tmp,
        cv::Size(3, 3),
        cv::Size(-1, -1),
        cv::TermCriteria((cv::TermCriteria::COUNT | cv::TermCriteria::EPS), 20, 0.03));

    // コーナーを描画する
    cv::drawChessboardCorners(img, {COL, ROW}, corners_tmp, true);
    cv::imshow(WINDOW_NAME, img);
    cv::waitKey(200);

    corners.push_back(corners_tmp);
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

void AbstCalibration::calibrate(IntrinsicParams& int_params, ExtrinsicParams& ext_params, const vv_point2f& corners, const vv_point3f& points, const cv::Size& size)
{
    std::cout << "\nCALIBRATION START" << std::endl;
    int_params.RMS = cv::calibrateCamera(
        points,
        corners,
        size,
        int_params.intrinsic,
        int_params.distortion,
        ext_params.rotation,
        ext_params.translation);
    std::cout << "CALIBRATION FINISH\n"
              << std::endl;
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