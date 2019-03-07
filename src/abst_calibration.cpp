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

void AbstCalibration::readImages(const std::string paths_file_path, std::vector<cv::Mat>& src_images)
{
    src_images.clear();

    std::ifstream ifs(paths_file_path);
    if (not ifs.is_open()) {
        std::cout << "[ERROR] can not open " << paths_file_path << std::endl;
        return;
    }

    // ファイル名だけ削除
    std::string dir = paths_file_path;
    while (true) {
        if (dir.empty() or *(dir.end() - 1) == '/')
            break;
        dir.erase(dir.end() - 1);
    }

    std::string file_name;
    std::getline(ifs, file_name);
    while (not file_name.empty()) {
        cv::Mat src_img = cv::imread(dir + file_name, cv::IMREAD_COLOR);
        if (src_img.empty()) {
            std::cout << "[ERROR] cannot open : " << dir + file_name << std::endl;
        } else {
            std::cout << "completely opened: " << dir + file_name << std::endl;
            src_images.push_back(src_img);
        }
        std::getline(ifs, file_name);
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

}  // namespace Calibration