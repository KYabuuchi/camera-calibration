#include "calibration/calibration.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace Calibration
{
void MonocularCalibration::init()
{
    m_src_images.clear();
    m_corners.clear();
    m_object_points.clear();
}

void MonocularCalibration::showParameters() const
{
    std::cout
        << "\nintrinsic\n " << m_int_params.intrinsic
        << "\ndistortion\n " << m_int_params.distortion
        << "\nresolusion\n " << m_int_params.resolution
        << "\nRMS\n " << m_int_params.RMS << std::endl;
}

void MonocularCalibration::writeConfig(const std::string file_path) const
{
    cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
    cv::write(fs, "intrinsic", m_int_params.intrinsic);
    cv::write(fs, "distortion", m_int_params.distortion);
    cv::write(fs, "resolution", m_int_params.resolution);
    cv::write(fs, "RMS", m_int_params.RMS);
    fs.release();
    std::cout << "\n"
              << file_path << " has outputed" << std::endl;
}

bool MonocularCalibration::readConfig(std::string file_path)
{
    cv::FileStorage fs(file_path, cv::FileStorage::READ);

    if (not fs.isOpened()) {
        std::cout << "[ERROR] can not open " << file_path << std::endl;
        return false;
    }

    fs["intrinsic"] >> m_int_params.intrinsic;
    fs["distortion"] >> m_int_params.distortion;
    fs["resolution"] >> m_int_params.resolution;
    fs["RMS"] >> m_int_params.RMS;
    fs.release();

    showParameters();
    return true;
}

void MonocularCalibration::rectify(const cv::Mat& src_image, cv::Mat& dst_image) const
{
    cv::undistort(src_image, dst_image, m_int_params.intrinsic, m_int_params.distortion);
}

bool MonocularCalibration::parser(const std::string& file_paths_file, std::vector<std::string>& file_paths)
{
    std::ifstream ifs(file_paths_file);
    if (not ifs.is_open()) {
        std::cout << "[ERROR] can not open " << file_paths_file << std::endl;
        return false;
    }

    std::string dir = directorize(file_paths_file);
    while (not ifs.eof()) {
        std::string file_path;
        std::getline(ifs, file_path);
        if (not file_path.empty())
            file_paths.push_back(dir + file_path);
    }
    ifs.close();
    return true;
}

int MonocularCalibration::calcParameters(const std::string paths_file_path, const std::string yaml_path)
{
    // 画像読み込み
    std::vector<std::string> file_paths;
    parser(paths_file_path, file_paths);
    readImages(file_paths, m_src_images);

    // コーナー検出
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WINDOW_NAME, 960, 720);
    for (size_t i = 0; i < m_src_images.size();) {
        std::vector<cv::Point2f> corners;
        if (not foundCorners(m_src_images.at(i), corners)) {
            m_src_images.erase(m_src_images.begin() + i);
        } else {
            m_corners.push_back(corners);
            i++;
        }
    }
    cv::destroyWindow(WINDOW_NAME);

    // 実世界での座標の登録
    for (size_t i = 0; i < m_src_images.size(); i++) {
        std::vector<cv::Point3f> objects_tmp;
        for (int j = 0; j < ROW; j++) {
            for (int k = 0; k < COL; k++) {
                objects_tmp.push_back(cv::Point3f(static_cast<float>(j) * SIZE, static_cast<float>(k) * SIZE, 0.0f));
            }
        }
        m_object_points.push_back(objects_tmp);
    }

    m_int_params.resolution = m_src_images.at(0).size();
    calibrate(m_int_params, m_corners, m_object_points, m_int_params.resolution);

    showParameters();

    writeConfig(yaml_path);

    return 0;
}

}  // namespace Calibration
