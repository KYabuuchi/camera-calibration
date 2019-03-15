#include "calibration/calibration.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace Calibration
{
void StereoCalibration::init()
{
    m_src_images1.clear();
    m_src_images2.clear();
    m_corners1.clear();
    m_corners1.clear();
    m_object_points.clear();
}

void StereoCalibration::showParameters() const
{
    std::cout
        << "\nintrinsic1\n " << m_int_params1.intrinsic
        << "\ndistortion1\n " << m_int_params1.distortion
        << "\nresolution1\n " << m_int_params1.resolution
        << "\nRMS1\n " << m_int_params1.RMS
        << "\nintrinsic2\n " << m_int_params2.intrinsic
        << "\ndistortion2\n " << m_int_params2.distortion
        << "\nresolution2\n " << m_int_params2.resolution
        << "\nRMS2\n " << m_int_params2.RMS
        << "\n  rotation\n " << m_ext_params.rotation
        << "\n  translation\n " << m_ext_params.translation
        << std::endl;
}

void StereoCalibration::writeConfig(const std::string file_path) const
{
    cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
    cv::write(fs, "intrinsic1", m_int_params1.intrinsic);
    cv::write(fs, "distortion1", m_int_params1.distortion);
    cv::write(fs, "resolution1", m_int_params1.resolution);
    cv::write(fs, "RMS1", m_int_params1.RMS);
    cv::write(fs, "intrinsic2", m_int_params2.intrinsic);
    cv::write(fs, "distortion2", m_int_params2.distortion);
    cv::write(fs, "resolution2", m_int_params2.resolution);
    cv::write(fs, "RMS2", m_int_params2.RMS);
    cv::write(fs, "rotation", m_ext_params.rotation);
    cv::write(fs, "translation", m_ext_params.translation);
    fs.release();
    std::cout << "\n"
              << file_path << " has outputed" << std::endl;
}

bool StereoCalibration::readConfig(std::string file_path)
{
    cv::FileStorage fs(file_path, cv::FileStorage::READ);

    if (not fs.isOpened()) {
        std::cout << "[ERROR] can not open " << file_path << std::endl;
        return false;
    }

    fs["intrinsic1"] >> m_int_params1.intrinsic;
    fs["distortion1"] >> m_int_params1.distortion;
    fs["resolution1"] >> m_int_params1.resolution;
    fs["RMS1"] >> m_int_params1.RMS;
    fs["intrinsic2"] >> m_int_params2.intrinsic;
    fs["distortion2"] >> m_int_params2.distortion;
    fs["resolution2"] >> m_int_params2.resolution;
    fs["RMS2"] >> m_int_params2.RMS;
    fs["rotation"] >> m_ext_params.rotation;
    fs["translation"] >> m_ext_params.translation;
    fs.release();

    showParameters();
    return true;
}

void StereoCalibration::rectify(const cv::Mat& src_image1, const cv::Mat& src_image2, cv::Mat& dst_image1, cv::Mat& dst_image2) const
{
    cv::undistort(src_image1, dst_image1, m_int_params1.intrinsic, m_int_params1.distortion);
    cv::undistort(src_image2, dst_image2, m_int_params2.intrinsic, m_int_params2.distortion);
}

bool StereoCalibration::parser(
    const std::string& file_paths_file,
    std::vector<std::string>& file_paths1,
    std::vector<std::string>& file_paths2)
{
    std::ifstream ifs(file_paths_file);
    if (not ifs.is_open()) {
        std::cout << "[ERROR] can not open " << file_paths_file << std::endl;
        return false;
    }

    std::string dir = directorize(file_paths_file);

    while (not ifs.eof()) {
        std::string dual_file_path;
        std::getline(ifs, dual_file_path);
        if (not dual_file_path.empty()) {
            std::istringstream stream(dual_file_path);
            std::string file_path;
            getline(stream, file_path, ' ');
            file_paths1.push_back(dir + file_path);
            getline(stream, file_path, ' ');
            file_paths2.push_back(dir + file_path);
        }
    }
    ifs.close();
    return true;
}

int StereoCalibration::calcParameters(const std::string paths_file_path, const std::string yaml_path)
{
    // 画像読み込み
    std::vector<std::string> file_paths1, file_paths2;
    parser(paths_file_path, file_paths1, file_paths2);
    readImages(file_paths1, m_src_images1);
    readImages(file_paths2, m_src_images2);
    if (m_src_images1.size() != m_src_images2.size()) {
        std::cout << "not equal nomber of src_images" << std::endl;
        return -1;
    }

    // コーナー検出
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::resizeWindow(WINDOW_NAME, 960, 720);
    for (size_t i = 0; i < m_src_images1.size();) {
        std::vector<cv::Point2f> corners1, corners2;
        if (foundCorners(m_src_images1.at(i), corners1) and foundCorners(m_src_images2.at(i), corners2)) {
            m_corners1.push_back(corners1);
            m_corners2.push_back(corners2);
            i++;
        } else {
            m_src_images1.erase(m_src_images1.begin() + i);
            m_src_images2.erase(m_src_images2.begin() + i);
        }
    }
    cv::destroyWindow(WINDOW_NAME);

    const size_t N = m_src_images1.size();

    // 実世界での座標の登録
    for (size_t i = 0; i < N; i++) {
        std::vector<cv::Point3f> objects_tmp;
        for (int j = 0; j < ROW; j++) {
            for (int k = 0; k < COL; k++) {
                objects_tmp.push_back(cv::Point3f(static_cast<float>(j) * SIZE, static_cast<float>(k) * SIZE, 0.0f));
            }
        }
        m_object_points.push_back(objects_tmp);
    }

    ExtrinsicParams tmp1, tmp2;
    m_int_params1.resolution = m_src_images1.at(0).size();
    m_int_params2.resolution = m_src_images2.at(0).size();
    calibrate(m_int_params1, tmp1, m_corners1, m_object_points, m_int_params1.resolution);
    calibrate(m_int_params2, tmp2, m_corners2, m_object_points, m_int_params2.resolution);

    // 外部パラメータの計算
    cv::Mat xi = cv::Mat3f(1, 1);
    cv::Mat t = cv::Mat3f(1, 1);
    for (int i = 0; i < static_cast<int>(N); i++) {
        xi += tmp2.rotation.row(i) - tmp1.rotation.row(i);
        t += tmp2.translation.row(i) - tmp1.translation.row(i);
    }
    xi /= static_cast<double>(N);
    t /= static_cast<double>(N);

    cv::Rodrigues(xi, m_ext_params.rotation);
    m_ext_params.translation = t.reshape(1).t();

    showParameters();
    writeConfig(yaml_path);

    return 0;
}


}  // namespace Calibration
