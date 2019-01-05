#include "calibration/calibration.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace Camera
{
Parameters Calibration::getParameters() const
{
    return m_parameters;
}

int Calibration::calcParameters(std::string paths_file_path, std::string xml_path)
{
    readImage(paths_file_path);

    // コーナー検出
    const std::string WINDOW_NAME = "Corner Detection";
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    for (size_t i = 0; i < m_src_imgs.size();) {
        if (not foundCorners(m_src_imgs.at(i), WINDOW_NAME)) {
            m_src_imgs.erase(m_src_imgs.begin() + i);
        } else {
            i++;
        }
    }
    cv::destroyWindow(WINDOW_NAME);

    // 実世界での座標の登録
    for (size_t i = 0; i < m_src_imgs.size(); i++) {
        std::vector<cv::Point3f> objects_tmp;
        for (int j = 0; j < ROW; j++) {
            for (int k = 0; k < COL; k++) {
                objects_tmp.push_back(cv::Point3f(static_cast<float>(j) * SIZE, static_cast<float>(k) * SIZE, 0.0f));
            }
        }
        m_object_points.push_back(objects_tmp);
    }

    calibrate();

    showParameters();

    outputXML(xml_path);

    return 0;
}

int Calibration::calcParametersWithPhoto(std::string output_dir, const std::string xml_path, const std::string device_path)
{
    // 動画の準備
    cv::VideoCapture video(device_path);
    const std::string WINDOW_NAME = "Calibration";
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    bool loop = true;
    int valid_image_num = 0;
    if (*(output_dir.end() - 1) != '/')
        output_dir.push_back('/');
    std::ofstream ofs(output_dir + "files.txt");

    // キー入力のたびに画像を保存
    while (loop) {
        std::cout << "PRESS 's' when the camera gazes tha pattern. PRESS 'q' to escape.\n"
                  << valid_image_num << " images are saved " << std::endl;

        cv::Mat src;
        int key = -1;
        while (key == -1) {
            key = cv::waitKey(10);
            video >> src;
            cv::imshow(WINDOW_NAME, src);
        }

        switch (key) {
        case 's':
            if (foundCorners(src, WINDOW_NAME)) {
                valid_image_num++;
                m_src_imgs.push_back(src);

                // 保存
                std::stringstream ss;
                ss << output_dir + "/image_" << std::setw(3) << std::setfill('0') << valid_image_num << ".png";
                ofs << ss.str() << std::endl;
                cv::imwrite(ss.str(), src);
            } else {
                std::cout << "NOT found the pattern completely" << std::endl;
            }

            break;
        case 'q':
            loop = false;
            break;
        default:
            break;
        }
    }
    video.release();
    cv::destroyWindow(WINDOW_NAME);

    // 実世界での座標の登録
    for (int i = 0; i < valid_image_num; i++) {
        std::vector<cv::Point3f> objects_points;
        for (int j = 0; j < ROW; j++) {
            for (int k = 0; k < COL; k++) {
                objects_points.push_back(cv::Point3f(static_cast<float>(j) * SIZE, static_cast<float>(k) * SIZE, 0.0f));
            }
        }
        m_object_points.push_back(objects_points);
    }

    calibrate();

    showParameters();

    outputXML(xml_path);

    return 0;
}

void Calibration::calibrate()
{
    std::cout << "\nCALIBRATION START" << std::endl;
    m_parameters.RMS = cv::calibrateCamera(
        m_object_points,
        m_corners,
        m_src_imgs.at(0).size(),
        m_parameters.intrinsic,
        m_parameters.distortion,
        m_parameters.rotation,
        m_parameters.translation);
    std::cout << "CALIBRATION FINISH\n"
              << std::endl;
}

bool Calibration::foundCorners(cv::Mat img, const std::string WINDOW_NAME)
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

    m_corners.push_back(corners_tmp);
    return true;
}

void Calibration::readImage(std::string paths_file_path)
{
    m_src_imgs.clear();

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
            m_src_imgs.push_back(src_img);
        }
        std::getline(ifs, file_name);
    }

    std::cout << " found " << m_src_imgs.size() << " images " << std::endl;
}

bool Calibration::readXML(const std::string xml_path)
{
    cv::FileStorage fs(xml_path, cv::FileStorage::READ);

    if (not fs.isOpened()) {
        std::cout << "[ERROR] can not open " << xml_path << std::endl;
        return false;
    }
    std::cout << "completely open " << xml_path << std::endl;

    fs["intrinsic"] >> m_parameters.intrinsic;
    fs["distortion"] >> m_parameters.distortion;
    // fs["rotation"] >> m_parameters.rotation;
    // fs["translation"] >> m_parameters.translation;
    fs["RMS"] >> m_parameters.RMS;

    showParameters();
    fs.release();

    return true;
}

void Calibration::outputXML(const std::string xml_path)
{
    cv::FileStorage fs(xml_path, cv::FileStorage::WRITE);
    cv::write(fs, "intrinsic", m_parameters.intrinsic);
    cv::write(fs, "distortion", m_parameters.distortion);
    // cv::write(fs, "rotation", m_parameters.rotation);
    // cv::write(fs, "translation", m_parameters.translation);
    cv::write(fs, "RMS", m_parameters.RMS);
    fs.release();
    std::cout << "" << xml_path << " has outputed" << std::endl;
}

Parameters Calibration::readParameters(const std::string xml_path)
{
    if (readXML(xml_path)) {
        return m_parameters;
    } else {
        std::cout << "[ERROR] can not open " << xml_path << std::endl;
        return Parameters();
    }
}

void Calibration::showParameters() const
{
    std::cout
        << "\nintrinsic\n " << m_parameters.intrinsic
        << "\ndistortion\n " << m_parameters.distortion
        // << "\n[rotation] " << m_parameters.rotation
        // << "\n[translation] " << m_parameters.translation
        << "\nRMS\n " << m_parameters.RMS << std::endl;
}

cv::Mat Calibration::rectify(const cv::Mat source_image) const
{
    cv::Mat destnation_image;
    cv::undistort(source_image, destnation_image, m_parameters.intrinsic, m_parameters.distortion);
    return destnation_image;
}

}  // namespace Camera
