#include "calibration/calibration.hpp"

int CameraCalibration::adaptParameter()
{
    // (0)初期化
    init();

    // (1)XMLファイルの読み込み
    readXML();

    // (2)歪補正
    compareCorrection();
}

int CameraCalibration::calcParameter()
{
    // (0)データ初期化
    init();

    // (1)キャリブレーション画像の読み込み
    readImage();

    // (2)キャリブレーションパターンのコーナー検出
    // (3)コーナー位置をサブピクセル精度に修正，描画
    std::string window_name = "Calibration";
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
    int tmp_image_num = valid_image_num;

    for (int i = 0; i < tmp_image_num; i++) {
        foundCorners(src_img.at(i), window_name);
    }
    cv::destroyWindow(window_name);

    // (4)3次元空間座標の設定
    for (int i = 0; i < valid_image_num; i++) {
        std::vector<cv::Point3f> objects_tmp;

        for (int j = 0; j < PAT_ROW; j++) {
            for (int k = 0; k < PAT_COL; k++) {
                objects_tmp.push_back(cv::Point3f(j * CHESS_SIZE, k * CHESS_SIZE, 0.0f));
            }
        }
        object_points.push_back(objects_tmp);
    }

    // (5)内部パラメータ，外部パラメータ，歪み係数の推定
    calcCalibration();

    // (6)XMLファイルへの書き出し
    outputXML();

    std::cout << "\n[parameter estimation is done]" << std::endl;

    return 0;
}

int CameraCalibration::calcParameterWithPhoto()
{
    // (0)データ初期化
    init();

    // (1)キャリブレーション画像の撮影
    // (2)キャリブレーションパターンのコーナー検出
    // (3)コーナー位置をサブピクセル精度に修正，描画
    cv::VideoCapture video(device);
    std::string window_name = "Calibration";
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

    valid_image_num = IMAGE_NUM_MAX;
    for (int i = 0; i < valid_image_num;) {
        std::cout << "PRESS KEY if a camera gazes tha pattern "
                  << (valid_image_num - i) << " left" << std::endl;

        cv::Mat src;
        while (cv::waitKey(10) == -1) {
            video >> src;
            std::stringstream ss;
            ss << "calib_img" << std::setw(2) << std::setfill('0') << i << ".png";
            cv::imwrite(ss.str(), src);
            cv::imshow(window_name, src);
        }

        if (foundCorners(src, window_name)) {
            i++;
            src_img.push_back(src);
        } else {
            std::cout << "NOT found the pattern completely" << std::endl;
        }
    }
    video.release();

    cv::destroyWindow(window_name);

    // (4)3次元空間座標の設定
    for (int i = 0; i < valid_image_num; i++) {
        std::vector<cv::Point3f> objects_tmp;

        for (int j = 0; j < PAT_ROW; j++) {
            for (int k = 0; k < PAT_COL; k++) {
                objects_tmp.push_back(cv::Point3f(j * CHESS_SIZE, k * CHESS_SIZE, 0.0f));
            }
        }
        object_points.push_back(objects_tmp);
    }


    // (5)内部パラメータ，外部パラメータ，歪み係数の推定
    calcCalibration();

    // (6)XMLファイルへの書き出し
    outputXML();

    // (7)歪補正
    compareCorrection();

    std::cout << "[parameter estimation is done]" << std::endl;

    return 0;
}

void CameraCalibration::calcCalibration()
{
    std::cout << "[calibration start]" << std::endl;
    double rms = cv::calibrateCamera(
        object_points,
        corners,
        src_img.at(0).size(),
        intrinsic,
        distortion,
        rotation,
        translation);
    std::cout << "RMS of error : " << rms << std::endl;
}

bool CameraCalibration::foundCorners(cv::Mat img, std::string window_name)
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
    cv::cvtColor(img, src_gray, CV_BGR2GRAY);

    cv::cornerSubPix(
        src_gray,
        corners_tmp,
        cv::Size(3, 3),
        cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

    cv::drawChessboardCorners(img, {PAT_COL, PAT_ROW}, corners_tmp, true);
    cv::imshow(window_name, img);

    cv::waitKey(200);
    corners.push_back(corners_tmp);

    return true;
}

void CameraCalibration::readImage()
{
    std::cout << "[reading stored images]" << std::endl;

    src_img.clear();

    int image_num = 0;
    for (int i = 0; i < IMAGE_NUM_MAX; i++) {
        std::stringstream ss;
        ss << input_dir << "/calib_img" << std::setw(2) << std::setfill('0') << i << ".png";
        cv::Mat tmp_img = cv::imread(ss.str(), CV_LOAD_IMAGE_COLOR);
        if (tmp_img.data == NULL) {
            std::cerr << "cannot read : " << ss.str() << std::endl;
        } else {
            src_img.push_back(tmp_img);
            image_num++;
        }
    }
    valid_image_num = image_num;
    std::cout << "[found " << valid_image_num << " images]" << std::endl;
}

void CameraCalibration::readXML()
{
    std::cout << "read " << output_dir << std::endl;
    cv::FileStorage fs(output_dir, cv::FileStorage::READ);

    fs["intrinsic"] >> intrinsic;
    fs["distortion"] >> distortion;
    fs["rotation"] >> rotation;
    fs["translation"] >> translation;
    fs.release();
}

void CameraCalibration::compareCorrection()
{
    std::cout << "[compare corrected image]" << std::endl;
    cv::VideoCapture video(device);
    std::string show_window_name = "show image";
    cv::namedWindow(show_window_name, CV_WINDOW_AUTOSIZE);
    std::cout << intrinsic << std::endl;

    while (cv::waitKey(10) == -1) {
        cv::Mat src, dst, merge;
        video.read(src);
        cv::undistort(src, dst, intrinsic, distortion);
        cv::putText(src, "src", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 200), 2, CV_AA);
        cv::putText(dst, "dst", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 200), 2, CV_AA);
        cv::hconcat(src, dst, merge);
        cv::imshow(show_window_name, merge);
    }
    cv::destroyWindow(show_window_name);
}

void CameraCalibration::outputXML()
{
    cv::FileStorage fs(output_dir, cv::FileStorage::WRITE);
    cv::write(fs, "intrinsic", intrinsic);
    cv::write(fs, "distortion", distortion);
    cv::write(fs, "rotation", rotation);
    cv::write(fs, "translation", translation);
    fs.release();
    std::cout << "['" << output_dir << "' has outputed]" << std::endl;
}

void CameraCalibration::init()
{
    src_img.clear();
    corners.clear();
    object_points.clear();
    std::cout << "[initialize done]" << std::endl;
}
