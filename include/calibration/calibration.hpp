#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace Calibration
{
using vv_point2f = std::vector<std::vector<cv::Point2f>>;
using vv_point3f = std::vector<std::vector<cv::Point3f>>;

// 内部パラメータ
struct IntrinsicParams {
    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat distortion = cv::Mat::zeros(1, 4, CV_32FC1);
    double RMS = -1;
};
// 外部パラメータ
struct ExtrinsicParams {
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat::zeros(1, 3, CV_32FC1);
};

// pure virtual class
class AbstCalibration
{
public:
    // 行数,列数,寸法[mm]
    explicit AbstCalibration(int row, int col, float size) : ROW(row), COL(col), NUM(row * col), SIZE(size), WINDOW_NAME("window") {}

    virtual int calcParameters(const std::string paths_file_path, const std::string yaml_path) = 0;
    virtual void showParameters() const = 0;
    virtual void writeYAML(const std::string file_path) const = 0;
    virtual bool readYAML(std::string file_path) = 0;

protected:
    // コーナーの行数・列数・個数
    const int ROW, COL, NUM;
    // 1マスのサイズ(mm)
    const float SIZE;
    const std::string WINDOW_NAME;


    bool foundCorners(const cv::Mat& img, vv_point2f& corners);
    void readImages(const std::vector<std::string>& file_paths, std::vector<cv::Mat>& src_images);
    void calibrate(IntrinsicParams& int_params, ExtrinsicParams& ext_params, const vv_point2f& corners, const vv_point3f& points, const cv::Size& size);
    std::string directorize(std::string file_path);
    virtual void init() = 0;
};

// for Monocular Camera
class MonocularCalibration : public AbstCalibration
{
public:
    // 行数,列数,寸法[mm]
    explicit MonocularCalibration(int row, int col, float size) : AbstCalibration(row, col, size) {}

    int calcParameters(const std::string paths_file_path, const std::string yaml_path) override;
    void showParameters() const override;
    void writeYAML(const std::string file_path) const override;
    bool readYAML(std::string file_path) override;
    void rectify(const cv::Mat& src_image, cv::Mat& dst_image) const;

protected:
    IntrinsicParams m_int_params;
    void init() override;

private:
    bool parser(
        const std::string& file_paths_file,
        std::vector<std::string>& file_paths);

    std::vector<cv::Mat> m_src_images;
    vv_point2f m_corners;
    vv_point3f m_object_points;
};

// for Stereo Camera
class StereoCalibration : public AbstCalibration
{
public:
    // 行数,列数,寸法[mm]
    explicit StereoCalibration(int row, int col, float size) : AbstCalibration(row, col, size) {}

    int calcParameters(const std::string paths_file_path, const std::string yaml_path) override;
    void showParameters() const override;
    void writeYAML(const std::string file_path) const override;
    bool readYAML(std::string file_path) override;
    void rectify(const cv::Mat& src_image1, const cv::Mat& src_image2, cv::Mat& dst_image1, cv::Mat& dst_image2) const;

protected:
    ExtrinsicParams m_ext_params;
    IntrinsicParams m_int_params1;
    IntrinsicParams m_int_params2;
    void init() override;

private:
    bool parser(const std::string& file_paths_file,
        std::vector<std::string>& file_paths1,
        std::vector<std::string>& file_paths2);

    std::vector<cv::Mat> m_src_images1;
    std::vector<cv::Mat> m_src_images2;
    vv_point2f m_corners1;
    vv_point2f m_corners2;
    vv_point3f m_object_points;
};

}  // namespace Calibration
