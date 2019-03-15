#include <opencv2/core.hpp>

namespace Calibration
{
using vv_point2f = std::vector<std::vector<cv::Point2f>>;
using vv_point3f = std::vector<std::vector<cv::Point3f>>;

// 内部パラメータ
struct IntrinsicParams {
    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat distortion = cv::Mat::zeros(1, 5, CV_32FC1);
    cv::Size2i resolution = cv::Size2i(640, 480);
    double RMS = -1;
};
// 外部パラメータ
struct ExtrinsicParams {
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat::zeros(1, 3, CV_32FC1);
};

}  // namespace Calibration