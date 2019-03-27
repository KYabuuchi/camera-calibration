#pragma once
#include <opencv2/core.hpp>

namespace Calibration
{
using vv_point2f = std::vector<std::vector<cv::Point2f>>;
using vv_point3f = std::vector<std::vector<cv::Point3f>>;

// 内部パラメータ
struct IntrinsicParams {
    double RMS = -1;
    cv::Mat1f intrinsic = cv::Mat1f(cv::Mat1f::eye(3, 3));
    cv::Mat1f distortion = cv::Mat1f(cv::Mat::zeros(1, 5, CV_32FC1));
    cv::Size2i resolution = cv::Size2i(640, 480);

    // alias
    cv::Mat1f K() const { return intrinsic; }
    cv::Mat1f D() const { return distortion; }
};

// 外部パラメータ
struct ExtrinsicParams {
    cv::Mat1f rotation = cv::Mat1f(cv::Mat1f::eye(3, 3));
    cv::Mat1f translation = cv::Mat1f(cv::Mat1f::zeros(1, 3));

    // alias
    cv::Mat1f R() const { return rotation; }
    cv::Mat1f t() const { return translation; }
    cv::Mat1f T() const
    {
        cv::Mat1f T(cv::Mat1f::eye(4, 4));
        R().copyTo(T.colRange(0, 3).rowRange(0, 3));
        t().copyTo(T.col(3).rowRange(0, 3));
        return T;
    }

    // inverse
    cv::Mat1f invR() const { return cv::Mat1f(rotation.t()); }
    cv::Mat1f invt() const { return cv::Mat1f(-translation); }
    cv::Mat1f invT() const
    {
        cv::Mat1f inv_T(cv::Mat1f::eye(4, 4));
        invR().copyTo(inv_T.colRange(0, 3).rowRange(0, 3));
        invt().copyTo(inv_T.col(3).rowRange(0, 3));
        return inv_T;
    }
};

}  // namespace Calibration