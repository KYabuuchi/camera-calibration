#include "calibration/params_struct.hpp"
#include <iostream>

namespace Calibration
{
class Loader
{
public:
    Loader() {}
    Loader(const std::string& file_path) { load(file_path); }

    bool load(const std::string& file_path)
    {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);

        if (not fs.isOpened()) {
            std::cout << "[ERROR] can not open " << file_path << std::endl;
            return false;
        }

        // monocular camera
        fs["intrinsic"] >> m_int_params.intrinsic;
        fs["distortion"] >> m_int_params.distortion;
        fs["resolution"] >> m_int_params.resolution;

        // stereo camera
        fs["intrinsic1"] >> m_int_params1.intrinsic;
        fs["distortion1"] >> m_int_params1.distortion;
        fs["resolution1"] >> m_int_params1.resolution;

        fs["intrinsic2"] >> m_int_params2.intrinsic;
        fs["distortion2"] >> m_int_params2.distortion;
        fs["resolution2"] >> m_int_params2.resolution;

        fs["rotation"] >> m_ext_params.rotation;
        fs["translation"] >> m_ext_params.translation;

        fs.release();
        return true;
    }

    IntrinsicParams monocular() { return m_int_params; }
    IntrinsicParams stereoLeft() { return m_int_params1; }
    IntrinsicParams stereoRight() { return m_int_params2; }
    IntrinsicParams rgb() { return m_int_params1; }
    IntrinsicParams color() { return m_int_params1; }
    IntrinsicParams depth() { return m_int_params2; }
    IntrinsicParams ir() { return m_int_params2; }

    ExtrinsicParams extrinsic() { return m_ext_params; }

    cv::Mat1f R() { return cv::Mat1f(m_ext_params.rotation); }
    cv::Mat1f t() { return cv::Mat1f(m_ext_params.translation); }
    cv::Mat1f T()
    {
        cv::Mat1f T(cv::Mat1f::eye(4, 4));
        if (m_ext_params.rotation.empty() or m_ext_params.translation.empty())
            return cv::Mat();

        R().copyTo(T.colRange(0, 3).rowRange(0, 3));
        t().copyTo(T.col(3).rowRange(0, 3));
        return T;
    }

    cv::Mat1f rotation() { return R(); }
    cv::Mat1f translation() { return t(); }
    cv::Mat1f pose() { return T(); }

private:
    IntrinsicParams m_int_params;
    IntrinsicParams m_int_params1;
    IntrinsicParams m_int_params2;
    ExtrinsicParams m_ext_params;
};

}  // namespace Calibration
