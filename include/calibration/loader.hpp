#pragma once
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

    IntrinsicParams monocular() const { return m_int_params; }
    IntrinsicParams stereoLeft() const { return m_int_params1; }
    IntrinsicParams stereoRight() const { return m_int_params2; }
    IntrinsicParams rgb() const { return m_int_params1; }
    IntrinsicParams color() const { return m_int_params1; }
    IntrinsicParams depth() const { return m_int_params2; }
    IntrinsicParams ir() const { return m_int_params2; }
    ExtrinsicParams extrinsic() const { return m_ext_params; }

    // extrinsic parameter
    cv::Mat1f R() const { return m_ext_params.R(); }
    cv::Mat1f t() const { return m_ext_params.t(); }
    cv::Mat1f T() const { return m_ext_params.T(); }

    // alias
    cv::Mat1f rotation() const { return R(); }
    cv::Mat1f translation() const { return t(); }
    cv::Mat1f pose() const { return T(); }

    // inverse
    cv::Mat1f invR() const { return m_ext_params.invt(); }
    cv::Mat1f invt() const { return m_ext_params.invt(); }
    cv::Mat1f invT() const { return m_ext_params.invT(); }

private:
    IntrinsicParams m_int_params;
    IntrinsicParams m_int_params1;
    IntrinsicParams m_int_params2;
    ExtrinsicParams m_ext_params;
};

}  // namespace Calibration
