#pragma once
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>


namespace Calibration
{
constexpr int IMAGE_NUM_MAX = 25;  // $B2hA|?t(B
constexpr int PAT_ROW = 7;         // $B%Q%?!<%s$N9T?t(B
constexpr int PAT_COL = 10;        // $B%Q%?!<%s$NNs?t(B
constexpr int PAT_SIZE = PAT_ROW * PAT_COL;
constexpr float CHESS_SIZE = 19.5f;  // $B%Q%?!<%s(B1$B%^%9$N(B1$BJU%5%$%:(B[mm]

struct CameraParameters {
    // $BFbIt%Q%i%a!<%?(B
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    // $B30It%Q%i%a!<%?(B
    cv::Mat rotation = cv::Mat(3, 3, CV_32FC1);
    cv::Mat translation = cv::Mat(1, 3, CV_32FC1);
    // $BOD%Q%i%a!<%?(B
    cv::Mat distortion = cv::Mat(1, 4, CV_32FC1);
    // $BEj1F8m:9$N(B Root Mean Square
    double RMS = 0;
};

class CameraCalibration
{
    using vv_point2f = std::vector<std::vector<cv::Point2f>>;
    using vv_point3f = std::vector<std::vector<cv::Point3f>>;

public:
    CameraCalibration() = default;

    // $B%G%#%l%/%H%jFb$K$"$k2hA|$rMxMQ$7!$%Q%i%a!<%?$r?dDj$7$F!$(BXML$B$X=PNO$9$k!%(B
    int calcParameters(std::string images_dir, std::string xml_dir);

    // $B<L??$r;#1F$7$F!$%G%#%l%/%H%jFb$K2hA|$rJ]B8!$$=$N$"$H%Q%i%a!<%?$r?dDj$7$F(BXML$B$X=PNO$9$k!%$5$i$KOD$_Jd@5:Q$_2hA|$rHf3S$9$k!%(B
    int calcParametersWithPhoto(std::string images_dir, std::string xml_dir, std::string device_dir);

    // XML$B$rFI$_9~$_!$%+%a%i$+$i$N1GA|$r9;@5$7$F%&%#%s%I%&$KI=<($9$k(B
    int adaptParameters(std::string xml_dir, std::string device_dir);

    // XML$B$rFI$_9~$_!$%Q%i%a!<%?$rJV$9(B
    CameraParameters readParameters(std::string xml_dir);

    // $B:#$b$C$F$$$k%Q%i%a!<%?$rI=<($9$k!%(B
    void showParameters();

private:
    // $BM-8z$J2hA|$NKg?t(B
    int valid_image_num;
    std::vector<cv::Mat> src_imgs;
    vv_point2f corners;
    vv_point3f object_points;

    // $B2f!9$,M_$7$$$b$N(B
    CameraParameters params;

    // $BJd@5$7$?2hA|$H@82hA|$rHf3S$9$k(B
    void compareCorrection(std::string device_dir);
    // $B%-%c%j%V%l!<%7%g%s$r;J$k(B
    void calibrate();
    // $B2hA|$+$i%3!<%J!<$r8!=P$7$FIA2h$9$k(B
    bool foundCorners(cv::Mat img, std::string window_name);
    // $BM?$($i$l$?%G%#%l%/%H%j$+$i2hA|$rC5$9(B
    void readImage(std::string images_dir);
    // $B%Q%i%a!<%?$r(BXML$B$X=PNO$9$k(B
    void outputXML(std::string xml_dir);
    // XML$B$+$i%Q%i%a!<%?$rF@$k(B
    bool readXML(std::string xml_dir);
    // $BJQ?t$N=i4|2=(B
    void init();
};

}  // namespace Calibration
