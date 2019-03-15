#include <filesystem>
#include <opencv2/opencv.hpp>
#include <sstream>

int main(int argc, char* argv[])
{
    const std::string dir_name = "images";
    std::string device = "/dev/video0";
    if (argc == 2)
        device = argv[1];

    // open device
    std::cout << "try to open " << device << std::endl;
    cv::VideoCapture camera(device);
    if (not camera.isOpened()) {
        std::cout << " cannot open " << device << std::endl;
        return -1;
    }

    // prepare directory
    std::uintmax_t num = std::filesystem::remove_all(dir_name);
    if (num > 0)
        std::cout << num << " files or directories are Deleted" << std::endl;
    std::filesystem::create_directory(dir_name);

    // window
    cv::namedWindow(device, CV_WINDOW_NORMAL);
    cv::resizeWindow(device, 960, 720);
    std::cout << "'q': quit , 's': save" << std::endl;

    int i = 0;
    while (true) {
        cv::Mat image;
        camera >> image;

        cv::imshow(device, image);
        int key = cv::waitKey(10);

        if (key == 's') {
            std::stringstream ss;
            ss << dir_name << "/image_" << std::setfill('0') << std::right << std::setw(3) << i++ << ".png";
            cv::imwrite(ss.str(), image);
            std::cout << "save " << ss.str() << std::endl;
        } else if (key == 'q') {
            break;
        }
    }
}