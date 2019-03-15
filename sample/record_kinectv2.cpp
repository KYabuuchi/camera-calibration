#include <filesystem>
#include <iostream>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/logger.h>
#include <libfreenect2/packet_pipeline.h>
#include <opencv2/opencv.hpp>
#include <sstream>

int main()
{
    // enumerate devices
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::None));
    libfreenect2::Freenect2 freenect2;
    if (freenect2.enumerateDevices() == 0) {
        std::cerr << "no device connected!" << std::endl;
        return 1;
    }

    // open device
    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    libfreenect2::PacketPipeline* pipeline = new libfreenect2::CpuPacketPipeline();
    libfreenect2::Freenect2Device* device = freenect2.openDevice(serial, pipeline);
    if (not device) {
        std::cerr << "failure opening device!" << std::endl;
        return 1;
    }

    // set listener
    libfreenect2::SyncMultiFrameListener listener{libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth};
    device->setColorFrameListener(&listener);
    device->setIrAndDepthFrameListener(&listener);

    // device start
    if (not device->start()) {
        std::cerr << "start failed!" << std::endl;
        return 1;
    }
    std::cout << "device serial: " << device->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << device->getFirmwareVersion() << std::endl;

    // prepare directory
    const std::string dir_name = "kinectv2";
    std::uintmax_t num = std::filesystem::remove_all(dir_name);
    if (num > 0)
        std::cout << num << " files or directories are Deleted" << std::endl;
    std::filesystem::create_directory(dir_name);

    cv::namedWindow("Color", 0);
    cv::namedWindow("Ir", 0);
    cv::namedWindow("Depth", 0);

    int frame_count = 0;
    while (true) {
        // Wait
        libfreenect2::FrameMap frames;
        cv::Mat color_mat, depth_mat, ir_mat;
        if (not listener.waitForNewFrame(frames, 10 * 1000)) {  // 10[sec]
            std::cerr << "timeout!" << std::endl;
            return 1;
        }

        {  // Color
            libfreenect2::Frame* color = frames[libfreenect2::Frame::Color];
            cv::Mat{
                static_cast<int>(color->height),
                static_cast<int>(color->width),
                CV_8UC4,
                (char*)(void*)(color->data)}
                .copyTo(color_mat);
            cv::imshow("Color", color_mat);
        }

        {  // IR
            libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
            cv::Mat{
                static_cast<int>(ir->height),
                static_cast<int>(ir->width),
                CV_32FC1,
                (char*)(void*)(ir->data)}
                .copyTo(ir_mat);
            ir_mat.convertTo(ir_mat, CV_16UC1);  // unknown unit
            cv::imshow("Ir", ir_mat);
        }

        {  // Depth
            libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];
            cv::Mat{
                static_cast<int>(depth->height),
                static_cast<int>(depth->width),
                CV_32FC1,
                (char*)(void*)(depth->data)}
                .copyTo(depth_mat);
            depth_mat.convertTo(depth_mat, CV_16UC1, 5);  // [mm/1000] => [mm/5000]
            cv::imshow("Depth", depth_mat);
        }

        int key = cv::waitKey(10);
        if (key == 'q')
            break;
        if (key == 's') {
            std::stringstream ss;
            ss << std::setfill('0') << std::right << std::setw(3) << frame_count++ << ".png";
            cv::imwrite(dir_name + "/ir_" + ss.str(), ir_mat);
            cv::imwrite(dir_name + "/rgb_" + ss.str(), color_mat);
            cv::imwrite(dir_name + "/depth_" + ss.str(), depth_mat);
            std::cout << "save " << ss.str() << std::endl;
        }

        listener.release(frames);
    }

    device->stop();
    device->close();
    return 0;
}