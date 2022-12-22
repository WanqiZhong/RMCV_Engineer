#include <iostream>
#include "uvc_v4l2.h"
#include <spdlog/spdlog.h>
#include <thread>
#include "umt.hpp"

using namespace cv;

// char devices0[40] = "/dev/video4";
char devices1[40] = "/dev/cam2"; // front
char devices2[40] = "/dev/cam1"; // hand
char devices3[40] = "/dev/cam3"; // back
char devices4[40] = "/dev/cam4"; // high
// char devices5[40] = "/dev/video5";

std::mutex img_mtx;      // extern

std::atomic<bool> running(false);
std::thread t[4];

void cam_start(std::vector<V4L2Capture *> &cams);
void cam_thread(V4L2Capture *cam, std::string channel);
void error_cam_thread(std::string channel);

void cam_start(std::vector<V4L2Capture *> &cams) {
    // for (int i = 0; i < cams_num; ++i) {
    //     stringstream ss;
    //     ss << "channel" << i;
    //     t[i] = std::thread(cam_thread, cams.at(i), ss.str());
    // }
    for (int i = 0; i < cams.size(); ++i) {
        stringstream ss;
        ss << "channel" << i;
        if ((*(cams.at(i))).init_success) {
            t[i] = std::thread(cam_thread, cams.at(i), ss.str());
        } else {
            t[i] = std::thread(error_cam_thread, ss.str());
        }
    }

    return;
}

void cam_thread(V4L2Capture *cam, std::string channel) {
    Mat tmp;
    umt::Publisher<cv::Mat> pub(channel);
    while (running) {
        img_mtx.lock();
        //运算符>>重载
        *cam >> tmp;
        img_mtx.unlock();
        // imshow("w", tmp);
        // waitKey(1);
        pub.push(tmp.clone());
    }
    return;
}

void error_cam_thread(std::string channel) {
    Mat tmp = Mat::zeros(480, 800, CV_8UC1);
    umt::Publisher<cv::Mat> pub(channel);
    while (running) {
        pub.push(tmp.clone());
    }
    return;
}

