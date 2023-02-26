
#include "sensor.hpp"
// #include <spdlog/spdlog.h>
#include <thread>
#define Laptop

using namespace cv;


std::atomic<bool> running(false);


void cam_thread(V4L2Capture *cam, std::string channel);
void error_cam_thread(std::string channel);

char devices1[40] = "/dev/video1"; // front

void Sensor::Run()
{
    #ifndef Laptop
    logger.info("Sensor Run");
    #else
    cout<<"Sensor Run"<<endl;
    #endif
    Sensor_Run();
}

void Sensor::Join()
{
    /* 未投入使用 */
    #ifndef Laptop
    logger.info("Waiting for [Sensor]");
    #else
    cout<<"Waiting for [Sensor]"<<endl;
    #endif
    #ifndef Laptop
    logger.sinfo("[Sensor] Join");
    #else
    cout<<"[Sensor] Join"<<endl;
    #endif
}

void Sensor::Sensor_Run()
{
    cout<<"Try open1"<<endl;
    V4L2Capture cap_hand(devices1, 1280, 720);
    vector<V4L2Capture *> caps;
    caps.push_back(&cap_hand);
    // caps.push_back(&cap_back);
    if (!caps.size()) {
        cout << "no captures" << endl;
        return;
    }
    //启动相机线程
    cout<<"Try start1"<<endl;
    cout<< "caps size: " << caps.size() << endl;
    Mat src;
    std::vector<std::unique_ptr<umt::Subscriber<cv::Mat>>> subs;
    for (int i = 0; i < caps.size(); i++) {
        stringstream ss;
        cout<< " Try channel"<<endl;
        ss << "channel" << i;
        subs.push_back(std::make_unique<umt::Subscriber<cv::Mat>>(ss.str()));
    }
    cout<< "out" <<endl;
    running = true;
    cam_start(caps);
}


void Sensor::cam_start(std::vector<V4L2Capture *> &cams) {
    // for (int i = 0; i < cams_num; ++i) {
    //     stringstream ss;
    //     ss << "channel" << i;
    //     t[i] = std::thread(cam_thread, cams.at(i), ss.str());
    // }
    for (int i = 0; i < cams.size(); ++i) {
        stringstream ss;
        ss << "channel" << i;
        cout<< "cam_start"<<endl;
        if ((*(cams.at(i))).init_success) {
            cout<<"thread"<<endl;
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
    cout<<"Thread"<<endl;
    while (running) {
        // img_mtx.lock();
        //运算符>>重载
        *cam >> tmp;
        // img_mtx.unlock();
        imshow("w", tmp);
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
