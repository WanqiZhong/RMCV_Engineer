#ifndef ENGCV_2023_SENSOR_HPP_
#define ENGCV_2023_SENSOR_HPP_

#include "rmconfig.h"
#include "parameter.h"
#include "uvc_v4l2.h"
#include "umt.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include "umt.hpp"
#include "data.hpp"
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>

using namespace std;
using namespace cv;

#define HALT -1
#define GoldMode 0
#define SilverMode 1
#define ChangeSiteMode 2

#define Laptop

class Sensor
{
    private:
        uint8_t mode=0;
        Mat img;
        #ifdef Laptop
        int side_num = 2; 
        #else 
        int side_num;
        #endif
        

    public:
        Sensor(){};
        ~Sensor(){};

        /* 主进程 */
        void Run();
        void Join();
        void cam_start(std::vector<V4L2Capture *> &cams);
        void Sensor_Run();
        std::thread t[1];
        // thread Sensor_thread;
};


#endif
