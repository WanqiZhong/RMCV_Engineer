#ifndef ENGCV_2023_SENSOR_HPP_
#define ENGCV_2023_SENSOR_HPP_

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
#include "UVC.hpp"
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <log.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

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
        Logger logger =  Logger("Sensor");
        uint8_t mode=0;
        Mat operator_img;
        Mat vision_img;
        std::mutex img_mtx;
        vector<string> cam_name = {"/dev/cam1","/dev/video0","/dev/cam2","/dev/cam3"};
        vector<VideoCapture> cap_set; // In the front of the engineer, default 2
        const int vision_index = 1;
        int operator_index = 1;
        VideoCapture vision_cap; // In the front of the engineer, default 2
        VideoCapture operator_cap; // For operator to use , default 2

        #ifdef Laptop
        int side_num = 2; 
        #else 
        int side_num;
        #endif

    public:
        Sensor(){
        
        };
        ~Sensor(){};

        /* 主进程 */
        void Run();
        void Join();

        void Sensor_Run();
        void imageRaw(int index, Mat& img);
        thread Sensor_thread;
};


#endif
