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
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

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
        std::mutex img_mtx; 
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

        void Sensor_Run();
        thread Sensor_thread;
};


#endif
