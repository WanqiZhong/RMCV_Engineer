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
#include "args.hpp"
#include "BaseCap.hpp"
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
#include <filesystem>
#include <map>


namespace fs = std::filesystem;
using namespace std;
using namespace cv;

const vector<int> writer_num = {1};
class Sensor : public BaseCap
{
    private:
        Logger logger =  Logger("Sensor");

        Mat operator_img;
        Mat vision_img;
        std::mutex img_mtx;

        vector<VideoCapture> cap_set; // In the front of the engineer, default 2
        VideoCapture vision_cap; // In the front of the engineer, default 2
        VideoCapture operator_cap; // For operator to use , default 2
        map<int, string> cam_name_maps = {{0,"/dev/cam1"},{1,"/dev/video0"},{2,"/dev/cam2"},{3,"/dev/cam3"}};
        map<int, VideoWriter> writer_map;

        int frame_index = 0;

    public:
        Sensor(){};
        ~Sensor(){};

        /* 主进程 */
        void Run();
        void Join();

        void Sensor_Run();

        void setCamera(int mode);
        void imageRaw(int index, Mat& img);
        void initVideoRaw();
        void videoRaw(Mat& img);
        void videoRaw(vector<Mat>& img);

        thread Sensor_thread;
};


#endif
