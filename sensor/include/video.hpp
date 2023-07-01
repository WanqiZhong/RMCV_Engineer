#ifndef ENGCV_2023_Video_HPP_
#define ENGCV_2023_Video_HPP_

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
#include "uvc.hpp"
#include "args.hpp"
#include "basecap.hpp"
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


class Video : public BaseCap
{
    private:
        Logger logger =  Logger("Video");
        Mat vision_img;
        VideoCapture vision_cap;
        std::mutex img_mtx;

    public:
        Video(){};
        ~Video(){};

        /* 主进程 */
        void Run();
        void Join();
        void Video_Run();
        thread Video_thread;
};


#endif
