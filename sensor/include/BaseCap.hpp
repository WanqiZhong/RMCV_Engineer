#ifndef ENGCV_2023_BaseCap_HPP_
#define ENGCV_2023_BaseCap_HPP_

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

class BaseCap
{
    public:
        virtual void Run() = 0;
        virtual void Join() = 0;
        const int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
        const double fps = 30.0;
        const Size frameSize = Size(1280, 720);
};


#endif
