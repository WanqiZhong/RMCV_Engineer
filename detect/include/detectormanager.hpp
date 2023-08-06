#ifndef ENGCV_2023_DETECTOR_HPP_
#define ENGCV_2023_DETECTOR_HPP_

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
#include <log.hpp>
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>
#include "basedetector.hpp"
#include "sitedetector.hpp"

using namespace std;
using namespace cv;

class Detectormanager
{
    private:
        shared_ptr<Basedetector> detector;
        Logger logger = Logger("Detectormanager");
        thread Detector_thread;
        MODE last_mode = Unknown;

    public:
        Detectormanager()= default;
        ~Detectormanager()= default;
        /* 主进程 */
        void Run();
        void Join();
        void Manager_Run();
        void UpdateParam();
};


#endif
