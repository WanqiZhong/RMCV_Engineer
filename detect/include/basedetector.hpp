#ifndef ENGCV_2023_BASEDETECTOR_HPP_
#define ENGCV_2023_BASEDETECTOR_HPP_

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

using namespace std;
using namespace cv;

typedef struct _debugUI{
    vector<Point> small_square_point;
    vector<double> small_square_area;
    vector<Point> poly;
    float area;
    float match_rate;
    float min_area;
    int min_index;
    bool right_flag;
    Point min_area_point;
}DebugUI;

class Basedetector
{
    private:
        map<int, VideoWriter> detector_writer_map;
    int frame_index = 0;

    protected:
        
    public:
        vector<vector<Point>> anchor_point;

        thread Detector_thread;
        Logger logger = Logger("Detector");

        Basedetector()= default;
        ~Basedetector()= default;

        /* 主进程 */
        virtual void Detector_Run(Mat &img) = 0;
        void clearAnchorPoint();
        vector<vector<Point>> getAnchorPoint();

        /* 图像增强工具 */
        void img_light_enhance(Mat &img, Mat &img_hsv);

        /* 内录 */
        void initVideoRaw();
        void writeVideoRaw(cv::Mat &img);
        void writeImageRaw(int index, Mat& img);
};

#endif
