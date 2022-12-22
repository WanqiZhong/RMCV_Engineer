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
#include "data.hpp"
#include "umt.hpp"
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>
#include "log.hpp"

using namespace std;
using namespace cv;

/* 金矿HSV参数 */
int hmin = 0;
int hmax = 255;
int smin = 37;
int smax = 255;
int vmin = 37;
int vmax = 255;

/* 图像增强参数 */
int contrast = 37;
int bright = 18;

class Detector
{
    private:
        bool mine_flag;
        vector<vector<Point>> contours;
        vector<vector<Point>> anchor_point;
        vector<Vec4i> hierarchy;
        vector<Rect> whole_rect;
        vector<Rect> half_rect;
        thread Detector_thread;
    public:
        Detector(){};
        ~Detector(){};
        
        void Run();
        void Join();

        void Detect_Run();

        Mat ImgUpdate();
        /* 寻找金矿 */
        void PreProcessMine(Mat& img);
        void FindSide(Mat &img);

        /* 图像处理分别为彩色和灰度 */
        void ImgProcess(const Mat& img,Scalar lower,Scalar upper);
        void ImgProcess(const Mat& img,int thresh,int maxval,int type);
        void ImgEnhance(Mat &img);
};

#endif