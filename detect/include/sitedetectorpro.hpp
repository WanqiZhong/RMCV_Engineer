#ifndef ENGCV_2023_SitedetectorPro_HPP_
#define ENGCV_2023_SitedetectorPro_HPP_

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <log.hpp>
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>
#include "umt.hpp"
#include "data.hpp"
#include "basedetector.hpp"

using namespace std;
using namespace cv;



class SitedetectorPro : public Basedetector
{
    public:
        SitedetectorPro() = default;
        ~SitedetectorPro() = default;

        /* 主进程 */
        void Detector_Run(Mat &img) override;

        /* Debug工具 */
        void draw_debug_ui(Mat &img, DebugUI &debug_ui);

        /* 分进程 */
        void find_corner(Mat &img); // 寻找到L形和正方形角点
        void get_anchor(Mat &img, const vector<Point>& four_station_contours, DebugUI &debug_ui, int index);
        void find_anchor(Mat &img);

        Mat thresh_output;
        vector<Point> station_contours;
        vector<Point> all_contours; // 符合条件的轮廓导入为一维数组
        vector<Point> anchor_contour; // 符合条件的二维数组中每一个一维数组的第一个元素导入为一维数组
        vector<vector<Point>> valid_contour; // 符合条件的轮廓导入为二维数组
        vector<Point> square_contour;

        int min_corner_index;
        double min_corner_rec;//最小面积的角点的外接矩形面积
        int corner_cnt = 0;
        Mat thresh;


};
#endif
