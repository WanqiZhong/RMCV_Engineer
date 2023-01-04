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

class Detector
{
    private:
	/* 金矿HSV参数 */
	int gold_hmin = 0;
	int gold_hmax = 255;
	int gold_smin = 37;
	int gold_smax = 255;
	int gold_vmin = 37;
	int gold_vmax = 255;

	int gold_thresh = 70;
	int gold_maxval = 255;

	/* 图像增强参数 */
	int contrast = 37;
	int bright = 18;

        uint8_t mode=0;
        // bool mine_flag;
        // #ifdef Laptop
        // int side_num = 2; 
        // #else 
        // int side_num;
        // #endif
    
        // vector<vector<Point>> gold_mine_contours;  //金矿轮廓 
        // vector<Vec4i> gold_mine_hierarchy; //金矿轮廓层次
        // vector<Rect> gold_mine_whole_rect; //金矿标识角
        // vector<Rect> gold_mine_half_rect; //金矿标识半角
        // vector<vector<Point>> anchor_point;

        // vector<vector<Point>> logo_R; //R_logo轮廓
        // vector<vector<Point>> gold_mine_side;  //金矿分面标识
        
        // vector<Rect> silver_mine_rect; //银矿外侧轮廓
        // vector<vector<Point>> silver_mine_contours;  //银矿标识
        
        
    public:
        thread Detector_thread;
        Detector(){};
        ~Detector(){};
        

        /* 主进程 */
        void Run();
        void Join();

        // /* 子进程 */
        // void GoldMineDetect_Run(Mat &img);
        // void GoldMineDetect_Run2(Mat &img);
        // void SilverMineDetect_Run(Mat &img);


        void Detect_Run();
        // Mat ImgUpdate();

        // /* 图像处理分别为彩色和灰度 */
        // /* 用于金矿 */
        // void process_gold_mine(const Mat& img,Scalar lower,Scalar upper);
        // void process_gold_mine(const Mat& img,int thresh,int maxval,int type);
        // void enhance_img(Mat &img);

        // /* 寻找金矿与分面识别 方案1 */
        // void find_gold_mine(Mat &img);
        // void get_gold_mine(Mat &img);

        // /* 寻找金矿与分面识别 方案2 */
        // void process_img_corner(Mat &img, int thresh, int maxual);
        // vector<vector<Point>> find_R(int side_number);
        // vector<int> sort_length(Point p);
        // vector<vector<Point>> store_side(vector<vector<Point>> logo_R,Mat img);
        // void draw_side(Mat img, vector<Point> side);


        // /* 银矿识别 */
        // void find_white_mineral(Mat &img, vector<Rect> &side_rect);
        // Mat get_white_mineral(Mat &img, Rect rec);
        // void process_white_corner(Mat &img, int thresh, int maxual,vector<vector<Point>> &corner_contour);
        // void get_white_corner(Mat &img, vector<vector<Point>> corner_contour);


};


#endif
