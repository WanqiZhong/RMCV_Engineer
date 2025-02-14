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

using namespace std;
using namespace cv;

#define Laptop

#define gold_hmin 0
#define gold_hmax 255
#define gold_smin 37
#define gold_smax 255
#define gold_vmin 37
#define gold_vmax 255

typedef struct _debugUI{
    vector<Point> small_square_point;
    vector<double> small_square_area;
    vector<Point> poly;
    float area;
    float match_rate;
    int min_index;
    bool right_flag;
}DebugUI;



class Detector
{
    private:
	/* 金矿HSV参数 */
        Logger logger = Logger("Detector");
        bool mine_flag;
        int side_num = 2;

        vector<vector<Point>> gold_mine_contours;  //金矿轮廓 
        vector<Vec4i> gold_mine_hierarchy; //金矿轮廓层次
        vector<Rect> gold_mine_whole_rect; //金矿标识角
        vector<Rect> gold_mine_half_rect; //金矿标识半角
        vector<vector<Point>> anchor_point;

        vector<vector<Point>> gold_mine_contours_2;
        vector<Vec4i> gold_mine_hierarchy_2;
        vector<Point> polycontours;

        vector<vector<Point>> logo_R; //R_logo轮廓
        vector<vector<Point>> gold_mine_side;  //金矿分面标识
        
        vector<Rect> silver_mine_rect; //银矿外侧轮廓
        vector<vector<Point>> silver_mine_contours;  //银矿标识
        vector<int> corner_number;

        int min_corner_index;
        vector<Point> station_contours;
        vector<Point> all_contours;
        vector<Point> square_contour;
        vector<Point> anchor_contour;
        vector<vector<Point>> valid_contour; // 符合标志物大小的完整轮廓
        double min_corner_rec;//最小面积的角点的外接矩形面积
        int corner_cnt = 0;
        Mat thresh;
        
        
        
    public:
        int frame_index = 0;
        int shot_index = 0;
        thread Detector_thread;
        map<int, VideoWriter> detector_writer_map;
  
        /* 图像增强参数 */
        int contrast = 37;
        int bright = 18;
        
        Detector(){};
        ~Detector(){};
        

        /* 主进程 */
        void Run();
        void Join();

        /* 子进程 */
        void GoldMineDetect_Run(Mat &img);
        void GoldMineDetect_Run2(Mat &img);
        void SilverMineDetect_Run(Mat &img);
        void ExchangeSite_Run(Mat &img);

        void Detect_Run();
        Mat ImgUpdate();

        /* 图像处理分别为彩色和灰度 */
        /* 用于金矿 */
        void process_gold_mine(const Mat& img,Scalar lower,Scalar upper);
        void process_gold_mine(const Mat& img,int thresh,int maxval,int type);
        void enhance_img(Mat &img);

        /* 寻找金矿与分面识别 方案1 */
        void find_gold_mine(Mat &img);
        void get_gold_mine(Mat &img);

        /* 寻找金矿与分面识别 方案2 */
        void img_light_enhance(Mat &img, Mat &img_hsv);
        Mat process_img_corner(const Mat &img, int thresh, int maxval);
        int find_R(vector<vector<Point>> &logo_R, Mat process);
        vector<int> sort_length(Point p);
        vector<vector<Point>> store_side(vector<vector<Point>> logo_R, vector<Point>& square,vector<int> &corner_number);
        void draw_side(Mat img, vector<Point> side, Point square, int corner_number);
        // void find_gold_mine_2(Mat &img);
        Mat get_gold_mine_2(Mat &img, Mat &colorhist);

        /* 银矿识别 */
        void find_white_mineral(Mat &img, vector<Rect> &side_rect);
        Mat get_white_mineral(Mat &img, Rect rec);
        void process_white_corner(Mat &img, int thresh, int maxual,vector<vector<Point>> &corner_contour);
        void get_white_corner(Mat &img, vector<vector<Point>> corner_contour);

        /* ExchangeSite  */
        void find_site_corner(Mat &img);
        void get_station_side(Mat &img);
        void get_station_corner(Mat &img, vector<Point> four_station_contours, DebugUI &debug_ui, int index);
        void draw_debug_ui(Mat &img, DebugUI &debug_ui);


        void initVideoRaw();
        void writeVideoRaw(cv::Mat &img);
        void writeImageRaw(int index, Mat& img);

};


#endif
