#ifndef ENGCV_2023_MINEDETECTOR_HPP_
#define ENGCV_2023_MINEDETECTOR_HPP_

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
#include "args.hpp"
#include "log.hpp"
#include "basedetector.hpp"
#include <fmt/format.h>
#include <thread>
#include <chrono>
#include <array>
#include <string>
#include <cstring>
#include <ie/inference_engine.hpp>

typedef struct _CornerContour{
    vector<Point> contour;
    double distance;
}CornerContour;

class Minedetector: public Basedetector
{
    
public:

    Minedetector(){};
    ~Minedetector() = default;

    // // init anchor_point
    // Minedetector(vector<vector<Point>> anchor_point){
    //     this->anchor_point = anchor_point;
    // }

    double square_area = 0;
    int offset = 100;
    int valid_cnt = 0;
    vector<Point> first_points = {};
    vector<Point> third_points = {};
    vector<vector<Point>> valid_contours = {};
    vector<Point> all_contours = {};
    vector<Point> square_contour = {};
    void Detector_Run_Withnet(Mat &img, vector<vector<Point>>& anchor_point);
    void drawLine(Mat &img, vector<Vec2f> lines, double rows, double cols, Scalar scalar, int n);

private:

    /* 主进程 */
    void find_mine(Mat &img);
    void get_main_corner_withnet(Mat &img);
    // Mat get_gold_mine(Mat &img, Mat &colorhist);
    void Detector_Run(Mat& img);
    // void perspective_transformation(const vector<Point2f> &final_points, Mat &src);
    void perspective_transformation(const vector<Point2f>& final_points, Mat& gray_src, Mat &src);
    void get_corner_withnet(Mat &img);
    void get_corner_withnet(Mat &img, vector<vector<Point>>& anchor_point);
    void get_main_corner_withnet(Mat &img, Mat &canvas);
    void get_main_corner_withnet(Mat &img, Mat &canvas, vector<Point> border, vector<Point>& net_point, int net_index);
    Point getTargetPoint(Point pt_origin, Mat warpMatrix);
    // void get_corner(Mat &img);
    void find_anchor(Mat &img);
    
    void get_anchor(Mat &img, const vector<Point> &four_station_contours, DebugUI &debug_ui, int index);
    void draw_debug_ui(Mat &img, DebugUI &debug_ui);
    void get_corner(Mat &gray_img, Mat &img);
    void enhance_img(Mat &img);
    void get_mine(Mat &img, Mat &mask);
    void get_mask(Mat &img, Mat &mask);
    void find_corner(Mat &img);
};

#endif

