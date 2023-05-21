#ifndef ENGCV_2023_CALCULATOR_HPP_
#define ENGCV_2023_CALCULATOR_HPP_

#define LABEL
#define OFFICIAL

#include<iostream>
#include<vector>
#include<map>
#include<set>
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<algorithm>
#include<Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "data.hpp"
#include "umt.hpp"
#include <thread>
#include <log.hpp>
#include <chrono>
#include <array>
#include <string>
#include <cstring>

using namespace std;
using namespace cv;

const int LENGTH = 150;
const int HALF_LENGTH = 75;

#define HALT -1
#define GoldMode 0
#define SilverMode 1
#define ChangeSiteMode 2

#define Laptop

class Calculator
{
    private:
        Logger logger = Logger("Calculator");
        uint8_t mode=0;
        vector<vector<Point>> anchor_point;
        Mat CameraMatrix;
        Mat DistCoeffs;
        vector<Mat> final_R; // 固定视角
        vector<Mat> final_T; // 固定视角
        double L = 0;
        double H = 0;
        Mat final_Rvec = (Mat_<double>(3,3) <<  0, 0, 1,
                                              -1, 0, 0,
                                               0,-1, 0);
        Mat final_Tvec = (Mat_<double>(3,1) << -L, 0, H); // 相机与吸盘转换矩阵
        Eigen::Vector3d ypr = Eigen::Vector3d(0,0,0);
        Mat position = (Mat_<double>(3,1) << 0, 0, 0);
        int view_type = 0;
        thread Calculator_thread;

    public:
        Calculator(){};
        ~Calculator(){};

        void Run();
        void Join();

        void Calculate_Run();

        void CalculateInit();
        void CalculateInit(Mat CameraMatrix, Mat DistCoeffs);
        void CalculatePnp();

};    

#endif
