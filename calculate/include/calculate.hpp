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
        uint8_t mode=0;
        vector<vector<Point>> anchor_point;
        Mat CameraMatrix;
        Mat DistCoeffs;
        Eigen::Vector3d ypr;
        thread Calculator_thread;
    public:
        Calculator(){};
        ~Calculator(){};

        void Run();
        void Join();

        void Calculate_Run();

        void CalculateInit();
        void CalculateInit(Mat CameraMatrix, Mat DistCoeffs);
        void CalculateMinePnp();

};    

#endif
