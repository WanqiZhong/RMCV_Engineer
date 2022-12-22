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

const int LENGTH = 150;
const int HALF_LENGTH = 75;

class Calculator
{
    private: 
        vector<vector<Point>> anchor_point;
        Mat CameraMatrix;
        Mat DistCoeffs;
        Eigen::Vector3d ypr(3);
        thread Calculator_Thread;
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