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

const float LENGTH = 275;
const float HALF_LENGTH = 275.0 / 2;

#define Laptop

class Calculator
{
private:
    Logger logger = Logger("Calculator");
    vector<vector<Point>> anchor_point;
    Mat CameraMatrix;
    Mat DistCoeffs;
    vector<Mat> final_R; // 固定视角
    vector<Mat> final_T; // 固定视角
    // Mat final_Rvec = (Mat_<double>(3,3) <<  1, 0, 0,
    //                                         0, 0, 1,
    //                                         0,-1, 0);
    Mat final_Rvec = (Mat_<double>(3,3) <<   0, 0, 1,
            -1, 0, 0,
            0,-1, 0);
    Mat final_Rvec_rpy = (Mat_<double>(3,3) <<  0, 0, -1,
            1, 0, 0,
            0,-1, 0);

    Mat final_Tvec = (Mat_<double>(3,1) << param.tran_tvecx+param.bias_tevcx, param.tran_tvecy+param.bias_tevcy, param.tran_tvecz+param.bias_tevcz); // 相机与吸盘转换矩阵
    Eigen::Vector3d ypr = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d position = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d eulerAngle2 = Eigen::Vector3d(0,0,0);
    ANGLE_DATA_MSG last_angle_data_msg;
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
    bool isRotationMatirx(Eigen::Matrix3d R);
    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

};

#endif
