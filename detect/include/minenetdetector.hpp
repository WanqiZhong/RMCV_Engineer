#ifndef ENGCV_2023_MINENETDETECTOR_HPP_
#define ENGCV_2023_MINENETDETECTOR_HPP_

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

class Minenetdetector: public Basedetector
{
private:
    double square_area = 0;
    int offset = 100;
    int valid_cnt = 0;
    vector<Point> first_points = {};
    vector<Point> third_points = {};
    vector<vector<Point>> valid_contours = {};
    vector<Point> all_contours = {};
    vector<Point> square_contour = {};
    /* 金矿HSV参数 */

    vector<int> corner_number;
    int type; // 0 for armor, 1 for windmill
    int INPUT_W;
    int INPUT_H;
    int NUM_CLASSES;
    int NUM_COLORS;
    int NUM_KPTS;
    float NMS_THRESH;
    float BBOX_CONF_THRESH;
    float MERGE_THRESH;
    std::vector<spl::OutLayer> output_layers;

    InferenceEngine::Core ie;
    InferenceEngine::CNNNetwork network;
    InferenceEngine::ExecutableNetwork executable_network;
    InferenceEngine::InferRequest infer_request;
    std::string model_xml;
    std::string model_bin;
    std::string input_name;
    std::vector<std::string> output_names;
    void copyBlob(std::vector<float>& blob, InferenceEngine::Blob::Ptr& ieBlob);

    int pad_w, pad_h;
    float scale;
    double iw, ih;
    double x_offset, y_offset;

    //for merge_nms
    struct pick_merge_store{
        int id;
        std::vector<cv::Point2f> merge_pts;
        std::vector<float> merge_confs;
    };
    struct armor_compare{
        bool operator ()(const armor::Armor& a,const armor::Armor& b) {
            return a.conf > b.conf;
        }
    };
    float calc_iou(const armor::Armor& a,const armor::Armor& b);
    void decode_outputs(const float *prob, std::vector<armor::Armor>& objects,
                        spl::OutLayer layer_info, const int img_w, const int img_h);
    std::vector<armor::Armor> do_nms(std::vector<armor::Armor>& objects);
    std::vector<armor::Armor> do_merge_nms(std::vector<armor::Armor>& objects);
    cv::Mat static_resize(cv::Mat& img);
    cv::Mat pad_image(cv::Mat image, cv::Size2i size);

public:
    void Run();
    void Join();

    // std::unique_ptr<Minedetector> minedetector;
    thread Detector_thread;
    int infer_cnt = 0;
    double resize_tot=0, infer_tot=0, decode_tot=0, total_tot=0;
    std::vector<armor::Armor> detect(std::vector<float>& blob);
    void draw(cv::Mat img, const std::vector<armor::Armor> &objects);

    Minenetdetector(std::string name, int type, int log_level); // type==0: amor, type==1: wind;
    ~Minenetdetector(){};

    /* 主进程 */
    void Detector_Run(Mat& img);
    void DetectorNet_Run();
    // void Detector_Run();
    void get_corner_withnet(Mat &img);
    void img2blob(cv::Mat &img, std::vector<float> &dst);
    void get_mask(Mat &img, Mat &mask);

};

#endif

