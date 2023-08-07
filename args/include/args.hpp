// 参数列表

#ifndef ENGCV_2023_ARGS_HPP_
#define ENGCV_2023_ARGS_HPP_
#include "data.hpp"
#include "toml.hpp"
#include "spdlog/spdlog.h"
#include <string>
#include <filesystem>
#include <map>
#include <vector>
#include <set>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>


namespace fs = std::filesystem;
using namespace std;

namespace args
{
    /**
     * @brief 警告: 在正式比赛之前必须确认所有的参数都设置为false
     * 
     */
    class DebugArgs
    {
            void init(const toml::value&);
        public:
            std::string video_path;
            friend class Args;
    };

    class DetectorArgs
    {
        void init(const toml::value&);
    public:
        // Path
        std::string path2model_mine; // Mine
        std::string path2model_exchangesite; // ExchangeSite
        std::string path2model_am; // 装甲板识别的模块文件名[无拓展]
        std::string path2model_wm; // 风车识别模块文件名[无拓展]

        // Thresh
        float nms_thresh[2], conf_thresh[2]; // 0 for armor, 1 for wind
        friend class Args;
    };

    class Args
    {
    private:
        MODE _run_mode; // 模式的设定, 使用带有读写锁的两个函数进行访问(get_run_mode() & set_run_mode())
    public:
        toml::value config;
        toml::value local_config;

        // ============= Camrea Config ===============
        Eigen::Matrix<double, 3, 3> F;  // 内参矩阵
        Eigen::Matrix<double, 1, 5> C;  // 畸变矩阵

        // ============== Visual =====================
        int visual_status;
        int view;
        int camp;

        float tran_tvecx;
        float tran_tvecy;
        float tran_tvecz;
        float bias_tevcx;
        float bias_tevcy;
        float bias_tevcz;

        float cali_x;
        float cali_y;
        float cali_z;
        float cali_roll;
        float cali_yaw;
        float cali_pitch;

        // ============== Camera =====================
        std::string serial_number;
        std::string camera_fov;
        float frame_rate;

        int operator_cam_index;
        int vision_cam_index;

        float exposure_time_exchangesite;    // microsecond(μs)
        float exposure_time_mine;    // microsecond(μs)
        int frame_width;
        int frame_height;
        int codec;

        // ============== Sensor ====================
        bool image_read;  // true - image / false - video
        bool image_log;   // true - read image or video log / false - read from camera
        std::string image_path;



        // ============== Detector ====================
        DetectorArgs detector_args;
        std::string constants_path_default;
        std::string constants_path;

        // ============== Bridge ====================

        uint32_t can_send_id, can_recv_id;
        std::string serial_name;
        bool use_can;
        int serial_rate;

        // ============== Runtime ====================
        // 存储运行时有关的参数
        pthread_rwlock_t rwlock_;   // 用来守护run_mode的读写锁, 提供给需要精细操作的部分
        MODE get_run_mode();
        void set_run_mode(const MODE target);
        std::chrono::_V2::steady_clock::time_point begin_time;
        std::string asc_begin_time;

        // ============== Logger =====================
        
        spdlog::level::level_enum log_level;
        // raw record
        bool sensor_need;
        bool detector_need;
        bool image_need;

        std::string sensor_prefix;
        std::string detector_prefix;
        std::string image_prefix;
        std::string shot_prefix;
        std::string video_prefix;

        int sensor_fps;
        int detector_fps;
        int image_stride;

        // log files
        std::string info_prefix;
        std::string error_prefix;
        map<int, string> cam_map;
        vector<int> cap_set;
        vector<int> writer_set;
        vector<int> detector_writer_set;
        string get_log_path(int num, string prefix, int index = 0);

        string get_video_log_path(int num, int index);

        // ================= Traditional ======================
        int gold_thresh;
        int gold_maxval;

        int bound_small;
        int bound_big;
        int w;
        int h;
        float ratio_thres;
        float area_ratio_thres;
        int corner_thresh;
        float ratio_thres_min;
        float area_ratio_thres_min;
        int corner_contour_area_min;
        int corner_contour_area_max;
        int corner_rec_area_min;
        float res_rate;
        int poly_area;
        float match_rate;

        float site_min_rate;
        float site_max_rate;
        int site_min_area;
        int site_max_area;
        float site_area_rate;

        int default_mode;

        DebugArgs debug;

        Args(std::string, std::string, std::string);
    };
}

extern args::Args param;

#endif // ENGCV_2023_ARGS_HPP_
