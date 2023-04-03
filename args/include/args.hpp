// 参数列表

#ifndef CRH_2022_ARGS_HPP_
#define CRH_2022_ARGS_HPP_
#include "data.hpp"
#include "toml.hpp"
#include "spdlog/spdlog.h"
#include <string>
#include <filesystem>

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
        // ================debug====================
        // 存储debug用的选项
        bool pseudo_camera;
        std::string video_path;
        float skip_minutes;
        bool pseudo_imu;
        // 使用文件作为imu输入
        bool file_imu;
        std::string imu_path;
        bool with_imshow_detector, with_imshow_predictor;
        bool disable_auto_shoot;
        bool pseudo_mode;   // True: 每隔5秒会自动切换模式(自瞄->小风车->大风车)
        friend class Args;
    };

    class DetectorArgs
    {
        void init(const toml::value&);
    public:
        // Path
        std::string path2model_am; // 装甲板识别的模块文件名[无拓展]
        std::string path2model_wm; // 风车识别模块文件名[无拓展]

        // Thresh
        float nms_thresh[2], conf_thresh[2]; // 0 for armor, 1 for wind

        friend class Args;
    };

    class Args
    {
    private:
        MODE _run_mode; // 机器人自瞄模式的设定, 使用带有读写锁的两个函数进行访问(get_run_mode() & set_run_mode())
    public:
        toml::value config;
        toml::value local_config;

        // ============== Camera =====================
        std::string serial_number;
        float frame_rate;                           // 使用float的原因采用微秒作为单位数值可能超出int最大值, 实际上还是必须是整数
        std::string camera_company;

        float exposure_time, energy_exposure_time;    // microsecond(μs)
                                                    // notes: 2022年东部赛区似乎出现过当曝光时间与交流电的频率不匹配的时候灯光频闪问题
        float gain, energy_gain;                    // dB
        int normal_grab_height, normal_grab_width;
        int energy_grab_height, energy_grab_width;  // 能量机关模式下捕获图像的宽和高
        int net_height, net_width;
        int offset_x, offset_y;
        int energy_offset_x, energy_offset_y;

        // ============== Detector =====================

        DetectorArgs detector_args;


        // ============== Predictor =====================
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
        // recorder of detector
        bool detector_need;
        std::string detector_prefix;
        float detector_fps;
        // raw record
        bool sensor_need;
        std::string sensor_prefix;
        float sensor_fps;
        // imu record
        bool imu_need;
        std::string imu_prefix;
        // log files
        std::string info_prefix;
        std::string error_prefix;

        DebugArgs debug;

        Args(std::string, std::string, std::string);
    };
}

extern args::Args param;

#endif // CRH_2022_ARGS_HPP_
