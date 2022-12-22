// 数据类型

#ifndef ENGCV_2023_DATA_HPP_
#define ENGCV_2023_DATA_HPP_
#include <opencv2/opencv.hpp>
#include <spdlog/fmt/fmt.h>
#include <vector>
#define UNKNOWN_ID (233)

namespace ml // machine learning
{
    struct OutLayer
    {

        int idx;
        int stride;
        int num_anchor;
        int num_out;
    };
} // namespace ml

enum MODE // 运行模式
{
    NO_AIM,   // 无瞄准
    AUTO_AIM, // 普通自瞄
    ANTI_ROT, // 反陀螺
    B_WM,     // 大风车
    S_WM,     // 小风车
    HALT,     // 停机
    Unknown,  // 未知
};

//机器人ID（DJI定义）
typedef enum Robot_id_e {
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
} Robot_id_dji;

//机器人ID（RMCV2022定义）
enum Robot_id {
    ROBOT_SENTRY = 0,
    ROBOT_HERO = 1,
    ROBOT_ENGINEER = 2,
    ROBOT_STANDARD_1 = 3,
    ROBOT_STANDARD_2 = 4,
    ROBOT_STANDARD_3 = 5,
    ROBOT_AERIAL = 6,
};

//装甲板ID（RMCV2022定义）
enum armor_type {
    SENTRY = 0,
    HERO = 1,
    ENGINEER = 2,
    STANDARD_1 = 3,
    STANDARD_2 = 4,
    STANDARD_3 = 5,
    OUTPOST = 6,
    BASE_SMALL = 7,
    BASE_BIG = 8,
};

//收imu信息

#pragma pack(1)

struct pure_IMU{
    float roll,pitch,yaw;
    pure_IMU(){
        roll = 0;
        pitch = 0;
        yaw = 0;
    }
    explicit pure_IMU(float r,float p,float y):roll(r),pitch(p),yaw(y){}
    explicit pure_IMU(double r,double p,double y): pure_IMU(static_cast<float>(r), static_cast<float>(p), static_cast<float>(y)){}
    pure_IMU operator- (const pure_IMU& b)const {
        return pure_IMU(roll-b.roll,pitch-b.pitch,yaw-b.yaw);
    }
    pure_IMU operator* (const double b)const {
        return pure_IMU(roll*b,pitch*b,yaw*b);
    }
    pure_IMU operator* (std::array<double, 3> b)const {
        return pure_IMU(roll * b[0], pitch * b[1], yaw * b[2]);
    }
    void operator+= (pure_IMU&& b){
        roll+=b.roll;
        pitch+=b.pitch;
        yaw+=b.yaw;
    }
};
struct IMU_DATA_MSG{
    //float x,y,z; // x-绕x轴转角-roll  y-绕y轴转角-pitch  z-绕z轴转角-yaw
    pure_IMU imu;
    uint8_t mode; // 0 for close, 1 for AUTO_AIM, 2 for S_WM, 3 for B_WM
    uint8_t id;
    float v;
    double time_stamp;
    std::string reprln()const;
};
#pragma pack()

struct SENSOR_DATA_MSG
{
    cv::Mat src;
    IMU_DATA_MSG imu_data; // TODO: add extra infomation
    double time_stamp;
    MODE run_mode;
};

struct CAMERA_DATA_MSG // simple version without IMU data
{
    cv::Mat src;
    double time_stamp;
}

struct ANGLE_DATA_MSG
{
    Eigen::Vector3d ypr(3);
};

struct MINE_POSITION_MSG
{
    std::vector<std::vector<cv::Point>> goal;
};

#pragma pack(1)
struct CONTROL_CMD
{
    float roll, pitch, yaw;
    uint8_t flag;
    CONTROL_CMD(){}
    CONTROL_CMD(float a,float b,float c,uint8_t d){roll=a;pitch=b;yaw=c;flag=d;}
    // 0 for no target
    // 1 for follow
    // 2 for fire once (only in windmill)
};
#pragma pack()

struct DETECT_MSG
{
    MODE mode;
    double time_stamp;   // 单位: millionsecond
    IMU_DATA_MSG imu_data; // 陀螺仪数据x,y,z
    std::vector<armor::Armor> res; // 检测到的所有可能的目标
    cv::Mat src;
};


class HaltEvent: std::exception // 停机事件
{ };


namespace wm
{
    enum WINDMILL_COLOR
    {
        UNNEEDED,
        TARGET,
    };
    enum WINDMILL_TYPE
    {
        BLUE,
        RED,
    };
} // namespace wm

// Watch Dog Heartbeat
struct HEART_BEAT
{
    
};


uint8_t get_simple_id(uint8_t);
// uint8_t get_simple_id(Robot_id_dji); // unneeded

MODE cast_run_mode(uint8_t);

#endif // CRH_2022_DATA_HPP_