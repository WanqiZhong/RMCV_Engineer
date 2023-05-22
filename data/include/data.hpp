// 数据类型

#ifndef ENGCV_2023_DATA_HPP_
#define ENGCV_2023_DATA_HPP_
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#define UNKNOWN_ID (233)
namespace armor
{
    struct Armor // 代表一个装甲板
    {
        cv::Rect_<float> rect;
        cv::Point2f pts[5];
        float conf;
        int color;
        int type;
    };
}

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
    GoldMode, // 0
    SilverMode, // 1
    ExchangeSiteMode, // 2
    WaitMode, // 3 NoVision
    HALT,     // 4 停机
    Unknown,  // 未知
};


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
    uint8_t view; // Operator view
    uint8_t visual_flag; // 0 for no vision, 1 for vision real time, 2 for vision last valid data
    uint8_t mode; // 0 for gold, 1 for silver, 2 for exchange site, 3 for halt
    uint8_t position_id; // 1 for left, 2 for center, 3 for right
    double time_stamp;
};
#pragma pack()


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

class HaltEvent: std::exception // 停机事件
{ };

// Watch Dog Heartbeat
struct HEART_BEAT
{
    enum TYPE
    {
        DEFAULT,    // a normal heart beat
        WAIT,       // wait for other modules
        CONTD,      // waiting done, continue
    };
    TYPE type;
};

#pragma pack(1)
struct ANGLE_DATA_MSG
{
    uint8_t is_valid;
    uint8_t ratation_right;
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
//    cv::Mat position;
//    Eigen::Vector3d angle;
};
#pragma pack()

struct MINE_POSITION_MSG
{
    std::vector<std::vector<cv::Point>> goal;
};

uint8_t get_simple_id(uint8_t);

MODE cast_run_mode(uint8_t);

#endif // ENGCV_2023_DATA_HPP_