#include "atomic"
#include <opencv4/opencv2/opencv.hpp>
#include <rmconfig.h>
#include <parameter.h>
using namespace cv;
McuConfig g_config_data; // extern
RmSerial g_rmSerial;     // extern
RmParameter g_para;      // extern
class Bridge{
public:
    McuConfig g_config_data; // intern
    RmSerial g_rmSerial;     // intern
    Bridge()
    {
        Init_CRC16();
        test_progress();
        //test_Serial();
    }
private:
    inline void test_Exchange_Station();
    inline void test_Small_Island();
    inline void test_Big_Island();
    inline void test_Twinkle_Light();
    inline void test_Free_Mode();
    void test_Serial();
    void test_progress();
};

void Bridge::test_progress()
{
    // test_Serial();
    test_Free_Mode();
    // test_Small_Island();
    // test_Twinkle_Light();
    // test_Big_Island();
    // #ifndef DEBUG
    g_rmSerial.init();
    // #endif
}
inline void Bridge::test_Exchange_Station() {
    g_config_data.start = 1;
    g_config_data.state = EXCHANGE_STATION_AUTO_RUN;
}
inline void Bridge::test_Small_Island() {
    g_config_data.start = 1;
    g_config_data.state = SMALL_RESOURCE_ISLAND_AUTO_RUN;
}
inline void Bridge::test_Big_Island() {
    g_config_data.start = 1;
    g_config_data.state = BIG_RESOURCE_ISLAND_AUTO_RUN;
}
inline void Bridge::test_Twinkle_Light() {
    g_config_data.start = 1;
    g_config_data.state = TWINKLE_LIGHT_AUTO_RUN;
}
inline void Bridge::test_Free_Mode() {
    g_config_data.start = 1;
    g_config_data.state = FREE_MODE;
}

uint64_t chhysb = 0;
void Bridge::test_Serial() {
    // Mat src = Mat::zeros(Size(720,480),CV_8UC1);
    g_rmSerial.init();
    shared_ptr<SendData> data(new SendData);
    // shared_ptr<McuData> data(new McuData);
    while (1) {
        chhysb++;
        // imshow("1",src);
        // waitKey();
        // // cout << "here" << endl;
        // if (g_config_data.state != 0) {
        //     cout << "1g_config_data.state:" << g_config_data.state  <<
        //     endl
        //          << "1g_config_data.start:" << g_config_data.start  <<
        //          endl
        //          << "1g_config_data.view:" << g_config_data.view  <<
        //          endl;
        //     cout << "quit" << endl;
        //     break;
        // }
        if (chhysb > 10000000) {
            printf("chhy\n");
            // printf("%ld\n",chhysb);
            // send_mtx.lock();
            data->start_flag = 's';
            data->end_flag = 'e';
            static int yzh;
            if (yzh) {
                data->error_x = 201;
                yzh = 0;
            } else {
                data->error_x = 188;
                yzh++;
            }
            data->direction_x = 'l';
            data->is_error_data = 0;
            data->is_standard = 1;
            data->len = 5;
            bool send_success = g_rmSerial.send_data(*data);
            chhysb = 1;
        }
    }
}