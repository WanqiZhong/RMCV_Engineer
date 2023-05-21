#include "cstdlib"
#include "detector.hpp"
#include "args.hpp"
#include "umt.hpp"
#include "log.hpp"
#include <spdlog/spdlog.h>
#include <array>
#include <thread>
#include "sensor.hpp"
#include "calculate.hpp"
#include "bridge.h"
#include <opencv4/opencv2/opencv.hpp>
#include <cstring>
using namespace cv;


int main(int argc, char **argv)
{
    Logger logger("main");
    if(argc>=2)
        param.set_run_mode((MODE)atoi(argv[1]));
    Sensor sensor;
    sensor.Run();
    Detector detect(param.get_run_mode());
    detect.Run();
    Calculator calculate;
    calculate.Run();
    Bridge bridge;
    bridge.Run();

    sensor.Join();
    detect.Join();
    calculate.Join();
    bridge.Join();
    
    while(param.get_run_mode() != HALT) {
        std::this_thread::sleep_for(3s);
    }
    cout<<"main end"<<endl;

    return 0;
}
