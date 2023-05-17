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
    int mode = 0;
    if(argc>=2)
        mode = atoi(argv[1]);
    
    cout<<mode<<endl;

    Sensor sensor;
    sensor.Run();
    // Detector detect(mode);
    // detect.Run();
    // Calculator calculate;
    // calculate.Run();
    Bridge bridge;
    bridge.Run();

    sensor.Join();
    // detect.Join();
    // calculate.Join();
    bridge.Join();
    
    while(mode != HALT) {
        std::this_thread::sleep_for(3s);
    }
    cout<<"main end"<<endl;

    return 0;
}
