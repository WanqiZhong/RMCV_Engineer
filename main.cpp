#include "cstdlib"
//#include "detector.hpp"
#include "detectormanager.hpp"
#include "minenetdetector.hpp"
#include "args.hpp"
#include "umt.hpp"
#include "log.hpp"
#include <spdlog/spdlog.h>
#include <array>
#include <thread>
#include "sensor.hpp"
#include "video.hpp"
#include "calculate.hpp"
#include "bridge.h"
#include "basecap.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cstring>
using namespace cv;


int main(int argc, char **argv)
{
    Logger logger("main");
    if(argc>=2)
        param.set_run_mode((MODE)atoi(argv[1]));

    shared_ptr<BaseCap> Basecap;
    if(param.image_log){
        Basecap = make_shared<Video>();
    }else{
        Basecap = make_shared<Sensor>();
    }
    Basecap->Run();


    if(!param.image_log){  [[likely]]
        Bridge bridge;
        bridge.Run();
        Minenetdetector minenetdetector(param.detector_args.path2model_am, 0, 0);
        minenetdetector.Run();
        Detectormanager detect;
        detect.Run();
        Calculator calculate;
        calculate.Run();
        Basecap->Join();
        bridge.Join();
        minenetdetector.Join();
        detect.Join();
        calculate.Join();
        
        while(param.get_run_mode() != HALT) {
            std::this_thread::sleep_for(3s);
        }
        cout<<"main end"<<endl;
    }else{    [[unlikely]]
        Detectormanager detect;
        detect.Run();
        Calculator calculate;
        calculate.Run();
        Bridge bridge;
        bridge.Run();
        Basecap->Join();
        detect.Join();
        calculate.Join();
        bridge.Join();
        while(param.get_run_mode() != HALT) {
            std::this_thread::sleep_for(3s);
        }
        cout<<"main end"<<endl;
    }
    return 0;
}
