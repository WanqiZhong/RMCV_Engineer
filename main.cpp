#include "cstdlib"
#include "detector.hpp"
#include <array>
#include <thread>
#include "sensor.hpp"
#include "calculate.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <cstring>
using namespace cv;


int main(int argc, char **argv)
{
    int mode = 0;
    if(argc>=2)
        mode = atoi(argv[1]);
    
    cout<<mode<<endl;

    Sensor sensor;
    sensor.Run();
    Detector detect(mode);
    detect.Run();
    Calculator calculate;
    calculate.Run();

    sensor.Join();
    detect.Join();
    calculate.Join();
    
    while(mode != HALT) {
        std::this_thread::sleep_for(3s);
    }
    cout<<"main end"<<endl;

    return 0;
}
