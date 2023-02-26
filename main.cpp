#include "cstdlib"
#include "detector.hpp"
#include <array>
#include <thread>
#include "sensor.hpp"
#include "calculate.hpp"
#include <opencv4/opencv2/opencv.hpp>
using namespace cv;


int main(int argc, char **argv)
{
    int mode = 0;

    Sensor sensor;
    sensor.Run();
    Detector detect;
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
