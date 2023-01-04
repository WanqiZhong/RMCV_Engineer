#include "cstdlib"
#include "detector.hpp"
#include <array>
#include <thread>
#include "sensor.hpp"

int main(int argc, char **argv)
{
    int mode = 0;

    Sensor sensor;
    sensor.Run();
    Detector detect;
    detect.Run();


    sensor.Join();
    detect.Join();
    
    while(mode != HALT) {
        std::this_thread::sleep_for(3s);
    }
    cout<<"main end"<<endl;

    return 0;
}
