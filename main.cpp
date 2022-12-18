#include "args.hpp"
#include "cstdlib"
#include "detect.hpp"
#include "calculate.hpp"
#include "log.hpp"
#include <spdlog/spdlog.h>
#include <array>
#include <thread>
#include "sensor.hpp"
#include "watchdog.hpp"

void signal_handler(int signum)
{
    Logger logger("signal");
    logger.info("SIGNAL {} received", signum);
    param.set_run_mode(HALT);
    return;
}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;
    signal(SIGINT, signal_handler);
    // Logger::Init();
    Logger logger("main");

    // load predict: constant if not exist
    if(!std::filesystem::exists(param.constants_path)) {
        logger.warn("Constants file not found, using default.");
        std::filesystem::copy_file(param.constants_path_default, param.constants_path);
    }

    // 开启传感器线程
    Sensor sensor;
    sensor.Run();
    //开启数据收发线程
    Bridge bridge;
    bridge.Run();
    //开启检测矿石线程
    Detect detect;
    detect.Run();
    //开启位姿解算线程
    Calculate calculate;
    calculate.Run();

    WatchDog dog("cnt.log");
    dog.Watch("BridgeReceive", bridge);
    dog.Watch("Sensor", sensor);
    dog.Watch("Detect", [](){
        return false;
    });
    dog.Watch("BridgeSend", bridge);
    dog.Run(5s, 5s);

    // param.set_run_mode(HALT);
    while(param.get_run_mode() != HALT) {
        std::this_thread::sleep_for(3s);
    }

    sensor.Join();
    detect.Join();
    calculate.Join();
    bridge.Join();
    dog.Join();

    logger.sinfo("Quit.");
    logger.flush();
    return 0;
}