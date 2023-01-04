#ifndef PERF_HPP_
#define PERF_HPP_
#include <chrono>
#include <string>
#include "log.hpp"

class Event{
private:
    std::chrono::_V2::steady_clock::time_point start_time;
    std::chrono::_V2::steady_clock::time_point end_time;
public:
    void start();
    void end();
    double duration();
};

class Perf{
private:
    double tot = 0, num = 0;
    int threshold;
    Logger logger;

public:
    Perf(std::string name, int _threshold=1000);
    double avg();
    void clear();
    void update(double val,int n=1);
};

#endif