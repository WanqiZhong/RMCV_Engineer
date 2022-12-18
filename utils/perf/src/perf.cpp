#include "perf.hpp"

void Event::start(){
    start_time = std::chrono::steady_clock::now();
}

void Event::end(){
    end_time = std::chrono::steady_clock::now();
}

double Event::duration(){
    return std::chrono::duration<double, std::milli>(start_time - end_time).count();
}

Perf::Perf(std::string name, int _threshold):logger(name){
    threshold = _threshold;
}

void Perf::update(double val,int n){
    tot += val;
    num += n;
    if(num > threshold){
        logger.sinfo(
            "[PERF] time: {:.3f}ms fps: {:.2f}",
            tot / num,
            val / num * 1000
        );
        clear();
    }
}

void Perf::clear(){
    tot = num = 0;
}

double Perf::avg(){
    return tot / num;
}