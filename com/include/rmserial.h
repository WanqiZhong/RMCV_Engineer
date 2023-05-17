#ifndef H_RMSERIAL
#define H_RMSERIAL
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <args.hpp>
#include <bridge.h>
#include <log.hpp>

class RmSerial{
public:
    RmSerial();
    ~RmSerial();
    bool send_buffer(int len);
    bool send_data(uint8_t* p_data,int len);

    bool receive_buffer(uint8_t len);
    bool receive_data(uint8_t* p_data,int len);
private:
    int s; //socketCAN存储
    serial::Serial* active_port;
    uint8_t* buffer;
};
#endif