#ifndef BRIDGE_HPP_
#define BRIDGE_HPP_
#include "rmcan.h"
#include "rmserial.h"
#include "data.hpp"
#include "umt.hpp"
#include "args.hpp"
#include "log.hpp"
#include "perf.hpp"

extern bool crc_inited;
extern uint16_t crc_tab16[256];

class Bridge{
private:
    Logger logger;
    std::thread send_thread;
    std::thread recv_thread;
public:
    void Join();
    void Run();
    bool Reset();
    Bridge(): logger("Bridge")
    { }
};

void can_send();

void can_receive();

void serial_send();

void serial_receive();

void Init_CRC16();

uint16_t CRC16_calc(uint8_t* data, uint32_t num_bytes);

#endif