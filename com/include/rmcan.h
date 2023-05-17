#ifndef H_RMCAN
#define H_RMCAN
#include <thread>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string>
#include <bits/stdc++.h>
#include <stdio.h>
#include <unistd.h>
#include <data.hpp>
#include <args.hpp>
#include <bridge.h>

#define CRC_POLY_16 0X8408
#define CRC_START_MODBUS 0xFFFF


class RmCan{
public:
    RmCan(std::string can_name);
    ~RmCan();
    bool send_buffer(uint32_t id,int len);
    bool send_data(uint32_t id,uint8_t* p_data,int len);

    bool receive_buffer(uint32_t &id,uint8_t &len);
    bool receive_data(uint32_t &id,uint8_t* p_data,int len);
    void add_filter(uint32_t id);
private:
    int s; //socketCAN存储
    struct sockaddr_can addr;
    struct ifreq ifr;
    uint8_t* buffer;
    std::vector<can_filter> rfilter;
};

void Init_CRC16();
uint16_t CRC16_calc(uint8_t* data, uint32_t num_bytes);


#endif