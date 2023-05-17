#include "rmcan.h"
#include <spdlog/spdlog.h>


//CAN send

bool RmCan::send_buffer(uint32_t id,int len) {
    struct can_frame frame;
    frame.can_id = id; //标识符
    bool is_send = true;
    uint8_t* p = buffer;
    while(len > 0){
        uint8_t l = std::min(8,len);
        frame.can_dlc = l;
        memcpy(frame.data,p,l);
        if(sizeof(frame) != write(s,&frame,sizeof(frame))) is_send = false;
        len -= 8;
        p += 8;
    }
    return is_send;
}

bool RmCan::send_data(uint32_t id,uint8_t* p_data,int len){
    buffer[0] = 's';
    buffer[1] = len;
    memcpy(buffer+2,p_data,len);
    uint16_t crc_code = CRC16_calc(buffer+1,len+1);
    memcpy(buffer+len+2,&crc_code,2);
    buffer[len+4] = 'e';
    return send_buffer(id,len+5);
}

//CAN receive

bool RmCan::receive_buffer(uint32_t &id,uint8_t &len) {
    struct can_frame frame;
    bool is_received = true;
    read(s,&frame,sizeof(frame));
    while(frame.data[0]!='s') {
        read(s,&frame,sizeof(frame));
    }
    id = frame.can_id;
    uint8_t* p = buffer;

    int l = frame.can_dlc;
    memcpy(p,frame.data,l);
    p += l;
    len = buffer[1];
    for(int i=9;i<=len+5;i+=8){
        read(s,&frame,sizeof(frame));
        l = frame.can_dlc;
        memcpy(p,frame.data,l);
        p += l;
    }
    return is_received;
}

bool RmCan::receive_data(uint32_t &id,uint8_t* p_data,int len){
    uint8_t buf_len;
    receive_buffer(id,buf_len);
    static Logger logger("Receiver");
    if(len != buf_len || buffer[len+4]!='e') {
        logger.critical("package length doesn't match!");
        return false;
    }
    uint16_t crc_code;
    memcpy(&crc_code,buffer+len+2,2);
    if(crc_code != CRC16_calc(buffer+1,len+1)) {
        logger.critical("CRC check failed!");
        return false;
    }
    memcpy(p_data,buffer+2,len);
    return true;
}

void RmCan::add_filter(uint32_t id){
    rfilter.emplace_back(can_filter{id,CAN_SFF_MASK});
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, rfilter.data(), rfilter.size() * sizeof(can_filter));//设置规则
}

RmCan::RmCan(std::string can_name){
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建SocketCAN套接字
    strcpy(ifr.ifr_name, can_name.c_str() );
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s, (struct sockaddr *)&addr, sizeof(addr)); 
    buffer = new uint8_t[100]; // 申请缓存
}

RmCan::~RmCan(){
    delete buffer; //释放缓存
}



