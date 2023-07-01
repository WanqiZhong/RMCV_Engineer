#include "bridge.h"
#include "thread_butcher.hpp"
#include <log.hpp>

bool Bridge::Reset(){
    crh::kill_thread(send_thread, "Bridge.send_thread");
    crh::kill_thread(recv_thread, "Bridge.recv_thread");
    Run();
    return true;
}

void Bridge::Run() {
    Init_CRC16();
    if(param.use_can) {
        logger.sinfo("using CAN");

        std::system("ip link set can0 down");
        std::system("ip link set can0 up type can bitrate 1000000");

        send_thread = std::thread(can_send);
        recv_thread = std::thread(can_receive);
    } else {
        logger.sinfo("using SERIAL");

        send_thread = std::thread(serial_send);
        recv_thread = std::thread(serial_receive);
    }
}

void Bridge::Join(){
    logger.info("Waiting for [Bridge]");
    send_thread.join();
    recv_thread.detach();
    logger.sinfo("[Bridge] Joined.");
}


uint16_t crc_tab16[256];
bool crc_inited = false;

void can_send() {
    Logger logger("CanSender");
    uint32_t send_id = param.can_send_id; // 发送id
    RmCan s0("can0");
    umt::Subscriber<CONTROL_CMD> send_sub("robot_data");
    umt::Publisher<HEART_BEAT> HB_pub("BridgeSend_HB");
    int HB_cnt = 0;
#ifdef PERF
    Event recorder;
    Perf perf("CanSender");
#endif
    while (param.get_run_mode() != HALT) {
#ifdef PERF
        recorder.start();
#endif
        CONTROL_CMD data_send;
        try {
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::WAIT});
            data_send = send_sub.pop();
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::CONTD});
        } catch(const HaltEvent&) {
            break;
        }
        if (!s0.send_data(send_id, (uint8_t *)&data_send, sizeof(CONTROL_CMD))){
            logger.warn("Send faild!");
        }
        if(HB_cnt>50){
            HB_cnt = 0;
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::DEFAULT});
        }
        ++HB_cnt;
#ifdef PERF
        recorder.end();
        perf.update(recorder.duration());
#endif
    }
    logger.sinfo("[Bridge] Sender exited.");
    return;
}
void can_receive(){
    Logger logger("CanReceiver");
    RmCan s1("can0");
    uint32_t receive_id = param.can_recv_id; // 接收id
    umt::Publisher<IMU_DATA_MSG> receive_pub("imu_data");
    umt::Publisher<HEART_BEAT> HB_pub("BridgeReceive_HB");
    int HB_cnt = 0;
    IMU_DATA_MSG data;
    s1.add_filter(receive_id);
    auto last_mode = param.get_run_mode();
#ifdef PERF
    Event recorder;
    Perf perf("CanReceiver");
#endif
    while(param.get_run_mode() != HALT){
#ifdef PERF
        recorder.start();
#endif
        uint32_t id;
        while(!s1.receive_data(id, (uint8_t *)&data, sizeof(IMU_DATA_MSG) - sizeof(double))){
            logger.error("Receive failed!");
            continue;
        }
        double time_stamp = std::chrono::duration<double>(std::chrono::steady_clock::now()- param.begin_time).count();
        data.time_stamp = time_stamp;
        auto mode = cast_run_mode(data.mode);
        if (mode != last_mode)
        {
            last_mode = mode;
            param.set_run_mode(mode);
        }
        if(id==receive_id) {
            receive_pub.push(data);
        }
        if(HB_cnt>50){
            HB_cnt = 0;
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::DEFAULT});
        }
        ++HB_cnt;
#ifdef PERF
        recorder.end();
        perf.update(recorder.duration());
#endif
    }
    logger.sinfo("[Bridge] Receiver exited.");
    return;
}

void serial_send() {
    try {
    Logger logger("SerialSender");
    RmSerial serial0;
    umt::Subscriber<ANGLE_DATA_MSG> send_sub("robot_data");
    umt::Publisher<HEART_BEAT> HB_pub("BridgeSend_HB");
    int HB_cnt = 0;
#ifdef PERF
    Event recorder;
    Perf perf("SerialSender");
#endif
    while (true) {
#ifdef PERF
        recorder.start();
#endif
        ANGLE_DATA_MSG data_send;
        try {
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::WAIT});
            data_send = send_sub.pop();
            logger.critical("{}",data_send.roll);
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::CONTD});
        } catch(const HaltEvent&) {
            break;
        }
        if (!serial0.send_data((uint8_t *)&data_send, sizeof(ANGLE_DATA_MSG))){
            logger.warn("Send failed!");
        }
        if(HB_cnt>50){
            HB_cnt = 0;
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::DEFAULT});
        }
        ++HB_cnt;
#ifdef PERF
        recorder.end();
        perf.update(recorder.duration());
#endif
    }
    logger.sinfo("[Bridge] Sender exited.");
    } catch(serial::IOException e) {
        return;
    }
}

void serial_receive(){
    try {
    Logger logger("SerialReceiver");
    RmSerial serial1;
    umt::Publisher<IMU_DATA_MSG> receive_pub("Electric_Data");
    umt::Publisher<HEART_BEAT> HB_pub("BridgeReceive_HB");
    int HB_cnt = 0;
    IMU_DATA_MSG data;
    auto last_mode = param.get_run_mode();
#ifdef PERF
    Event recorder;
    Perf perf("SerialReceiver");
#endif
    while(param.get_run_mode() != HALT){
#ifdef PERF
        recorder.start();
#endif
        if(!serial1.receive_data((uint8_t *)&data, sizeof(IMU_DATA_MSG)-sizeof(double))){
            logger.warn("Serial No Data");
            continue;
        }
        auto mode = cast_run_mode(data.mode);
        if (mode != last_mode)
        {
            last_mode = mode;
            param.set_run_mode(mode);
        }
        double time_stamp = std::chrono::duration<double>(std::chrono::steady_clock::now()- param.begin_time).count();
        // logger.info("{},{},{},{}",data.x,data.y,data.z,data.mode);
        // logger.info("cam_id:{},position_id:{},mode:{}",data.cam_id,data.position_id,data.mode);
        data.time_stamp = time_stamp;
        receive_pub.push(data);
        if(HB_cnt>50){
            HB_cnt = 0;
            HB_pub.push(HEART_BEAT{HEART_BEAT::TYPE::DEFAULT});
        }
        ++HB_cnt;
#ifdef PERF
        recorder.end();
        perf.update(recorder.duration());
#endif
    }
    logger.sinfo("[Bridge] Receiver exited.");
    } catch(serial::IOException e) {
        return;
    }
}

void Init_CRC16() {
    if(crc_inited) return;
    for (uint16_t i = 0; i < 256; ++i) {
        uint16_t crc = 0, c = i;
        for (uint16_t j = 0; j < 8; ++j) {
            if ((crc ^ c) & 0x0001) {
                crc = (crc >> 1) ^ CRC_POLY_16;
            } else
                crc = crc >> 1;
            c = c >> 1;
        }
        crc_tab16[i] = crc;
    }
    crc_inited = true;
}

uint16_t CRC16_calc(uint8_t* data, uint32_t num_bytes) {
    uint16_t crc = CRC_START_MODBUS;
    while (num_bytes--) crc = (crc >> 8) ^ crc_tab16[(crc ^ *data++) & 0xff];
    return crc;
}
