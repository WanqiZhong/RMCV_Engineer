#include "../serial/v8sydint.h"
#include <iostream>
#include <rmserial.h>
#include <thread>

using namespace serial;

std::mutex receive_mtx;
const std::string uart_port = "/dev/ttyACM0";

bool RmSerial::send_data(uint8_t *p_data, size_t size) {
    *(p_data + 1) = size - 5;
    uint16_t crc_code = CRC16_calc(p_data + 1, size - 4);
    memcpy(p_data + size - 3, &crc_code, 2);
    // cout << p_data[0] << endl;
    // cout << p_data[4] << endl;
    // cout << (int)p_data[5] << endl;
    // cout << "sending" << endl;
    return active_port->write(p_data, size);
}

bool RmSerial::send_data(const SendData &data) {
    return send_data((uint8_t *)(&data), sizeof(SendData));
}

bool RmSerial::send_data(const McuData &data) {
    return send_data((uint8_t *)(&data), sizeof(McuData));
}

void process_data(uint8_t *buff,
                  uint8_t *buff_tail) { // cout << "process" << endl;
    if ((uint8_t)*buff != 's') {
        cout << "Not Starter with 's' " << endl;
        return;
    }
    // cout << buff_tail-buff << endl; //参数：头指针，尾指针
    // cout << buff_tail[-6]<<endl;
    if (buff_tail - buff != sizeof(McuData)) {
        cout << "buff_tail - buff != sizeof(McuData)" << endl;
        return;
    }
    uint16_t crc_code;
    memcpy(&crc_code, buff + 4, 2);
    // cout << crc_code << endl;
    // uint8_t a = *(buff + 2);
    // a += '0';
    // int b=(int)a;
    // cout << a << endl;
    // cout << "calc:" << CRC16_calc(buff + 1, sizeof(McuData) - 4) << endl;
    if (crc_code != CRC16_calc(buff + 1, sizeof(McuData) - 4)) {
        cout << "CRC error!" << endl;
        return;
    } else {
        // cout << "crc correct" << endl;
    }
    McuData mcu_data;
    memcpy(&mcu_data, buff, sizeof(McuData));

    receive_mtx.lock();
    // cout << mcu_data.view << endl;
    readEngineerMcuData(&mcu_data, &g_config_data.view, &g_config_data.state);
    receive_mtx.unlock();
    // cout <<g_config_data.state
    cout << "g_config_data.view:" << (int)g_config_data.view << endl;
    // cout << "g_config_data.state:" << g_config_data.state << endl
    //  << "g_config_data.start:" << g_config_data.start << endl
    //  << "g_config_data.view:" << g_config_data.view << endl
    //  << "g_config_data.barrier_ok:" << g_config_data.barrier_ok << endl;
}

void receive_data(RmSerial *rm_serial) {
    cout << "receiving data." << endl;
    uint8_t *buffer_tail = rm_serial->buff;
    serial::Serial *port = rm_serial->active_port;
    size_t wait_in_buffer;
    port->flush();
    while (rm_serial->thread_running) {
        wait_in_buffer = port->available();
        // cout << "wait_in_buffer:" << wait_in_buffer << endl;
        if (wait_in_buffer) {
            port->read(buffer_tail, 1);
            buffer_tail += 1;
            // cout << "port reading." << endl;
            if (buffer_tail[-1] == 'e') {
                process_data(rm_serial->buff, buffer_tail);
                buffer_tail = rm_serial->buff;
            } else {
                // // cout << buffer_tail - rm_serial->buff << endl;
                // cout << buffer_tail[-2]<<endl;
                // cout << buffer_tail[-1]<<endl;
                // cout << buffer_tail[1]<<endl;
                // // cout << "Not end with 'e'." << endl;
            }
        }
    }
}

// bool receive_buffer(RmSerial *rm_serial, uint8_t len) {
// serial::Serial *active_port = rm_serial->active_port;
// int time = 0;
// while (++time < 100) {
// if (active_port->read((rm_serial->buff), 1) == 1 &&
// (rm_serial->buff)[0] == 's') {
// return active_port->read((rm_serial->buff) + 1, len - 1) == len - 1;
// }
// }
//     return false;
// }

// bool receive_data(RmSerial *rm_serial, uint8_t *p_data, int len) {
//     serial::Serial *active_port = rm_serial->active_port;
//     bool state = receive_buffer(rm_serial, len + 5);
//     if (!state)
//         return false;
//     if (len != (rm_serial->buff)[1]) {
//         return false;
//     }
//     if ((rm_serial->buff)[0] != 's' || (rm_serial->buff)[len + 4] != 'e')
//         return false;
//     uint16_t crc_code;
//     memcpy(&crc_code, (rm_serial->buff) + len + 2, 2);
//     if (crc_code != CRC16_calc((rm_serial->buff) + 1, len + 1))
//         return false;
//     memcpy(p_data, (rm_serial->buff) + 2, len);
//     return true;
// }

void RmSerial::start_thread() {
    if (init_success)
        thread_running = true;
    std::thread task(receive_data, this);
    // int size = sizeof(McuData);
    // std::thread task(receive_data, this, &g_config_data, &size);
    task.detach();
    cout << "thread starts." << endl;
}

void RmSerial::stop_thread() {

    if (init_success) {
        thread_running = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool RmSerial::init() {
    active_port =
        new Serial(uart_port, 115200, serial::Timeout::simpleTimeout(1000));
    buff = new uint8_t[100];
    init_success = true;
    //开启数据接受线程
    start_thread();
    if (active_port->isOpen()) {
        cout << "Successfully initialized port " << uart_port << endl;
        return true;
    } else {
        cout << "Failed to initialize port " << uart_port << endl;
        return false;
    }
}

RmSerial::~RmSerial() { delete buff; }