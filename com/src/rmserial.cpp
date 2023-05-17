#include <rmserial.h>

#define MCU_PAN_TYPE 0
#define MCU_CONFIG_TYPE 1
#define MCU_ENERGY_TYPE 2
#define MCU_SPEED_TYPE 3

// #include <runtime.h>

// receive data

RmSerial::RmSerial(){
    std::string uart_port = param.serial_name;
    active_port = new serial::Serial(uart_port, param.serial_rate,serial::Timeout::simpleTimeout(1000));
    buffer = new uint8_t[100];
}

RmSerial::~RmSerial(){
    delete buffer;
}

bool RmSerial::send_buffer(int len) {
    return active_port->write(buffer, len) == len;
}
bool RmSerial::send_data(uint8_t* p_data,int len) {
    buffer[0] = 's';
    buffer[1] = len; //data type
    memcpy(buffer+2,p_data,len);
    uint16_t crc_code = CRC16_calc(buffer+1,len+1);
    memcpy(buffer+len+2,&crc_code,2);
    buffer[len+4] = 'e';
    return send_buffer(len+5);
}

bool RmSerial::receive_buffer(uint8_t len){
    int time = 0;
    while(++time < 100){
        if(active_port->read(buffer,1)!=1) break;
        if(buffer[0]=='s'){
            return active_port->read(buffer+1, len-1) == len-1;
        }
    }
    return false;
}

bool RmSerial::receive_data(uint8_t* p_data,int len){
    bool state = receive_buffer(len+5);
    if(!state) return false;
    static Logger logger("Receiver");
    if(len!= buffer[1]){
        logger.critical("package length doesn't match!");
    }
    if(buffer[0]!='s' || buffer[len+4]!='e') return false;
    uint16_t crc_code;
    memcpy(&crc_code,buffer+len+2,2);
    if(crc_code != CRC16_calc(buffer+1,len+1)) {
        logger.critical("CRC check failed!");
        return false;
    }
    memcpy(p_data,buffer+2,len);
    return true;
}