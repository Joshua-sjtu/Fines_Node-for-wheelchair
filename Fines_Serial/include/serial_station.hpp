#ifndef FINES_NODE_SERIAL_STATION_HPP
#define FINES_NODE_SERIAL_STATION_HPP

#include <queue>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "serial/serial.h"
#include "msg_types.hpp"

typedef struct
{
    const std::string port; // e.g. "/dev/ttyUSB0"
    uint32_t baudrate;
    serial::Timeout timeout;
    serial::bytesize_t bytesize;
    serial::parity_t parity;
    serial::stopbits_t stopbits;
    serial::flowcontrol_t flowcontrol;
    int rx_handle_period; // 接收处理周期，单位ms
} SerialConfig_t;

class SerialStation : public rclcpp::Node
{
public:
    explicit SerialStation(SerialConfig_t &config);
    ~SerialStation();

    void bindEncodeFunc(std::function<void(std::vector<uint8_t>&)> && encode_func) {
        encode_func_ = encode_func;
    }

    void bindDecodeFunc(std::function<void(std::vector<uint8_t>)> && decode_func) {
        decode_func_ = decode_func;
    }

    void loadAndTransmit(std::vector<uint8_t> &data);

private:

    void rxCallback();


private:
    serial::Serial serial_;

    rclcpp::TimerBase::SharedPtr rx_timer_;

    std::vector<uint8_t> rx_buffer_; // 串口接收缓冲区，即取即刷

    std::function<void(std::vector<uint8_t>&)> encode_func_ = nullptr;
    std::function<void(std::vector<uint8_t>)> decode_func_ = nullptr;

};

#endif //FINES_NODE_SERIAL_STATION_HPP
