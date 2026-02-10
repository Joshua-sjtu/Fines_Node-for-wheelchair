#include "serial_station.hpp"
#include <fmt/core.h>
#include "utils/data_convert.hpp"

void SerialStation::loadAndTransmit(std::vector<uint8_t> &data)
{

    convert_velocity(data, velocity);
    convert_omega(data, omega);

    if (encode_func_)
    {
        encode_func_(data);
        serial_.write(data);
    }
}


SerialStation::SerialStation(SerialConfig_t &config) : Node("serial_station")
{
    // Set up the serial port
    serial_.setPort(config.port);
    serial_.setBaudrate(config.baudrate);
    serial_.setTimeout(config.timeout);
    serial_.setBytesize(config.bytesize);
    serial_.setParity(config.parity);
    serial_.setStopbits(config.stopbits);
    serial_.setFlowcontrol(config.flowcontrol);

    // Open the serial port
    try
    {
        serial_.open();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s", config.port.c_str());
    }
    if (serial_.isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "Serial port %s is open", config.port.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port %s is not open", config.port.c_str());
    }

    serial_.flush();

    rx_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(config.rx_handle_period),
        [this]()
        { this->rxCallback(); });
}

SerialStation::~SerialStation()
{
    serial_.close();
}




void SerialStation::rxCallback()
{
    rx_buffer_.clear(); // 清空缓冲区
    auto bytes_read = serial_.read(rx_buffer_, 200);
    if (bytes_read > 0)
    {
        if (decode_func_)
        { // 复制一份缓冲区数据
            std::vector<uint8_t> data_rx(rx_buffer_.begin(), rx_buffer_.begin() + bytes_read);
            bool flag=decode_func_(data_rx);
            if(flag)
            {
                std::vector<uint8_t> data_tx;
                loadAndTransmit(data_tx);
            }
        }
    }
}
