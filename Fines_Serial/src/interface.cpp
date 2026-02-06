#include <iostream>
#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>
#include "serial_station.hpp"
#include "msg_types.hpp"
#include "utils/crc.hpp"
#include "utils/data_convert.hpp"

// 带有帧标识符、长度和 CRC 的编码函数
void encode(std::vector<uint8_t> &data)
{
    const uint16_t HEAD1 = 0xEB;
    const uint16_t HEAD2 = 0x90;
    const uint16_t FB = 0xBB;
    const uint16_t LENGTH = 0x11;
    const uint16_t ORDER = 0x3A;
    const uint16_t STA = 0x00;
    const uint16_t SPEEDGRAD = 0x01;
    const uint16_t TAIL1 = 0xCC;
    const uint16_t TAIL2 = 0x33;
    const uint16_t TAIL3 = 0xC3;
    const uint16_t TAIL4 = 0x3C;

    // 在 data 的开头插入帧头
    data.insert(data.begin(), ORDER);
    data.insert(data.begin(), LENGTH);
    data.insert(data.begin(), FB);
    data.insert(data.begin(), HEAD2);
    data.insert(data.begin(), HEAD1);
    // 在 data 的结尾插入帧尾和 CRC
    data.push_back(STA);
    data.push_back(SPEEDGRAD);
    data.push_back(TAIL1);
    data.push_back(TAIL2);
    data.push_back(TAIL3);
    data.push_back(TAIL4);
    uint8_t crcH, crcL;
    GetCRC16(data, 15, &crcH, &crcL);
    data.push_back(crcH);
    data.push_back(crcL);
}

// 解码函数
void decode(std::vector<uint8_t> data)
{
    uint8_t pc_addr = 0xB7; // 你的 PC 的 485 地址

    if (data.size() > 3 && data[3] == pc_addr)
    {
        RCLCPP_INFO(rclcpp::get_logger("SerialStation"),
                    "Received: [%s]", data_str.c_str());
        loadAndTransmit(data);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("SerialStation"),
                    "Address Mismatch: [%s]", data_str.c_str());
    }
}

// 全局变量
float velocity = 0.0f;
float omega = 0.0f;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    /***************************** 参数处理 ********************************/
    std::string serial_port = "/dev/ttyUSB0";
    uint32_t baudrate = 9600;
    int rx_handle_period = 1; // ms

    /***************************** 串口配置 ********************************/
    SerialConfig_t config = {
        serial_port,     // serial_port
        baudrate,        // baudrate
        serial::Timeout( // 如果只有最长超时，每次read都会超时
            0,           // inter_byte_timeout_
            0,           // read_timeout_constant_
            0,           // read_timeout_multiplier_
            0,           // write_timeout_constant_
            0            // write_timeout_multiplier_
            ),
        serial::eightbits,        // byte_size
        serial::parity_none,      // parity
        serial::stopbits_one,     // stopbits
        serial::flowcontrol_none, // flowcontrol
        rx_handle_period,         // rx_handle_period, unit: ms
    };
    auto serial_station = std::make_shared<SerialStation>(config);

    serial_station->bindEncodeFunc(encode);
    serial_station->bindDecodeFunc(decode);

    /***************************** 速度指令 ********************************/
    auto cmd_vel_sub = serial_station->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        [serial_station](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            velocity = static_cast<float>(msg->linear.x);
            omega = static_cast<float>(msg->angular.z);
        });

    rclcpp::spin(serial_station);

    rclcpp::shutdown();

    return 0;
}
