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
    GetCRC16(data.data(), 15, &crcH, &crcL);
    data.push_back(crcH);
    data.push_back(crcL);
}
// 解码函数, 检查地址是否匹配
bool decode(std::vector<uint8_t>& data_rx)
{
    uint8_t pc_addr = 0xB7; // 你的 PC 的 485 地址
    std::string hex_str = bytes_to_hex(data_rx);
    if (data_rx.size() > 3 && data_rx[3] == pc_addr)
    {
        RCLCPP_INFO(rclcpp::get_logger("SerialStation"),
                "Received valid frame (addr 0x%02X): %s",
                pc_addr, hex_str.c_str());
        return true;

    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("SerialStation"),
                    "Address Mismatch. Received addr: 0x%02X, expected: 0x%02X, data: %s",
                    data_rx[3], pc_addr, hex_str.c_str());
        return false;
    }
}

// 全局变量
float velocity = 0.0f;
float omega = 0.0f;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    //参数处理 
    std::string serial_port = "/dev/ttyUSB0";
    uint32_t baudrate = 9600;
    int rx_handle_period = 1; // ms

    //串口配置
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


    //速度指令 
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

//测试代码
/*
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    //参数处理 
    std::string serial_port = "/dev/ttyUSB0";
    uint32_t baudrate = 9600;
    int rx_handle_period = 1; // ms

    //串口配置
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
    auto node = std::make_shared<SerialStation>(config);
    // 创建节点实例（用于调用成员函数和日志）
    node->bindEncodeFunc(encode);
    node->bindDecodeFunc(decode);
    std::cout << "\n=== 速度 & 角速度 测试模式 ===\n";
    std::cout << "输入格式：速度(m/s) 角速度(rad/s)   示例：0.5 -0.3\n";
    std::cout << "输入 q 或 quit 退出\n\n";

    while (true) {
        std::cout << "> ";

        std::string line;
        std::getline(std::cin, line);

        // 去除前后空白
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);

        if (line.empty()) continue;

        // 退出判断
        if (line == "q" || line == "quit" || line == "exit") {
            std::cout << "退出测试模式\n";
            break;
        }

        // 解析两个浮点数
        std::istringstream iss(line);
        float velocity_ = 0.0f;
        float omega_ = 0.0f;

        if (!(iss >> velocity_ >> omega_)) {
            std::cout << "输入格式错误，请输入两个数字（速度 角速度）\n";
            continue;
        }

        // 设置到节点成员变量（假设类里有 public 或你能访问的 velocity_ 和 omega_）
        velocity = velocity_;
        omega    = omega_;

        // 准备一个空的 vector，让 loadAndTransmit 填充
        std::vector<uint8_t> data;

        // 调用核心函数
        node->loadAndTransmit(data);

        // 打印结果
        std::cout << "\n输入: 速度 = " << velocity << " m/s, 角速度 = " << omega << " rad/s\n";
        std::cout << "输出字节 (" << data.size() << " bytes): " 
                  << bytes_to_hex(data) << "\n\n";
    }

    rclcpp::shutdown();
    return 0;
}
*/