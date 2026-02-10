#include <cstdint>
#include <vector>
#include <algorithm>
#include "utils/data_convert.hpp"
#include <iostream>
#include <sstream>     // std::stringstream, std::hex
#include <iomanip>     // std::setfill, std::setw

std::string bytes_to_hex(const std::vector<uint8_t>& bytes) {
    if (bytes.empty()) return "[empty]";
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (size_t i = 0; i < bytes.size(); ++i) {
        ss << std::setw(2) << static_cast<unsigned>(bytes[i]);
        if (i < bytes.size() - 1) ss << " ";
    }
    return ss.str();
}

void convert_velocity(std::vector<uint8_t> &bytes, float value)
{
    // 1. 限幅，防止越界
    value = std::clamp(value, -100.0f, 100.0f);

    // 2. 线性映射：(-100, 100) -> (400, 3704)
    constexpr float in_min = -100.0f;
    constexpr float in_max = 100.0f;
    constexpr int out_min = 400;
    constexpr int out_max = 3704;

    float mapped_f =
        out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);

    // 3. 转为整数（四舍五入）
    uint16_t mapped = static_cast<uint16_t>(mapped_f + 0.5f);

    // 4. 拆成高低字节（小端序）
    uint8_t high = static_cast<uint8_t>((mapped >> 8) & 0xFF);
    uint8_t low = static_cast<uint8_t>(mapped & 0xFF);

    // 5. 加入到 bytes 末尾
    bytes.push_back(low);
    bytes.push_back(high);
}

void convert_omega(std::vector<uint8_t> &bytes, float value)
{
    // 1. 限幅，防止越界
    value = std::clamp(value, -100.0f, 100.0f);

    // 2. 线性映射：(-100, 100) -> (400, 3704)
    constexpr float in_min = -100.0f;
    constexpr float in_max = 100.0f;
    constexpr int out_min = 400;
    constexpr int out_max = 3704;

    float mapped_f =
        out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);

    // 3. 转为整数（四舍五入）
    uint16_t mapped = static_cast<uint16_t>(mapped_f + 0.5f);

    // 4. 拆成高低字节（小端序）
    uint8_t high = static_cast<uint8_t>((mapped >> 8) & 0xFF);
    uint8_t low = static_cast<uint8_t>(mapped & 0xFF);

    // 5. 加入到 bytes 末尾
    bytes.push_back(low);
    bytes.push_back(high);
}