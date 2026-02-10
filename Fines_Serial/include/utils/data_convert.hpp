//
// Created by fins on 24-10-28.
//

#ifndef FINES_SERIAL_DATA_CONVERT_HPP
#define FINES_SERIAL_DATA_CONVERT_HPP


#include <cstdint>
#include <vector>
#include <algorithm>
#include <iostream>
extern float velocity;
extern float omega;

std::string bytes_to_hex(const std::vector<uint8_t>& bytes);
void convert_velocity(std::vector<uint8_t> &bytes, float value);
void convert_omega(std::vector<uint8_t> &bytes, float value);




#endif // FINES_SERIAL_DATA_CONVERT_HPP
