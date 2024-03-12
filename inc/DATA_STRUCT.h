#pragma once
//
// Created by Yang on 2023/5/27.
//


#include <cstdint>
#define DTS_SIZE 24
#define DFS_SIZE 20
#define DTG_SIZE_P 12
#define DFG_SIZE_P 8
//TODO: Update the struct size of the grap torque motor
#define DTG_SIZE_T 12
#define DFG_SIZE_T 8

#define servoNums 7
#define servoNums 7
#define MONITOR_Hz 20
#define Socket_Hz 1000 / 100
#define motorLagRate 1.2
#define electronicGearRatio 10
#define UP_NEEDED
#define electronicGear
#include "winsock2.h"
// hpp 文件请在最头文件加载，并且pragma once
//  或者在cpp中直接使用头文件
#include <libInterpolate/Interpolate.hpp>
//
#include <memory>
#include <vector>
static std::vector<int32_t> pulse_offset{-507555, -905323126, 6492578, 17654595, -394883, 234933, 0, 0, 0};
using DTS = struct Data_To_Servo {
    uint16_t Control_Word = 0;
    int32_t Target_Pos = 0;
    uint32_t Profile_Velocity = 0x00000000;
    uint32_t Max_Velocity = 3000;// rpm
    int8_t Mode_of_Operation = 1;
    int16_t Target_Torque = 0;
    uint16_t Max_Torque = 1500;
    uint16_t Place_Holder = 0;
};
//using pDTS = DTS *;

using DFS = struct Data_From_Servo {
    uint16_t Status_Word = 0;
    int8_t Mode_of_Operation_disp = 0;
    int32_t Actual_Pos = 0;
    int32_t Actual_Vec = 0;
    int32_t Following_error = 0;
    int16_t Actual_Torque = 0;
};
using DFG_P = struct Data_From_Grap_Position {
    uint16_t Status_Word;
    int32_t Actual_Pos;
};
using DTG_P = struct Data_To_Grap_Position {
    uint16_t Control_Word=0;
    int32_t Target_pos =0;
    int8_t Mode_of_Operation=1;
};
//TODO: Update the date struct of the torque Motor
using DFG_T = struct Data_From_Grap_Torque{
   uint16_t Status_Word;
};
using DTG_T = struct Data_To_Grap_Torque{
   uint16_t Control_Word=0;
};
using _recv = struct Recv_from_Client {
    int Head_check{22};                                                 //int32
    int Command{};                                                      //int32
    std::vector<float> Joint_Position_set{std::vector<float>(9, 0)};    //float
    std::vector<float> Cartesian_Position_set{std::vector<float>(6, 0)};//float
    std::vector<float> Joint_Velocity_set{std::vector<float>(9, 0)};    //Joint Velocity theta per sec
    std::vector<float> Cartesian_Velocity_set{std::vector<float>(6, 0)};//float
    int Tail_check{};

    const int Head_check_location = 0;
    const int Command_location = 1;
    const int Joint_Position_set_location = 2;
    const int Cartesian_Position_set_location = 2 + 9;
    const int Joint_Velocity_set_location = 2 + 9 + 6;
    const int Cartesian_Velocity_set_location = 2 + 9 + 6 + 9;
    const int Tail_check_location = Cartesian_Velocity_set_location + 6;
    const int total_recv_size = Tail_check_location + 1;
};
using _send = struct Send_to_Client {
    int Head_check{};
    int Status{};
    std::vector<float> Joint_Position{std::vector<float>(9, 0.0f)};
    std::vector<float> Cartesian_Position{std::vector<float>(6, 0.0f)};
    std::vector<float> Joint_Velocity{std::vector<float>(9, 0.0f)};
    std::vector<float> Cartesian_Velocity{std::vector<float>(6, 0.0f)};
    int Tail_check{};

    const int Head_check_location = 0;
    const int Status_location = 1;
    const int Joint_Position_location = 2;
    const int Cartesian_Position_location = 2 + 9;
    const int Joint_Velocity_real_location = 2 + 9 + 6;
    const int Cartesian_Velocity_real_location = 2 + 9 + 6 + 9;
    const int Tail_check_location = Cartesian_Velocity_real_location + 6;
    const int total_recv_size = Tail_check_location + 1;
};
