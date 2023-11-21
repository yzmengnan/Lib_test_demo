#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
//
// Created by 91418 on 2023/11/6.
//

#ifndef JACOBDEMO_TRANSFORM_HPP
#define JACOBDEMO_TRANSFORM_HPP
#include "mat.hpp"
#include <cmath>
inline auto angles2rad(const vector<float> &data) -> vector<float> {
    vector<float> rads{};
    for (const auto &d: data) {
        rads.push_back(d / 180 * 3.1415F);
    }
    return rads;
}

inline auto joint2Position(const vector<float> &joint_data) -> vector<float> {
    if (joint_data.size() < 6) {
        return vector<float>{};
    }
    auto joint_rad = angles2rad(joint_data);
    mat T01{{{cos(joint_rad[0]), -sin(joint_rad[0]), 0, 0},
             {sin(joint_rad[0]), cos(joint_rad[0]), 0, 0},
             {0, 0, 1, 0},
             {0, 0, 0, 1}}};
    mat T12{{{cos(joint_rad[1]), -sin(joint_rad[1]), 0, 0},
             {0, 0, 1, 1.002},
             {-sin(joint_rad[1]), -cos(joint_rad[1]), 0, 0},
             {0, 0, 0, 1}}};
    mat T23{{{cos(joint_rad[2]), -sin(joint_rad[2]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_rad[2]), cos(joint_rad[2]), 0, 0},
             {0, 0, 0, 1}}};
    mat T34{{{cos(joint_rad[3]), -sin(joint_rad[3]), 0, 0},
             {0, 0, 1, 1.1115},
             {-sin(joint_rad[3]), -cos(joint_rad[3]), 0, 0},
             {0, 0, 0, 1}}};
    mat T45{{{cos(joint_rad[4]), -sin(joint_rad[4]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_rad[4]), cos(joint_rad[4]), 0, 0},
             {0, 0, 0, 1}}};
    mat T56{{{cos(joint_rad[5]), -sin(joint_rad[5]), 0, 0},
             {0, 0, 1, 0},
             {-sin(joint_rad[5]), -cos(joint_rad[5]), 0, 0},
             {0, 0, 0, 1}}};
    mat T02 = T01 * T12;
    mat T03 = T02 * T23;
    mat T04 = T03 * T34;
    mat T05 = T04 * T45;
    mat T06 = T05 * T56;
    float beta = atan2(-T06.data[2][0], sqrt(T06.data[0][0] * T06.data[0][0] + T06.data[1][0] * T06.data[1][0]));
    float alpha = atan2(T06.data[1][0] / cos(beta), T06.data[0][0] / cos(beta));
    float gama = atan2(T06.data[2][1] / cos(beta), T06.data[2][2] / cos(beta));
    return vector<float>{T06.data[0][3], T06.data[1][3], T06.data[2][3], alpha * 180 / 3.1415F, beta * 180 / 3.1415F, gama * 180 / 3.1415F};
}

inline auto jacob0(vector<float> &joint_data) -> mat {
    //only used for 6-DOF
    if (joint_data.size() != 6) {
        return {};
    }

    auto joint_rad = angles2rad(joint_data);

    vector<float> jacobv1{-1.002F * cos(joint_rad[0]) - 1.1115F * cos(joint_rad[2]) * cos(joint_rad[0]) + 1.115F * sin(joint_rad[0]) * cos(joint_rad[1]) * sin(joint_rad[2]),
                          1.1115F * cos(joint_rad[0]) * sin(joint_rad[1]) * sin(joint_rad[2]),
                          1.1115F * sin(joint_rad[2]) * sin(joint_rad[0]) - 1.1115F * cos(joint_rad[0]) * cos(joint_rad[1]) * cos(joint_rad[2]),
                          0, 0, 0};
    vector<float> jacobv2{-1.002F * sin(joint_rad[0]) - 1.1115F * sin(joint_rad[0]) * cos(joint_rad[2]) - 1.1115F * cos(joint_rad[1]) * cos(joint_rad[0]) * sin(joint_rad[2]),
                          1.1115F * sin(joint_rad[0]) * sin(joint_rad[1]) * sin(joint_rad[2]),
                          -1.1115F * cos(joint_rad[0]) * sin(joint_rad[2]) - 1.1115F * sin(joint_rad[0]) * cos(joint_rad[1]) * cos(joint_rad[2]),
                          0, 0, 0};
    vector<float> jacobv3{0,
                          1.1115F * cos(joint_rad[1]) * sin(joint_rad[2]),
                          1.1115F * sin(joint_rad[1]) * cos(joint_rad[2]),
                          0, 0, 0};
    mat T01{{{cos(joint_rad[0]), -sin(joint_rad[0]), 0, 0},
             {sin(joint_rad[0]), cos(joint_rad[0]), 0, 0},
             {0, 0, 1, 0},
             {0, 0, 0, 1}}};
    mat T12{{{cos(joint_rad[1]), -sin(joint_rad[1]), 0, 0},
             {0, 0, 1, 1.002},
             {-sin(joint_rad[1]), -cos(joint_rad[1]), 0, 0},
             {0, 0, 0, 1}}};
    mat T23{{{cos(joint_rad[2]), -sin(joint_rad[2]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_rad[2]), cos(joint_rad[2]), 0, 0},
             {0, 0, 0, 1}}};
    mat T34{{{cos(joint_rad[3]), -sin(joint_rad[3]), 0, 0},
             {0, 0, 1, 1.1115},
             {-sin(joint_rad[3]), -cos(joint_rad[3]), 0, 0},
             {0, 0, 0, 1}}};
    mat T45{{{cos(joint_rad[4]), -sin(joint_rad[4]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_rad[4]), cos(joint_rad[4]), 0, 0},
             {0, 0, 0, 1}}};
    mat T56{{{cos(joint_rad[5]), -sin(joint_rad[5]), 0, 0},
             {0, 0, 1, 0},
             {-sin(joint_rad[5]), -cos(joint_rad[5]), 0, 0},
             {0, 0, 0, 1}}};
    mat T02 = T01 * T12;
    mat T03 = T02 * T23;
    mat T04 = T03 * T34;
    mat T05 = T04 * T45;
    mat T06 = T05 * T56;
    mat jacobw{
            {vector<float>{T01.data[0][2], T01.data[1][2], T01.data[2][2]},
             vector<float>{T02.data[0][2], T02.data[1][2], T02.data[2][2]},
             vector<float>{T03.data[0][2], T03.data[1][2], T03.data[2][2]},
             vector<float>{T04.data[0][2], T04.data[1][2], T04.data[2][2]},
             vector<float>{T05.data[0][2], T05.data[1][2], T05.data[2][2]},
             vector<float>{T06.data[0][2], T06.data[1][2], T06.data[2][2]}}};
    jacobw = jacobw.T();
    return mat({jacobv1, jacobv2, jacobv3, jacobw.data[0], jacobw.data[1], jacobw.data[2]});
}
inline auto jacobe(vector<float> &joint_data) -> mat {
    //only used for 6-DOF
    if (joint_data.size() != 6) {
        return {};
    }
    auto joint_rad = angles2rad(joint_data);
    vector<float> jacobv1{-1.002F * cos(joint_rad[0]) - 1.1115F * cos(joint_rad[2]) * cos(joint_rad[0]) + 1.115F * sin(joint_rad[0]) * cos(joint_rad[1]) * sin(joint_rad[2]),
                          1.1115F * cos(joint_rad[0]) * sin(joint_rad[1]) * sin(joint_rad[2]),
                          1.1115F * sin(joint_rad[2]) * sin(joint_rad[0]) - 1.1115F * cos(joint_rad[0]) * cos(joint_rad[1]) * cos(joint_rad[2]),
                          0, 0, 0};
    vector<float> jacobv2{-1.002F * sin(joint_rad[0]) - 1.1115F * sin(joint_rad[0]) * cos(joint_rad[2]) - 1.1115F * cos(joint_rad[1]) * cos(joint_rad[0]) * sin(joint_rad[2]),
                          1.1115F * sin(joint_rad[0]) * sin(joint_rad[1]) * sin(joint_rad[2]),
                          -1.1115F * cos(joint_rad[0]) * sin(joint_rad[2]) - 1.1115F * sin(joint_rad[0]) * cos(joint_rad[1]) * cos(joint_rad[2]),
                          0, 0, 0};
    vector<float> jacobv3{0,
                          1.1115F * cos(joint_rad[1]) * sin(joint_rad[2]),
                          1.1115F * sin(joint_rad[1]) * cos(joint_rad[2]),
                          0, 0, 0};

    mat T01{{{cos(joint_rad[0]), -sin(joint_rad[0]), 0, 0},
             {sin(joint_rad[0]), cos(joint_rad[0]), 0, 0},
             {0, 0, 1, 0},
             {0, 0, 0, 1}}};
    mat T12{{{cos(joint_rad[1]), -sin(joint_rad[1]), 0, 0},
             {0, 0, 1, 1.002},
             {-sin(joint_rad[1]), -cos(joint_rad[1]), 0, 0},
             {0, 0, 0, 1}}};
    mat T23{{{cos(joint_rad[2]), -sin(joint_rad[2]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_rad[2]), cos(joint_rad[2]), 0, 0},
             {0, 0, 0, 1}}};
    mat T34{{{cos(joint_rad[3]), -sin(joint_rad[3]), 0, 0},
             {0, 0, 1, 1.1115},
             {-sin(joint_rad[3]), -cos(joint_rad[3]), 0, 0},
             {0, 0, 0, 1}}};
    mat T45{{{cos(joint_rad[4]), -sin(joint_rad[4]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_rad[4]), cos(joint_rad[4]), 0, 0},
             {0, 0, 0, 1}}};
    mat T56{{{cos(joint_rad[5]), -sin(joint_rad[5]), 0, 0},
             {0, 0, 1, 0},
             {-sin(joint_rad[5]), -cos(joint_rad[5]), 0, 0},
             {0, 0, 0, 1}}};
    mat T02 = T01 * T12;
    mat T03 = T02 * T23;
    mat T04 = T03 * T34;
    mat T05 = T04 * T45;
    mat T06 = T05 * T56;
    mat jacobw{
            {vector<float>{T01.data[0][2], T01.data[1][2], T01.data[2][2]},
             vector<float>{T02.data[0][2], T02.data[1][2], T02.data[2][2]},
             vector<float>{T03.data[0][2], T03.data[1][2], T03.data[2][2]},
             vector<float>{T04.data[0][2], T04.data[1][2], T04.data[2][2]},
             vector<float>{T05.data[0][2], T05.data[1][2], T05.data[2][2]},
             vector<float>{T06.data[0][2], T06.data[1][2], T06.data[2][2]}}};
    jacobw = jacobw.T();
    auto jacob0 = mat({jacobv1, jacobv2, jacobv3, jacobw.data[0], jacobw.data[1], jacobw.data[2]});

    mat Re0{{{T06.data[0][0], T06.data[1][0], T06.data[2][0], 0, 0, 0},
             {T06.data[0][1], T06.data[1][1], T06.data[2][1], 0, 0, 0},
             {T06.data[0][2], T06.data[1][2], T06.data[2][2], 0, 0, 0},
             {0, 0, 0, T06.data[0][0], T06.data[1][0], T06.data[2][0]},
             {0, 0, 0, T06.data[0][1], T06.data[1][1], T06.data[2][1]},
             {0, 0, 0, T06.data[0][2], T06.data[1][2], T06.data[2][2]}}};
    return Re0 * jacob0;
}
#endif//JACOBDEMO_TRANSFORM_HPP

#pragma clang diagnostic pop