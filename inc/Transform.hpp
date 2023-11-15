//
// Created by 91418 on 2023/11/6.
//

#ifndef JACOBDEMO_TRANSFORM_HPP
#define JACOBDEMO_TRANSFORM_HPP
#include "mat.hpp"
#include "math.h"
vector<float> joint2Position(const vector<float> &joint_data) {
    if (joint_data.size() < 6) {
        return vector<float>{};
    }
    mat T01{{{cos(joint_data[0]), -sin(joint_data[0]), 0, 0},
             {sin(joint_data[0]), cos(joint_data[0]), 0, 0},
             {0, 0, 1, 0},
             {0, 0, 0, 1}}};
    mat T12{{{cos(joint_data[1]), -sin(joint_data[1]), 0, 0},
             {0, 0, 1, 1.002},
             {-sin(joint_data[1]), -cos(joint_data[1]), 0, 0},
             {0, 0, 0, 1}}};
    mat T23{{{cos(joint_data[2]), -sin(joint_data[2]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_data[2]), cos(joint_data[2]), 0, 0},
             {0, 0, 0, 1}}};
    mat T34{{{cos(joint_data[3]), -sin(joint_data[3]), 0, 0},
             {0, 0, 1, 1.1115},
             {-sin(joint_data[3]), -cos(joint_data[3]), 0, 0},
             {0, 0, 0, 1}}};
    mat T45{{{cos(joint_data[4]), -sin(joint_data[4]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_data[4]), cos(joint_data[4]), 0, 0},
             {0, 0, 0, 1}}};
    mat T56{{{cos(joint_data[5]), -sin(joint_data[5]), 0, 0},
             {0, 0, 1, 0},
             {-sin(joint_data[5]), -cos(joint_data[5]), 0, 0},
             {0, 0, 0, 1}}};
    mat T02 = T01 * T12;
    mat T03 = T02 * T23;
    mat T04 = T03 * T34;
    mat T05 = T04 * T45;
    mat T06 = T05 * T56;
    return vector<float>{T06.data[0][3], T06.data[1][3], T06.data[2][3]};
}
mat jacob0(const vector<float> &joint_data) {
    //only used for 6-DOF
    if(joint_data.size()!=6){
        return mat();
    }
    vector<float> jacobv1{static_cast<float>(-1.002 * cos(joint_data[0]) - 1.1115 * cos(joint_data[2]) * cos(joint_data[0]) + 1.115 * sin(joint_data[0]) * cos(joint_data[1]) * sin(joint_data[2])),
                          static_cast<float>(1.1115 * cos(joint_data[0]) * sin(joint_data[1]) * sin(joint_data[2])),
                          static_cast<float>(1.1115 * sin(joint_data[2]) * sin(joint_data[0]) - 1.1115 * cos(joint_data[0]) * cos(joint_data[1]) * cos(joint_data[2])),
                          0, 0, 0};
    vector<float> jacobv2{static_cast<float>(-1.002 * sin(joint_data[0]) - 1.1115 * sin(joint_data[0]) * cos(joint_data[2]) - 1.1115 * cos(joint_data[1]) * cos(joint_data[0]) * sin(joint_data[2])),
                          static_cast<float>(1.1115 * sin(joint_data[0]) * sin(joint_data[1]) * sin(joint_data[2])),
                          static_cast<float>(-1.1115 * cos(joint_data[0]) * sin(joint_data[2]) - 1.1115 * sin(joint_data[0]) * cos(joint_data[1]) * cos(joint_data[2])),
                          0, 0, 0};
    vector<float> jacobv3{0,
                          static_cast<float>(1.1115 * cos(joint_data[1]) * sin(joint_data[2])),
                          static_cast<float>(1.1115 * sin(joint_data[1]) * cos(joint_data[2])),
                          0, 0, 0};
    mat T01{{{cos(joint_data[0]), -sin(joint_data[0]), 0, 0},
             {sin(joint_data[0]), cos(joint_data[0]), 0, 0},
             {0, 0, 1, 0},
             {0, 0, 0, 1}}};
    mat T12{{{cos(joint_data[1]), -sin(joint_data[1]), 0, 0},
             {0, 0, 1, 1.002},
             {-sin(joint_data[1]), -cos(joint_data[1]), 0, 0},
             {0, 0, 0, 1}}};
    mat T23{{{cos(joint_data[2]), -sin(joint_data[2]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_data[2]), cos(joint_data[2]), 0, 0},
             {0, 0, 0, 1}}};
    mat T34{{{cos(joint_data[3]), -sin(joint_data[3]), 0, 0},
             {0, 0, 1, 1.1115},
             {-sin(joint_data[3]), -cos(joint_data[3]), 0, 0},
             {0, 0, 0, 1}}};
    mat T45{{{cos(joint_data[4]), -sin(joint_data[4]), 0, 0},
             {0, 0, -1, 0},
             {sin(joint_data[4]), cos(joint_data[4]), 0, 0},
             {0, 0, 0, 1}}};
    mat T56{{{cos(joint_data[5]), -sin(joint_data[5]), 0, 0},
             {0, 0, 1, 0},
             {-sin(joint_data[5]), -cos(joint_data[5]), 0, 0},
             {0, 0, 0, 1}}};
    mat T02 = T01 * T12;
    mat T03 = T02 * T23;
    mat T04 = T03 * T34;
    mat T05 = T04 * T45;
    mat T06 = T05 * T56;
    //    mat jacobw1{{{T01.data[0][2], T01.data[1][2], T01.data[2][2]}}};
    //    mat jacobw2{{{T02.data[0][2], T02.data[1][2], T02.data[2][2]}}};
    //    mat jacobw3{{{T03.data[0][2], T03.data[1][2], T03.data[2][2]}}};
    //    mat jacobw4{{{T04.data[0][2], T04.data[1][2], T04.data[2][2]}}};
    //    mat jacobw5{{{T05.data[0][2], T05.data[1][2], T05.data[2][2]}}};
    //    mat jacobw6{{{T06.data[0][2], T06.data[1][2], T06.data[2][2]}}};
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

#endif//JACOBDEMO_TRANSFORM_HPP
