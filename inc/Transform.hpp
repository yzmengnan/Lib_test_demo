#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
//
// Created by 91418 on 2023/11/6.
//
#ifndef JACOBDEMO_TRANSFORM_HPP
#define JACOBDEMO_TRANSFORM_HPP
#include <armadillo>
using namespace std;
using namespace arma;
inline auto angles2Rad(const vector<double> &data) -> vector<double> {
    vector<double> rads{};
    for (const auto &d: data) {
        rads.push_back(d / 180 * M_PI);
    }
    return rads;
}
inline auto rad2Angles(const vector<double> &data) {
    vector<double> degrees{};
    for (const auto &d: data) {
        degrees.push_back(d / M_PI * 180);
    }
    return degrees;
}
/*!
 * @details from joint angles(degrees) to operational space positions, the posture is shown
 *          as RPY angles
 * @param jointData degrees
 * @return
 */
inline vector<double> fkine(const vector<double> &jointData) {
    if (jointData.size() < 6) {
        return vector<double>{};
    }
    auto jointRad = angles2Rad(jointData);
    mat t01 = {{cos(jointRad[0]), -sin(jointRad[0]), 0, 0},
               {sin(jointRad[0]), cos(jointRad[0]), 0, 0},
               {0, 0, 1, 0},
               {0, 0, 0, 1}};
    mat t12 = {{cos(jointRad[1]), -sin(jointRad[1]), 0, 0},
               {0, 0, 1, 1.002},
               {-sin(jointRad[1]), -cos(jointRad[1]), 0, 0},
               {0, 0, 0, 1}};
    mat t23 = {{cos(jointRad[2]), -sin(jointRad[2]), 0, 0},
               {0, 0, -1, 0},
               {sin(jointRad[2]), cos(jointRad[2]), 0, 0},
               {0, 0, 0, 1}};
    mat t34 = {{cos(jointRad[3]), -sin(jointRad[3]), 0, 0},
               {0, 0, 1, 1.1115},
               {-sin(jointRad[3]), -cos(jointRad[3]), 0, 0},
               {0, 0, 0, 1}};
    mat t45 = {{cos(jointRad[4]), -sin(jointRad[4]), 0, 0},
               {0, 0, -1, 0},
               {sin(jointRad[4]), cos(jointRad[4]), 0, 0},
               {0, 0, 0, 1}};
    mat t56 = {{cos(jointRad[5]), -sin(jointRad[5]), 0, 0},
               {0, 0, 1, 0},
               {-sin(jointRad[5]), -cos(jointRad[5]), 0, 0},
               {0, 0, 0, 1}};
    mat t02 = t01 * t12;
    mat t03 = t02 * t23;
    mat t04 = t03 * t34;
    mat t05 = t04 * t45;
    mat t06 = t05 * t56;
//    t06.print("t06: ");
    double alpha{};
    double gama{};
    double beta = atan2(-t06(2, 0), sqrt(t06(0, 0) * t06(0, 0) + t06(1, 0) * t06(1, 0)));
    if(abs(abs(t06(0,2))-1)<0.001){
        alpha = 0;
        if(t06(0,2)>0){
            gama = atan2(t06(2,1),t06(1,1));
        }
        else{
            gama = -atan2(t06(1,0),t06(2,0));
        }
        beta = asin(t06(0,2));
    }
    else{
        alpha = -atan2(t06(0,1),t06(0,0));
        gama  = -atan2(t06(1,2),t06(2,2));
        vector<double> temp_data = {abs(t06(0,0)),abs(t06(0,1)),abs(t06(1,2)),abs(t06(2,2))};
        auto k = std::distance(temp_data.begin(), max_element(temp_data.begin(),temp_data.end()));
        switch (k){
            case 0:
                beta = atan(t06(0,2)*cos(alpha)/t06(0,0));
                break;
            case 1:
                beta = -atan(t06(0,2)*sin(alpha)/t06(0,1));
                break;
            case 2:
                beta = -atan(t06(0,2)*sin(gama)/t06(1,2));
                break;
            case 3:
                beta = atan(t06(0,2)*cos(gama)/t06(2,2));
                break;
        };
    }
    /*
    if (abs(beta - M_PI_2) < 0.001) {
        beta = M_PI_2;
        alpha = 0;
        gama = atan2(t06(0, 1), t06(1, 1));
    } else if (abs(beta + M_PI_2) < 0.001) {
        beta = -M_PI_2;
        alpha = 0;
        gama = -atan2(t06(0, 1), t06(1, 1));
    } else {
        alpha = atan2(t06(1, 0) / cos(beta), t06(0, 0) / cos(beta));
        gama = atan2(t06(2, 1) / cos(beta), t06(2, 2) / cos(beta));
    }
     */
    return vector<double>{t06(0, 3), t06(1, 3), t06(2, 3), alpha * 180 / M_PI, beta * 180 / M_PI, gama * 180 / M_PI};
}
/*!
 * @details from operational space position to joint space position, the posture should be discrbied as RPY angles
 * @param positionData operational space positions
 * @param currentAngles current close positions
 * @return
 */
inline auto ikine(const vector<double> &positionData, const vector<double> &currentAngles) -> vector<double> {
    // transform the angles to rad
    vector<double> p{positionData.begin(), positionData.begin() + 3};
    p.push_back(positionData[3] / 180 * M_PI);
    p.push_back(positionData[4] / 180 * M_PI);
    p.push_back(positionData[5] / 180 * M_PI);
    auto ca = angles2Rad(currentAngles);
    vector<double> jointData(6, 0);
    const float d4 = 1.1115;
    const float d2 = 1.002;
    const auto r = (pow(p[0], 2) + pow(p[1], 2) + pow(p[2], 2));
    jointData[2] = acos((r - d4 * d4 - d2 * d2) / (2 * d4 * d2));
//    jointData[2] = abs(jointData[2] - ca[2]) > abs(-jointData[2] - ca[2]) ? -jointData[2] : jointData[2];
    jointData[1] = asin(p[2] / d4 / sin(jointData[2]));
//    jointData[1] = abs(jointData[1] - ca[1]) > abs(M_PI - jointData[1] - ca[1]) ? M_PI - jointData[1] : jointData[1];
    auto a = 1.1115 * cos(jointData[1]) * sin(jointData[2]);
    auto b = 1.1115 * cos(jointData[2]) + 1.002;
    auto c = -p[0];
    jointData[0] = asin(c / sqrt(pow(a, 2) + pow(b, 2))) - atan(a / b);
    jointData[0] = abs(jointData[0] - ca[0]) > abs(M_PI - jointData[0] - ca[0]) ? M_PI - jointData[0] : jointData[0];
//    jointData[0] = jointData[2] > M_PI ? M_PI * 2 - jointData[2] : jointData[2];
    double c1 = cos(jointData[0]);
    double c2 = cos(jointData[1]);
    double c3 = cos(jointData[2]);
    double s1 = sin(jointData[0]);
    double s2 = sin(jointData[1]);
    double s3 = sin(jointData[2]);
    mat r01 = {{c1, -s1, 0}, {s1, c1, 0}, {0, 0, 1}};
    mat r12 = {{c2, -s2, 0}, {0, 0, 1}, {-s2, -c2, 0}};
    mat r23 = {{c3, -s3, 0}, {0, 0, -1}, {s3, c3, 0}};
    mat r03 = r01 * r12 * r23;
    mat rzAlpha = {{cos(p[3]), -sin(p[3]), 0},
                   {sin(p[3]), cos(p[3]), 0},
                   {0, 0, 1}};
//    rzAlpha.print("rz");
    mat ryBeta = {{cos(p[4]), 0, sin(p[4])},
                  {0, 1, 0},
                  {-sin(p[4]), 0, cos(p[4])}};
//    ryBeta.print("ry");
    mat rxGama = {{1, 0, 0},
                  {0, cos(p[5]), -sin(p[5])},
                  {0, sin(p[5]), cos(p[5])}};
//    rxGama.print("rx");
    mat r06 = mat(rxGama) * mat(ryBeta) * mat(rzAlpha);
//    r06.print("r06: ");
    mat r36 = (r03.t()) * r06;
    jointData[4] = atan2(sqrt(r36(1, 0) * r36(1, 0) + r36(1, 1) * r36(1, 1)), r36(1, 2));
    jointData[4] = abs(jointData[4] - ca[4]) > abs(M_PI - jointData[4] - ca[4]) ? M_PI - jointData[4] : jointData[4];
    if (abs(jointData[4]) < 0.001) {
        jointData[4] = 0;
        jointData[3] = 0;
        jointData[5] = atan2(-r36(2, 1), r36(0, 0));
    } else if (abs(abs(jointData[4]) - M_PI) <= 0.001) {
        jointData[4] = M_PI;
        jointData[3] = 0;
        jointData[5] = atan2(r36(2, 0), r36(0, 0));
    } else {
        jointData[3] = atan2(r36(2, 2) / sin(jointData[4]), -r36(0, 2) / sin(jointData[4]));
        jointData[3] = abs(jointData[3] - ca[3]) > abs(M_PI - jointData[3] - ca[3]) ? M_PI - jointData[3] : jointData[3];
        jointData[5] = acos(r36(1, 0) / sin(jointData[4]));
        jointData[5] = abs(jointData[5] - ca[5]) > abs(-jointData[5] - ca[5]) ? -jointData[5] : jointData[5];
    }
    return rad2Angles(jointData);
}
/*!
 * @details get jacob of the robot base frame
 * @param jointData
 * @return
 */
inline auto jacob0(vector<double> &jointData) -> mat {
    //only used for 6-DOF
    if (jointData.size() != 6) {
        return {};
    }

    auto jointRad = angles2Rad(jointData);

    rowvec jacobvx{-1.002F * cos(jointRad[0]) - 1.1115F * cos(jointRad[2]) * cos(jointRad[0]) + 1.1115F * sin(jointRad[0]) * cos(jointRad[1]) * sin(jointRad[2]),
                   1.1115F * cos(jointRad[0]) * sin(jointRad[1]) * sin(jointRad[2]),
                   1.1115F * sin(jointRad[2]) * sin(jointRad[0]) - 1.1115F * cos(jointRad[0]) * cos(jointRad[1]) * cos(jointRad[2]),
                   0, 0, 0};
    rowvec jacobv2{-1.002F * sin(jointRad[0]) - 1.1115F * sin(jointRad[0]) * cos(jointRad[2]) - 1.1115F * cos(jointRad[1]) * cos(jointRad[0]) * sin(jointRad[2]),
                   1.1115F * sin(jointRad[0]) * sin(jointRad[1]) * sin(jointRad[2]),
                   -1.1115F * cos(jointRad[0]) * sin(jointRad[2]) - 1.1115F * sin(jointRad[0]) * cos(jointRad[1]) * cos(jointRad[2]),
                   0, 0, 0};
    rowvec jacobv3{0,
                   1.1115F * cos(jointRad[1]) * sin(jointRad[2]),
                   1.1115F * sin(jointRad[1]) * cos(jointRad[2]),
                   0, 0, 0};
    mat t01{{cos(jointRad[0]), -sin(jointRad[0]), 0, 0},
            {sin(jointRad[0]), cos(jointRad[0]), 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}};
    mat t12{{cos(jointRad[1]), -sin(jointRad[1]), 0, 0},
            {0, 0, 1, 1.002},
            {-sin(jointRad[1]), -cos(jointRad[1]), 0, 0},
            {0, 0, 0, 1}};
    mat t23{{cos(jointRad[2]), -sin(jointRad[2]), 0, 0},
            {0, 0, -1, 0},
            {sin(jointRad[2]), cos(jointRad[2]), 0, 0},
            {0, 0, 0, 1}};
    mat t34{{cos(jointRad[3]), -sin(jointRad[3]), 0, 0},
            {0, 0, 1, 1.1115},
            {-sin(jointRad[3]), -cos(jointRad[3]), 0, 0},
            {0, 0, 0, 1}};
    mat t45{{cos(jointRad[4]), -sin(jointRad[4]), 0, 0},
            {0, 0, -1, 0},
            {sin(jointRad[4]), cos(jointRad[4]), 0, 0},
            {0, 0, 0, 1}};
    mat t56{{cos(jointRad[5]), -sin(jointRad[5]), 0, 0},
            {0, 0, 1, 0},
            {-sin(jointRad[5]), -cos(jointRad[5]), 0, 0},
            {0, 0, 0, 1}};
    mat t02 = t01 * t12;
    mat t03 = t02 * t23;
    mat t04 = t03 * t34;
    mat t05 = t04 * t45;
    mat t06 = t05 * t56;
    mat jacobw{
            {t01(0, 2), t01(1, 2), t01(2, 2)},
            {t02(0, 2), t02(1, 2), t02(2, 2)},
            {t03(0, 2), t03(1, 2), t03(2, 2)},
            {t04(0, 2), t04(1, 2), t04(2, 2)},
            {t05(0, 2), t05(1, 2), t05(2, 2)},
            {t06(0, 2), t06(1, 2), t06(2, 2)}};
    jacobw = jacobw.t();
    return join_vert(jacobvx, jacobv2, jacobv3, jacobw);
}
/*!
 * @details get jacob of the robot endeffector frame
 * @param joint_data
 * @return
 */
inline auto jacobe(vector<double> &joint_data) -> mat {
    //only used for 6-DOF
    if (joint_data.size() != 6) {
        return {};
    }
    auto jointRad = angles2Rad(joint_data);
    rowvec jacobv1{-1.002F * cos(jointRad[0]) - 1.1115F * cos(jointRad[2]) * cos(jointRad[0]) + 1.1115F * sin(jointRad[0]) * cos(jointRad[1]) * sin(jointRad[2]),
                1.1115F * cos(jointRad[0]) * sin(jointRad[1]) * sin(jointRad[2]),
                1.1115F * sin(jointRad[2]) * sin(jointRad[0]) - 1.1115F * cos(jointRad[0]) * cos(jointRad[1]) * cos(jointRad[2]),
                0, 0, 0};
    rowvec jacobv2{-1.002F * sin(jointRad[0]) - 1.1115F * sin(jointRad[0]) * cos(jointRad[2]) - 1.1115F * cos(jointRad[1]) * cos(jointRad[0]) * sin(jointRad[2]),
                1.1115F * sin(jointRad[0]) * sin(jointRad[1]) * sin(jointRad[2]),
                -1.1115F * cos(jointRad[0]) * sin(jointRad[2]) - 1.1115F * sin(jointRad[0]) * cos(jointRad[1]) * cos(jointRad[2]),
                0, 0, 0};
    rowvec jacobv3{0,
                1.1115F * cos(jointRad[1]) * sin(jointRad[2]),
                1.1115F * sin(jointRad[1]) * cos(jointRad[2]),
                0, 0, 0};

    mat t01{{cos(jointRad[0]), -sin(jointRad[0]), 0, 0},
            {sin(jointRad[0]), cos(jointRad[0]), 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1}};
    mat t12{{cos(jointRad[1]), -sin(jointRad[1]), 0, 0},
            {0, 0, 1, 1.002},
            {-sin(jointRad[1]), -cos(jointRad[1]), 0, 0},
            {0, 0, 0, 1}};
    mat t23{{cos(jointRad[2]), -sin(jointRad[2]), 0, 0},
            {0, 0, -1, 0},
            {sin(jointRad[2]), cos(jointRad[2]), 0, 0},
            {0, 0, 0, 1}};
    mat t34{{cos(jointRad[3]), -sin(jointRad[3]), 0, 0},
            {0, 0, 1, 1.1115},
            {-sin(jointRad[3]), -cos(jointRad[3]), 0, 0},
            {0, 0, 0, 1}};
    mat t45{{cos(jointRad[4]), -sin(jointRad[4]), 0, 0},
            {0, 0, -1, 0},
            {sin(jointRad[4]), cos(jointRad[4]), 0, 0},
            {0, 0, 0, 1}};
    mat t56{{cos(jointRad[5]), -sin(jointRad[5]), 0, 0},
            {0, 0, 1, 0},
            {-sin(jointRad[5]), -cos(jointRad[5]), 0, 0},
            {0, 0, 0, 1}};
    mat t02 = t01 * t12;
    mat t03 = t02 * t23;
    mat t04 = t03 * t34;
    mat t05 = t04 * t45;
    mat t06 = t05 * t56;
    mat jacobw{
            {t01(0, 2), t01(1, 2), t01(2, 2)},
            {t02(0, 2), t02(1, 2), t02(2, 2)},
            {t03(0, 2), t03(1, 2), t03(2, 2)},
            {t04(0, 2), t04(1, 2), t04(2, 2)},
            {t05(0, 2), t05(1, 2), t05(2, 2)},
            {t06(0, 2), t06(1, 2), t06(2, 2)}};
    jacobw = jacobw.t();
    mat j0 = join_vert(jacobv1, jacobv2, jacobv3, jacobw);
    mat re0{{t06(0, 0), t06(1, 0), t06(2, 0), 0, 0, 0},
            {t06(0, 1), t06(1, 1), t06(2, 1), 0, 0, 0},
            {t06(0, 2), t06(1, 2), t06(2, 2), 0, 0, 0},
            {0, 0, 0, t06(0, 0), t06(1, 0), t06(2, 0)},
            {0, 0, 0, t06(0, 1), t06(1, 1), t06(2, 1)},
            {0, 0, 0, t06(0, 2), t06(1, 2), t06(2, 2)}};
    return re0 * j0;
}
/*!
 * @details get jacob of the robot endeffector frame, the postrue is related to RPY vecs
 * @param joint_data
 * @return
 */
inline auto jacoba(vector<float> &joint_data) -> mat { return {}; }
/*!
 * @details get the manipulibility of the robot configurations at current joint position
 *          the returned value is scalar and should be well scalared
 * @param a
 * @return
 */
inline auto getManipulability(const mat &a) {
    return det(a * a.t());
}
#endif//JACOBDEMO_TRANSFORM_HPP

#pragma clang diagnostic pop