//
// Created by 91418 on 2023/7/29.
//

#ifndef MAIN_DRIVERSEVER_H
#define MAIN_DRIVERSEVER_H


#include "DATA_STRUCT.h"
#include "Driver.h"
#include "comSocket.h"
#include "iostream"
using namespace std;
class driverSever :  public MotionV1,comSocket{
public:
    /*
     * @description: driverServer类为中位机操作总类，包含套接字对象与驱动控制器对象
     */
    driverSever(const int &port, Tc_Ads &ads_handle);
    int state{};
private:
    void servoData_to_socketData(const Driver &d, const vector<DFS> &data);
    int enableFlag{};
    int ppFlag{};
};


#endif//MAIN_DRIVERSEVER_H
