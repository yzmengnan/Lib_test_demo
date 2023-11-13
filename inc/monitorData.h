/*
 * @Author: YangQ
 * @Date: 2023-02-28 16:41:33
 * @LastEditors: YangQ
 * @LastEditTime: 2023-03-08 21:34:27
 * @FilePath: \Demo0\INC\monitorData.h
 * @Description:
 *
 * Copyright (c) 2023 by YangQ, All Rights Reserved.
*/
#pragma once
//#ifndef __MYTHREAD__
//#define __MYTHREAD__

#include "DATA_STRUCT.h"
#include "Driver.h"
#include "Tc_Ads.h"
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include<memory>
#include "Multi_Process.h"
#include "Driver.h"
using namespace std;
class MotionV1;
using mt = class monitorData {
public:
    void sendMessage(const MotionV1& m1,const string& name);
private:
    int error_code = 0;
};

//#endif