/*
 * @Author: YangQ
 * @Date: 2023-03-08 09:46:45
 * @LastEditors: YangQ
 * @LastEditTime: 2023-03-08 10:01:14
 * @FilePath: \Demo0\SRC\TimerCounter.cpp
 * @Description:
 *
 * Copyright (c) 2023 by YangQ, All Rights Reserved.
 */
// #include "StdAfx.h"
#include "TimerCounter.h"

using namespace std;

TimerCounter::TimerCounter() {
    QueryPerformanceFrequency(&freq); // 获取CPU时钟
}

// TimerCounter::~TimerCounter()
//     = default;
void TimerCounter::Start() {
    QueryPerformanceCounter(&startCount); // 计数
}

void TimerCounter::Stop() {
    QueryPerformanceCounter(&endCount); // 停止计数

    dbTime = ((double) endCount.QuadPart - (double) startCount.QuadPart) / (double) freq.QuadPart; // 计时
}
