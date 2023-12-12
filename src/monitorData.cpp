/*
 * @Author: YangQ
 * @Date: 2023-02-28 16:43:54
 * @LastEditors: YangQ
 * @LastEditTime: 2023-03-08 11:10:46
 * @FilePath: \Demo0\SRC\monitorData.cpp
 * @Description:
 *
 * Copyright (c) 2023 by YangQ, All Rights Reserved.
 */
#include "monitorData.h"
#include "motionDataTransform.hpp"
#include <sstream>
#include <string>

void monitorData::sendMessage(const MotionV1 &m1, const string &name) {
    bool enableFlag = true;
    SetConsoleOutputCP(CP_UTF8);
    cout << "创建管道！" << endl;
    SECURITY_ATTRIBUTES saAttr;
    saAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
    saAttr.bInheritHandle = TRUE;
    saAttr.lpSecurityDescriptor = NULL;

    // Create a pipe for the child process's STDIN.
    HANDLE handle_read;
    HANDLE handle_write;
    bool ret = CreatePipe(&handle_read, &handle_write, &saAttr, 0);
    if (!ret) {
        cout << "创建管道失败：create pipe fail" << endl;
        enableFlag = false;
    }

    //设置写句柄不可以被子进程继承,不设置也不影响。 Ensure the write handle to the pipe for STDIN is not inherited.
    if (!SetHandleInformation(handle_write, HANDLE_FLAG_INHERIT, 0)) {
        cout << "设置句柄失败：set handle fail!" << endl;
        enableFlag = false;
    }
    Multi_Process p;
    if (p.data_monitor(name, handle_read) != 0) {
        enableFlag = false;
        cout << "监视进程创建失败！" << endl;
    }
    this_thread::sleep_for(chrono::milliseconds (2000));
    cout << "监视器开启！" << endl;
    DWORD len;
    ostringstream message;
    while (enableFlag) {
        //        更新:使用MotionV1版本内的MotGetData源

        //        vector<DFS> GetData(servoNUMs);
        //        auto servoData_success = m1.GetDataUpdate(GetData);
        //        if (servoData_success < 0) {
        //            cout << "Get Monitor Data error! Ads error: " << servoData_success << endl;
        //            ExitProcess(1);
        //        }
        auto now = std::chrono::system_clock::now();
        //通过不同精度获取相差的毫秒数
        uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
                                   std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
        time_t tt = std::chrono::system_clock::to_time_t(now);
        auto time_tm = localtime(&tt);
        char strTime[25] = {};
        sprintf(strTime, "%02d-%02d %02d:%02d:%02d %03d",
                time_tm->tm_mon + 1, time_tm->tm_mday, time_tm->tm_hour,
                time_tm->tm_min, time_tm->tm_sec, (int) dis_millseconds);
        message << "[" << string(strTime) << "]";
        auto fadd_space = [](float data) {
            string s = to_string(data);
            s = s.substr(0, s.find('.') + 4);
            while (s.size() < 8)
                s = " " + s;
            return s;
        };
        vector<vector<double>> data{MDT::getAngles(m1, m1.MotGetData),
                                   MDT::getVecs(m1, m1.MotGetData),
                                   MDT::getMoments(m1, m1.MotGetData)};
        vector<string> keys{"angles: ", "vecs: ", "moments: "};
        for (int i = 0; i < 3; i++) {
            message << keys[i];
            for (int j = 0; j < servoNums; j++) {
                message<<fadd_space(data[i][j]);
                if(j!=servoNums-1)
                message <<",";
            }
            message<<" ";
        }
        auto message_data = message.str();
        WriteFile(handle_write, message_data.data(), message_data.size(), &len, NULL);
        this_thread::sleep_for(chrono::milliseconds(1000/MONITOR_Hz));
        message.str(" ");
    }
    cout << "后台线程数据显示进程启动失败！" << endl;
}
