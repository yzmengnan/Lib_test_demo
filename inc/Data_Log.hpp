//
// Created by 91418 on 2023/7/9.
//

#ifndef SEAL_DEMO_DATA_LOG_HPP
#define SEAL_DEMO_DATA_LOG_HPP
#include "DATA_STRUCT.h"
#include "motionDataTransform.hpp"
#include <cstdint>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>
using namespace std;
using file_log = class Data_Log {
public:
    void finishWrite() {
        cout << "finish record!" << endl;
        this->writeEnable = false;
        fp.close();
        fv.close();
        fm.close();
    }
    void writeFile(const MotionV1 &m) {
        SYSTEMTIME systime = {0};
        GetSystemTime(&systime);
        stringstream file_name;
        file_name << systime.wMonth << "-" << systime.wDay << "-"
                  << ((systime.wHour + 8) > 24 ? (systime.wHour + 8 / 24) + 1 : systime.wHour + 8)
                  << "-" << systime.wMinute;

        string file_address = "../data/";
        string file_position = file_address + file_name.str() + " angles.txt";
        string file_vec = file_address + file_name.str() + " vecs.txt";
        string file_moments = file_address + file_name.str() + " moments.txt";
        fp.open(file_position, ios::out);
        fv.open(file_vec, ios::out);
        fm.open(file_moments, ios::out);
        int index{};
        if (writeEnable) {
            cout << "start data record!" << endl;
        }
        TimerCounter tc;
        tc.Start();
        auto func = [&]() {
        while (this->writeEnable) {
            if (tc.dbTime * 1000 > 10) {
                tc.Start();
                {
                    stringstream data;
                    data << index;
                    for (auto d: MDT::getAngles(m, m.MotGetData)) {
                        data << " " << d;
                    }
                    fp << data.str() << endl;
                }
                {
                    stringstream data;
                    data << index;
                    for (auto d: MDT::getVecs(m, m.MotGetData)) {
                        data << " " << d;
                    }
                    fv << data.str() << endl;
                }
                {
                    stringstream data;
                    data << index;
                    for (auto d: MDT::getMoments(m, m.MotGetData)) {
                        data << " " << d;
                    }
                    fm << data.str() << endl;
                }
                index++;
            }
            tc.Stop();
            //            this_thread::sleep_for(chrono::milliseconds(10));
        } };
        thread t(func);
        t.detach();
    }
    virtual ~Data_Log() {
        finishWrite();
    }

private:
    vector<vector<float>> _angles{};
    vector<vector<float>> _moments{};
    vector<vector<float>> _velocities{};
    bool writeEnable = true;
    fstream fp, fv, fm;
};


#endif//SEAL_DEMO_DATA_LOG_HPP
