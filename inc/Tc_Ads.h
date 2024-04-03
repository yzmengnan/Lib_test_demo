//
// Created by 91418 on 2023/6/26.
//

#ifndef SEAL_DEMO_TC_ADS_H
#define SEAL_DEMO_TC_ADS_H


#include "winsock2.h"
//
#include "ADDRESS_DEFINE.h"
#include "DATA_STRUCT.h"
#include "thread"
#include <vector>
//
#include "TcAdsDef.h"
#include "windows.h"
//
#include "TcAdsAPI.h"

#define PLC2_PORT 852
#define PLC1_PORT 851
using namespace std;
class Tc_Ads {
public:
    Tc_Ads();
    auto set(vector<DTS> &SendData) -> int;
    auto get(vector<DFS> &GetData) -> int;
    PAmsAddr pAddr = &Addr;

private:
    long nPort{}, nErr{};
    AmsAddr Addr{};
};
using pTc_Ads = Tc_Ads *;

// 创建两个类，Grap为手爪Ethercat 控制部分，需要独立的Ethercat地址与操作业务逻辑
class TcAds_Grap_Position_Control {
public:
    TcAds_Grap_Position_Control();
    int set(vector<DTG_P> &SendData);
    int get(vector<DFG_P> &GetData);
    PAmsAddr pAddr = &Addr;

private:
    AmsAddr Addr{};
    long nPort{}, nErr{};
};
class TcAds_Grap_Torque_Control {
public:
    TcAds_Grap_Torque_Control();
    int set(vector<DTG_T> &SendData);
    int get(vector<DFG_T> &GetData);
    PAmsAddr pAddr = &Addr;
private:
    long nPort{}, nErr{};
    AmsAddr Addr{};
};
#endif//SEAL_DEMO_TC_ADS_H
