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
#include "windows.h"
#include "TcAdsDef.h"
//
#include "TcAdsAPI.h"

using namespace std;
class Tc_Ads {
public: Tc_Ads();
    auto set(vector<DTS> &SendData)->int;
    auto get(vector<DFS> &GetData)->int;
    PAmsAddr pAddr = &Addr;
private:
    long nPort{},nErr{};
    AmsAddr Addr{};
};
using pTc_Ads = Tc_Ads*;
#endif//SEAL_DEMO_TC_ADS_H
