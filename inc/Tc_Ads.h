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

    /**
     * \brief 用于继承类的编写，调用基类的ads发送与接受
     * @tparam a
     * @tparam b
     * @tparam c
     * @param outputBase
     * @param offsetAddress
     * @param dataSize
     * @param data
     * @return
     */
    template<typename a, typename b, typename  c>
    int ads_send(const a& outputBase, const unsigned long& offsetAddress, const b& dataSize, const c& data)
    {
        nErr = AdsSyncWriteReq(pAddr, outputBase, offsetAddress, dataSize, data);
        if (nErr != 0)
        {
            std::cout << "Error: Ads send error: " << nErr << '\n';
            return nErr;
        }
        return 0;
    }

    template<typename a, typename b,typename  c>
    int ads_receive(const a& inputBase, const unsigned long& offsetAddress,const b& dataSize, const c& data)
    {
        nErr = AdsSyncReadReq(pAddr, inputBase, offsetAddress, dataSize, data);
        if (nErr != 0)
        {
            std::cout << "Error: Ads receive error: " << nErr << '\n';
            return nErr;
        }
        return 0;
    }

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

class gp_ads:public Tc_Ads{
public:
    gp_ads(const ptr_v<DTG_P> &tx, const ptr_v<DFG_P> &rx);
    void send();
    void receive();
private:
    std::shared_ptr<std::vector<DTG_P>> tx_data=nullptr;
    std::shared_ptr<std::vector<DFG_P>> rx_data=nullptr;
    int plc_port{};
};
class gt_ads:public Tc_Ads{
public:
    gt_ads(const ptr_v<DTG_T> &tx, const ptr_v<DFG_T> &rx);
    void send();
    void receive();
private:
    std::shared_ptr<std::vector<DTG_T>> tx_data=nullptr;
    std::shared_ptr<std::vector<DFG_T>> rx_data=nullptr;
    int plc_port{};
};
#endif//SEAL_DEMO_TC_ADS_H
