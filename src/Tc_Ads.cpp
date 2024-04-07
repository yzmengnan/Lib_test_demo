//
// Created by 91418 on 2023/6/26.
//

#include "Tc_Ads.h"
#include <iostream>
/*
 * @description: build pAddr
 */
Tc_Ads::Tc_Ads() {
    nPort = AdsPortOpen();
    nErr = AdsGetLocalAddress(pAddr);
    if (nErr != 0)
        cout << "Error: Ads: Open port: " << nErr << endl;
    pAddr->port = PLC1_PORT;
    cout << "ADS CONNECTION BUILT!" << endl;
}
/*
 * &description: send data
 */
auto Tc_Ads::set(vector<DTS> &SendData) -> int {
    nErr = AdsSyncWriteReq(pAddr, OUTPUT_BASE, OUTPUT_OFFSET, DTS_SIZE * SendData.size(), SendData.data());
    if (nErr) {
        cout << "Error: Ads send error: " << nErr << endl;
        return -101;
    }
    return 0;
}
/*
 * @description: get data
 */
auto Tc_Ads::get(vector<DFS> &GetData) -> int {
    nErr = AdsSyncReadReq(pAddr, INPUT_BASE, INPUT_OFFSET, DFS_SIZE * GetData.size(), GetData.data());
    if (nErr) {
        //        cout<<"Error: Ads get error: "<<nErr<<endl;
        return -102;
    }
    return 0;
}

TcAds_Grap_Position_Control::TcAds_Grap_Position_Control() {
    nPort = AdsPortOpen();
    nErr = AdsGetLocalAddress(pAddr);
    if (nErr != 0)
        cout << "Error: Ads: Open port: " << nErr << endl;
    pAddr->port = PLC2_PORT;
    cout << "GRAP ADS CONNECTION BUILT!" << endl;
}
int TcAds_Grap_Position_Control::set(vector<DTG_P> &SendData) {

    nErr = AdsSyncWriteReq(pAddr, OUTPUT_BASE, OUTPUT_OFFSET, DTG_SIZE_P * SendData.size(), SendData.data());
    if (nErr) {
        cout << "Error: Ads send error: " << nErr << endl;
        return -101;
    }
    return 0;
}
int TcAds_Grap_Position_Control::get(vector<DFG_P> &GetData) {

    nErr = AdsSyncReadReq(pAddr, INPUT_BASE, INPUT_OFFSET, DFG_SIZE_P * GetData.size(), GetData.data());
    if (nErr) {
        //        cout<<"Error: Ads get error: "<<nErr<<endl;
        return -102;
    }
    return 0;
}
TcAds_Grap_Torque_Control::TcAds_Grap_Torque_Control() {
    nPort = AdsPortOpen();
    nErr = AdsGetLocalAddress(pAddr);
    if (nErr != 0)
        cout << "Error: Ads: Open port: " << nErr << endl;
    pAddr->port = PLC2_PORT;
    cout << "GRAP ADS CONNECTION BUILT!" << endl;
}
int TcAds_Grap_Torque_Control::set(vector<DTG_T> &SendData) {

    nErr = AdsSyncWriteReq(pAddr, OUTPUT_BASE, OUTPUT_OFFSET_GRAP_TORQUE, DTG_SIZE_T * SendData.size(), SendData.data());
    if (nErr) {
        cout << "Error: Ads send error: " << nErr << endl;
        return -101;
    }
    return 0;
}
int TcAds_Grap_Torque_Control::get(vector<DFG_T> &GetData) {

    nErr = AdsSyncReadReq(pAddr, INPUT_BASE, INPUT_OFFSET_GRAP_TORQUE, DFG_SIZE_T * GetData.size(), GetData.data());
    if (nErr) {
        //        cout<<"Error: Ads get error: "<<nErr<<endl;
        return -102;
    }
    return 0;
}
gp_ads::gp_ads(const ptr_v<DTG_P> &tx, const ptr_v<DFG_P> &rx) : plc_port{PLC2_PORT} {
    this->pAddr->port = PLC2_PORT;
    this->tx_data = tx;
    this->rx_data = rx;
    std::cout << "grap position ads built!" << std::endl;
}
void gp_ads::send() {
    ads_send(OUTPUT_BASE, OUTPUT_OFFSET, DTG_SIZE_P * Grap_Position_Servo_Nums, tx_data->data());
}
void gp_ads::receive() {
    ads_receive(INPUT_BASE, INPUT_OFFSET, DFG_SIZE_P * Grap_Position_Servo_Nums, rx_data->data());
}
gt_ads::gt_ads(const ptr_v<DTG_T> &tx, const ptr_v<DFG_T> &rx) : plc_port{PLC2_PORT} {
    this->pAddr->port = PLC2_PORT;
    this->tx_data = tx;
    this->rx_data = rx;
    std::cout << "grap position ads built!" << std::endl;
}
void gt_ads::send() {
    ads_send(OUTPUT_BASE, OUTPUT_OFFSET_GRAP_TORQUE, DTG_SIZE_T * Grap_Torque_Servo_Nums, tx_data->data());
}
void gt_ads::receive() {
    ads_receive(INPUT_BASE, INPUT_OFFSET_GRAP_TORQUE, DFG_SIZE_T * Grap_Torque_Servo_Nums, rx_data->data());
}
