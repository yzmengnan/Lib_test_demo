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
    if (nErr)
        cout << "Error: Ads: Open port: " << nErr << endl;
    pAddr->port = 851;
    cout<<"ADS CONNECTION BUILT!"<<endl;
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
    nErr = AdsSyncReadReq(pAddr,INPUT_BASE,INPUT_OFFSET,DFS_SIZE*GetData.size(),GetData.data());
    if(nErr){
//        cout<<"Error: Ads get error: "<<nErr<<endl;
        return -102;
    }
    return 0;
}
