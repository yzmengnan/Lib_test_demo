//
// Created by 91418 on 2023/7/29.
//

#include "driverSever.h"
driverSever::driverSever(const int &port, Tc_Ads &ads_handle) : MotionV1{ads_handle}, comSocket{port} {
    this_thread::sleep_for(std::chrono::seconds(2));
    cout << "driver Server start!" << endl;
    //lamda: 将伺服器数据实时转译为套接字数据
    auto Server_StatusBuilder = [&]() {
        //driver_errcode 表示操作伺服器对象错误码
        //       while(driver_errcode>=0){
        //Test
        while (true) {
            servoStatusLock.lock();
            this->servoData_to_socketData(*this, this->MotGetData);
            servoStatusLock.unlock();
        }
    };
    //lamda: 解释套接字数据
    auto Server_Controller = [&]() {
        //        while (driver_errcode >=0) {
        //Test
        while (true) {
            //如果套接字通讯错误,则直接退出
            if (this->socketResult < 0) {
                cout << "Command: socket error! Check: " << socketResult << endl;
                break;
            }
            //使能
            if (this->socketRecv->Command & 0b10) {
                if (!enableFlag) {
                    cout << "Command: operation Enable!" << endl;
                    driver_errcode = this->Enable();
                    if (driver_errcode < 0) {
                        cout << "Command error! Check: " << driver_errcode << endl;
                        break;
                    } else
                        enableFlag = 1;
                }
                // 所有伺服活动都在使能状态下进行
                //PP
                if (this->socketRecv->Command & 0b100) {
                    if (!ppFlag) {
                        cout << "Command: PP Enable!" << endl;
                        ppFlag = 1;
                    }
                    this->servoFinishCS();
                    this->setSyncrpm(100);
                    driver_errcode = this->Write('1',
                                                 this->socketRecv->Joint_Position_set[0], this->socketRecv->Joint_Position_set[1],
                                                 this->socketRecv->Joint_Position_set[2], this->socketRecv->Joint_Position_set[3],
                                                 this->socketRecv->Joint_Position_set[4], this->socketRecv->Joint_Position_set[5],
                                                 this->socketRecv->Joint_Position_set[6], this->socketRecv->Joint_Position_set[7],
                                                 this->socketRecv->Joint_Position_set[8]);
                    if (driver_errcode < 0) {
                        cout << "Command error in PP! check: " << driver_errcode << endl;
                    }
                }
                else{
                    ppFlag=0;
                    this->servoOperationModeSet(ppFlag,cspFlag,0);
                }
                //CSP
                if (this->socketRecv->Command & 0b1000) {
                    if (ppFlag) {
                        cout << "Command error: pp is enable now!" << endl;
                        continue;
                    }
                    if (!cspFlag) {
                        cout << "Command: CSP Enable!" << endl;
                        cspFlag = 1;
                        //refresh sendData
                        for (auto &s: sendData) {
                            s.Control_Word = 15;
                        }
                    }
                    vector<float> angles{
                            this->socketRecv->Joint_Position_set[0],
                            this->socketRecv->Joint_Position_set[1],
                            this->socketRecv->Joint_Position_set[2],
                            this->socketRecv->Joint_Position_set[3],
                            this->socketRecv->Joint_Position_set[4],
                            this->socketRecv->Joint_Position_set[5],
                            this->socketRecv->Joint_Position_set[6],
                            this->socketRecv->Joint_Position_set[7],
                            this->socketRecv->Joint_Position_set[8],
                    };
                    MDT::fromAnglesToPulses(*this, angles, sendData);
                    driver_errcode = this->servoCSP(sendData, getData);
                    if (driver_errcode < 0) {
                        cout << "Command error in CSP! check: " << driver_errcode << endl;
                    }
                } else {
                    cspFlag = 0;
                    this->servoOperationModeSet(ppFlag,cspFlag,0);
                }
            }
            // 如果没有上使能指令，则下使能
            else {
                driver_errcode = this->Disable();
                enableFlag = 0;
                ppFlag = 0;
            }
            if (driver_errcode < 0) {
                cout << "Command error! Check: " << driver_errcode << endl;
                break;
            }
        }
        //任何异常退出后，driverServer类的状态标志置-1
        state = -1;
    };
    thread f1(Server_Controller);
    f1.detach();
    thread f2(Server_StatusBuilder);
    f2.detach();
}
void driverSever::servoData_to_socketData(const Driver &d, const vector<DFS> &data) {
    this->socketSend->Tail_check = 0;
    this->socketSend->Head_check = 22;
    if (this->enableFlag) {
        this->socketSend->Status |= 0b10;
    } else {
        this->socketSend->Status &= ~((uint32_t) 2);
    }
    this->socketSend->Joint_Position = MDT::getAngles(d, data);
    this->socketSend->Joint_Velocity = MDT::getVecs(d, data);
}
