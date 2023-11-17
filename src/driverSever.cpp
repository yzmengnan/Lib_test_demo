//
// Created by 91418 on 2023/7/29.
//

#include "driverSever.h"
driverSever::driverSever(const int &port, Tc_Ads &ads_handle) : MotionV1{ads_handle}, comSocket{port} {
    this_thread::sleep_for(std::chrono::seconds(2));
    cout << "driver Server start!" << '\n';
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
                cout << "Command: socket error! Check: " << socketResult << '\n';
                break;
            }
            //使能
            if ((this->socketRecv->Command & 0b10) != 0) {
                if (enableFlag == 0) {
                    cout << "Command: operation Enable!" << '\n';
                    driver_errcode = this->Enable();
#ifndef SOCKET_TEST
                    if (driver_errcode < 0) {
                        cout << "Command error! Check: " << driver_errcode << '\n';
                        break;
                    }
#endif
                    enableFlag = 1;
                }
                // 所有伺服活动都在使能状态下进行
                //PP
                if ((this->socketRecv->Command & 0b100) != 0) {
                    if (ppFlag == 0) {
                        //重置CS flag，关闭后台Cyclic 线程
                        this->servoFinishCS();
                        cout << "Command: PP Enable!" << '\n';
                        ppFlag = 1;
                    }
                    this->setSyncrpm(100);
                    driver_errcode = this->Write('1',
                                                 this->socketRecv->Joint_Position_set[0], this->socketRecv->Joint_Position_set[1],
                                                 this->socketRecv->Joint_Position_set[2], this->socketRecv->Joint_Position_set[3],
                                                 this->socketRecv->Joint_Position_set[4], this->socketRecv->Joint_Position_set[5],
                                                 this->socketRecv->Joint_Position_set[6], this->socketRecv->Joint_Position_set[7],
                                                 this->socketRecv->Joint_Position_set[8]);
#ifndef SOCKET_TEST
                    if (driver_errcode < 0) {
                        cout << "Command error in PP! check: " << driver_errcode << '\n';
                    }
#endif
                } else {
                    ppFlag = 0;
                    this->servoOperationModeSet(ppFlag, cspFlag, 0);
                }
                //CSP--online
                if ((this->socketRecv->Command & 0b1000) != 0) {
                    if (ppFlag != 0) {
                        //若指令为0b1110,则PP模式与CSP共存，此时以PP优先，并warning
                        cout << "Command error: pp is enable now!" << '\n';
                        continue;
                    }
                    if (!cspFlag) {
                        cout << "Command: CSP Enable!" << '\n';
                        cspFlag = 1;
                        //refresh sendData
                        for (auto &s: sendData) {
                            s.Control_Word = 15;
                        }
                    }
                    //此处更新csp的位置数据
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
                    //servoCSP内部有自检，只会启动一次csp线程
                    driver_errcode = this->servoCSP(sendData, this->MotGetData);
#ifndef SOCKET_TEST
                    if (driver_errcode < 0) {
                        cout << "Command error in CSP! check: " << driver_errcode << '\n';
                    }
#endif
                } else {
                    cspFlag = 0;
                    this->servoOperationModeSet(ppFlag, cspFlag, 0);
                }
                //CSP--offline
                //data buffer
                if ((this->socketRecv->Command & 0b10000) != 0) {
                    //                    if (offline_pathPointsNums == 0){
                    //                        offline_pathPointsNums = this->socketRecv->Tail_check;
                    //                        this->socketRecv->Tail_check=0;
                    //                    }
                    //此处更新csp的位置数据
                    if (offline_pathPoints.size() == this->socketRecv->Tail_check - 1) {
                        cout << "Tail Check: " << this->socketRecv->Tail_check << endl;
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
                        offline_pathPoints.push_back(angles);
                        for (const auto &data: offline_pathPoints) {
                            for (const auto &data_second: data) {
                                cout << data_second << ",";
                            }
                            cout << endl;
                        }
                    }
                }
                //data runner
                //此处启动offline csp运动，生成轨迹，并本地csp方式执行轨迹
                if ((this->socketRecv->Command & 0b100000) != 0 && offline_pathPoints.size()) {
                    if (ppFlag != 0) {
                        //若指令为0b1110,则PP模式与CSP共存，此时以PP优先，并warning
                        cout << "Command error: pp is enable now!" << '\n';
                        continue;
                    }
                    if (!cspFlag) {
                        cout << "Command: CSP Enable!" << '\n';
                        //refresh sendData
                        for (auto &s: sendData) {
                            s.Control_Word = 15;
                        }
                    }
                    //清空路径插值数据
                    vector<vector<float>>().swap(offline_traj_data);
                    offline_traj_data = my_traj::_jtraj_Linear(offline_pathPoints, 10);
                    //清空获取的路径点数据
                    vector<vector<float>>().swap(offline_pathPoints);
                    MDT::fromAnglesToPulses(*this, offline_traj_data[0], sendData);
                    //demo版本，本地执行固定轨迹
                    driver_errcode = this->servoCSP(sendData, this->MotGetData);
#ifndef SOCKET_TEST
                    if (driver_errcode < 0) {
                        cout << "offline csp error!" << endl;
                        break;
                    }
#endif
                    auto offline_csp = [&]() {
                        cspFlag=1;
                        for (const auto &data: offline_traj_data) {
                            MDT::fromAnglesToPulses(*this, data, sendData);
                            this_thread::sleep_for(chrono::milliseconds(10));
                        }
                        cout<<"offline csp finish!"<<endl;
                        servoFinishCS();
                    };
                    if (cspFlag == 0) {
                        thread t_offline_csp(offline_csp);
                        t_offline_csp.detach();
                        this->servoOperationModeSet(ppFlag, cspFlag, 0);
                    }

                } else {
                    cspFlag = 0;
                    this->servoOperationModeSet(ppFlag, cspFlag, 0);
                }


            }
            // 如果没有上使能指令，则下使能
            else {
                driver_errcode = this->Disable();
                enableFlag = 0;
                ppFlag = 0;
                this->servoOperationModeSet(ppFlag, cspFlag, 0);
            }
#ifndef SOCKET_TEST
            if (driver_errcode < 0) {
                cout << "Command error! Check: " << driver_errcode << '\n';
                break;
            }
#endif
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
    if (this->enableFlag != 0) {
        this->socketSend->Status |= 0b10;
    } else {
        this->socketSend->Status = 0;
    }
    this->socketSend->Joint_Position = MDT::getAngles(d, data);
    //            for(const auto & joint:this->socketSend->Joint_Position){
    //                cout<<joint<<',';
    //            }
    //            cout<<endl;
    this->socketSend->Joint_Velocity = MDT::getVecs(d, data);
}
