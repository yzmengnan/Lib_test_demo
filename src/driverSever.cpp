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
                        //                        break;
                        continue;
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
                    //operational space motion by jacob0
                    if (this->socketRecv->Command & 0x10000000) {
                        auto rate = abs(this->socketRecv->Cartesian_Velocity_set[0]) >= 0.01 ? this->socketRecv->Cartesian_Velocity_set[0] : 1.0f;
                        this->setSyncrpm(rate * DEFAULT_SYNC_RPM_C);
                        vector<float> c_vecs = {this->socketRecv->Cartesian_Position_set[0],
                                                this->socketRecv->Cartesian_Position_set[1],
                                                this->socketRecv->Cartesian_Position_set[2],
                                                this->socketRecv->Cartesian_Position_set[3],
                                                this->socketRecv->Cartesian_Position_set[4],
                                                this->socketRecv->Cartesian_Position_set[5]};
                        driver_errcode = this->opSpaceMotionByJacob0(c_vecs);
                    }
                    //operation space moiton by jacobe
                    else if (this->socketRecv->Command & 0x20000000) {
                        auto rate = abs(this->socketRecv->Cartesian_Velocity_set[0]) >= 0.01 ? this->socketRecv->Cartesian_Velocity_set[0] : 1.0f;
                        this->setSyncrpm(rate * DEFAULT_SYNC_RPM_C);
                        vector<float> c_vecs = {this->socketRecv->Cartesian_Position_set[0],
                                                this->socketRecv->Cartesian_Position_set[1],
                                                this->socketRecv->Cartesian_Position_set[2],
                                                this->socketRecv->Cartesian_Position_set[3],
                                                this->socketRecv->Cartesian_Position_set[4],
                                                this->socketRecv->Cartesian_Position_set[5]};
#ifndef USE_RL
                        driver_errcode = this->opSpaceMotionByJacobe(c_vecs);
#else
                        static Robot r("../myrobot.xml");
                        driver_errcode = this->opSpaceMotionByJacobe_RL(c_vecs, r);
#endif
                    }
                    //PP set continus position motion
                    else if (this->socketRecv->Command & 0x8000000) {
                        //                        this->showSyncrpm();
                        // TODO: 检查单关机连续点动
                        vector<double> singleJointNeedMoving(9, 0);
                        auto jointDataNow = MDT::getAngles(*this, this->MotGetData);
                        int index{};
                        for (auto &item: singleJointNeedMoving) {
                            item = jointDataNow[index];
                            if (this->socketRecv->Joint_Position_set[index] >= 0.5f) {
                                item += 1.0f;
                            }
                            if (this->socketRecv->Joint_Position_set[index] <= -0.5f) {
                                item -= 1.0f;
                            }
                            index++;
                        }
                        double rate{};
                        if ((this->socketRecv->Joint_Velocity_set[0] >= 0.01)) {
                            rate = abs(this->socketRecv->Joint_Velocity_set[0]) >= 1 ? 1 : this->socketRecv->Joint_Velocity_set[0];
                            this->setSyncrpm(rate * DEFAULT_SYNC_RPM);
                            this->isSingleAxisMotion = true;
                        }
                        driver_errcode = this->Write('3',
                                                     singleJointNeedMoving[0], singleJointNeedMoving[1],
                                                     singleJointNeedMoving[2], singleJointNeedMoving[3],
                                                     singleJointNeedMoving[4], singleJointNeedMoving[5],
                                                     singleJointNeedMoving[6], singleJointNeedMoving[7],
                                                     singleJointNeedMoving[8]);
                    }
                    // PP set disceret position motion
                    else {
                        auto rate = abs(this->socketRecv->Joint_Velocity_set[0]) >= 0.01 ? this->socketRecv->Joint_Velocity_set[0] : 1.0f;
                        this->setSyncrpm(rate * DEFAULT_SYNC_RPM);
                        driver_errcode = this->Write('1',
                                                     this->socketRecv->Joint_Position_set[0], this->socketRecv->Joint_Position_set[1],
                                                     this->socketRecv->Joint_Position_set[2], this->socketRecv->Joint_Position_set[3],
                                                     this->socketRecv->Joint_Position_set[4], this->socketRecv->Joint_Position_set[5],
                                                     this->socketRecv->Joint_Position_set[6], this->socketRecv->Joint_Position_set[7],
                                                     this->socketRecv->Joint_Position_set[8]);
                        this->isSingleAxisMotion = false;
                    }
#ifndef SOCKET_TEST
                    if (driver_errcode < 0) {
                        cout << "Command error in PP! check: " << driver_errcode << '\n';
                    }
#endif
                } else {
                    ppFlag = 0;
                    this->isSingleAxisMotion = false;
                    // TODO: 检查是否基类Driver中的控制模式标志变化
                    //                    this->servoOperationModeSet(ppFlag, cspFlag, 0);
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
                    if (cspFirststep) {
                        cspFirststep = false;
                        auto jointDataNow = vec(MDT::getAngles(*this, this->MotGetData).data(), 6);
                        auto jointDataTarget = vec(vector<double>{angles.begin(), angles.begin() + 6});
                        if (norm(jointDataNow - jointDataTarget) > 10) {
                            cout << "Error! CSP start point is too far! Vector Norm is " << norm(jointDataNow - jointDataTarget) << endl;
                            cspFirststep = true;
                            continue;
                        }
                    }
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
                    // TODO: 检查是否基类Driver中的控制模式标志变化
                    //                    this->servoOperationModeSet(ppFlag, cspFlag, 0);
                }
                //CSP--offline
                {
                    //data buffer
                    if ((this->socketRecv->Command & 0b10000) != 0) {
                        //此处更新csp的位置数据
                        if (offline_pathPoints.size() == this->socketRecv->Tail_check - 1) {
                            cout << "Tail Check: " << this->socketRecv->Tail_check << endl;
                            vector<double> angles{
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
                            //                            for (const auto &data: offline_pathPoints) {
                            //                                for (const auto &data_second: data) {
                            //                                    cout << data_second << ",";
                            //                                }
                            //                                cout << endl;
                            //                            }
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
                        vector<vector<double>>().swap(offline_traj_data);
                        //: TODO check the new interpolation method
                        offline_traj_data = _interpolation::_1_Order_Linear_no_time_adjustment(offline_pathPoints, COST_TIME);
//                        offline_traj_data = _interpolation::_3_Order_CubicSpline_is_time_adjustment(offline_pathPoints, COST_TIME);
                        //                        //跑到初始位置
                        auto jointDataNow = vec(MDT::getAngles(*this, this->MotGetData).data(), 6);
                        auto jointDataFirstTarget = vec(vector<double>{offline_pathPoints[0].begin(), offline_pathPoints[0].begin() + 6});
                        if (norm(jointDataNow - jointDataFirstTarget) > 10) {
                            cout << "running to the first path point!" << endl;
                            //pp 1 csp 0 cst 0
                            //                            this->servoOperationModeSet(true, false, false);
                            this->ppFlag = true;
                            this->cspFlag = false;
                            this->cst_Flag = false;
                            this->setSyncrpm(50);
                            driver_errcode = this->Write('x', offline_pathPoints[0][0],
                                                         offline_pathPoints[0][1],
                                                         offline_pathPoints[0][2],
                                                         offline_pathPoints[0][3],
                                                         offline_pathPoints[0][4],
                                                         offline_pathPoints[0][5],
                                                         offline_pathPoints[0][6]);
                            Sleep(100);
                            this->Disable();
                            Sleep(100);
                            if (!cspFlag) {
                                cout << "Command: CSP Enable!" << '\n';
                                //refresh sendData
                                for (auto &s: sendData) {
                                    s.Control_Word = 15;
                                }
                            }
                            this->Enable();
                            this->ppFlag = false;
                            this->cspFlag = false;
                            this->cst_Flag = false;
                            cout << "Change mode to CSP!" << endl;
                            // 通过上下使能，切换操作模式
                            ////                            this->servoOperationModeSet(false, true, false);
                        }
                        //清空获取的路径点数据
                        vector<vector<double>>().swap(offline_pathPoints);
                        MDT::fromAnglesToPulses(*this, offline_traj_data[0], sendData);
                        //demo版本，本地执行固定轨迹
                        //open csp motion
                        //send cyclic data to the servo automatically background
                        //the data is upgrade by local thread :offline_csp
                        driver_errcode = this->servoCSP(sendData, this->MotGetData);
#ifndef SOCKET_TEST
                        if (driver_errcode < 0) {
                            cout << "offline csp error!" << endl;
                            break;
                        }
#endif
                        // lambda function:
                        // only update the data, not excuate the driver
                        auto offline_csp = [&]() {
                            offlinecspFlag = true;
                            for (const auto &data: offline_traj_data) {
                                MDT::fromAnglesToPulses(*this, data, sendData);
                                this_thread::sleep_for(chrono::milliseconds(10));
                            }
                            cout << "offline csp finish!" << endl;
                            servoFinishCS();
                            vector<vector<double>>().swap(offline_traj_data);
                        };
                        if (!offlinecspFlag) {
                            thread t_offline_csp(offline_csp);
                            t_offline_csp.detach();
                            //TODO: 检查是否基类Driver中的控制模式标志变化
                            //  this->servoOperationModeSet(ppFlag, cspFlag, 0);
                        }

                    } else {
                        offlinecspFlag = false;
                        //TODO: 检查是否基类Driver中的控制模式标志变化
                        //  this->servoOperationModeSet(ppFlag, cspFlag, 0);
                    }
                }

            }
            // 如果没有上使能指令，则下使能
            else {
                driver_errcode = this->Disable();
                enableFlag = 0;
                ppFlag = 0;
                //TODO: 检查是否基类Driver中的控制模式标志变化
                //  this->servoOperationModeSet(ppFlag, cspFlag, 0);
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

    this->socketSend->Status &= ~((!enableFlag << 1));
    this->socketSend->Status |= enableFlag << 1;

    this->socketSend->Status &= ~((!ppFlag << 2));
    this->socketSend->Status |= (ppFlag << 2);

    this->socketSend->Status &= ~((!ppFlag << 2));
    this->socketSend->Status |= (ppFlag << 2);

    if ((this->socketRecv->Command & 0b100000) != 0) {
        this->socketSend->Status &= ~((!cspFlag << 4));
        this->socketSend->Status |= (cspFlag << 4);
    } else {
        this->socketSend->Status &= ~((!cspFlag << 3));
        this->socketSend->Status |= (cspFlag << 3);
    }

    vector<double> j_angles = MDT::getAngles(d, data);
    j_angles[6] = j_angles[6];
    this->socketSend->Joint_Position = vector<float>(j_angles.begin(), j_angles.end());
    vector<double> j_vec = MDT::getVecs(d, data);
    this->socketSend->Joint_Velocity = vector<float>(j_vec.begin(), j_vec.end());
    vector<double> c_Position = fkine(MDT::getAngles(d, data));
    this->socketSend->Cartesian_Position = vector<float>(c_Position.begin(), c_Position.end());
}
