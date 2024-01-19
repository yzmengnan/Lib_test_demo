#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-non-const-global-variables"
#pragma ide diagnostic ignored "readability-qualified-auto"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "modernize-use-trailing-return-type"
#pragma ide diagnostic ignored "readability-braces-around-statements"
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
/*
 * @Author: YangQ
 * @Date: 2023-02-27 15:57:49
 * @LastEditors: YangQ
 * @LastEditTime: 2023-03-08 11:10:28
 * @FilePath: \Demo0\SRC\Servo_Driver.cpp
 * @Description:
 *
 * Copyright (c) 2023 by YangQ, All Rights Reserved.
 */
#include "Driver.h"
#include "Transform.hpp"
#include "motionDataTransform.hpp"
#include <algorithm>
#include <memory>
#include <thread>
mutex adsLock;
Driver::Driver(Tc_Ads &adsHandle) : p_ads(&adsHandle) {
}
Driver::~Driver() {
    vector<DTS> sendData(servoNums);
    servoDisable(sendData);
    cout << "Driver controller disable!" << '\n';
}
auto Driver::servoEnable(std::vector<DTS> &SendData, std::vector<DFS> &GetData) -> int {
    for (int try_count = 0; try_count < 3; try_count++) {
        uint8_t state = 0;
        error_code = p_ads->get(GetData);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Get Data Error:" << error_code << '\n';
        }
        Sleep(60);
        //first check ,if servo is enabled, quit!
        for (auto child: GetData) {
            state += static_cast<int>((child.Status_Word &= 0x37) == 0x37);
        }
        if (state == servoNums) {
            cout << "All servo has been enabled!" << '\n';
            //需要更新当前控制字的引用
            for (auto &child: SendData) {
                child.Control_Word |= 0x000f;
            }
            state = 0;
            enableFlag = true;
            return 0;
        }
        for (DFS child_servo: GetData) {
            state += static_cast<int>((child_servo.Status_Word &= 0x40) == 0x40);
        }
        if (state == servoNums) {
            std::cout << "All Servos Ready!" << '\n';
            state = 0;
        } else {
            std::cout << "Servo Enable trying, time_counts:" << try_count + 1 << '\n';
            continue;
        }
        for (DTS &child_servo: SendData) {
            child_servo.Control_Word = 0x0006;
        }
        error_code = p_ads->set(SendData);
        Sleep(120);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Set Data Error:" << error_code << '\n';
        }
        error_code = p_ads->get(GetData);
        Sleep(60);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Get Data Error:" << error_code << '\n';
        }
        for (DFS child_servo: GetData) {
            state += static_cast<int>((child_servo.Status_Word &= 0x21) == 0x21);
        }
        if (state == servoNums) {
            std::cout << "All Servos Ready to switch on!" << '\n';
            state = 0;
        } else {
            std::cout << "Servo Enable trying, time_counts:" << try_count + 1 << '\n';
            continue;
        }
        for (DTS &child_servo: SendData) {
            child_servo.Control_Word = 0x0007;
        }
        p_ads->set(SendData);
        Sleep(100);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Set Data Error:" << error_code << '\n';
        }
        error_code = p_ads->get(GetData);
        Sleep(60);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Get Data Error:" << error_code << '\n';
        }
        for (DFS child_servo: GetData) {
            state += static_cast<int>((child_servo.Status_Word &= 0x23) == 0x23);
        }
        if (state == servoNums) {
            std::cout << "All Servos Switched on!" << '\n';
            state = 0;
        } else {
            std::cout << "Servo Enable trying, time_counts:" << try_count + 1 << '\n';
            continue;
        }
        for (DTS &child_servo: SendData) {
            child_servo.Control_Word = 0x000F;
        }
        error_code = p_ads->set(SendData);
        Sleep(100);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Set Data Error:" << error_code << '\n';
        }
        error_code = p_ads->get(GetData);
        if (error_code < 0) {
            cout << "SERVO ENABLE: Get Data Error: " << error_code << '\n';
        }
        for (DFS child_servo: GetData) {
            state += static_cast<int>((child_servo.Status_Word &= 0x37) == 0x37);
        }
        if (state == servoNums) {
            std::cout << "All Servos Operation enabled!" << '\n';
            servoBreak(true);
            this_thread::sleep_for(chrono::milliseconds(120));
            state = 0;
            enableFlag = true;
            return 0;
        }
        std::cout << "Servo Enable trying, time_counts:" << try_count + 1 << '\n';
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    servoBreak(false);
    std::cout << "Servo Enable Failure......" << '\n';
    error_code = -1001;
    return error_code;
}

auto Driver::servoDisable(std::vector<DTS> &SendData) -> int {
    if (enableFlag) {
        if (*cyclicFlag) {
            this->servoFinishCS();
        }
        servoBreak(false);
        pp_Flag = false;
        cst_Flag = false;
        csp_Flag = false;
        for (auto &child_servo: SendData) {
            child_servo.Mode_of_Operation = 1;
            child_servo.Control_Word = 0;
        }
        error_code = p_ads->set(SendData);
        if (error_code < 0) {
            cout << "Servo Operation disabled failure !!" << '\n';
            cout << "Extreme Warning! Please Shut Down the Power Immediately!!!" << '\n';
            return error_code;
        }
        std::cout << "All Servos Operation disabled!" << '\n';
        enableFlag = false;
    } else {
        servoBreak(false);
        for (auto &child_servo: SendData) {
            child_servo.Mode_of_Operation = 1;
            child_servo.Control_Word = 0;
        }
        error_code = p_ads->set(SendData);
    }
    return 0;
}
/*
 * servoPP0: point to point move!
 */
auto Driver::servoPP0(std::vector<DTS> &SendData, std::vector<DFS> &GetData) -> int {
    //若伺服未使能
    if (!enableFlag) {
        cout << "禁止！请上使能！" << '\n';
        return -2999;
    }
    // 若此时伺服未设置PP模式
    if (static_cast<int>(pp_Flag) == 0) {
        //设置PP工作模式
        for (auto &child_servo: SendData) {
            child_servo.Mode_of_Operation = 1;
            child_servo.Max_Velocity = 3000;
        }
        error_code = p_ads->set(SendData);
        if (error_code < 0) {
            return error_code;
        }
        this_thread::sleep_for(chrono::milliseconds(120));
        error_code = p_ads->get(GetData);
        if (error_code < 0) {
            return error_code;
        }
        //检查伺服是否为PP工作模式
        for (auto child_servo: GetData) {
            if (child_servo.Mode_of_Operation_disp != 1) {
                std::cout << "Servo Operation Mode Change Failure!" << '\n';
                error_code = -3000;
                return error_code;
            }
        }
        //若未return，则设置伺服模式标志位
        pp_Flag = true;
        cst_Flag = false;
        csp_Flag = false;
    }

    //伺服已设置为PP模式
    adsLock.lock();
    // 更新607Ah（Target Position）的值
    error_code = p_ads->set(SendData);
    adsLock.unlock();
    if (error_code < 0) {
        return error_code;
    }
    //控制字BIT4为1，通知伺服器目标位置开始有效
    for (auto &child_servo: SendData)
        child_servo.Control_Word |= 0x10;
    adsLock.lock();
    error_code = p_ads->set(SendData);
    adsLock.unlock();
    // 检查伺服是否收到目标点，否则，循环发送控制字的bit4为1；
    if (error_code < 0) {
        return error_code;
    }
    // 开启线程th1，设置延迟最大20ms即退出
    //标志位指针
    shared_ptr<bool> servoLag_flag = make_shared<bool>(true);
    auto th = [servoLag_flag] {
        this_thread::sleep_for(chrono::milliseconds(120));
        *servoLag_flag = false;
    };

    thread th1(th);
    th1.detach();
    bool target_ack = true;
    while (target_ack && *servoLag_flag) {
        int statusReadyCount = 0;
        // 获取伺服状态字
        adsLock.lock();
        error_code = p_ads->get(GetData);
        adsLock.unlock();
        if (error_code != 0) {
            return error_code;
        }
        for (auto child_servo: GetData) {
            if ((child_servo.Status_Word & 0x1000) != 0)
                statusReadyCount++;
        }
        if (statusReadyCount == servoNums) {
            target_ack = false;
        }
    }
    // 如果是伺服均收到新的坐标位置，更新控制字，准备下一次位置更新
    if (!target_ack) {
        for (auto &child_servo: SendData) {
            child_servo.Control_Word &= 0xffef;//控制字BIT4置0
        }
        adsLock.lock();
        error_code = p_ads->set(SendData);
        adsLock.unlock();
        return error_code;
    }
    // 否则，则是*servoLag_flag=0退出 由th1最大延时后，伺服依旧没有响应
    std::cout << "Servo lag!" << '\n';
    error_code = -3001;
    return error_code;
}

/*!
 * @disription false meaning lock drivers
 * @param state
 * @return 0 if set success
 */
auto Driver::servoBreak(const bool &state) -> int {
    //    int break_state[servoNums] = {};
    //    for (auto &b: break_state) {
    //        b = state;
    //    }
    //
    int break_state{};
#ifdef DISTRIBUTE_BREAK
    if (state) {
        for (int i = 0; i < servoNUMs; i++) {
            break_state |= 0b1 << i;
        }
    }
#else
    if (state)
        break_state = 1;
#endif
    auto nErr = AdsSyncWriteReq(p_ads->pAddr, OUTPUT_BASE, DIGITAL_IO_OFFSET, 4, &break_state);
    if (nErr != 0) {
        std::cout << "Ads set error: " << nErr << '\n';
    }
    return 0;
}
/*!
 * @details 设置伺服器为CST模式，此函数执行后，通过参数SendData的引用实时控制力矩
 * @param SendData
 * @param GetData
 * @return
 */
auto Driver::servoCST(vector<DTS> &SendData, vector<DFS> &GetData) -> int {
    //若伺服未使能
    if (!enableFlag) {
        cout << "禁止！请先上使能！" << '\n';
    }
    //如果CST模式已经进入，则直接退出
    if (cst_Flag) {
        cout << "CST MODE has been running!" << '\n';
        return 0;
    }
    if (csp_Flag || pp_Flag || enableFlag) {
        cout << "禁止！ 请下使能后再切换模式！" << '\n';
        vector<DTS> temp(servoNums);
        this->servoDisable(temp);
    } else {
        for (auto &child: SendData) {
            child.Mode_of_Operation = 10;
            child.Max_Torque = 1500;
            child.Max_Velocity = 3000;
        }
        error_code = p_ads->set(SendData);
        if (error_code < 0) {
            cout << "Error: set CST MODE! " << error_code << '\n';
        }
        this_thread::sleep_for(chrono::milliseconds(100));
        error_code = p_ads->get(GetData);
        if (error_code < 0) {
            cout << "Error: get CST MODE! " << error_code << '\n';
        }
        for (auto child: GetData) {
            if (child.Mode_of_Operation_disp != 10) {
                cout << "Error! CST MODE failed!" << '\n';
                this->servoDisable(SendData);
                return -4000;
            }
        }
        //CST模式设置成功
        pp_Flag = false;
        cst_Flag = true;
        csp_Flag = false;
    }
    //设置cyclicFlag 为真，表示Driver开启了循环同步子线程
    *cyclicFlag = true;
    auto cst_func = [&]() {
        this->f_Cyclic(SendData, GetData);
    };
    thread t(cst_func);
    t.detach();
    return 0;
}
/*!
 * @details 设置伺服器为CSP模式，此函数执行后，通过参数SendData的引用实时控制力矩
 * @param SendData
 * @param GetData
 * @return
 */
auto Driver::servoCSP(vector<DTS> &SendData, vector<DFS> &GetData) -> int {
    //如何未使能，则直接退出
    if(!enableFlag){
        cout<<"禁止！请先上使能！"<<endl;
        return -2998;
    }
    //如果CSP模式已经进入，则直接退出
    if (csp_Flag) {
        //        cout << "CSP MODE is running:"<<endl;
        return 0;
    }
    //任意模式设置，禁止模式切换
    if (cst_Flag || pp_Flag) {
        cout << "禁止！请下使能后再切换模式" << '\n';
        vector<DTS> temp(servoNums);
        this->servoDisable(temp);
    } else {
        for (auto &child: SendData) {
            child.Mode_of_Operation = 8;
            child.Max_Velocity = 3000;
            child.Max_Torque = 1500;
        }
        error_code = p_ads->set(SendData);
        if (error_code < 0) {
            cout << "Error: set CSP MODE! " << error_code << '\n';
        }
        this_thread::sleep_for(chrono::milliseconds(30));
        error_code = p_ads->get(GetData);
        if (error_code != 0) {
            cout << "Error: get CSP MODE! " << error_code << '\n';
        }
        for (auto child: GetData) {
            if (child.Mode_of_Operation_disp != 8) {
                cout << "Error! CSP MODE failed!" << '\n';
                this->servoDisable(SendData);
                return -4001;
            }
        }
        pp_Flag = false;
        csp_Flag = true;
        cst_Flag = false;
    }
    //设置cyclicFlag为真，表示Driver开启了循环同步子线程
    *cyclicFlag = true;
    auto pSendData = make_shared<vector<DTS>>(SendData);
    auto csp_func = [&]() {
        this->f_Cyclic(SendData, GetData);
    };
    thread t(csp_func);
    t.detach();
    return error_code;
}
auto Driver::cutToolOperation(const int8_t &flag) -> int {
    int ioState = (0 | static_cast<int>(this->enableFlag));
#ifndef DISTRIBUTE_BREAK
    if (flag == 0) {
        ioState &= 0b001;
    } else if (flag == 1) {
        ioState &= 0b001;
        ioState |= 0b100;
    } else if (flag == 2) {
        ioState &= 0b001;
        ioState |= 0b010;
    } else
        return 0;
    error_code = AdsSyncWriteReq(p_ads->pAddr, OUTPUT_BASE, DIGITAL_IO_OFFSET, 4, &ioState);
    if (error_code != 0) {
        cout << "Error! cutToolOperation Error: " << error_code << '\n';
        return -1;
    }
    return 0;
#else
    cout << "Warning! Work in distribute break connections!" << endl;
#endif
}
/*
 * servoPP0: point to point move continusly!
 */
auto Driver::servoPP1(std::vector<DTS> &SendData, std::vector<DFS> &GetData) -> int {
    //若伺服未使能
    if (!enableFlag) {
        cout << "禁止！请上使能！" << '\n';
        return -2999;
    }
    // 若此时伺服未设置PP模式
    if (static_cast<int>(pp_Flag) == 0) {
        //设置PP工作模式
        for (auto &child_servo: SendData) {
            child_servo.Mode_of_Operation = 1;
            child_servo.Max_Velocity = 3000;
        }
        error_code = p_ads->set(SendData);
        if (error_code < 0) {
            return error_code;
        }
        this_thread::sleep_for(chrono::milliseconds(120));
        error_code = p_ads->get(GetData);
        if (error_code < 0) {
            return error_code;
        }
        //检查伺服是否为PP工作模式
        for (auto child_servo: GetData) {
            if (child_servo.Mode_of_Operation_disp != 1) {
                std::cout << "Servo Operation Mode Change Failure!" << '\n';
                error_code = -3000;
                return error_code;
            }
        }
        //若未return，则设置伺服模式标志位
        pp_Flag = true;
        cst_Flag = false;
        csp_Flag = false;
    }

    //伺服已设置为PP模式
    adsLock.lock();
    // 更新607Ah（Target Position）的值
    error_code = p_ads->set(SendData);
    adsLock.unlock();
    if (error_code < 0) {
        return error_code;
    }
    //控制字BIT4为1，通知伺服器目标位置开始有效
    //BIT5=1 move cotinuesly
    for (auto &child_servo: SendData)
        child_servo.Control_Word |= (0x10 | 0b100000);
    adsLock.lock();
    error_code = p_ads->set(SendData);
    adsLock.unlock();
    // 检查伺服是否收到目标点，否则，循环发送控制字的bit4为1；
    if (error_code < 0) {
        return error_code;
    }
    // 开启线程th1，设置延迟最大20ms即退出
    //标志位指针
    shared_ptr<bool> servoLag_flag = make_shared<bool>(true);
    auto th = [servoLag_flag] {
        this_thread::sleep_for(chrono::milliseconds(80));
        *servoLag_flag = false;
    };

    thread th1(th);
    th1.detach();
    bool target_ack = true;
    while (target_ack && *servoLag_flag) {
        int statusReadyCount = 0;
        // 获取伺服状态字
        adsLock.lock();
        error_code = p_ads->get(GetData);
        adsLock.unlock();
        if (error_code != 0) {
            return error_code;
        }
        for (auto child_servo: GetData) {
            if ((child_servo.Status_Word & 0x1000) != 0)
                statusReadyCount++;
        }
        if (statusReadyCount == servoNums) {
            target_ack = false;
        }
    }
    // 如果是伺服均收到新的坐标位置，更新控制字，准备下一次位置更新
    if (!target_ack) {
        for (auto &child_servo: SendData) {
            child_servo.Control_Word &= 0xffef;//控制字BIT4置0
        }
        adsLock.lock();
        error_code = p_ads->set(SendData);
        adsLock.unlock();
        return error_code;
    }
    // 否则，则是*servoLag_flag=0退出 由th1最大延时后，伺服依旧没有响应
    std::cout << "Servo lag!" << '\n';
    error_code = -3001;
    return error_code;
}

MotionV1::MotionV1(Tc_Ads &ads_handle) : Driver{ads_handle} {
    cout << "MotionV1 control module built!" << '\n';
    auto dataUpdating_MOTIONV1 = [&]() {
        while (true) {
            driver_errcode = this->GetDataUpdate(MotGetData);
            if (driver_errcode < 0) {
                cout << "Error updating servo data error in MotionV1 err:  " << driver_errcode << '\n';
                break;
            }
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    };
    thread t_Motion_V1(dataUpdating_MOTIONV1);
    t_Motion_V1.detach();
    cout << "MotionV1 is updating the servo data background!" << '\n';
}
int MotionV1::Enable() {
    auto err = servoEnable(MotSendData, MotGetData);
    if (err < 0) {
        cout << "Error: Enable the Drive : " << err << '\n';
    }
    return err;
}
int MotionV1::Disable() {
    auto err = servoDisable(MotSendData);
    if (err < 0) {
        cout << "Error: Enable the Drive : " << err << '\n';
    }
    return err;
}
vector<DTS> &MotionV1::gearRatioScalar(initializer_list<double> args) {
    char i{};
    vector<float> angles;
    for (auto index = args.begin(); index != args.end(); index++, i++) {
        if (i >= servoNums) {
            // modify servo send data
            break;
        }
        angles.push_back(*index);
    }
    MDT::fromAnglesToPulses(*this, angles, this->MotSendData);
    return MotSendData;
}
MotionV1::~MotionV1() {
    this->Disable();
    cout << "Motion V1 controller disable!" << '\n';
}
auto MotionV1::opSpaceMotion(const vector<double> &target_c_position) -> int {
    auto qd = ikine(target_c_position, MDT::getAngles(*this, this->MotGetData));
    vec qd_vec = qd;
    qd_vec.print("qdesired");
    return this->Write('x', qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
}
/* Canceld
 *
 *
 *
auto MotionV1::opSpaceMotion(const vector<double> &target_c_position, int rate) -> int {
    //first, get cartisen vecs
    auto qNow = MDT::getAngles(*this, this->MotGetData);
    vec c_Now = fkine(vector<double>(qNow.begin(), qNow.begin() + 6));
    c_Now.print("c_Now");
    vec c_Target = target_c_position;
    const vec c_vecs = (c_Target - c_Now) / (double) rate;
    c_vecs.print("c_vecs");
    // run c_vecs each time
    while (true) {
        qNow = MDT::getAngles(*this, this->MotGetData);
        c_Now = fkine(vector<double>(qNow.begin(), qNow.begin() + 6));
        //for some reason ,some element of c_Now will be changed
        //one method here is to compensate the deviation
        vec delta = c_Now - c_Target;
        for (int i{}; i < delta.n_rows; i++) {
            delta(i) = c_vecs(i) == 0 ? delta(i) / 10 : 0;
        }
        vec c_vecs_modified = c_vecs - delta;
        vec cdesired = c_vecs_modified + c_Now;
        auto qd = ikine(vector<double>(cdesired.begin(), cdesired.end()), MDT::getAngles(*this, this->MotGetData));
        this->Write('2', qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
        cout << "sum: " << sum(abs(delta)) << endl;
        if (sum(abs(delta)) < 0.03) {
            break;
        }
    }
    return 0;
}
 */
void MotionV1::showOperationalSpaceData() {
    auto data = fkine(MDT::getAngles(*this, this->MotGetData));
    for (const auto &d: data) {
        cout << d << ",";
    }
    cout << '\n';
}
int MotionV1::opSpaceMotionByJacobe(const vector<float> &c_vecs) {
    //modify operational velocities
    vec c_temp = mat(vector<double>{c_vecs.begin(), c_vecs.end()});
    auto c_max = abs(c_temp).max();
    vector<double> c_vecs_Modified{};
    if (c_max == 0) {
        cout << "no capable velocity!" << endl;
        return 0;
    }
    for (const auto &c: c_vecs) {
        c_vecs_Modified.push_back(c / c_max);
    }
    rowvec c{c_vecs_Modified};
//    c.print("vector");
    //get current q position
    auto q = MDT::getAngles(*this, this->MotGetData);
    //modify q position, only need the formar 6 axis
    vector<double> q_6Axis{q.begin(), q.begin() + 6};
    //get jacob matrix of the end effector
    mat J = jacobe(q_6Axis);
//    J.print("J");
    //bad condition
//    if (cond(J) > 300) {
//        cout << "Bad posture!" << endl;
//    }
    mat Jinv{};
    try {
        Jinv = inv(J);
    } catch (const std::runtime_error &e) {
        Jinv = pinv(J);
    }
    vec q_dot = Jinv * vec(c_vecs_Modified);
    vec qd = vec(q_6Axis) + q_dot * 0.1;
//    rowvec q_dotShow  = q_dot.t();
//    q_dotShow.print("q_dot: ");
    qd.t().print("qd:");
    //    cout<<"manipulability: "<<getManipulability(J)<<'\n';
    return this->Write('3', qd(0), qd(1), qd(2), qd(3), qd(4), qd(5));
}
int MotionV1::opSpaceMotionByJacob0(const vector<float> &c_vecs) {
    //modify operational velocities
    vec c_temp = mat(vector<double>{c_vecs.begin(), c_vecs.end()});
    auto c_max = abs(c_temp).max();
    vector<double> c_vecs_Modified{};
    if (c_max == 0) {
        cout << "no capable velocity!" << endl;
        return 0;
    }
    for (const auto &c: c_vecs) {
        c_vecs_Modified.push_back(c / c_max);
    }
    //get current q position
    auto q = MDT::getAngles(*this, this->MotGetData);
    //modify q position, only need the formar 6 axis
    vector<double> q_6Axis{q.begin(), q.begin() + 6};
    //get jacob matrix of the end effector
    mat J = jacob0(q_6Axis);
    //bad condition
    if (cond(J) > 300) {
        cout << "Bad posture!" << endl;
    }
    mat Jinv{};
    try {
        Jinv = inv(J);
    } catch (const std::runtime_error &e) {
        Jinv = pinv(J);
    }
    vec q_dot = Jinv * vec(c_vecs_Modified);
//    q_dot.print("q_dot: ");
    vec qd = vec(q_6Axis) + q_dot * 0.01;
    //    cout<<"manipulability: "<<getManipulability(J)<<'\n';
    return this->Write('3', qd(0), qd(1), qd(2), qd(3), qd(4), qd(5));
}
int MotionV1::opSpaceMotionByJacobe_RL(const vector<float> &c_vecs,Robot&robot) {
    //modify operational velocities
    Eigen::VectorXd c_temp(6);
    c_temp<<c_vecs[0],c_vecs[1],c_vecs[2],c_vecs[3],c_vecs[4],c_vecs[5];
    Eigen::VectorXd c_temp_abs = c_temp.array().abs();
    double bigone = c_temp_abs.maxCoeff();
    if(bigone<=0.001)
    {
//        cout<<"no capable order velocity!"<<endl;
        return 0;
    }
    else{
        cout<<endl<<c_temp<<endl;
    }
    c_temp.normalize();
    cout<<"order operational vecs:"<<c_temp<<endl;
    //get current q position
    auto q = MDT::getAngles(*this, this->MotGetData);
    //modify q position, only need the formar 6 axis
    vector<double> q_6Axis{q.begin(), q.begin() + 6};
    //get jacob matrix of the end effector
    auto Jinv = robot.getInverseJacobe(q_6Axis);
    rl::math::Vector6 q_dot = Jinv*c_temp;
    rl::math::Vector6 q_now;
    q_now<<q_6Axis[0],q_6Axis[1],q_6Axis[2],q_6Axis[3],q_6Axis[4],q_6Axis[5];
    auto qd = q_now+q_dot*0.01;
    return this->Write('3', qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]);
}

#pragma clang diagnostic pop