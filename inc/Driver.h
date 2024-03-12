/*
 *
 */
#pragma once

#include "winsock2.h"
//
#include "DATA_STRUCT.h"
#include "Tc_Ads.h"
#include "TimerCounter.h"
#include "robot.h"
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>
using namespace std;
extern mutex adsLock;
#define ANGLES_2_PULSES (8388608 / 360)
#define PULSETO_MOTOR_SPEED_RATE 0.000715
//#define DEFAULT_SYNC_RPM 2000
#define DEFAULT_SYNC_RPM_C 10
using sd = class Driver {
public:
    vector<double> _driver_gearRatioScalar{-284.9231 * ANGLES_2_PULSES,
                                           -213.7 * ANGLES_2_PULSES,
                                           -171.0 * 5 / 3 / 90 * 82 * ANGLES_2_PULSES,
                                           -181.22 * ANGLES_2_PULSES,
                                           -144.9 * 5 / 3 * ANGLES_2_PULSES,
                                           -33.0 * 5 / 3 * ANGLES_2_PULSES,
                                           -25 * 1000 * 1.5 * ANGLES_2_PULSES};
    //    vector<double> _driver_gearRatioScalar{
    //            -284.9231 * ANGLES_2_PULSES,
    //            -213.7 * ANGLES_2_PULSES,
    //            -153.9 * ANGLES_2_PULSES,
    //            -146 * ANGLES_2_PULSES,
    //            -144.9 * ANGLES_2_PULSES,
    //            -33.0 * ANGLES_2_PULSES,
    //            -25 * 1.5 * ANGLES_2_PULSES};


public:
    Driver(Tc_Ads &adsHandle);
    auto servoEnable(std::vector<DTS> &SendData, std::vector<DFS> &GetData) -> int;
    auto servoDisable(std::vector<DTS> &SendData) -> int;
    void setProfileVelocity(vector<float> degreesPerSeconds, std::vector<DTS> &SendData) {
        int i{};
        for (auto &s: SendData) {
            if (i >= degreesPerSeconds.size()) {
                break;
            }
            s.Profile_Velocity = degreesPerSeconds[i] * this->_driver_gearRatioScalar[i];
            i++;
        }
        int err = p_ads->set(SendData);
        if (err < 0) {
            cout << "set profile velocity error!" << err << endl;
        }
    }
    void setGearRatioScalar(initializer_list<float> r) {
        int i{};
        for (auto scalar: r) {
            i++;
            if (i > servoNums) {
                break;
            }
            _driver_gearRatioScalar[i - 1] = scalar;
        }
    }
    int GetDataUpdate(vector<DFS> &GetData) {
        adsLock.lock();
        auto err = p_ads->get(GetData);
        adsLock.unlock();
        if (err < 0) {
            cout << "Get Data Update error :" << err << endl;
            return err;
        }
        return 0;
    }
    /**
     * @description: PP运动驱动程序,动作例1,执行点到点单独运动
     */
    auto servoPP0(std::vector<DTS> &SendData, std::vector<DFS> &GetData) -> int;
    auto servoPP1(std::vector<DTS> &SendData, std::vector<DFS> &GetData) -> int;
    auto servoCST(vector<DTS> &SendData, vector<DFS> &GetData) -> int;
    auto servoCSP(vector<DTS> &SendData, vector<DFS> &GetData) -> int;
    /*!
     * @details 修改伺服器工作模式，根据手册，伺服器模式的更换需要下使能
     * @param pp
     * @param csp
     * @param cst
     */
    void servoOperationModeSet(int pp, int csp, int cst) {
        // 修改逻辑
        //        pp_Flag = pp;
        //        csp_Flag = csp;
        //        cst_Flag = cst;
        if (pp_Flag && csp) {
            pp_Flag = false;
            vector<DTS> a(servoNums);
            vector<DFS> b(servoNums);
            this->servoDisable(a);
            for (auto &i: a) {
                i.Mode_of_Operation = 8;
            }
            p_ads->set(a);
            this->servoEnable(a, b);
            //  csp_Flag 不能通过其他方式改变
            //            csp_Flag = true;
        } else if (pp_Flag && cst) {
            pp_Flag = false;
            vector<DTS> a(servoNums);
            vector<DFS> b(servoNums);
            this->servoDisable(a);
            for (auto &i: a) {
                i.Mode_of_Operation = 10;
            }
            p_ads->set(a);
            this->servoEnable(a, b);
            //            cst_Flag = true;
        } else if ((csp_Flag || cst_Flag) && pp) {
            csp_Flag = false;
            cst_Flag = false;
            vector<DTS> a(servoNums);
            vector<DFS> b(servoNums);
            this->servoDisable(a);
            for (auto &i: a) {
                i.Mode_of_Operation = 1;
            }
            p_ads->set(a);
            this->servoEnable(a, b);
            pp_Flag = true;
        }
    }

    virtual ~Driver();

    //Test part
    void TestBREAK(const bool &state) {
        servoBreak(state);
    }
    bool enableFlag = false;

private:
    auto servoBreak(const bool &state) -> int;

public:
    /*
     *
     * @param flag flag=0:hold
     *              flag = 1; catch
     *              flag =2 ; release
     * @return
     */
    auto cutToolOperation(const int8_t &flag) -> int;

protected:
    bool pp_Flag = false; //=1表示pp就位，=0表示未就位
    bool cst_Flag = false;// 1 ready, 0 not ready
    bool csp_Flag = false;// 1 ready, 0 not ready
private:
    shared_ptr<bool> cyclicFlag = make_shared<bool>(false);
    pTc_Ads p_ads = nullptr;
    int error_code = 0;
    void f_Cyclic(vector<DTS> &SendData, const vector<DFS> &GetData) {
        cout << "Cyclic START!" << endl;
        TimerCounter tc;
        tc.Start();
        vector<int32_t> pulseLast{};
        for (const auto &f: SendData) {
            pulseLast.push_back(f.Target_Pos);
        }
        auto motor_speed_adjust = [&]() {
            int i{};
            for (auto &data: SendData) {
#ifdef electronicGear
                data.Max_Velocity = abs(pulseLast[i++] - data.Target_Pos) * PULSETO_MOTOR_SPEED_RATE / motorLagRate;
#else
                data.Max_Velocity = abs(pulseLast[i++] - data.Target_Pos) * PULSETO_MOTOR_SPEED_RATE / motorLagRate;
#endif
            }
            i = 0;
            for (const auto &lastPulses: GetData) {
                pulseLast[i++] = lastPulses.Actual_Pos;
            }
        };
        while (*cyclicFlag) {
            if (tc.dbTime * 1000 >= 10) {
                if (csp_Flag)
                    motor_speed_adjust();
                p_ads->set(SendData);
                //                cout<<"Target_Pos: ";
                //                for (const auto &data: SendData) {
                //                    cout << data.Target_Pos << ',';
                //                }
                //                cout << endl;
                tc.Start();
            }
            tc.Stop();
        }
        cout << "Cyclic QUIT!" << endl;
    }

public:
    void servoFinishCS() {
        *cyclicFlag = false;
    }
};

class MotionV1 : public Driver {
public:
    MotionV1(Tc_Ads &ads_handle);
    int Enable();
    int Disable();
    template<typename T, typename... T2>
    /*!
     *
     * @tparam T
     * @tparam T2
     * @param operationMode  '0': 此状态下，执行有缓冲的Profile运动，当前位置执行中，新的位置发送时，
     *                          新的位置作为缓存进入伺服器的缓存器中，当前位置执行结束后，立即执行有效的缓存器内的动作
     *                       '1': 此状态下，执行各轴同步速度的'0'
     *                       '2': 此状态下，执行无缓冲的Profile运动，当前位置执行中，新的位置发送时，
     *                          立即运行到新的位置，设置同步速度为最高关节速度。
     *                       '3': 此状态下，执行无缓冲的Profile运动，当前位置执行中，新的位置发送时，
     *                          立即运行到新的位置,设置同步速度为最低关节速度
     *
     *                        请使用setSyncrpm函数调整同步速度大小
     *
     * @param args
     * @return
     */
    int Write(T operationMode = '0', T2... args) {
        //update the actual position to the command first
        //this part is to ensure the joint with no signals can do nothing
        for (int i{}; i < servoNums; i++) {
            MotSendData[i].Target_Pos = MotGetData[i].Actual_Pos;
        }
        //update target position with gearRatio_Scalar anyway!
        MotSendData = gearRatioScalar({args...});
        if (operationMode == '0') {
            // Normal motion with no sync-vec and no target change immediately
            int err = servoPP0(MotSendData, MotGetData);
            if (err < 0) {
                cout << "MotionV1 : pp0 error" << err << endl;
                return err;
            }
            return 0;
        } else if (operationMode == '1') {
            setSpeedOnHighestAxis();
            auto err = servoPP0(MotSendData, MotGetData);
            if (err < 0) {
                cout << "MotionV1 :pp1 error" << err << endl;
                return err;
            }
            return 0;

        } else if (operationMode == '2') {
            //motion with target changing immediately， sync speed is the highest speed
            setSpeedOnHighestAxis();
            auto err = servoPP1(MotSendData, MotGetData);
            if (err < 0) {
                cout << "MotionV1 :pp1 error" << err << endl;
                return err;
            }
            return 0;
        } else if (operationMode == '3') {
            //motion with target changing immediately, sync speed is the lowest speed
            //            if (isSingleAxisMotion)
            //                setNormalSpeed();
            //            else
            setSpeedOntheLowestAxis();
            auto err = servoPP1(MotSendData, MotGetData);
            if (err < 0) {
                cout << "MotionV1 :pp1 error" << err << endl;
                return err;
            }
            return 0;
        } else if (operationMode == 'x' || operationMode == 'X') {
            setSpeedOnHighestAxis();
            auto err = servoPP0(MotSendData, MotGetData);
            while (true) {
                int delta{};
                for (int i{}; i < servoNums; i++) {
                    delta += abs(MotSendData[i].Target_Pos - MotGetData[i].Actual_Pos);
                }
                if (delta < 20000) {
                    this_thread::sleep_for(chrono::milliseconds(200));
                    break;
                }
            }
        } else {
            cout << "wrong operation mode set!" << endl;
            return -2;
        }
        return 0;
    }
    template<typename... T>
    void setProfileVelocity(T... args) {
        vector<float> dps;
        for (auto dps_k: {args...}) {
            dps.push_back(dps_k);
        }
        this->Driver::setProfileVelocity(dps, this->MotSendData);
    }
    int driver_errcode{};
    ~MotionV1();
    /*!
     * @Description 注意，设置同步速度时，考虑不同轴减速比不同，无法定义同步转速，
     *              因此选取单位为rpm，表达最高轴的伺服轴输出转速
     * @param rpm
     */
    void setSyncrpm(double rpm) {
        this->sync_rpm = rpm;
    }
    void showSyncrpm() {
        cout << "this sync rpm is :" << this->sync_rpm;
    }
    /*!
     *
     * @param target_c_position
     * @param eor0  1:jacobe 0:jacob0
     * @return
     */
    int opSpaceMotion(const vector<double> &target_c_position);
    //    int opSpaceMotion(const vector<double> &target_c_position, int rate);
    int opSpaceMotionByJacobe(const vector<float> &c_vecs);
    int opSpaceMotionByJacobe_RL(const vector<float> &c_vecs, Robot &robot);
    int opSpaceMotionByJacob0(const vector<float> &c_vecs);
    int opSpaceMotionByJacob0_RL(const vector<float> &c_vecs);

    /*！
     *
     *
     */
    void showOperationalSpaceData();
    /*!
     * @details 返回MotionV1内部的发送数据
     * @return
     */
    vector<DTS> getSendData() {
        return this->MotSendData;
    }
    vector<DFS> MotGetData{vector<DFS>(servoNums)};
    void setSpeedOnHighestAxis() {
#ifdef USE_EIGEN
        Eigen::RowVectorXd Delta(7);
        Delta << abs(this->MotSendData[0].Target_Pos - this->MotGetData[0].Actual_Pos),
                abs(this->MotSendData[1].Target_Pos - this->MotGetData[1].Actual_Pos),
                abs(this->MotSendData[2].Target_Pos - this->MotGetData[2].Actual_Pos),
                abs(this->MotSendData[3].Target_Pos - this->MotGetData[3].Actual_Pos),
                abs(this->MotSendData[4].Target_Pos - this->MotGetData[4].Actual_Pos),
                abs(this->MotSendData[5].Target_Pos - this->MotGetData[5].Actual_Pos),
                abs(this->MotSendData[6].Target_Pos - this->MotGetData[6].Actual_Pos);
        Delta.normalize();
        double k = 1.0F / Delta.maxCoeff();
        for (int vec_index = 0; vec_index < servoNums; vec_index++) {
            MotSendData[vec_index].Profile_Velocity = sync_rpm * Delta[vec_index] * k * 8388608 / 60;
            MotSendData[vec_index].Max_Velocity = 3000;
        }
#else
        vector<uint32_t> Delta{};
        for (int i = 0; i < servoNums; i++) {
            Delta.push_back(abs(MotSendData[i].Target_Pos - MotGetData[i].Actual_Pos));
        }
        uint32_t maxDelta = *max_element(Delta.begin(), Delta.end());
        //calculate and update each joint`s velocity
        for (int vec_index = 0; vec_index < servoNums; vec_index++) {
            MotSendData[vec_index].Profile_Velocity = sync_rpm * (float) Delta[vec_index] / maxDelta * 8388608 / 60;
            MotSendData[vec_index].Max_Velocity = 3000;
        }
#endif
    }
    void setSpeedOntheLowestAxis() {
#ifdef USE_EIGEN
        Eigen::RowVectorXd Delta(7);
        Delta << abs(this->MotSendData[0].Target_Pos - this->MotGetData[0].Actual_Pos),
                abs(this->MotSendData[1].Target_Pos - this->MotGetData[1].Actual_Pos),
                abs(this->MotSendData[2].Target_Pos - this->MotGetData[2].Actual_Pos),
                abs(this->MotSendData[3].Target_Pos - this->MotGetData[3].Actual_Pos),
                abs(this->MotSendData[4].Target_Pos - this->MotGetData[4].Actual_Pos),
                abs(this->MotSendData[5].Target_Pos - this->MotGetData[5].Actual_Pos),
                abs(this->MotSendData[6].Target_Pos - this->MotGetData[6].Actual_Pos);
        Delta.normalize();
        //find the smallest element
        double smallone{1.0f};
        for (const auto &item: Delta) {
            if (abs(item) >= 0.001)
                smallone = item <= smallone ? item : smallone;
        }
        double k = 1.0F / smallone;
        for (int vec_index = 0; vec_index < servoNums; vec_index++) {
            MotSendData[vec_index].Profile_Velocity = sync_rpm * Delta[vec_index] * k * 8388608 / 60;
            MotSendData[vec_index].Max_Velocity = 3000;
        }
#else
        vector<uint32_t> Delta{};
        for (int i = 0; i < servoNums; i++) {
            Delta.push_back(abs(MotSendData[i].Target_Pos - MotGetData[i].Actual_Pos));
        }
        uint32_t minDelta = *min_element(Delta.begin(), Delta.end());
        if (minDelta < 100) {
            for (int vec_index = 0; vec_index < servoNums; vec_index++) {
                MotSendData[vec_index].Profile_Velocity = sync_rpm * 8388608 / 60;
                MotSendData[vec_index].Max_Velocity = 3000;
            }
        } else {
            minDelta = minDelta < 10000 ? 10000 : minDelta;
            //calculate and update each joint`s velocity
            for (int vec_index = 0; vec_index < servoNums; vec_index++) {
                MotSendData[vec_index].Profile_Velocity = sync_rpm * (float) Delta[vec_index] / minDelta * 8388608 / 60;
                MotSendData[vec_index].Max_Velocity = 3000;
            }
        }
#endif
    }
    void setNormalSpeed() {
        for (auto &d: this->MotSendData) {
            d.Profile_Velocity = sync_rpm * 8388608 / 60;
            d.Max_Velocity = 3000;
        }
    }
    bool isSingleAxisMotion{false};

private:
    vector<DTS> MotSendData{vector<DTS>(servoNums)};
    vector<DTS> &gearRatioScalar(initializer_list<double> args);
    double sync_rpm{DEFAULT_SYNC_RPM};
};

class Grap_Driver {
public:
    //    Grap_Driver(TcAds_Grap_Position_Control &adsHandle) : TcAds_ptr(&adsHandle) {}
    //T1 send T2 get
    template<typename T1, typename T2, typename T3>
    int Enable(T1 &send, T2 &get, const T3 &TcAds_ptr) {
        for (int enable_try_count{}; enable_try_count < 3; enable_try_count++) {
            uint8_t state{};
            int16_t error_code{};
            error_code = TcAds_ptr->get(get);
            if (error_code < 0)
                cout << "GRAP SERVO ENABLE: GET DATA ERROR: " << error_code << endl;
            Sleep(10);
            //第一次检查，检验是否已经上过使能了
            for (const auto child: get)
                state += (child.Status_Word & 0x37) == 0x37;
            if (state == Grap_Position_Servo_Nums) {
                cout << "All servos has been Enabled!" << endl;
                //需要更新当前控制字的引用
                for (auto &child: send)
                    child.Control_Word |= 0xf;
                state = 0;
                this->enable_flag = true;
                return 0;
            }
            //简化版本，根据当前零差伺服控制器版本，只校验状态字BIT12是否为0
            //(BIT12为1状态时，可能需要重新上下电
            for (const auto child: get)
                state += (child.Status_Word & 0x1000) >> 12;
            if (state != 0) {
                cout << "Servo Enable Invalid! Please re-power the servo!" << endl;
                return -1;
            }
            for (auto &child: send)
                child.Control_Word = 0x26;
            error_code = TcAds_ptr->set(send);
            Sleep(40);
            for (auto &child: send)
                child.Control_Word = 0x27;
            error_code = TcAds_ptr->set(send);
            Sleep(40);
            //Servo Enable!
            for (auto &child: send)
                child.Control_Word = 0x2f;
            error_code = TcAds_ptr->set(send);
            if (error_code < 0)
                cout << "SERVO ENABLE FAILURE: " << error_code << endl;
            else {
                Sleep(100);
                error_code = TcAds_ptr->get(get);
                if (error_code < 0)
                    cout << "GRAP SERVO ENABLE: GET DATA ERROR: " << error_code << endl;
                Sleep(10);
                for (const auto child: get)
                    state += (child.Status_Word & 0x37) == 0x37;
                if (state == Grap_Position_Servo_Nums) {
                    cout << "All servos  Enabled Success!" << endl;
                    this->enable_flag = true;
                    return 0;
                }
            }
            cout << "Grap Servo Enable Try counts: " << enable_try_count << endl;
        }
        cout << "Grap Servo Enable failure!" << endl;
        return -2;
    }
    template<typename T1, typename T2, typename T3>
    int Disable(T1 &send, T2 &get, const T3 &TcAds_ptr) {
        for (auto &child: send)
            child.Control_Word = 0x20;
        auto error_code = TcAds_ptr->set(send);
        if (error_code < 0) {
            cout << "SERVO DISABLE: Get Data Error:" << error_code << '\n';
            return -1;
        } else
            cout << "SERVO DISABLE SUCCESS!" << endl;
        this->enable_flag = false;
        return 0;
    }
    virtual int Enable() = 0;
    virtual int Disable() = 0;
    virtual int Motion(initializer_list<int32_t> target_list) = 0;
    bool enable_flag = false;
    virtual ~Grap_Driver() { cout << "Grap Driver QUIT!" << endl; };
};

class Grap_Driver_Position : public Grap_Driver {
public:
    Grap_Driver_Position(TcAds_Grap_Position_Control &adsHandle) {
        this->driver_p_ads = &adsHandle;
    }
    ~Grap_Driver_Position() {
        if (this->enable_flag)
            this->d_Disable();
    }
    auto d_Disable() -> int {
        return Disable();
    }
    virtual auto Enable() -> int {
        return Grap_Driver::Enable(this->SendData_P, this->GetData_P, this->driver_p_ads);
    }
    virtual auto Disable() -> int {
        return Grap_Driver::Disable(this->SendData_P, this->GetData_P, this->driver_p_ads);
    }
    virtual int Motion(initializer_list<int32_t> target_list);

private:
    vector<DFG_P> GetData_P{vector<DFG_P>(Grap_Position_Servo_Nums)};
    vector<DTG_P> SendData_P{vector<DTG_P>(Grap_Position_Servo_Nums)};
    TcAds_Grap_Position_Control *driver_p_ads = nullptr;
};
class Grap_Driver_Torque : public Grap_Driver {
public:
    Grap_Driver_Torque(TcAds_Grap_Torque_Control&adsHandle){
            this->driver_t_ads = &adsHandle;
    };
    virtual auto Enable() -> int final {
        return Grap_Driver::Enable(this->SendData_T, this->GetData_T, this->driver_t_ads);
    }
    virtual auto Disable()->int final{
       return Grap_Driver::Disable(this->SendData_T,this->GetData_T,this->driver_t_ads);
    }
    virtual int Moition(initializer_list<int32_t> target_list) final;

private:
    vector<DFG_T> GetData_T{vector<DFG_T>(Grap_Position_Servo_Nums)};
    vector<DTG_T> SendData_T{vector<DTG_T>(Grap_Position_Servo_Nums)};
    TcAds_Grap_Torque_Control *driver_t_ads = nullptr;
};