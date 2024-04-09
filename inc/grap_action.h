//
// Created by Yang on 2024/4/1.
//

#ifndef FINAL_GRAP_ACTION_H
#define FINAL_GRAP_ACTION_H

#include "Driver.h"
#define Grap_Tool_limit_therthold 40000
enum {
    GRAP_CLOSE = 0,
    GRAP_OPEN,
    ROTATE_FORWARD = 0,
    ROTATE_BACKWARD
};

using namespace std;
#ifdef EndEffector_Histroy_Func
class grap_action {
public:
    grap_action();
    grap_action(gp &m1, gt &m2);
    grap_action(TcAds_Grap_Position_Control &ads_p, TcAds_Grap_Torque_Control &ads_t);
    ~grap_action();

    void Enable() {
        int err;
        err += m1.Enable();
        err += m2.Enable();
        if (err != 0)
            cout << "Enable error!" << endl;
        else
            isEnabled = true;
    }

    void Disable() {
        m1.Disable();
        m2.Disable();
        isEnabled = false;
    }

    void fast_tool_move(const int &dist);

    void ftmr(bool f_or_b);
    void grap_tool(bool flag);


private:
    gp m1;
    gt m2;
    bool isEnabled{false};
    double fast_tool_moving_ratio{50000.0f};
    int fast_tool_moving_offset{-3447630};
    int uptool_open_position{94000}, uptool_close_position{3660000};
    int downtool_open_position{-2550000}, downtool_close_position{940000};
    int grap_torque = 450;
    int grap_torque_threshold = 750;
    int overtime = 30;
};

#endif
class EndEffector_Motion {

public:
    EndEffector_Motion() {
        m1 = Ep();
        m2 = Et();
    }
    void Enable() {
        int err;
        err += m1.Enable();
        err += m2.Enable();
        if (err != 0)
            cout << "Enable error!" << endl;
        else
            isEnabled = true;
    }

    void Disable() {
        m1.Disable();
        m2.Disable();
        isEnabled = false;
    }

    void fast_tool_move(const int &dist);
    void ftmr(bool f_or_b);
    void grap_tool(bool flag);

private:
    Ep m1;
    Et m2;
    bool isEnabled{false};
    double fast_tool_moving_ratio{50000.0f};
    int fast_tool_moving_offset{-3447630};
    int uptool_open_position{94000}, uptool_close_position{3660000};
    int downtool_open_position{-2550000}, downtool_close_position{940000};
    int grap_torque = 450;
    int grap_torque_threshold = 750;
    int overtime = 40;
};

#endif//FINAL_GRAP_ACTION_H
