//
// Created by 91418 on 2024/3/9.
//
#include "grap_action.h"
int main() {
    //    grap_action gg;
    //    gg.Enable();
    //	gg.fast_tool_move(10);

    //    gg.grap_tool(GRAP_OPEN);
    //    gg.ftmr(ROTATE_FORWARD);
    TcAds_Grap_Torque_Control ads1;
    Grap_Driver_Torque g1(ads1);
    g1.Enable();
    g1.Motion({0, 0, 400, 400});
    system("pause");
    Sleep(20);
    return 0;
}