//
// Created by 91418 on 2024/3/9.
//
#include "Driver.h"

int main() {
    TcAds_Grap_Position_Control gp_ads{};
    TcAds_Grap_Torque_Control gt_ads{};
    Grap_Driver_Position gp(gp_ads);
    Grap_Driver_Torque gt(gt_ads);
//    Grap_Driver *g=&gp;
    Grap_Driver *g=&gt;
    g->Enable();
    g->Motion({200});
    Sleep(2000);
    g = &gp;
    g->Enable();
    g->Motion({1000,200000});
    Sleep(5000);
    g->Disable();
    g = &gt;
    g->Disable();
//    gp.Disable();
}