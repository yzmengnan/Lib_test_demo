//
// Created by 91418 on 2024/3/9.
//
#include "Driver.h"

int main() {
    TcAds_Grap_Position_Control gp_ads{};
    Grap_Driver_Position gp(gp_ads);
    Grap_Driver *g=&gp;
    g->Enable();
    g->Motion({10});
    Sleep(5000);
//    gp.Disable();
}