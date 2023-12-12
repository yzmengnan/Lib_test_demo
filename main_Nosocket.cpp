#include "DATA_STRUCT.h"
#include "Data_Log.hpp"
#include "Multi_Process.h"
#include "Transform.hpp"
#include "driverSever.h"
#define BasicTest
#define operationalControlTest
using namespace std;
int main() {
    Tc_Ads ads_ptr;
    Multi_Process p;
    auto pi = p.safety_monitor_build("SAFE-CHECK.exe");
    auto d = make_shared<MotionV1>(ads_ptr);
    //    auto fl = file_log();
    //    fl.writeFile(*d);
    d->Enable();
#ifdef BasicTest
    d->setSyncrpm(100);
    d->Write('2', 10.0F, 10.0F,10.0F, 10.0F, 10.0F,10.0F);
    Sleep(10000);
    cout << "Finished!" << endl;
#else
    vector<DTS> cspSendData(servoNums, DTS{});
    vector<DFS> cspGetData(servoNums, DFS{});
    while (true) {
        d->servoCSP(cspSendData, cspGetData);
    }
#endif
#ifdef operationalControlTest
    auto c_begin = mat(fkine(MDT::getAngles(*d, d->MotGetData)));
    c_begin.print("operational position: ");
//    vector<float> c_target{-0.551,2.032,0.034,32.135F,-29.162F,-84.174F};
    vector<float> c_vecs{-1,0,0,0,0,0};

    d->setSyncrpm(50);
    for(int i{};i<100*6;i++){
        d->opSpaceMotionByJacob(c_vecs);
        Sleep(5);
    }
    vec  c_end = mat(fkine(MDT::getAngles(*d,d->MotGetData)));
    c_end.print("operational position end:");
    vec c_delta = {-0.2,0,0,0,0,0};
    vec c_newTarget = {-0.529,2.036,0.097,39.0059,-47.2695,-73.2263};
    c_newTarget.print("c_New target:");
    d->setSyncrpm(100);
    d->opSpaceMotion(vector<double>{c_newTarget.begin(),c_newTarget.end()});
    Sleep(8000);
    vec c_newEnd = fkine(MDT::getAngles(*d,d->MotGetData));
    c_newEnd.print("c_new end: ");
#endif
    p.processDelete(pi);
    //    system("pause");
    return 0;
}
