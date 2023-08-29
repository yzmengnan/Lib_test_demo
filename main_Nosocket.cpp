#include "DATA_STRUCT.h"
#include "Multi_Process.h"
#include "driverSever.h"
#include "Data_Log.hpp"
using namespace std;

int main() {
    Tc_Ads ads_ptr;
    Multi_Process p;
    auto pi = p.safety_monitor_build("SAFE-CHECK.exe");
    auto d = make_shared<MotionV1>(ads_ptr);
//    auto fl = file_log();
//    fl.writeFile(*d);
    d->Enable();
    d->setSyncrpm(100);
    d->Write('1',0.0f,0.0f,0.0f);
    Sleep(5000);
    d->Write('1',10.0f,10.0f,10.0f);
    Sleep(5000);
    cout<<"Finished!"<<endl;
    p.processDelete(pi);
//    system("pause");
    return 0;
}
