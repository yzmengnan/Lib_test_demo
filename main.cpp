#include "DATA_STRUCT.h"
#include "Multi_Process.h"
#include "driverSever.h"
#include "Data_Log.hpp"
using namespace std;

int main() {
    Tc_Ads ads_ptr;
    Multi_Process p;
    auto pi = p.safety_monitor_build("SAFE-CHECK.exe");
    auto server = make_shared<driverSever>(1115,ads_ptr);
    auto fl = file_log();
    fl.writeFile(*server);
    while(server->state>=0){

    }
    cout<<"Finished!"<<endl;
    p.processDelete(pi);
//    system("pause");
    return 0;
}
