#include "DATA_STRUCT.h"
#include "Multi_Process.h"
#include "comSocket.h"
#include "driverSever.h"
using namespace std;

int main() {
    Tc_Ads ads_ptr;
    auto server = make_shared<driverSever>(1115,ads_ptr);
    Multi_Process p;
    auto pi = p.safety_monitor_build("SAFE-CHECK.exe");
    while(server->state>=0){

    }
    cout<<"Finish!"<<endl;
    p.processDelete(pi);
//    system("pause");
    return 0;
}
