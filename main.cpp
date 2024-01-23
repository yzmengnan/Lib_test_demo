
#include "DATA_STRUCT.h"
#include "Data_Log.hpp"
#include "Multi_Process.h"
#include "driverSever.h"
#include "monitorData.h"
using namespace std;

int main() {
    Tc_Ads ads_ptr;
    Multi_Process p;
    auto pi = p.safety_monitor_build("SAFE-CHECK.exe");
    auto server = make_shared<driverSever>(SOCKET_PORT, ads_ptr);
    //    monitorData md;
    //    thread monitor(&monitorData::sendMessage,md,ref(*server),"dataMonitor.exe");
    //    monitor.detach();
    //    auto fl = file_log();
    //    fl.writeFile(*server);
    while (server->state >= 0) {
    }
    cout << "Finished!" << endl;
    p.processDelete(pi);
    //    system("pause");
    return 0;
}
