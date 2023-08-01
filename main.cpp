#include "comSocket.h"
#include "DATA_STRUCT.h"
#include "driverSever.h"
using namespace std;

int main() {
    Tc_Ads ads_ptr;
    auto server = make_shared<driverSever>(1115,ads_ptr);
    while(server->state>=0){

    }
    cout<<"Finish!"<<endl;
//    system("pause");
    return 0;
}
