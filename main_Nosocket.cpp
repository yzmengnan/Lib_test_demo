#include "DATA_STRUCT.h"
#include "Data_Log.hpp"
#include "Multi_Process.h"
#include "Transform.hpp"
#include "driverSever.h"
#define BasicTest
//#define operationalControlTest
//#define VISUAL
using namespace std;
auto getVisualSocket() -> shared_ptr<SOCKET> {
    const int DEFAULT_BUFLEN = 56;
    const string DEFAULT_PORT = "8899";
    WSADATA wsaData;
    auto ConnectSocket = make_shared<SOCKET>(INVALID_SOCKET);
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    int iResult;
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return make_shared<SOCKET>(INVALID_SOCKET);
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT.data(), &hints, &result);
    if (iResult != 0) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return make_shared<SOCKET>(INVALID_SOCKET);
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

        // Create a SOCKET for connecting to server
        *ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
                                ptr->ai_protocol);
        if (*ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return make_shared<SOCKET>(INVALID_SOCKET);
        }

        // Connect to server.
        iResult = connect(*ConnectSocket, ptr->ai_addr, (int) ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(*ConnectSocket);
            *ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }
    freeaddrinfo(result);
    if (*ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return make_shared<SOCKET>(INVALID_SOCKET);
    }
    // Receive until the peer closes the connection
    cout << "start receiving!" << endl;
    return ConnectSocket;
    //    while (TRUE) {
    //        iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
    //        cout << "received data" << endl;
    //        auto c_vecs = (double *) &recvbuf;
    //        cout << c_vecs[0] << "," << c_vecs[1] << "," << c_vecs[2] << "," << c_vecs[3] << "," << c_vecs[4] << "," << c_vecs[5] << endl;
    //    }
}
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
    d->Write('x', 20.0F, 0.0F, -40.0F, 0.0F, 20.0F, 0.0F);
    cout << "Finished!" << endl;
#endif
#ifdef operationalControlTest
    auto c_begin = mat(fkine(MDT::getAngles(*d, d->MotGetData)));
    c_begin.print("operational position: ");
    //    vector<float> c_target{-0.551,2.032,0.034,32.135F,-29.162F,-84.174F};
    vector<float> c_vecs{0, 0, 0, 0, 0, 0};
    d->setSyncrpm(100);
    for (int i{}; i < 100 * 6; i++) {
        d->opSpaceMotionByJacob(c_vecs);
        Sleep(5);
    }
    return 0;
    vec c_end = mat(fkine(MDT::getAngles(*d, d->MotGetData)));
    c_end.print("operational position end:");
    vec c_delta = {0, 0, 0.5, 0, 0, 0};
    vec c_newTarget = c_end + c_delta;
    c_newTarget.print("c_New target:");
    d->setSyncrpm(50);
    d->opSpaceMotion(vector<double>{c_newTarget.begin(), c_newTarget.end()}, 100);
    //    Sleep(20000);
    vec c_newEnd = fkine(MDT::getAngles(*d, d->MotGetData));
    c_newEnd.print("c_new end: ");
#endif
#ifdef VISUAL
    auto socketptr = getVisualSocket();
    if (*socketptr != INVALID_SOCKET) {
        char recvbuf[56] = {};
        while (recv(*socketptr, recvbuf, 56, 0)) {
            auto c_vecs = (double *) &recvbuf;
            cout << c_vecs[0] << "," << c_vecs[1] << "," << c_vecs[2] << "," << c_vecs[3] << "," << c_vecs[4] << "," << c_vecs[5] << endl;
            d->opSpaceMotionByJacob(vector<float>{(float) c_vecs[0], (float) c_vecs[1], (float) c_vecs[2], 0, 0, 0});
            Sleep(5);
        }
    }

#endif
    p.processDelete(pi);
    //    system("pause");
    return 0;
}
