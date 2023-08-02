#pragma once
#include <winsock2.h>
//
#include "DATA_STRUCT.h"
#include "motionDataTransform.hpp"
#include "ws2tcpip.h"
#include <array>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>
using namespace std;
class comSocket {
public:
    comSocket(const int &port, const shared_ptr<_send> &sendData);
    comSocket(const int &port);
    shared_ptr<_recv> socketRecv{make_shared<_recv>()};
    shared_ptr<_send> socketSend{make_shared<_send>()};
    int comSocket_build();
    void comSocket_send();
    //        void comSocket_send(shared_ptr<_send>& send_data,int);
    void comSocket_receive();
    int socketResult{0};
protected:
    int port{1115};

public:
    mutex servoStatusLock{};
private:
    SOCKET com_socket{};
    int recv_res{0};
    int send_res{0};
};
