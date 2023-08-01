// vs_Low.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "comSocket.h"
#include <iostream>
#include <thread>
using namespace std;

int main() {
    auto sendData = make_shared<_send>();
    auto socketService = make_shared<comSocket>(1115,sendData);
    auto addf = [](string data) {
        data = data.substr(0, data.find('.') + 3);
        while (data.size() < 4) {
            data = " " + data;
        }
        return data;
    };
    ostringstream message{};
    while (true) {
        message << "===H: " << (socketService->socketRecv->Head_check) << "|CW: " << (socketService->socketRecv->Command);
        message << "|Joint Data:";
        int i{};
        for (auto j: socketService->socketRecv->Joint_Position_set) {
            message << "|No:" << i << ": " << addf(to_string(j));
            i++;
            if(i>=2){
                break;
            }
        }
        cout << message.str() << "===";
        cout << endl;
        message.str("");
        Sleep(20);
    }
    return 0;
}

#pragma clang diagnostic pop