//
// Created by 91418 on 2023/7/5.
//
// main.cpp
#include "DATA_STRUCT.h"
#include "iostream"
#include <windows.h>
#define buffsize 4096
using namespace std;

int main(int argc, char* argv[])
{
	//    SetConsoleOutputCP(CP_UTF8);
	cout << "\n ** This is a message from the data_monitor process. ** \n";
	string buf(buffsize, 0);
	DWORD len;
	HANDLE handle_read;
	handle_read = GetStdHandle(STD_INPUT_HANDLE);
	if (handle_read == INVALID_HANDLE_VALUE)
		ExitProcess(1);

	for (;;)
	{
		// Read from standard input and stop on error or no data.
		bool ret = ReadFile(handle_read, buf.data(), buffsize, &len, NULL);
		if (!ret || len == 0)
		{
			cout << "ReadFile error!" << endl;
			break;
		}
		cout << buf << endl;
		Sleep(1000 / MONITOR_Hz);
	}
	cout << "finish reading!" << endl;
	//    freopen("CON", "r", stdin);    // 重定向输入,否则system("pause")会无效
	// CloseHandle(handle_read);
	//    system("pause");
	return 0;
}
