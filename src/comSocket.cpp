#include "comSocket.h"

using namespace std;

comSocket::comSocket(const int& port, const shared_ptr<_send>& sendData)
    : port {port}, socketSend(sendData)
{
	cout << "Server Socket initial!" << endl;
	comSocket_build();
	if (socketResult == 0)
	{
		auto f1 = [&]() {
			while (true)
			{
				comSocket_receive();
				if (recv_res < 0)
					break;
			}
		};
		auto f2 = [&]() {
			while (true)
			{
				comSocket_send();
				if (send_res < 0)
					break;
			}
		};
		cout << "socket start!" << endl;
		thread receive(f1);
		receive.detach();
		thread send(f2);
		send.detach();
	}
}
comSocket::comSocket(const int& port)
{
	cout << "Server Socket initial!" << endl;
	comSocket_build();
	if (socketResult == 0)
	{
		auto f1 = [&]() {
			while (true)
			{
				comSocket_receive();
				if (recv_res < 0)
					break;
			}
		};
		auto f2 = [&]() {
			while (true)
			{
				comSocket_send();
				if (send_res < 0)
					break;
			}
		};
		cout << "socket start!" << endl;
		thread receive(f1);
		receive.detach();
		thread send(f2);
		send.detach();
	}
}
int comSocket::comSocket_build()
{
	WSADATA wsadata;
	auto ListenSocket       = INVALID_SOCKET;
	auto ClientSocket       = INVALID_SOCKET;
	struct addrinfo* result = nullptr;
	struct addrinfo hints {
	};
	// Initialize Winsock
	socketResult = WSAStartup(MAKEWORD(2, 2), &wsadata);
	if (socketResult != 0)
	{
		cout << "WSAStratup failed with error:" << socketResult << endl;
		return -1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family   = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags    = AI_PASSIVE;
	stringstream PORT;
	PORT << port;
	string SERVER_NAME;
	PORT >> SERVER_NAME;
	// Resolve the server address and port
	socketResult = getaddrinfo(nullptr, SERVER_NAME.data(), &hints, &result);
	if (socketResult != 0)
	{
		cout << "getdaddrinfo failed with error: " << socketResult << endl;
		WSACleanup();
		return -1;
	}
	// create a socket for the server to listen for client connections.
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET)
	{
		cout << "socket failed with error: " << WSAGetLastError() << endl;
		freeaddrinfo(result);
		WSACleanup();
		return -1;
	}
	// set up the tcp listening socket
	socketResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (socketResult == SOCKET_ERROR)
	{
		cout << "bind failed with error: " << WSAGetLastError() << endl;
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		return -1;
	}
	freeaddrinfo(result);
	cout << "start listening!" << endl;
	socketResult = listen(ListenSocket, SOMAXCONN);
	if (socketResult == SOCKET_ERROR)
	{
		cout << "Listen failed with error: " << WSAGetLastError() << endl;
		closesocket(ListenSocket);
		WSACleanup();
		return -1;
	}
	// accepte a client socket
	ClientSocket = accept(ListenSocket, nullptr, nullptr);
	if (ClientSocket == INVALID_SOCKET)
	{
		cout << "accept failed with error: " << WSAGetLastError() << endl;
		closesocket(ListenSocket);
		WSACleanup();
		return -1;
	}
	cout << "Listen Socket Success, Server Socket has been built!" << endl;
	closesocket(ListenSocket);
	com_socket = ClientSocket;
	return socketResult;
}

void comSocket::comSocket_receive()
{
	std::array<char, 1024> recvbuf {};
	while (recv_res >= 0)
	{
		recv_res                     = recv(com_socket, &recvbuf.front(), 132, 0);
		this->socketRecv->Head_check = *((int*)&recvbuf.front());
		this->socketRecv->Command    = *((int*)&recvbuf.front() + socketRecv->Command_location);
		socketRecv->Joint_Position_set =
		    vector<float> {(float*)&recvbuf.front() + socketRecv->Joint_Position_set_location,
		                   (float*)&recvbuf.front() + socketRecv->Joint_Position_set_location + 9};
		socketRecv->Joint_Velocity_set =
		    vector<float> {(float*)&recvbuf.front() + socketRecv->Joint_Velocity_set_location,
		                   (float*)&recvbuf.front() + socketRecv->Joint_Velocity_set_location + 9};
		socketRecv->Cartesian_Position_set = vector<float> {
		    (float*)&recvbuf.front() + socketRecv->Cartesian_Position_set_location,
		    (float*)&recvbuf.front() + socketRecv->Cartesian_Position_set_location + 6};
		socketRecv->Cartesian_Velocity_set = vector<float> {
		    (float*)&recvbuf.front() + socketRecv->Cartesian_Velocity_set_location,
		    (float*)&recvbuf.front() + socketRecv->Cartesian_Velocity_set_location + 6};
		socketRecv->Tail_check = *((int*)&recvbuf.front() + socketRecv->Tail_check_location);
	}
	cout << "Socket Communication Error:" << recv_res << endl;
	socketResult = -1;
}

void comSocket::comSocket_send()
{
	//_send 的数据应该在外部引用更新
	while (send_res >= 0)
	{
		servoStatusLock.lock();
		vector<char> byte_data {};
		// 头校验字节添加
		char* temp = (char*)&socketSend->Head_check;
		// 将数据取地址后，使用char型指针指向地址
		for (int i = 0; i < 4; i++)
		{
			// 对temp指针加1，得到char型指针的单个字节，随后对每个指针解引用，实现了将数据拆分为字节型
			byte_data.push_back(*(temp + i));
		}
		// 状态字字节添加
		temp = (char*)&socketSend->Status;
		for (int i = 0; i < 4; i++)
		{
			byte_data.push_back(*(temp + i));
		}
		// 关节空间位置信息添加
		temp = (char*)(socketSend->Joint_Position.data());
		for (int j = 0; j < 4 * 9; j++)
			byte_data.push_back(*(temp + j));
		// 笛卡尔位置信息添加
		temp = (char*)socketSend->Cartesian_Position.data();
		for (int j = 0; j < 4 * 6; j++)
			byte_data.push_back(*(temp + j));
		// 关节速度信息添加
		temp = (char*)socketSend->Joint_Velocity.data();
		for (int j = 0; j < 4 * 9; j++)
			byte_data.push_back(*(temp + j));
		// 笛卡尔速度信息添加
		temp = (char*)socketSend->Cartesian_Velocity.data();
		for (int j = 0; j < 4 * 6; j++)
		{
			byte_data.push_back(*(temp + j));
		}
		// 尾校验
		temp = (char*)&socketSend->Tail_check;
		for (int j = 0; j < 4; j++)
			byte_data.push_back(*temp + j);
		servoStatusLock.unlock();
		send_res = send(com_socket, byte_data.data(), byte_data.size(), 0);
		this_thread::sleep_for(chrono::milliseconds(Socket_Hz));
	}
	cout << "send error :" << send_res << endl;
	socketResult = -2;
}
