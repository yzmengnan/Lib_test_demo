#include "DATA_STRUCT.h"
#include "Data_Log.hpp"
#include "Multi_Process.h"
#include "Transform.hpp"
#include "driverSever.h"
// #define BasicTest
// #define operationalControlTest
#define VISUAL
// #define jacobeMotionTest

using namespace std;
auto getVisualSocket() -> shared_ptr<SOCKET>
{
	const int DEFAULT_BUFLEN  = 56;
	const string DEFAULT_PORT = "8899";
	WSADATA wsaData;
	auto ConnectSocket      = make_shared<SOCKET>(INVALID_SOCKET);
	struct addrinfo *result = NULL, *ptr = NULL, hints;
	int iResult;
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0)
	{
		printf("WSAStartup failed with error: %d\n", iResult);
		return make_shared<SOCKET>(INVALID_SOCKET);
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family   = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT.data(), &hints, &result);
	if (iResult != 0)
	{
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return make_shared<SOCKET>(INVALID_SOCKET);
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{
		// Create a SOCKET for connecting to server
		*ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
		if (*ConnectSocket == INVALID_SOCKET)
		{
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return make_shared<SOCKET>(INVALID_SOCKET);
		}

		// Connect to server.
		iResult = connect(*ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR)
		{
			closesocket(*ConnectSocket);
			*ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}
	freeaddrinfo(result);
	if (*ConnectSocket == INVALID_SOCKET)
	{
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
	//        cout << c_vecs[0] << "," << c_vecs[1] << "," << c_vecs[2] << "," << c_vecs[3] << ","
	//        << c_vecs[4] << "," << c_vecs[5] << endl;
	//    }
}
int main(int argc, char* argv[])
{
	Tc_Ads ads_ptr;
	Multi_Process p;
	auto pi = p.safety_monitor_build("SAFE-CHECK.exe");
	auto d  = make_shared<MotionV1>(ads_ptr);
	//    auto fl = file_log();
	//    fl.writeFile(*d);
	d->Enable();
#ifdef BasicTest
	d->setSyncrpm(100);
	d->Write('x', -10.0F, 20.0F, 20.0F, 20.0F, 20.0F, 20.0F);
	cout << "Finished!" << endl;
#endif
#ifdef operationalControlTest
	d->setSyncrpm(10);
	if (argc == 7)
	{
		vector<double> c_vecs {atof(argv[1]), atof(argv[2]), atof(argv[3]),
		                       atof(argv[4]), atof(argv[5]), atof(argv[6])};
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(vector<float> {c_vecs.begin(), c_vecs.begin() + 6});
			Sleep(5);
		}
	}
#	ifdef jacobeMotionTest
	{
		vector<float> c_vecs {0, 1, 0, 0, 0, 0};
		cout << "move y axis!" << endl;
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(c_vecs);
			Sleep(5);
			vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
			//            c_end.t().print("operational position end:");
		}
		Sleep(1000);
		cout << "move -y axis!" << endl;
		c_vecs = {0, -1, 0, 0, 0, 0};
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(c_vecs);
			Sleep(5);
			vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
			//            c_end.t().print("operational position end:");
		}
		Sleep(1000);
		cout << "move x axis!" << endl;
		c_vecs = {1, 0, 0, 0, 0, 0};
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(c_vecs);
			Sleep(5);
			vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
			//            c_end.t().print("operational position end:");
		}
		Sleep(1000);
		cout << "move -x axia!" << endl;
		c_vecs = {-1, 0, 0, 0, 0, 0};
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(c_vecs);
			Sleep(5);
			vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
			//            c_end.t().print("operational position end:");
		}
		Sleep(1000);
		cout << "rotate with x" << endl;
		c_vecs = {0, 0, 0, 1, 0, 0};
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(c_vecs);
			Sleep(5);
			vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
			//            c_end.t().print("operational position end:");
		}
		Sleep(1000);
		cout << "rotate with -x" << endl;
		c_vecs = {0, 0, 0, -1, 0, 0};
		for (int i {}; i < 100 * 6; i++)
		{
			d->opSpaceMotionByJacobe(c_vecs);
			Sleep(5);
			vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
			//            c_end.t().print("operational position end:");
		}
		return 0;
	}
#	endif
//    vec c_end = mat(fkine_W(MDT::getAngles(*d, d->MotGetData)));
//    c_end.print("operational position end:");
//    vec c_delta = {-0.1, 0, 0, 0, 0, 0};
//    vec c_newTarget = c_end + c_delta;
//    c_newTarget.print("c_New target:");
//    d->setSyncrpm(10);
//        d->opSpaceMotion(vector<double>{c_newTarget.begin(), c_newTarget.end()});
//        Sleep(20000);
//    vec c_newEnd = fkine(MDT::getAngles(*d, d->MotGetData));
//    c_newEnd.print("c_new end: ");
#endif
#ifdef VISUAL
	d->setSyncrpm(1);
	auto socketptr = getVisualSocket();
	if (*socketptr != INVALID_SOCKET)
	{
		char recvbuf[56] = {};
		while (recv(*socketptr, recvbuf, 56, 0))
		{
			auto c_vecs = (double*)&recvbuf;
			cout << c_vecs[0] << "," << c_vecs[1] << "," << c_vecs[2] << "," << c_vecs[3] << ","
			     << c_vecs[4] << "," << c_vecs[5] << endl;
			d->opSpaceMotionByJacobe(vector<float> {(float)c_vecs[1], -(float)c_vecs[0],
			                                        (float)c_vecs[2], (float)c_vecs[3],
			                                        (float)c_vecs[4], (float)c_vecs[5]});
			Sleep(5);
		}
	}

#endif
	p.processDelete(pi);
	//        system("pause");
	return 0;
}
