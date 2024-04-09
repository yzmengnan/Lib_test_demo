#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
#include "driverSever.h"
//
#include "Multi_Process.h"
#include <iostream>
#include <thread>
using namespace std;

// DVP API 依赖
#include "DVPCamera.h"
#include <opencv2/opencv.hpp>
#define MOVE_LOG
#define MOVE_DISTANCE 1000
#define MOTION_BY_TARGET
#define NO_VIDEO

#ifndef NO_VIDEO
class myCamera
{
public:
	myCamera(void* p)
	{
		char* name = (char*)p;
		status     = dvpOpenByName(name, OPEN_NORMAL, &h);
		if (status != DVP_STATUS_OK)
		{
			cout << "dvpOpenByName failed with err: " << status << endl;
		}
	}
	~myCamera()
	{
		stop();
	}
	dvpStatus start()
	{
		if (status != DVP_STATUS_OK)
			return DVP_STATUS_DENIED;
		cout << "start camera" << endl;
		run = true;
		return dvpStart(h);
	}
	dvpStatus stop()
	{
		status = dvpStop(h);
		status = dvpClose(h);
		run    = false;
		return status;
	}
	bool getMat(cv::Mat& srcImg)
	{
		if (run)
			status = dvpGetFrame(h, &frame, &pBuffer, 2000);
		if (status != DVP_STATUS_OK)
		{
			cout << "Failed to get a frame in continuous mode" << endl;
			//            cout << status << endl;
		}
		srcImg = cv::Mat(frame.iHeight, frame.iWidth, CV_8UC3, (unsigned char*)pBuffer);
		if (NULL == srcImg.data)
		{
			return false;
		}
		return true;
	};

	bool run = false;

private:
	dvpStatus status;
	dvpHandle h;
	void* pBuffer;
	dvpFrame frame;
};
#endif
int main(int argc, char* argv[])
{
	cout << "start servo" << endl;
	Tc_Ads ads_ptr;
	auto d = make_shared<MotionV1>(ads_ptr);
	Multi_Process pp;
	auto pi = pp.safety_monitor_build("SAFE-CHECK.exe");
	d->Enable();
	d->setSyncrpm(100);
	d->Write('x', -17.81, 1.65, 30.86, 1.08, -17.62, -2.50);
#ifndef NO_VIDEO
	cv::Mat showImage;
	dvpUint32 count = 0, num = -1;
	dvpCameraInfo info[8];
	/* 枚举设备 */
	dvpRefresh(&count);
	if (count > 8)
		count = 8;

	for (int i = 0; i < (int)count; i++)
	{
		if (dvpEnum(i, &info[i]) == DVP_STATUS_OK)
		{
			printf("[%d]-Camera FriendlyName : %s\r\n", i, info[i].FriendlyName);
		}
	}
	num    = 0;
	auto c = myCamera(&num);
	cout << "start number: " << c.start();
	c.getMat(showImage);
#endif
//	this_thread::sleep_for(chrono::seconds(5));
#ifdef USE_RL
	static Robot r("../myrobot.xml");
#endif

	auto angles_now                  = MDT::getAngles(*d, d->MotGetData);
	auto angles_6axis                = vector<double>{angles_now.begin(), angles_now.begin() + 6};
	auto c_position_now              = r.fkine(angles_6axis);
	vector<double> target_c_position = c_position_now;
	string name;
	if (argc >= 2)
	{
		int movingflag = atoi(argv[1]);
		switch (movingflag)
		{
		case 0:
			target_c_position[0] += 0.2;
			break;
		case 1:
			target_c_position[1] += 0.2;
			break;
		case 2:
			target_c_position[0] -= 0.2;
			break;
		case 3:
			target_c_position[1] -= 0.2;
			break;
		}
		auto movef = [&]() {
			vector<float> c_vecs{};
#ifdef MOTION_BY_TARGET
			int i{};
			while (true)
#else
			for (int i{}; i < MOVE_DISTANCE; i++)
#endif
			{
				//				cout << "count: " << i << endl;
				auto angles_now     = MDT::getAngles(*d, d->MotGetData);
				auto angles_6axis   = vector<double>{angles_now.begin(), angles_now.begin() + 6};
				auto c_position_now = r.fkine(angles_6axis);
				vector<double> c_Delta;
				for (int j{}; j < 6; j++)
				{
					cout << c_position_now[j] << ",";
					c_Delta.push_back(target_c_position[j] - c_position_now[j]);
				}
				cout << endl;

				for (int j{}; j < 6; j++)
				{
					cout << target_c_position[j] << ",";
				}
				cout << endl;
#ifdef MOTION_BY_TARGET
				//				auto jointDataNow    = vec(angles_now.data(), 6);
				//				auto jointDataTarget = vec(target_c_position.data(), 6);
				double delta{};
				for (int c{}; c < 6; c++)
				{
					if (c >= 3)
						delta += abs(c_Delta[c]) * rl::math::constants::deg2rad;
					else
						delta += abs(c_Delta[c]);
				}
				//				if (norm(jointDataNow - jointDataTarget) < 0.05)
				cout << "delta: " << delta << endl;
				if (delta <= 0.05)
				{
					cout << "finish,reach the target" << endl;
					break;
				}
				else
				{
					i++;
				}
#endif
				switch (movingflag)
				{
				case 0:
					c_vecs = {1, 0, 0, 0, 0, 0};
					name   = "../img/new/0/";
					break;
				case 1:
					c_vecs = {0, 1, 0, 0, 0, 0};
					name   = "../img/new/1/";
					break;
				case 2:
					c_vecs = {-1, 0, 0, 0, 0, 0};
					name   = "../img/new/2/";
					break;
				case 3:
					c_vecs = {0, -1, 0, 0, 0, 0};
					name   = "../img/new/3/";
					break;
				}

#ifdef USE_RL
#ifdef MOTION_BY_TARGET
				d->opSpaceMotionByJacobe_RL(c_Delta, r);
				std::this_thread::sleep_for(chrono::milliseconds(10));
			}
#else
				d->opSpaceMotionByJacobe_RL(c_vecs, r);
#endif
#else

				d->opSpaceMotionByJacobe(c_vecs);
#endif
#ifdef MOVE_LOG
		};
		thread task(movef);
		task.join();
	}
	//	else if (argc == 3)
	//
	//	{
	//		name += "reverse/";
	//	}
	//
	else
	{
		return 0;
	}

	this_thread::sleep_for(chrono::milliseconds(5));
#ifndef NO_VIDEO
	cv::imwrite(name + to_string(i) + ".jpg", showImage);
	c.stop();
#endif
#endif
#ifndef NO_VIDEO
	auto getFrame = [&]() {
		for (;;)
		{
			if (!c.run)
				break;
			c.getMat(showImage);
		}
	};
#endif
#ifndef NO_VIDEO
	thread task2(getFrame);
	task2.detach();
#endif

	d->Disable();
	return 0;
}
#pragma clang diagnostic pop