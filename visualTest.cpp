#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-avoid-magic-numbers"
#include "driverSever.h"
//
#include <iostream>
#include <thread>

#include "Multi_Process.h"
using namespace std;

// DVP API 依赖
#include <opencv2/opencv.hpp>

#include "DVPCamera.h"
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
	~myCamera() { stop(); }
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

	auto angles_now     = MDT::getAngles(*d, d->MotGetData);
	auto angles_6axis   = vector<double> {angles_now.begin(), angles_now.begin() + 6};
	auto c_position_now = r.fkine(angles_6axis);
	rowvec cp {c_position_now};
	cout << "base position: " << cp;
	Sleep(2000);
	vector<double> delta_based_on_end {0.0f, 0.0f, 0.0f, 1.0f};
	string name;
	int i {};
	if (argc >= 2)
	{
		int movingflag = atoi(argv[1]);
		switch (movingflag)
		{
			case 0: delta_based_on_end[0] += 0.2; break;
			case 1: delta_based_on_end[1] += 0.2; break;
			case 2: delta_based_on_end[0] -= 0.2; break;
			case 3: delta_based_on_end[1] -= 0.2; break;
		}
			// start motion
#ifdef MOTION_BY_TARGET
		thread time_cnt([&]() {
			while (true)
			{
				this_thread::sleep_for(chrono::milliseconds(10));
				i++;
			}
		});
		time_cnt.detach();
		d->opSpaceTargetMotion(c_position_now, delta_based_on_end, r);
#else
		auto movef = [&]() {
			vector<float> c_vecs {};
			for (int i {}; i < MOVE_DISTANCE; i++)
			{
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

#	ifdef USE_RL
				d->opSpaceMotionByJacobe(c_vecs, r);
				std::this_thread::sleep_for(chrono::milliseconds(20));
#	else
				d->opSpaceMotionByJacobe_RL(c_vecs, r);
#	endif
			}
			cv::imwrite(name + to_string(i) + ".jpg", showImage);
			c.stop();
		};
		thread task(movef);
		task.join();
#endif
	}
	else
	{
		return 0;
	}

	this_thread::sleep_for(chrono::milliseconds(5));
#ifndef NO_VIDEO

	auto getFrame = [&]() {
		for (;;)
		{
			if (!c.run)
				break;
			c.getMat(showImage);
		}
	};
	thread task2(getFrame);
	task2.detach();
#endif

	d->Disable();
	cout << "i:" << i << endl;
	return 0;
}