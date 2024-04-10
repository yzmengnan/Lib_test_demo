//
// Created by 91418 on 2024/3/26.
//
// ReadMem.cpp : 此文件为读共享内存

#include "ShareMemory.h"
#include <Windows.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	int index = 0;
	SHAREDMEMORY sharemem;
	if (sharemem.state == INITSUCCESS)
	{
		// read video frame from shared memory.s
		int64 t0 = cv::getTickCount();
		;
		int64 t1    = 0;
		string fps  = "";
		int nFrames = 0;
		namedWindow("ReadMemShow", 0);

		while (true)
		{
			nFrames++;
			Mat frame = sharemem.ReceiveMat(index * FRAME_NUMBER);

			if (!frame.empty())
			{
				if (nFrames % 10 == 0)
				{
					const int N = 10;
					int64 t1    = cv::getTickCount();
					fps = " Average FPS:" + to_string((double)getTickFrequency() * N / (t1 - t0))
					    + "fps";
					t0 = t1;
				}
				cv::putText(frame, fps, Point(100, 200), cv::FONT_HERSHEY_COMPLEX, 1,
				            cv::Scalar(100, 200, 200), 1);
				imshow("ReadMemShow", frame);
			}
			if ((waitKey(1) & 0xFF) == 'q')
				break;
		}

		// char* str = sharemem.RecieveStr();
	}
	destroyAllWindows();
	return 0;
}