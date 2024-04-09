//
// Created by Yang on 2024/3/26.
//

#include "windows.h"
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace std;
//
#include "DVPCamera.h"
#include "ShareMemory.h"
class IMG_BASE
{
public:
	bool Convert2Mat(dvpFrame* pFrameInfo, unsigned char* pData, cv::Mat& srcImage)
	{
		if (pFrameInfo->format == FORMAT_MONO)
		{
			srcImage = cv::Mat(pFrameInfo->iHeight, pFrameInfo->iWidth, CV_8UC1, pData);
			printf("MONO convert to cv::Mat OK.\n");
		}
		else if (pFrameInfo->format == FORMAT_BGR24)
		{
			srcImage = cv::Mat(pFrameInfo->iHeight, pFrameInfo->iWidth, CV_8UC3, pData);
			//            printf("BGR24 convert to cv::Mat OK.\n");
		}
		else
		{
			printf("unsupported pixel format\n");
			return false;
		}

		if (NULL == srcImage.data)
		{
			printf("Invalid data!\n");
			return false;
		}

		/* 保存图片 */
		try
		{
			cv::imwrite("MatImage.bmp", srcImage);
		}
		catch (cv::Exception& ex)
		{
			fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
		}

		return true;
	}
	void scanDevice()
	{
		dvpUint32 count = 0, num = -1;

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
		} /* 没发现设备 */
		if (count == 0)
		{
			printf("No device found!\n");
		}
		else
		{
			cout << "use device: 0" << endl;
		}
	}
	void getIMG(shared_ptr<bool> flag, shared_ptr<cv::Mat> Image)
	{
		dvpStatus status;
		dvpHandle h;
		char* name = (char*)info[0].FriendlyName;

		status = dvpOpenByName(name, OPEN_NORMAL, &h);
		if (status != DVP_STATUS_OK)
		{
			cout << "Open device failed!" << endl;
		}
		dvpFrame frame;
		void* pBuffer;

		// start vedio stream
		status = dvpStart(h);
		while (*flag == true)
		{
			status = dvpGetFrame(h /*相机句柄*/, &frame /*帧信息*/, &pBuffer /*图像数据的内存首地址,切勿手动释放*/, 3000 /*超时时间（毫秒）*/);
			if (status != DVP_STATUS_OK)
			{
				printf("Fail to get a frame in continuous mode \r\n");
				break;
			}

			Convert2Mat(&frame, (unsigned char*)pBuffer, *Image);
			this_thread::sleep_for(chrono::milliseconds(20));
		}
		status = dvpStop(h);

		this_thread::sleep_for(chrono::milliseconds(200));
		status = dvpClose(h);
		cout << "close device" << endl;
	}
	void showIMG(const cv::Mat& imgdata)
	{
		cv::namedWindow("send data", cv::WINDOW_NORMAL);
		//        cv::resizeWindow("send data", 2568 / 2, 1920 / 2);
		for (;;)
		{
			if (imgdata.empty())
			{
				continue;
			}
			cv::imshow("send data", imgdata);
			cv::waitKey(1);
		}
	}
	void showIMG()
	{
		dvpStatus status;
		dvpHandle h;
		char* name = (char*)info[0].FriendlyName;
		do {
			/* 打开设备 */
			status = dvpOpenByName(name, OPEN_NORMAL, &h);
			if (status != DVP_STATUS_OK)
			{
				printf("dvpOpenByName failed with err:%d\r\n", status);
				break;
			}

			dvpFrame frame;
			void* pBuffer;

			/* 开始视频流 */
			status = dvpStart(h);
			if (status != DVP_STATUS_OK)
			{
				break;
			}

			cv::Mat showImage;
			/* 获取帧 */
			//		for (int j = 0; j < GRABCOUNT; j++)
			for (;;)
			{
				status = dvpGetFrame(h /*相机句柄*/, &frame /*帧信息*/, &pBuffer /*图像数据的内存首地址,切勿手动释放*/, 3000 /*超时时间（毫秒）*/);
				if (status != DVP_STATUS_OK)
				{
					printf("Fail to get a frame in continuous mode \r\n");
					break;
				}

				Convert2Mat(&frame, (unsigned char*)pBuffer, showImage);
				//                        cout<<"width: "<<showImage.cols<<" height: "<<showImage.rows<<endl;
				cv::namedWindow("ImageShow", cv::WINDOW_GUI_NORMAL);
				cv::resizeWindow("ImageShow", 2568 / 2, 1920 / 2);
				cv::imshow("ImageShow", showImage);
				cv::waitKey(20); /*每张图片显示20ms*/
			}

			/* 停止视频流 */
			status = dvpStop(h);
			if (status != DVP_STATUS_OK)
			{
				break;
			}
		} while (0);

		status = dvpClose(h);

		printf("test quit, %s, status:%d\r\n", name, status);
	}

private:
	dvpCameraInfo info[8];
};
void img_transfer(shared_ptr<bool> f)
{
	cout << "start..." << endl;
	auto img = shared_ptr<cv::Mat>(new cv::Mat);
	IMG_BASE i;
	thread task([&]() {
		i.scanDevice();
		i.getIMG(f, img);
	});

	thread imgLocalShow([&]() {
		i.showIMG(*img);
	});
	cout << "get img data" << endl;
	task.detach();
	imgLocalShow.detach();
	SHAREDMEMORY share;
	int nFrames = 0;
	string fps;
	auto t0 = cv::getTickCount();
	int index = 0;
	while (*f)
	{
		const auto frame = *img;
		//        resize(frame, frame, cv::Size(FRAME_W, FRAME_H));
		nFrames++;
		if (!frame.empty())
		{
			if (nFrames % 10 == 0)
			{
				const int N = 10;
				int64 t1 = cv::getTickCount();
				fps = " Send FPS:" + to_string((double)cv::getTickFrequency() * N / (t1 - t0)) + "fps";
				t0 = t1;
				cout << fps << endl;
			}
			//            cv::putText(frame, fps, cv::Point(100, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 1);
		}
		this_thread::sleep_for(chrono::milliseconds(1));
		share.SendMat(frame, index * FRAME_NUMBER);
	}
}
int main()
{
	printf("start...\r\n");
	auto flag = make_shared<bool>(true);
	img_transfer(flag);

	system("pause");
	return 0;
}
