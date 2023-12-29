/*****************************************************************************
* @FileName:dvp2_opencv.cpp
* @Author: chenyn
* @Mail:mexbochen@foxmail.com
* @CreatTime: 2020/8/24 14:42
* @Descriptions: 编译前请配置好opencv、dvp2 api有关的库路径，避免编译报错；
* @Version: ver 1.0
* @Copyright(c) 2020 Do3Think All Rights Reserved.
*****************************************************************************/
#include <iostream>
#include <thread>
#include <windows.h>
using namespace std;

// DVP API 依赖
#include "DVPCamera.h"
// OpenCV依赖一共需要配置4处，内容如下：
//1、项目属性->VC++目录->包含目录中加入$(ProjectDir)OpenCV-3.49\include;$(ProjectDir)OpenCV-3.49\include\opencv2
//2、项目属性->VC++目录->库目录中加入$(ProjectDir)OpenCV-3.49\lib\x86\vc15
//3、项目属性->链接器->输入->附件依赖项中加入opencv_world349d.lib或者opencv_world349.lib。
//4、项目属性->调试->环境，
//		- 编译64位添加：path=%path%;$(ProjectDir)OpenCV-3.49\bin\x64\vc15，或者直接把dll库拷贝到生成的exe程序目录
#include <opencv2/opencv.hpp>

#pragma warning(disable:4996)

#define GRABCOUNT 50

//RGB to BGR
bool RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
	if (NULL == pRgbData)
	{
		return false;
	}

	for (unsigned int j = 0; j < nHeight; j++)
	{
		for (unsigned int i = 0; i < nWidth; i++)
		{
			unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
			pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
			pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
		}
	}
	return true;
}

// 把获取到的buffer转成Mat格式
bool Convert2Mat(dvpFrame* pFrameInfo, unsigned char* pData,cv::Mat& srcImage)
{	if (pFrameInfo->format == FORMAT_MONO)
    	{
    		srcImage = cv::Mat(pFrameInfo->iHeight, pFrameInfo->iWidth, CV_8UC1, pData);
    		printf("MONO convert to cv::Mat OK.\n");
    	}
    	else if (pFrameInfo->format == FORMAT_BGR24)
    	{
    		srcImage = cv::Mat(pFrameInfo->iHeight, pFrameInfo->iWidth, CV_8UC3, pData);
//    		printf("BGR24 convert to cv::Mat OK.\n");
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
    	try {
    		cv::imwrite("MatImage.bmp", srcImage);
    	}
    	catch (cv::Exception& ex) {
    		fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
    	}


	return true;
}

void test(void* p)
{
	dvpStatus status;
	dvpHandle h;
	char* name = (char*)p;

	printf("Test start,camera is %s\r\n", name);

	do
	{
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
        int i{};
        for (;;)
		{
			status = dvpGetFrame(h/*相机句柄*/, &frame/*帧信息*/, &pBuffer/*图像数据的内存首地址,切勿手动释放*/, 3000/*超时时间（毫秒）*/);
			if (status != DVP_STATUS_OK)
			{
				printf("Fail to get a frame in continuous mode \r\n");
				break;
			}

			Convert2Mat(&frame, (unsigned char*)pBuffer, showImage);

			cv::namedWindow("ImageShow", 0);
//			cv::resizeWindow("ImageShow", 640, 480);
			cv::imshow("ImageShow", showImage);
            string name="../img/a";
            cv::imwrite(name+to_string(i++)+".jpg",showImage);
			cv::waitKey(1);	/*每张图片显示20ms*/
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


int main()
{
	printf("start...\r\n");

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

	/* 没发现设备 */
	if (count == 0)
	{
		printf("No device found!\n");
		return 0;
	}

	while (num < 0 || num >= count)
	{
		printf("Please enter the number of the camera you want to open: \r\n");
		scanf("%d", &num);
	}

	thread task(test, (void*)info[num].FriendlyName);
	task.join();

	system("pause");
	return 0;
}