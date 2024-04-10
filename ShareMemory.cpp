#include "ShareMemory.h"
// ShareMemory.cpp : 此文件包含信息定义SHAREDMEMOR类的实现
// Author : MJJ
// Update : 2020/11/27
// Modefied : YangQiang
#ifndef ShareMemory_CPP
#	define ShareMemory_CPP

#	include "ShareMemory.h"
#	include <iostream>

using namespace cv;
using namespace std;

/*************************************************************************************
FuncName  :SHAREDMEMORY::~SHAREDMEMORY()
Desc      :构造函数创建共享内存
Input     :None
Output    :None
**************************************************************************************/
SHAREDMEMORY::SHAREDMEMORY()
{
	hShareMem = CreateFileMapping(INVALID_HANDLE_VALUE,// use paging file
	                              NULL,                // default security
	                              PAGE_READWRITE,      // read/write access
	                              0,                   // maximum object size(high-order DWORD)
	                              MEMORY_SIZE,         // maximum object size(low-order DWORD)
	                              sShareMemName);      // name of mapping object

	if (hShareMem)
	{
		//  映射对象视图，得到共享内存指针，设置数据
		pBuf = (LPTSTR)MapViewOfFile(hShareMem,          // handle to map object
		                             FILE_MAP_ALL_ACCESS,// read/write permission
		                             0, 0, MEMORY_SIZE);
		cout << "memory size:" << MEMORY_SIZE << endl;

		// 若映射失败退出
		if (pBuf == NULL)
		{
			std::cout << "Could not map view of framebuffer file." << GetLastError() << std::endl;
			CloseHandle(hShareMem);
			state = MAPVIEWFAILED;
		}
	}
	else
	{
		std::cout << "Could not create file mapping object." << GetLastError() << std::endl;
		state = CREATEMAPFAILED;
	}
	state = INITSUCCESS;
}

/*************************************************************************************
FuncName  :SHAREDMEMORY::~SHAREDMEMORY()
Desc      :析构函数释放
Input     :None
Output    :None
**************************************************************************************/
SHAREDMEMORY::~SHAREDMEMORY()
{
	std::cout << "unmap shared addr." << std::endl;
	UnmapViewOfFile(pBuf);// 释放；
	CloseHandle(hShareMem);
}

/*************************************************************************************
FuncName  :void SHAREDMEMORY::SendMat(cv::Mat img, char indexAddress)
Desc      :发送Mat数据
Input     :
    Mat img               发送图像
    char indexAddress     共享内存中起始位置，若只有一路视频则无偏移
Output    :None
**************************************************************************************/
void SHAREDMEMORY::SendMat(cv::Mat img, char indexAddress)
{
	ImgInf img_head;
	img_head.width  = img.cols;
	img_head.height = img.rows;
	img_head.type   = img.type();

	if (img_head.type == CV_64FC1)
	{
		memcpy((char*)pBuf + indexAddress, &img_head, sizeof(ImgInf));
		memcpy((char*)pBuf + indexAddress + sizeof(ImgInf),         // Address of dst
		       img.data,                                            // Src data
		       img.cols * img.rows * img.channels() * sizeof(double)// size of data
		);
	}
	else
	{
		memcpy((char*)pBuf + indexAddress, &img_head, sizeof(ImgInf));
		memcpy((char*)pBuf + indexAddress + sizeof(ImgInf),// Address of dst
		       img.data,                                   // Src data
		       img.cols * img.rows * img.channels()        // size of data
		);
	}
	//    cout << "write shared mem successful." << endl;
}

/*************************************************************************************
FuncName  :cv::Mat SHAREDMEMORY::ReceiveMat(char indexAddress)
Desc      :接收Mat数据
Input     :
    char indexAddress     共享内存中起始位置，若只有一路视频则无偏移
Output    :Mat图像
**************************************************************************************/
cv::Mat SHAREDMEMORY::ReceiveMat(char indexAddress)
{
	ImgInf img_head;
	cv::Mat img;
	memcpy(&img_head, (char*)pBuf + indexAddress, sizeof(ImgInf));
	img.create(img_head.height, img_head.width, img_head.type);
	if (img_head.type == CV_64FC1)
	{
		memcpy(img.data, (char*)pBuf + indexAddress + sizeof(ImgInf),
		       img.cols * img.rows * img.channels() * sizeof(double));
	}
	else
	{
		memcpy(img.data, (char*)pBuf + indexAddress + sizeof(ImgInf),
		       img.cols * img.rows * img.channels());
	}
	return img;
}

/*************************************************************************************
FuncName  :void SHAREDMEMORY::SendStr(cv::Mat img, char indexAddress)
Desc      :发送str数据
Input     :
    Mat img               发送图像
    char indexAddress     共享内存中起始位置，若只有一路视频则无偏移
Output    :None
**************************************************************************************/
void SHAREDMEMORY::SendStr(const char data[], int length)
{
	memcpy((char*)pBuf, data, length);
	cout << "write shared mem successful." << endl;
	getchar();
}

/*************************************************************************************
FuncName  :void SHAREDMEMORY::ReceiveStr()
Desc      :接收str数据
Input     :None
Output    :获取的字符串
**************************************************************************************/
char* SHAREDMEMORY::ReceiveStr()
{
	char* str = (char*)pBuf;
	cout << "receive is:" << str << endl;
	return str;
}
#endif// !ShareMemory_CPP