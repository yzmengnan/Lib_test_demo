// ShareMemory.h : 此文件包含共享内存数据定义、大小确定、位置分配、信息定义
// Author : Jiejing.Ma
// Update : 2020/11/27
// Modefied: YangQiang
#pragma once
#ifndef ShareMemory_H
#define ShareMemory_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>  // cv::Canny()
#include <opencv2/opencv.hpp>

#include <Windows.h>

//=================================共享内存数据定义=================================
typedef struct {
    int width;
    int height;
    int type;
}ImgInf;       //图像信息
//=================================共享内存大小确定=================================
// 为图像分配空间
#define FRAME_NUMBER         1               // 图像路数
#define FRAME_W              2568
#define FRAME_H              1920
#define FRAME_W_H            FRAME_W*FRAME_H
// 图像分辨率：彩色图（3通道）+图信息结构体
#define FRAME_SIZE           FRAME_W_H*sizeof(unsigned char)*3+sizeof(ImgInf)

#define MEMORY_SIZE          FRAME_NUMBER*FRAME_SIZE

//=================================共享内存信息定义=================================
#define INITSUCCESS      0
#define CREATEMAPFAILED  1
#define MAPVIEWFAILED    2

class SHAREDMEMORY
{
public:
    SHAREDMEMORY();
    ~SHAREDMEMORY();
    void SendMat(cv::Mat img, char indexAddress);
    cv::Mat  ReceiveMat(char indexAddress);
    [[maybe_unused]]void SendStr(const char data[],int length);
    char* ReceiveStr();

public:
    int state;
private:
    HANDLE hShareMem;                               //共享内存句柄
    TCHAR sShareMemName[30] = TEXT("ShareMedia");   // 共享内存名称
    LPCTSTR pBuf;
};

#endif // !ShareMemory_H