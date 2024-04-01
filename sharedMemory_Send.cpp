//
// Created by 91418 on 2024/3/26.
//
#include "ShareMemory.h"
#include <iostream>

using namespace std;
using namespace cv;

// 读图片或视频
void send_img(SHAREDMEMORY sharedsend) {
    int index = 0;
    int64 t0 = cv::getTickCount();
    ;
    int64 t1 = 0;
    string fps = "";
    int nFrames = 0;

    cv::Mat frame;

    cout << "Opening video..." << endl;
    VideoCapture cap(0,CAP_V4L2);
    VideoWriter vw;
    cap.open(0);
    cap.set(CAP_PROP_FPS,30);
    cap.set(CAP_PROP_FOURCC,vw.fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    if (!cap.isOpened()) {
        cout << "ERROR! Unable to open camera!" << endl;
    }
    while (cap.isOpened()) {
        cap >> frame;
        cout << "frame width: " << frame.cols << " frame height: " << frame.rows << endl;
        //        imshow("Live",frame);
        cv::namedWindow("local test", cv::WINDOW_GUI_NORMAL | cv::WINDOW_AUTOSIZE);
        imshow("local test", frame);
                waitKey(10);
        if (frame.empty()) {
            std::cerr << "ERROR: Can't grab video frame." << endl;
            break;
        }
        resize(frame, frame, Size(FRAME_W, FRAME_H));

        nFrames++;

        if (!frame.empty()) {
            if (nFrames % 10 == 0) {
                const int N = 10;
                int64 t1 = cv::getTickCount();
                fps = " Send FPS:" + to_string((double) getTickFrequency() * N / (t1 - t0)) + "fps";
                t0 = t1;
            }
            cv::putText(frame, fps, Point(100, 100), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 1);
        }
        sharedsend.SendMat(frame, index * FRAME_NUMBER);

//        if ((waitKey(1) & 0xFF) == 'q') break;
    }
}


int main() {
    SHAREDMEMORY sharedmem;
    //char str[] = "hello";
    if (sharedmem.state == INITSUCCESS) send_img(sharedmem);
    //if (sharedmem.state == INITSUCCESS) sharedmem.SendStr(str);

    return 0;
}