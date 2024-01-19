#include "Multi_Process.h"
#include "driverSever.h"
#include <iostream>
#include <thread>
#include <windows.h>
using namespace std;

// DVP API 依赖
#include "DVPCamera.h"
#include <opencv2/opencv.hpp>
#define MOVE_LOG
#define MOVE_DISTANCE 1000
class myCamera {
public:
    myCamera(void *p) {
        char *name = (char *) p;
        status = dvpOpenByName(name, OPEN_NORMAL, &h);
        if (status != DVP_STATUS_OK) {
            cout << "dvpOpenByName failed with err: " << status << endl;
        }
    }
    ~myCamera() {
        stop();
    }
    dvpStatus start() {
        if (status != DVP_STATUS_OK)
            return DVP_STATUS_DENIED;
        cout << "start camera" << endl;
        run = true;
        return dvpStart(h);
    }
    dvpStatus stop() {
        status = dvpStop(h);
        status = dvpClose(h);
        run = false;
    }
    bool getMat(cv::Mat &srcImg) {
        if (run)
            status = dvpGetFrame(h, &frame, &pBuffer, 3000);
        if (status != DVP_STATUS_OK) {
            cout << "Failed to get a frame in continuous mode" << endl;
        }
        srcImg = cv::Mat(frame.iHeight, frame.iWidth, CV_8UC3, (unsigned char *) pBuffer);
        if (NULL == srcImg.data) {
            return false;
        }
        return true;
    };

    bool run = false;

private:
    dvpStatus status;
    dvpHandle h;
    void *pBuffer;
    dvpFrame frame;
};
int main(int argc, char *argv[]) {
    cout << "start servo" << endl;
    Tc_Ads ads_ptr;
    auto d = make_shared<MotionV1>(ads_ptr);
    Multi_Process pp;
    auto pi = pp.safety_monitor_build("SAFE-CHECK.exe");
    d->Enable();
    d->setSyncrpm(10);

    cv::Mat showImage;
    dvpUint32 count = 0, num = -1;
    dvpCameraInfo info[8];
    /* 枚举设备 */
    dvpRefresh(&count);
    if (count > 8)
        count = 8;

    for (int i = 0; i < (int) count; i++) {
        if (dvpEnum(i, &info[i]) == DVP_STATUS_OK) {
            printf("[%d]-Camera FriendlyName : %s\r\n", i, info[i].FriendlyName);
        }
    }
    num = 0;
    auto c = myCamera(&num);
    c.start();
    c.getMat(showImage);
    if (argc >= 2) {
        int movingflag = atoi(argv[1]);
        auto movef = [&]() {
            this_thread::sleep_for(chrono::milliseconds(3000));
            vector<float> c_vecs{};
            string name;
            for (int i{}; i < MOVE_DISTANCE; i++) {
                cout << "count: " << i << endl;
                switch (movingflag) {
                    case 0:
                        c_vecs = {1, 0, 0, 0, 0, 0};
                        d->opSpaceMotionByJacobe(c_vecs);
                        name = "../img/0/";
                        break;

                    case 1:
                        c_vecs = {0, 1, 0, 0, 0, 0};
                        d->opSpaceMotionByJacobe(c_vecs);
                        name = "../img/1/";
                        break;

                    case 2:
                        c_vecs = {-1, 0, 0, 0, 0, 0};
                        d->opSpaceMotionByJacobe(c_vecs);
                        name = "../img/2/";
                        break;
                    case 3:
                        c_vecs = {0, -1, 0, 0, 0, 0};
                        d->opSpaceMotionByJacobe(c_vecs);
                        name = "../img/3/";
                        break;
                }
#ifdef MOVE_LOG
                if (argc == 3) {
                    name += "reverse/";
                }
                cv::imwrite(name + to_string(i) + ".jpg", showImage);
                //                this_thread::sleep_for(chrono::milliseconds(5));
#endif
            }
            c.stop();
        };
        auto getFrame = [&]() {
            for (;;) {
                if (c.run == false)
                    break;
                c.getMat(showImage);
            }
        };
        thread task(movef);
        thread task2(getFrame);
        task2.detach();
        task.join();
    }
    d->Disable();
    return 0;
}