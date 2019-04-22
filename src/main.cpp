#include <pkQueueTS.hpp>
#include "flycam.h"
#include "zedcam.h"

#include <unistd.h>
#include <sys/time.h>
#include <stdlib.h>
#include <chrono>    //C++11
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <iomanip>
#include <atomic>

bool paraChange = false;
pkQueueTS<double> queueOfTimeStamp;

std::mutex data_mutex;
std::condition_variable data_var;
bool flag = true;//thread tongbu

cv::Size displaySize(640, 360);

sl::Camera zed;
FlyCapture2::Camera  FlyCamL, FlyCamR;//相机对象
int camFlagL = 0;
int camFlagR = 1;	//相机指针


int main(int argc,char** argv)
{
    ZedCamera zedcam;
    FlyCamera flycam;
    std::cout << "------Main Thread start...------" << std::endl;
    //ConfigPara paraRead;  
    ConfigPara paraRead = zedcam.readConfigFile();
    //zed.open
    if(!zedcam.initZedCamera(&zed)){
        std::cout << "-- ZED camera init failed, exiting..." << std::endl;
        exit(0);
    }
    zedcam.setZedCameraParameters(&zed, paraRead);
    int height = zed.getResolution().height;
    int width = zed.getResolution().width;

    bool CamL = flycam.ConnectToCamera(&FlyCamL, camFlagL);
    bool CamR = flycam.ConnectToCamera(&FlyCamR, camFlagR);  //相机初始化		//TODO:加入传参
    
    std::thread flygrabbingThread(&FlyCamera::grabFlyImg, &flycam, &FlyCamL, &FlyCamR);
    std::thread zedgrabbingThread(&ZedCamera::grabZedImg, &zedcam, &zed, height, width, 1);

    MatCapsule elemFlyLeft;
    MatCapsule elemFlyRight; 
    MatCapsule elemZedLeft;
    MatCapsule elemZedRight;

    double timeStamp;
    long flyimgCnt = 0;    
    long zedimgCnt = 0;
    std::string addr=".";
    string timeStampDir = addr + "/times.txt";
    ofstream fileTime(timeStampDir, ofstream::out);
    fileTime << setiosflags(ios::fixed);
    
    while(1)
    {       
        pkQueueResults res_flyleft = flycam.queueOfMatsFlyLeft.Pop(elemFlyLeft, 2000);
        pkQueueResults res_flyright = flycam.queueOfMatsFlyRight.Pop(elemFlyRight, 2000);
        pkQueueResults res_zedleft = zedcam.queueOfMatsZedLeft.Pop(elemZedLeft, 2000);
        pkQueueResults res_zedright = zedcam.queueOfMatsZedRight.Pop(elemZedRight, 2000);

        pkQueueResults res_time = queueOfTimeStamp.Pop(timeStamp, 2000);
        cout << flycam.queueOfMatsFlyLeft.Size() << " imgs in (Fly)left queue, " 
             << flycam.queueOfMatsFlyRight.Size() << " imgs in (Fly)right queue, " 
             << zedcam.queueOfMatsZedLeft.Size() << " imgs in (Zed)left queue, " 
             << zedcam.queueOfMatsZedRight.Size() << " imgs in (Zed)right queue, " 
             << queueOfTimeStamp.Size() << " timeStamps." << endl;

        if (res_flyleft != PK_QTS_OK){
            if (res_flyleft == PK_QTS_TIMEOUT)
                cout << "WARNING: time out reading from the queue!" << endl;
            if (res_flyleft == PK_QTS_EMPTY)
                cout << "INFO: the (Flyleft)queue is empty!" << endl;
            // pass the control to next threads
            this_thread::yield();
            continue;
        }
        if (res_flyright != PK_QTS_OK){
            if (res_flyright == PK_QTS_TIMEOUT)
                cout << "WARNING: time out reading from the queue!" << endl;
            if (res_flyright == PK_QTS_EMPTY)
                cout << "INFO: the (Flyright)queue is empty!" << endl;
            // pass the control to next threads
            this_thread::yield();
            continue;
        }
        if (res_zedleft != PK_QTS_OK){
            if (res_zedleft == PK_QTS_TIMEOUT)
                cout << "WARNING: time out reading from the queue!" << endl;
            if (res_zedleft == PK_QTS_EMPTY)
                cout << "INFO: the queue is empty!" << endl;
            // pass the control to next threads
            this_thread::yield();
            continue;
        }
        if (res_zedright != PK_QTS_OK){
            if (res_zedright == PK_QTS_TIMEOUT)
                cout << "WARNING: time out reading from the queue!" << endl;
            if (res_zedright == PK_QTS_EMPTY)
                cout << "INFO: the queue is empty!" << endl;
            // pass the control to next threads
            this_thread::yield();
            continue;
        }

        // process data
        if(paraRead.ifSaveImgs)
        {
            zedcam.saveZedImagePairs(addr, elemZedLeft.mat, elemZedLeft.mat, zedimgCnt);
            flycam.saveFlyImagePairs(addr, elemFlyLeft.mat, elemFlyRight.mat, flyimgCnt);
            fileTime << setprecision(6) << timeStamp << endl;
        }
        flyimgCnt++;
        zedimgCnt++;


        cv::Mat flyleftImgShow, flyrightImgShow, zedleftImgShow;
        cv::resize(elemFlyLeft.mat, flyleftImgShow, displaySize);
        cv::resize(elemFlyRight.mat, flyrightImgShow, displaySize);
        cv::resize(elemZedLeft.mat, zedleftImgShow, displaySize);
        cv::imshow("Image from Fly left queue", flyleftImgShow);
        cv::imshow("Image from Fly right queue", flyrightImgShow);
        cv::imshow("Image from Zed left queue", zedleftImgShow);

        
        char key = cv::waitKey(5);
        if( key == 'q' ){
            flycam.flygrabOn = false;
            zedcam.zedgrabOn = false;
        }
        if( flycam.flygrabOn == false && zedcam.zedgrabOn == false && flycam.queueOfMatsFlyLeft.Size()==0 && flycam.queueOfMatsFlyRight.Size()==0 && zedcam.queueOfMatsZedLeft.Size()==0 && zedcam.queueOfMatsZedRight.Size()==0)
        //if( flycam.flygrabOn == false  && flycam.queueOfMatsFlyLeft.Size()==0 && flycam.queueOfMatsFlyRight.Size()==0) 
            break;
    }
 
    flygrabbingThread.join();
    zedgrabbingThread.join();
    std::cout << "------Main Thread Closing...------" << std::endl;
    
    bool CamoutL = flycam.DisconnectCamera(&FlyCamL);
    bool CamoutR = flycam.DisconnectCamera(&FlyCamR);
    zed.close();
    return 0;
}

