#ifndef FLYCAMERA_H
#define FLYCAMERA_H

#include "FlyCapture2.h"
#include <pkQueueTS.hpp>
#include "zedcam.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

using namespace std;
using namespace FlyCapture2;

class FlyCamera
{
public:
    FlyCamera();
    atomic<bool> flygrabOn;
    pkQueueTS<MatCapsule> queueOfMatsFlyLeft;
    pkQueueTS<MatCapsule> queueOfMatsFlyRight;

    FlyCapture2::Error m_Error;
    FlyCapture2::TriggerMode m_triggerMode;

	//定义相机对象
    FlyCapture2::Camera m_Cam;

public:
    void grabFlyImg(FlyCapture2::Camera* m_CamL, FlyCapture2::Camera* m_CamR);
    void saveFlyImagePairs( string addr, cv::Mat left, cv::Mat right, long num );


	//连接相机函数，输入0连接第一个相机，输入1连接第二个相机
    bool ConnectToCamera(FlyCapture2::Camera* m_Cam, unsigned int camFlag);
    //set camera para
    bool setFlyCameraParameters(FlyCapture2::Camera* m_Cam);

	//断开相机的连接
    bool DisconnectCamera(FlyCapture2::Camera* m_Cam);
	//显示错误
    void PrintError( FlyCapture2::Error error );
	//输出相机信息
    void PrintCameraInfo( CameraInfo* myCamInfo );
	//发射拍摄信号
    bool CheckSoftwareTriggerPresence(FlyCapture2::Camera* m_Cam);
    bool PollForTriggerReady( FlyCapture2::Camera* m_Cam );
    bool FireSoftwareTrigger( FlyCapture2::Camera* m_Cam );
	//拍摄图片
    void GrabAPicture(FlyCapture2::Camera* m_Cam, Image& myImage);
	//设置曝光时间
    bool SetExposureTime(float ms);
};

#endif // MyCAMERA_H
