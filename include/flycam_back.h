#ifndef MyCAMERA_H
#define MyCAMERA_H

#include<iostream>
#include "fstream"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "FlyCapture2.h"

using namespace std;
using namespace FlyCapture2;

class MyCamera
{
public:
    MyCamera();
    Error m_Error;
	//定义相机对象
    Camera m_Cam;
    TriggerMode m_TriggerMode;

public:
	//连接相机函数，输入0连接第一个相机，输入1连接第二个相机
    bool ConnectToCamera(unsigned int camFlag);
	//断开相机的连接
    bool DisconnectCamera();
	//显示错误
    void PrintError( Error error );
	//输出相机信息
    void PrintCameraInfo( CameraInfo* myCamInfo );
	//发射拍摄信号
    bool PollForTriggerReady( Camera* myCam );
    bool FireSoftwareTrigger( Camera* myCam );
	//拍摄图片
    void GrabAPicture(Image& myImage);
	//设置曝光时间
    bool SetExposureTime(float ms);
};

#endif // MyCAMERA_H
