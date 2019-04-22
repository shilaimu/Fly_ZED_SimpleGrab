#ifndef ZEDCAMERA_H
#define ZEDCAMERA_H
#include <pkQueueTS.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sl/Camera.hpp>

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
using namespace cv;
using namespace sl;

typedef struct _CameraPara{
    int fps;
    float exposure;
    float gain;
    float brightness;
    float contrast;
    float hue;
    float saturation;
    //sl::RESOLUTION resolution;
    int width;
    int height;
    sl::Resolution resolution;
}CameraPara;

typedef struct _configPara{
    int ifColor;
    int ifSaveImgs;
    int ifShowImgs; 
    CameraPara zedPara;
    CameraPara flyPara;
}ConfigPara;

class ZedCamera
{
public:
    ZedCamera();
    sl::Camera m_zed;
    atomic<bool> zedgrabOn;
    pkQueueTS<MatCapsule> queueOfMatsZedLeft;
    pkQueueTS<MatCapsule> queueOfMatsZedRight;
    ConfigPara readConfigFile();

    bool initZedCamera(sl::Camera* m_zed);
    void setZedCameraParameters(sl::Camera* m_zed, ConfigPara para );
    void grabZedImg(sl::Camera* m_zed, int height, int width, bool ifcolor);
    void saveZedImagePairs( string addr, cv::Mat left, cv::Mat right, long num );

};

#endif // ZEDCAMERA_H
