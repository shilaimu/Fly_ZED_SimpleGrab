#include "zedcam.h"

ZedCamera::ZedCamera()
{

}

ConfigPara ZedCamera::readConfigFile(){
    ConfigPara paraRead;
    cv::FileStorage fs("config.yaml", cv::FileStorage::READ);

    paraRead.ifColor = fs["Func.ifColor"];
    paraRead.ifSaveImgs = fs["Func.ifSaveImgs"];
    paraRead.ifShowImgs = fs["Func.ifShowImgs"];

    paraRead.flyPara.fps = fs["FlyCamera.fps"];
    paraRead.flyPara.exposure = fs["FlyCamera.exposure"];
    paraRead.flyPara.gain = fs["FlyCamera.gain"];
    paraRead.flyPara.brightness = fs["FlyCamera.brightness"];
    paraRead.flyPara.contrast = fs["FlyCamera.contrast"];
    paraRead.flyPara.hue = fs["FlyCamera.hue"];
    paraRead.flyPara.saturation = fs["FlyCamera.saturation"];
    paraRead.flyPara.width = fs["FlyCamera.width"];
    paraRead.flyPara.height = fs["FlyCamera.height"];

    paraRead.zedPara.fps = fs["ZedCamera.fps"];
    paraRead.zedPara.exposure = fs["ZedCamera.exposure"];
    paraRead.zedPara.gain = fs["ZedCamera.gain"];
    paraRead.zedPara.brightness = fs["ZedCamera.brightness"];
    paraRead.zedPara.contrast = fs["ZedCamera.contrast"];
    paraRead.zedPara.hue = fs["ZedCamera.hue"];
    paraRead.zedPara.saturation = fs["ZedCamera.saturation"];
    int _zedResolution = fs["ZedCamera.resolution"];    
    
    switch(_zedResolution)
    {
        case 0: 
            paraRead.zedPara.resolution.width =672;
        paraRead.zedPara.resolution.height =376;
            break;
        case 1: 
            paraRead.zedPara.resolution.width =1080;
            paraRead.zedPara.resolution.height =720;
            break;
        case 2: 
            paraRead.zedPara.resolution.width =1920;
            paraRead.zedPara.resolution.height =1080;
            break;
        case 3: 
            paraRead.zedPara.resolution.width =2208;
            paraRead.zedPara.resolution.height =1242;
            break;
        default:
            std::cout << "YAML file ERROR!" << std::endl;
            exit(-1);
    }
    fs.release();
    std::cout << "ifColor:" << paraRead.ifColor << std::endl;    
    std::cout << "ifSaveImgs:" << paraRead.ifSaveImgs << std::endl;    
    std::cout << "zedfps:" << paraRead.zedPara.fps << "      " << "flyfps:" << paraRead.flyPara.fps << std::endl;    
    std::cout << "zedexposure:" << paraRead.zedPara.exposure << "      " << "flyexposure:" << paraRead.flyPara.exposure << std::endl;    
    std::cout << "zedgain:" << paraRead.zedPara.gain << "      " << "flygain:" << paraRead.flyPara.gain << std::endl;    
    std::cout << "zedbrightness:" << paraRead.zedPara.brightness << "      " << "flybrightness:" << paraRead.flyPara.brightness << std::endl;    
    std::cout << "zedcontrast:" << paraRead.zedPara.contrast << "      " << "flycontrast:" << paraRead.flyPara.contrast << std::endl;    
    std::cout << "zedhue:" << paraRead.zedPara.hue << "      " << "flyhue:" << paraRead.flyPara.hue << std::endl;    
    std::cout << "zedsaturation:" << paraRead.zedPara.saturation << "      " << "flysaturation:" << paraRead.flyPara.saturation << std::endl;    
    std::cout << "zedresolution:" 
<< paraRead.zedPara.resolution.width <<"*"
<< paraRead.zedPara.resolution.height << "flyresolution:" 
<< paraRead.flyPara.width << "*" 
<< paraRead.flyPara.height << std::endl;   
    std::cout << "-- Config file read finish." << std::endl;   
    return paraRead;
}

bool ZedCamera::initZedCamera(sl::Camera* m_zed){
    // Define a struct of parameters for the initialization
    sl::InitParameters params;
    ERROR_CODE err = m_zed->open(params);
    std::cerr << "Initialization status : " << sl::errorCode2str(err) << std::endl;
    if (err != sl::SUCCESS) {
        // Exit if an error occurred
        m_zed->close();
        return 0;
    }
    else return 1;
}

void ZedCamera::setZedCameraParameters( sl::Camera* m_zed, ConfigPara para ){

    // The depth is limited to 20 METERS, as defined in zed::init()
    m_zed->setDepthMaxRangeValue(10000);
    m_zed->setCameraFPS( para.zedPara.fps );
    m_zed->setCameraSettings( sl::CAMERA_SETTINGS_EXPOSURE, para.zedPara.exposure );
    m_zed->setCameraSettings( sl::CAMERA_SETTINGS_BRIGHTNESS, para.zedPara.brightness );
    m_zed->setCameraSettings( sl::CAMERA_SETTINGS_CONTRAST, para.zedPara.contrast );
    m_zed->setCameraSettings( sl::CAMERA_SETTINGS_HUE, para.zedPara.hue );
    m_zed->setCameraSettings( sl::CAMERA_SETTINGS_SATURATION, para.zedPara.saturation );
    m_zed->setCameraSettings( sl::CAMERA_SETTINGS_GAIN, para.zedPara.gain );   
    std::cout << "-- Parameters setting finished" << std::endl;
}

void ZedCamera::grabZedImg(sl::Camera* m_zed, int height, int width, bool ifcolor){
    std::cout << "------zed Grabbing Thread start...------" << std::endl;
    extern  pkQueueTS<double> queueOfTimeStamp;
    extern std::mutex data_mutex;
    extern std::condition_variable data_var;
    extern bool flag;//thread tongbu

    sl::Mat zed_image;
    cv::Mat imgLeft(height, width, CV_8UC4);
    cv::Mat imgRight(height, width, CV_8UC4);
    //cv::Mat imgLeftGrey(height, width, CV_8UC1); 
    //cv::Mat imgRightGrey(height, width, CV_8UC1);

    MatCapsule elemLeft;
    MatCapsule elemRight;  

    auto t_firstGrabbing = std::chrono::system_clock::now();
    auto t_start = std::chrono::system_clock::now();
    auto t_now = std::chrono::system_clock::now();

    zedgrabOn = true;                   /* Lock free var to control the grab thread */
    long grabCnt = 0;
    std::cout << "Press 'q' to exit" << std::endl;
    // Loop until 'q' is pressed
    
    while (zedgrabOn) 
    {
        std::unique_lock<std::mutex> lck(data_mutex) ;
        data_var.wait(lck,[]{return !flag;});
        flag = true;
        data_var.notify_one();

        bool res = m_zed->grab();
        if (!res) {
            m_zed->retrieveImage(zed_image,VIEW_LEFT);
            imgLeft = cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU));
            m_zed->retrieveImage(zed_image,VIEW_RIGHT);
            imgRight = cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU));

 
            elemLeft.mat = imgLeft;
            elemRight.mat = imgRight;
            queueOfMatsZedLeft.Push(elemLeft);
            queueOfMatsZedRight.Push(elemRight);

            if(grabCnt==0) //initial value
            {
                t_firstGrabbing = std::chrono::system_clock::now();
                queueOfTimeStamp.Push(0.0);
            }           
            else
            {
                t_now   = std::chrono::system_clock::now();
                auto durationFromStart = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_firstGrabbing);
                auto durationDeltaT = std::chrono::duration_cast<std::chrono::microseconds>(t_now - t_start);
                double grabTimeStamp = double(durationFromStart.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
                double deltaT = double(durationDeltaT.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;              
                std::cout << "FPS = " << 1/deltaT << "\tgrabTimeStamp = " << grabTimeStamp << "\tthread: "<< std::this_thread::get_id() << "   printf: " << "B" <<std::endl;
                queueOfTimeStamp.Push(grabTimeStamp);
            }
            t_start = std::chrono::system_clock::now();          
            grabCnt++; 

        } else usleep(5000);
    }

    std::cout << "------zed Grabbing Thread Closing...------" << std::endl;
}

void ZedCamera::saveZedImagePairs( string addr, cv::Mat left, cv::Mat right, long num ){
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << num;
    std::string addrImgLeft  = addr + "/image_0/" + ss.str() + ".png";
    std::string addrImgRight = addr + "/image_1/" + ss.str() + ".png";
    cv::imwrite(addrImgLeft,left);
    cv::imwrite(addrImgRight,right);
}