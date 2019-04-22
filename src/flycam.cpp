#include "flycam.h"
#define SOFTWARE_TRIGGER_CAMERA

FlyCamera::FlyCamera()
{

}


void FlyCamera::grabFlyImg(FlyCapture2::Camera* m_CamL, FlyCapture2::Camera* m_CamR)
{
    std::cout << "------Flycapture Grabbing Thread start...------" << std::endl;
    extern  pkQueueTS<double> queueOfTimeStamp;
    extern std::mutex data_mutex;
    extern std::condition_variable data_var;
    extern bool flag;//thread tongbu

    MatCapsule elemLeft;
    MatCapsule elemRight;  //存储图像队列
    Image FlyimageL, FlyimageR;     

    auto t_firstGrabbing = std::chrono::system_clock::now();
    auto t_start = std::chrono::system_clock::now();
    auto t_now = std::chrono::system_clock::now();    

    int flygrabCnt = 0;
    flygrabOn = true;
    std::cout << "Press 'q' to exit" << std::endl;    // Loop until 'q' is pressed

    while(flygrabOn)
    {
        //std::this_thread::sleep_for(std::chrono::seconds(1));
        std::unique_lock<std::mutex> lck(data_mutex) ;
        data_var.wait(lck,[]{return flag;});
        flag = false;
        data_var.notify_one();

        //cout<<"here"<<endl;
        GrabAPicture(m_CamL, FlyimageL);
        GrabAPicture(m_CamR, FlyimageR);
        
      /*  if(flygrabCnt < 20)
        {
            flygrabCnt++;
            continue;
        }*/
        //Convert to OpenCV Mat
        unsigned int rowLBytes = (double)FlyimageL.GetReceivedDataSize()/(double)FlyimageL.GetRows();
        unsigned int rowRBytes = (double)FlyimageR.GetReceivedDataSize()/(double)FlyimageR.GetRows();
        cv::Mat imgLeft = cv::Mat(FlyimageL.GetRows(), FlyimageL.GetCols(), CV_8UC3, FlyimageL.GetData(), rowLBytes);
        cv::Mat imgRight = cv::Mat(FlyimageR.GetRows(), FlyimageR.GetCols(), CV_8UC3, FlyimageR.GetData(), rowRBytes);
        elemLeft.mat = imgLeft;
        elemRight.mat = imgRight;
        queueOfMatsFlyLeft.Push(elemLeft);
        queueOfMatsFlyRight.Push(elemRight);    //图片放到队列中

        if(flygrabCnt==0) //initial value
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
            std::cout << "FPS = " << 1/deltaT << "\tgrabTimeStamp = " << grabTimeStamp << "\tthread: "<< std::this_thread::get_id() << "   printf: " << "A" <<std::endl;
            queueOfTimeStamp.Push(grabTimeStamp);
        }
        t_start = std::chrono::system_clock::now();          
        flygrabCnt++; 
            
    }

    std::cout << "------Fly Grabbing Thread Closing...------" << std::endl;
}

void FlyCamera::saveFlyImagePairs( string addr, cv::Mat left, cv::Mat right, long num ){
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << num;
    std::string addrImgLeft  = addr + "/image_2/" + ss.str() + ".png";
    std::string addrImgRight = addr + "/image_3/" + ss.str() + ".png";
    cv::imwrite(addrImgLeft,left);
    cv::imwrite(addrImgRight,right);
}


bool FlyCamera::ConnectToCamera(FlyCapture2::Camera* m_Cam, unsigned int camFlag)
{
	BusManager busMgr;
	unsigned int numCameras;
	m_Error = busMgr.GetNumOfCameras(&numCameras);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}
    cout << "Number of cameras detected: " << numCameras << endl;
	if (numCameras < 1)
	{
		printf("Insufficient number of cameras... exiting\n");
		return -1;
	}

	PGRGuid guid;
	m_Error = busMgr.GetCameraFromIndex(camFlag, &guid);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}
    // Connect to a camera
	m_Error = m_Cam->Connect(&guid);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}
    setFlyCameraParameters( m_Cam );

    // Power on the camera
    const unsigned int k_cameraPower = 0x610;
    const unsigned int k_powerVal = 0x80000000;
    m_Error = m_Cam->WriteRegister(k_cameraPower, k_powerVal);
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }

    const unsigned int millisecondsToSleep = 100;
    unsigned int regVal = 0;
    unsigned int retries = 10;

    // Wait for camera to complete power-up
    do
    {
        struct timespec nsDelay;
        nsDelay.tv_sec = 0;
        nsDelay.tv_nsec = (long)millisecondsToSleep * 1000000L;
        nanosleep(&nsDelay, NULL);

        m_Error = m_Cam->ReadRegister(k_cameraPower, &regVal);
        if (m_Error == PGRERROR_TIMEOUT)
        {
            // ignore timeout errors, camera may not be responding to
            // register reads during power-up
        }
        else if (m_Error != PGRERROR_OK)
        {
            PrintError(m_Error);
            return -1;
        }

        retries--;
    } while ((regVal & k_powerVal) == 0 && retries > 0);

    // Check for timeout errors after retrying
    if (m_Error == PGRERROR_TIMEOUT)
    {
        PrintError(m_Error);
        return -1;
    }

	// Get the camera information
	CameraInfo camInfo;
	m_Error = m_Cam->GetCameraInfo(&camInfo);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}
	PrintCameraInfo(&camInfo);

#ifndef SOFTWARE_TRIGGER_CAMERA
    // Check for external trigger support
  	TriggerModeInfo triggerModeInfo;
 	m_Error = m_Cam->GetTriggerModeInfo(&triggerModeInfo);
  	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}
	if (triggerModeInfo.present != true)
	{
		printf("Camera does not support external trigger! Exiting...\n");
		return -1;
	}
#endif

	// Get current trigger settings
	m_Error = m_Cam->GetTriggerMode(&m_triggerMode);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}

	// Set camera to trigger mode 0
	m_triggerMode.onOff = true;
	m_triggerMode.mode = 0;
	m_triggerMode.parameter = 0;

#ifdef SOFTWARE_TRIGGER_CAMERA
	// A source of 7 means software trigger
	m_triggerMode.source = 7;
#else
	// Triggering the camera externally using source 0.
    m_triggerMode.source = 0;
#endif

    m_Error = m_Cam->SetTriggerMode(&m_triggerMode);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}

    // Poll to ensure camera is ready
    bool retVal = PollForTriggerReady( m_Cam );
    if( !retVal )
    {
        cout << endl;
        cout << "Error polling for trigger ready!" << endl;
        return -1;
    }


	// Get the camera configuration
	FC2Config config;
	m_Error = m_Cam->GetConfiguration(&config);
	if (m_Error != PGRERROR_OK)
	{
		PrintError(m_Error);
		return -1;
	}
    
    // Set the grab timeout to 5 seconds
    config.grabTimeout = 5000;
    // Set the camera configuration
    m_Error = m_Cam->SetConfiguration(&config);
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }

	// Camera is ready, start capturing images
    m_Error = m_Cam->StartCapture();
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }

#ifdef SOFTWARE_TRIGGER_CAMERA
    if (!CheckSoftwareTriggerPresence( m_Cam ))
    {
        cout << "SOFT_ASYNC_TRIGGER not implemented on this camera! Stopping "
                "application"
             << endl;
        return -1;
    }
#else
    cout << "Trigger the camera by sending a trigger pulse to GPIO"
         << m_triggerMode.source << endl;
#endif

    return true;

}

bool FlyCamera::setFlyCameraParameters(FlyCapture2::Camera* m_Cam){

    ////////////////初始化相机参数/////////////////////////
    //m_Cam->SetVideoModeandFrameRate( VIDEOMODE_640x480Y8 , FRAMERATE_60 );
    Property pFR;
    pFR.type = FRAME_RATE;
    m_Cam->GetProperty(&pFR);
    cout<< "FPS of Fly " << pFR.absValue<<endl;
    //camera parameters

   /* Format7ImageSettings f7setting;
    //size setting
    unsigned int packaage_size;
    float persentage;
    m_Cam->GetFormat7Configuration(&f7setting, &packaage_size, &persentage);

    f7setting.mode = Mode(MODE_1);
    f7setting.offsetX = 1000;
    f7setting.offsetY = 1000;
    f7setting.width = 100;
    f7setting.height = 100;

    f7setting.pixelFormat = PIXEL_FORMAT_RAW8;


    packaage_size = 4092;
    m_Cam->SetFormat7Configuration(&f7setting, packaage_size);

    Property exposure;
    exposure.type = AUTO_EXPOSURE;
    exposure.absControl = true;
    exposure.onePush = false;
    exposure.onOff = true;
    exposure.autoManualMode = false;
    exposure.absValue = 1.2;
    m_Error = m_Cam->SetProperty(&exposure);
    //shutter setting

    Property pshutter;
    pshutter.type = GAMMA;
    pshutter.absControl = true;
    pshutter.onePush = false;
    pshutter.onOff = true;
    pshutter.autoManualMode = false;
    pshutter.absValue = 10;
    m_Error = m_Cam->SetProperty(&pshutter);

    Property pgain;
    pgain.type = GAIN;
    pgain.absControl = true;
    pgain.onePush = false;
    pgain.onOff = true;
    pgain.autoManualMode = false;
    pgain.absValue = 0;
    m_Error = m_Cam->SetProperty(&pgain);
    //gain setting

    bool returnstatus;
    Format7PacketInfo info;
    m_Error = m_Cam->ValidateFormat7Settings(&f7setting, &returnstatus, &info);
    if (m_Error != FlyCapture2::PGRERROR_OK)
    {
        //cout<<"ValidateFormat7Settings"<<endl;
        printf("ValidateFormat7Settings:\n");
        m_Error.PrintErrorTrace();
    }

    Property pFR;
    pFR.type = FRAME_RATE;
    pFR.absControl = true;
    pFR.onePush = false;
    pFR.onOff = true;
    pFR.autoManualMode = false;
    pFR.absValue = 30;
    m_Error = m_Cam->SetProperty(&pFR);
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }
    //frame rate setting

    Property pWB;
    pWB.type = WHITE_BALANCE;
    pWB.absControl = true;
    pWB.onePush = false;
    pWB.onOff = true;
    pWB.autoManualMode = true;
    pWB.valueA = 578;
    pWB.valueB = 803;
    m_Error = m_Cam->SetProperty(&pWB);
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }
    //m_Cam->GetProperty(&pWB);

    //关闭相机自动曝光
    Property prop; //Declare a Property struct.
    prop.type = SHUTTER; //Define the property to adjust.
    prop.onOff = true; //Ensure the property is on.
    prop.autoManualMode = false; //Ensure auto-adjust mode is off.
    prop.absControl = true; //Ensure the property is set up to use absolute value control.
    prop.absValue = 3; //Set the absolute value.
    m_Error = m_Cam->SetProperty(&prop); //Set the property.
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }
    //关掉自动增益模式
    prop.type = GAIN; //Define the property to adjust.
    prop.onOff = true; //Ensure the property is on.
    prop.autoManualMode = false; //Ensure auto-adjust mode is off.
    m_Error = m_Cam->SetProperty(&prop); //Set the property.
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }
    prop.type = BRIGHTNESS; //Define the property to adjust.
    prop.onOff = true; //Ensure the property is on.
    prop.autoManualMode = false; //Ensure auto-adjust mode is off.
    m_Error = m_Cam->SetProperty(&prop); //Set the property.
    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return -1;
    }*/
        
}


	


bool FlyCamera::DisconnectCamera(FlyCapture2::Camera* m_Cam)
{
    printf( "\nStopping grabbing images...\n" );

    // Turn trigger mode off.
    m_triggerMode.onOff = false;
    m_Error = m_Cam->SetTriggerMode( &m_triggerMode );
    if (m_Error != PGRERROR_OK)
    {
        PrintError( m_Error );
        return -1;
    }

    // Stop capturing images
    m_Error = m_Cam->StopCapture();
    if (m_Error != PGRERROR_OK)
    {
        PrintError( m_Error );
        return -1;
    }

    // Disconnect the camera
    m_Error = m_Cam->Disconnect();
    if (m_Error != PGRERROR_OK)
    {
        PrintError( m_Error );
        return -1;
    }

    return 1;
}


void FlyCamera::PrintError( FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}

void FlyCamera::PrintCameraInfo( CameraInfo* myCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n",
        myCamInfo->serialNumber);
}

bool FlyCamera::CheckSoftwareTriggerPresence(FlyCapture2::Camera* m_Cam)
{
    const unsigned int k_triggerInq = 0x530;

    FlyCapture2::Error m_error;
    unsigned int regVal = 0;

    m_Error = m_Cam->ReadRegister(k_triggerInq, &regVal);

    if (m_Error != PGRERROR_OK)
    {
        PrintError(m_Error);
        return false;
    }

    if ((regVal & 0x10000) != 0x10000)
    {
        return false;
    }

    return true;
}

bool FlyCamera::FireSoftwareTrigger( FlyCapture2::Camera* m_Cam )
{
    const unsigned int k_softwareTrigger = 0x62C;
    const unsigned int k_fireVal = 0x80000000;
    FlyCapture2::Error m_Error;

    m_Error = m_Cam->WriteRegister( k_softwareTrigger, k_fireVal );
    if (m_Error != PGRERROR_OK)
    {
        PrintError( m_Error );
        return false;
    }

    return true;
}


bool FlyCamera::PollForTriggerReady( FlyCapture2::Camera* m_Cam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    FlyCapture2::Error m_Error;
    unsigned int regVal = 0;

    do
    {
        m_Error = m_Cam->ReadRegister( k_softwareTrigger, &regVal );
        if (m_Error != PGRERROR_OK)
        {
            PrintError( m_Error);
            return false;
        }

    } while ( (regVal >> 31) != 0 );

    return true;
}

void FlyCamera::GrabAPicture(FlyCapture2::Camera* m_Cam, Image& myImage)
{
#ifdef SOFTWARE_TRIGGER_CAMERA
    // Check that the trigger is ready
    PollForTriggerReady( m_Cam);
    cout << "Waiting for a software trigger" << endl;

    // Fire software trigger
    bool retVal = FireSoftwareTrigger( m_Cam );
    if ( !retVal )
    {
        printf("\nError firing software trigger!\n");
        return ;
    }
#endif

    // Retrieve an image
    Image rawImage;
    m_Error = m_Cam->RetrieveBuffer( &rawImage );
    if (m_Error != PGRERROR_OK)
    {
        PrintError( m_Error );
        return ;
    }
    rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &myImage);
    return ;
}
