#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include <boost/make_shared.hpp> // Required for cv_bridge shared pointers

namespace camera
{
    std::vector<cv::Mat> frames;
    std::vector<bool> frame_emptys;
    std::vector<pthread_mutex_t> mutexs;
    bool SysteamTime;

    // Camera availability flags
    bool left_camera_available;
    bool right_camera_available;

    image_transport::CameraPublisher imageL_pub;
    image_transport::CameraPublisher imageR_pub;
    sensor_msgs::Image imageL_msg;
    sensor_msgs::Image imageR_msg;
    sensor_msgs::CameraInfo cameraL_info_msg;
    sensor_msgs::CameraInfo cameraR_info_msg;

    cv_bridge::CvImagePtr cv_ptr_l;
    cv_bridge::CvImagePtr cv_ptr_r;

    ros::Time ConvertToROSTime(uint32_t nDevTimeStampHigh, uint32_t nDevTimeStampLow);

    struct ThreadData {
        int ndevice;
        void* handle;
        MVCC_INTVALUE stParam;
    };

    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        void RunCamera(int ndevice, void* &handle); // 初始化和启动相机
        static void* WorkThread(void* p_handle); // 工作线程函数

    private:
        std::vector<void*> handles; // 创建多个相机句柄
        MV_CC_DEVICE_INFO_LIST stDeviceList;  // 枚举设备
        std::vector<pthread_t> threads; // 存储线程ID

        int nRet;
        int TriggerMode;
        // 触发模式：MV_TRIGGER_SOURCE_LINE0 = 0,MV_TRIGGER_SOURCE_LINE1 = 1,MV_TRIGGER_SOURCE_LINE2 = 2,
        // MV_TRIGGER_SOURCE_LINE3 = 3,MV_TRIGGER_SOURCE_COUNTER0 = 4,MV_TRIGGER_SOURCE_SOFTWARE = 7,
        // MV_TRIGGER_SOURCE_FrequencyConverter = 8
        int TriggerSource;

        // CameraInfoManager members (removed - now using ROS params directly)
        // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_l_;
        // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_r_;
        
        // Helper function to load camera info from ROS parameters
        void loadCameraInfoFromParams(ros::NodeHandle &node, const std::string &camera_name, sensor_msgs::CameraInfo &camera_info);
        
        // Helper function to check if camera parameters exist
        bool checkCameraParamsExist(ros::NodeHandle &node, const std::string &camera_name);
    };

    Camera::Camera(ros::NodeHandle &node)
    {
        //读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效
        node.param("TriggerMode", TriggerMode, 0);    //0为不启用触发，1为启用
        node.param("TriggerSource", TriggerSource, 0);  //设置触发模式
        node.param("SysteamTime", SysteamTime, false);

        // Load camera info from ROS parameters
        ROS_INFO("Loading camera info from ROS parameters...");
        
        // Check which cameras are available
        left_camera_available = checkCameraParamsExist(node, "camera_left");
        right_camera_available = checkCameraParamsExist(node, "camera_right");
        
        if (!left_camera_available && !right_camera_available)
        {
            ROS_ERROR("No camera parameters found in ROS parameter server!");
            exit(-1);
        }
        
        // Load camera info only for available cameras
        if (left_camera_available)
        {
            loadCameraInfoFromParams(node, "camera_left", cameraL_info_msg);
            ROS_INFO("Left camera parameters loaded successfully");
        }
        else
        {
            ROS_WARN("Left camera parameters not found, left camera will be disabled");
        }
        
        if (right_camera_available)
        {
            loadCameraInfoFromParams(node, "camera_right", cameraR_info_msg);
            ROS_INFO("Right camera parameters loaded successfully");
        }
        else
        {
            ROS_WARN("Right camera parameters not found, right camera will be disabled");
        }

        image_transport::ImageTransport main_cam_image(node);
        
        // Create publishers only for available cameras
        if (left_camera_available)
        {
            imageL_pub = main_cam_image.advertiseCamera("/hikrobot_camera_L/image_raw", 1000);
            cv_ptr_l = boost::make_shared<cv_bridge::CvImage>();
            cv_ptr_l->encoding = sensor_msgs::image_encodings::RGB8;
        }
        
        if (right_camera_available)
        {
            imageR_pub = main_cam_image.advertiseCamera("/hikrobot_camera_R/image_raw", 1000);
            cv_ptr_r = boost::make_shared<cv_bridge::CvImage>();
            cv_ptr_r->encoding = sensor_msgs::image_encodings::RGB8;
        }

        // 枚举设备
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        unsigned int nIndex = 0;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);

                void* handle = NULL;
                handles.push_back(handle);

                pthread_mutex_t mutex;
                pthread_mutex_init(&mutex, NULL);  // 初始化互斥锁
                mutexs.push_back(mutex);

                cv::Mat frame;
                frames.push_back(frame);

                bool frame_empty = 0;
                frame_emptys.push_back(frame_empty);
            }
            printf("Find Devices: %d\n", stDeviceList.nDeviceNum);
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        // 选择设备初始设置并取流
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            RunCamera(i, handles[i]);
        }
    }

    // 初始化和启动相机
    void Camera::RunCamera(int ndevice, void* &handle)
    {
        //选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[ndevice]);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        nRet = MV_CC_OpenDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[ndevice]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        // // 设置触发模式为off
        // nRet = MV_CC_SetEnumValue(handle, "TriggerMode", TriggerMode);
        // if (MV_OK != nRet)
        // {
        //     printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        //     exit(-1);
        // }

        // if(TriggerMode){
        //     // 设置触发源
        //     nRet = MV_CC_SetEnumValue(handle, "TriggerSource", TriggerSource);
        //     if (MV_OK != nRet)
        //     {
        //         printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
        //         exit(-1);
        //     }
        // }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            exit(-1);
        }

        // 开始取流
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 创建工作线程
        ThreadData* data = new ThreadData;
        data->ndevice = ndevice;
        data->handle = handle;
        data->stParam = stParam;

        pthread_t nThreadID;
        threads.push_back(nThreadID);
        nRet = pthread_create(&threads[ndevice], NULL, WorkThread, static_cast<void*>(data));
        if (nRet != 0)
        {
            printf("thread create failed. ret = %d\n", nRet);
            exit(-1);
        }
    }

    // 工作线程函数
    void* Camera::WorkThread(void* p_handle)
    {
        int nRet;
        int image_empty_count = 0; //空图帧数
        unsigned char *pDataForRGB = NULL;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

        ThreadData* data = static_cast<ThreadData*>(p_handle);
        int ndevice = data->ndevice;
        void* handle = data->handle;
        MVCC_INTVALUE stParam = data->stParam;

        unsigned char * pData = NULL; 
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            printf("pData is null\n");
            // It's important to also free data if allocated before exit
            delete static_cast<ThreadData*>(p_handle); // Clean up ThreadData
            exit(-1);
        }
        unsigned int nDataSize = stParam.nCurValue;

        while (ros::ok())
        {
            nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    // Clean up before exiting
                    free(pData);
                    delete static_cast<ThreadData*>(p_handle);
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数

            pDataForRGB = (unsigned char*)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
            if (NULL == pDataForRGB)
            {
                printf("pDataForRGB is null\n");
                free(pData);  // 释放pData
                // Clean up before exiting
                delete static_cast<ThreadData*>(p_handle);
                exit(-1);
            }

            // 像素格式转换
            // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
            // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight *  4 + 2048;
            nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
            if (MV_OK != nRet)
            {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                free(pData);  // 释放pData
                free(pDataForRGB);  // 释放pDataForRGB
                // Clean up before exiting
                delete static_cast<ThreadData*>(p_handle);
                exit(-1);
            }
            pthread_mutex_lock(&mutexs[ndevice]);
            if(ndevice == 0 && left_camera_available){
                cv_ptr_l->image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB).clone();
                imageL_msg = *(cv_ptr_l->toImageMsg());
                if(SysteamTime)
                {
                    imageL_msg.header.stamp = ros::Time::now(); 
                    // ROS_INFO("Using cameraL system time: %f", imageR_msg.header.stamp.toSec());
                }
                else
                {
                    imageL_msg.header.stamp = ConvertToROSTime(stImageInfo.nDevTimeStampHigh, stImageInfo.nDevTimeStampLow);
                    // ROS_INFO("Using cameraL time: %f", imageL_msg.header.stamp.toSec()); // Reduced verbosity
                }
                imageL_msg.header.frame_id = "hikrobot_camera"; // Or a more specific frame like "camera_left_link"
                
                // cameraL_info_msg is already populated with calibration data by loadCameraInfoFromParams
                // We only need to update the header stamp and frame_id to match the image message
                cameraL_info_msg.header.stamp = imageL_msg.header.stamp;
                cameraL_info_msg.header.frame_id = imageL_msg.header.frame_id; 

                imageL_pub.publish(imageL_msg, cameraL_info_msg);
            }
            else if(ndevice == 1 && right_camera_available){
                cv_ptr_r->image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB).clone();
                imageR_msg = *(cv_ptr_r->toImageMsg());
                if(SysteamTime)
                {
                    imageR_msg.header.stamp = ros::Time::now(); 
                    // ROS_INFO("Using cameraR system time: %f", imageR_msg.header.stamp.toSec());
                }
                else
                {
                    imageR_msg.header.stamp = ConvertToROSTime(stImageInfo.nDevTimeStampHigh, stImageInfo.nDevTimeStampLow);
                    // ROS_INFO("Using cameraR time: %f", imageR_msg.header.stamp.toSec()); // Reduced verbosity
                }
                imageR_msg.header.frame_id = "hikrobot_camera"; // Or a more specific frame like "camera_right_link"

                // cameraR_info_msg is already populated with calibration data by loadCameraInfoFromParams
                // We only need to update the header stamp and frame_id to match the image message
                cameraR_info_msg.header.stamp = imageR_msg.header.stamp; // Match right image stamp
                cameraR_info_msg.header.frame_id = imageR_msg.header.frame_id;

                imageR_pub.publish(imageR_msg, cameraR_info_msg);
            }
            else if(ndevice >= 2){
                printf("目前只支持两个相机，多个相机需要在此处添加一些代码。");
                exit(-1);
            }
            // If camera is not available, skip processing but continue loop
            pthread_mutex_unlock(&mutexs[ndevice]);

            free(pDataForRGB);  // 释放pDataForRGB
        }
        free(pData);
        delete data;  // 释放data
        return 0;
    }

    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 // 当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); // 用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    ros::Time ConvertToROSTime(uint32_t nDevTimeStampHigh, uint32_t nDevTimeStampLow)
    {
        uint64_t timestamp = static_cast<uint64_t>(nDevTimeStampHigh) << 32 | nDevTimeStampLow;
        uint64_t seconds = timestamp / 1000000000UL;
        uint64_t nanoseconds = timestamp % 1000000000UL;
        return ros::Time(seconds, nanoseconds);
    }

    // Load camera info from ROS parameters
    void Camera::loadCameraInfoFromParams(ros::NodeHandle &node, const std::string &camera_name, sensor_msgs::CameraInfo &camera_info)
    {
        std::string param_prefix = camera_name + "/";
        
        // Load basic camera info
        int width, height;
        std::string camera_name_str, distortion_model;
        
        if (!node.getParam(param_prefix + "image_width", width) ||
            !node.getParam(param_prefix + "image_height", height) ||
            !node.getParam(param_prefix + "camera_name", camera_name_str) ||
            !node.getParam(param_prefix + "distortion_model", distortion_model))
        {
            ROS_ERROR("Failed to load basic camera info for %s", camera_name.c_str());
            return;
        }
        
        camera_info.width = width;
        camera_info.height = height;
        camera_info.distortion_model = distortion_model;

        // Load camera matrix
        std::vector<double> camera_matrix_data;
        if (node.getParam(param_prefix + "camera_matrix/data", camera_matrix_data) && camera_matrix_data.size() == 9)
        {
            for (int i = 0; i < 9; ++i)
            {
                camera_info.K[i] = camera_matrix_data[i];
            }
        }
        else
        {
            ROS_ERROR("Failed to load camera matrix for %s", camera_name.c_str());
        }

        // Load distortion coefficients
        std::vector<double> distortion_data;
        if (node.getParam(param_prefix + "distortion_coefficients/data", distortion_data))
        {
            camera_info.D = distortion_data;
        }
        else
        {
            ROS_ERROR("Failed to load distortion coefficients for %s", camera_name.c_str());
        }

        // Load rectification matrix
        std::vector<double> rectification_data;
        if (node.getParam(param_prefix + "rectification_matrix/data", rectification_data) && rectification_data.size() == 9)
        {
            for (int i = 0; i < 9; ++i)
            {
                camera_info.R[i] = rectification_data[i];
            }
        }
        else
        {
            ROS_ERROR("Failed to load rectification matrix for %s", camera_name.c_str());
        }

        // Load projection matrix
        std::vector<double> projection_data;
        if (node.getParam(param_prefix + "projection_matrix/data", projection_data) && projection_data.size() == 12)
        {
            for (int i = 0; i < 12; ++i)
            {
                camera_info.P[i] = projection_data[i];
            }
        }
        else
        {
            ROS_ERROR("Failed to load projection matrix for %s", camera_name.c_str());
        }

        ROS_INFO("Successfully loaded camera info for %s", camera_name.c_str());
    }

    // Check if camera parameters exist in ROS parameter server
    bool Camera::checkCameraParamsExist(ros::NodeHandle &node, const std::string &camera_name)
    {
        std::string param_prefix = camera_name + "/";
        
        // Check if essential parameters exist
        if (node.hasParam(param_prefix + "image_width") &&
            node.hasParam(param_prefix + "image_height") &&
            node.hasParam(param_prefix + "camera_matrix/data") &&
            node.hasParam(param_prefix + "distortion_coefficients/data") &&
            node.hasParam(param_prefix + "rectification_matrix/data") &&
            node.hasParam(param_prefix + "projection_matrix/data"))
        {
            return true;
        }
        
        return false;
    }

    Camera::~Camera()
    {
        // 销毁线程
        for (int i = 0; i < threads.size(); i++)
        {
            pthread_join(threads[i], NULL);
        }

        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            // 停止取流
            nRet = MV_CC_StopGrabbing(handles[i]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
                break;
            }

            // 关闭设备
            nRet = MV_CC_CloseDevice(handles[i]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
                break;
            }

            // 销毁句柄
            nRet = MV_CC_DestroyHandle(handles[i]);
            if (MV_OK != nRet)
            {
                printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
                break;
            }
            handles[i] = NULL;
        }

        // 销毁互斥锁
        for (int i = 0; i < mutexs.size(); i++)
        {
            pthread_mutex_destroy(&mutexs[i]);
        }
    }
}
#endif
