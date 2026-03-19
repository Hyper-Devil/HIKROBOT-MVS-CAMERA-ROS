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
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <vector>
#include <set>

namespace camera
{
    std::vector<cv::Mat> frames;
    std::vector<bool> frame_emptys;
    std::vector<pthread_mutex_t> mutexs;
    bool SystemTime;

    std::vector<int> camera_device_indices;
    std::vector<std::string> camera_yaml_names;
    std::vector<std::string> configured_camera_ips;

    std::vector<image_transport::CameraPublisher> image_pubs;
    std::vector<image_transport::CameraPublisher> rect_image_pubs;
    std::vector<sensor_msgs::Image> image_msgs;
    std::vector<sensor_msgs::CameraInfo> camera_info_msgs;
    std::vector<sensor_msgs::CameraInfo> rect_camera_info_msgs;

    std::vector<cv::Mat> rect_map_xs;
    std::vector<cv::Mat> rect_map_ys;
    std::vector<cv::Rect> rect_crop_rois;
    std::vector<bool> rect_enabled_flags;

    int publish_queue_size = 10;

    std::vector<cv_bridge::CvImagePtr> cv_ptrs;

    ros::Time ConvertToROSTime(uint32_t nDevTimeStampHigh, uint32_t nDevTimeStampLow, int64_t freq);

    bool buildRectifyArtifacts(const sensor_msgs::CameraInfo &raw_info,
                               cv::Size image_size,
                               cv::Mat &map_x,
                               cv::Mat &map_y,
                               cv::Rect &crop_roi,
                               sensor_msgs::CameraInfo &rect_info);

    static bool setEnumWithVerify(void* handle,
                                  const char* key,
                                  unsigned int desired_value,
                                  const char* desired_name,
                                  const char* fallback_name = NULL);

    struct ThreadData {
        int ndevice;
        void* handle;
        MVCC_INTVALUE stParam;
        bool trigger_enabled;
        int trigger_source;
        int64_t nTimestampFreq;
    };

    class Camera
    {
    public:
        Camera(ros::NodeHandle &node);
        ~Camera();

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        void RunCamera(int logical_index, int physical_device_index, void* &handle); // 初始化和启动相机
        static void* WorkThread(void* p_handle); // 工作线程函数

    private:
        std::vector<void*> handles; // 创建多个相机句柄
        MV_CC_DEVICE_INFO_LIST stDeviceList;  // 枚举设备
        std::vector<pthread_t> threads; // 存储线程ID

        int nRet;
        int TriggerMode;
        int TriggerSource;
        double FrameRate;
        // 触发来源：
        // MV_TRIGGER_SOURCE_LINE0 = 0,MV_TRIGGER_SOURCE_LINE1 = 1,MV_TRIGGER_SOURCE_LINE2 = 2,
        // MV_TRIGGER_SOURCE_LINE3 = 3,MV_TRIGGER_SOURCE_COUNTER0 = 4,MV_TRIGGER_SOURCE_SOFTWARE = 7,
        // MV_TRIGGER_SOURCE_FrequencyConverter = 8

        // CameraInfoManager members (removed - now using ROS params directly)
        // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_l_;
        // boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_r_;
        
        // Helper function to load camera info from ROS parameters
        void loadCameraInfoFromParams(ros::NodeHandle &node, const std::string &camera_name, sensor_msgs::CameraInfo &camera_info);
        
        // Helper function to check if camera parameters exist
        bool checkCameraParamsExist(ros::NodeHandle &node, const std::string &camera_name);

        // Helper function to convert device IP to string
        std::string ipToString(uint32_t raw_ip);

        // Helper function to compare configured ip with device raw ip
        bool ipMatches(const std::string &configured_ip, uint32_t device_raw_ip);

        // Helper function to find device index by configured ip
        int findDeviceIndexByIp(const std::string &configured_ip, std::vector<bool> &used_devices);

        // Helper function to convert trigger source to string
        std::string triggerSourceToString(unsigned int trigger_source);

        // Discover camera names from ROS parameter server
        std::vector<std::string> discoverCameraNames(ros::NodeHandle &node);

        // Helper function to print current device configuration once after startup
        void logCurrentConfig(int logical_index, int physical_device_index, const std::string &yaml_camera_name, void* handle);
    };

    static bool setEnumWithVerify(void* handle,
                                  const char* key,
                                  unsigned int desired_value,
                                  const char* desired_name,
                                  const char* fallback_name)
    {
        int ret = MV_OK;

        // Try setting by desired name first
        if (desired_name != NULL)
        {
            ret = MV_CC_SetEnumValueByString(handle, key, desired_name);
            if (ret == MV_OK)
            {
                // Verify if it was set successfully
                MVCC_ENUMVALUE current = {0};
                ret = MV_CC_GetEnumValue(handle, key, &current);
                if (ret == MV_OK)
                {
                    return true;
                }
            }
        }

        // Try setting by fallback name
        if (fallback_name != NULL)
        {
            ret = MV_CC_SetEnumValueByString(handle, key, fallback_name);
            if (ret == MV_OK)
            {
                MVCC_ENUMVALUE current = {0};
                ret = MV_CC_GetEnumValue(handle, key, &current);
                if (ret == MV_OK)
                {
                    return true;
                }
            }
        }

        // Try setting by desired value as last resort
        ret = MV_CC_SetEnumValue(handle, key, desired_value);
        if (ret == MV_OK)
        {
            MVCC_ENUMVALUE current = {0};
            ret = MV_CC_GetEnumValue(handle, key, &current);
            if (ret == MV_OK && current.nCurValue == desired_value)
            {
                return true;
            }
        }

        ROS_ERROR("Set %s failed using name(%s), fallback(%s) or value(%u)", 
                  key, desired_name ? desired_name : "none", 
                  fallback_name ? fallback_name : "none", desired_value);
        return false;
    }

    Camera::Camera(ros::NodeHandle &node)
    {
        //读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效
        node.param("TriggerMode", TriggerMode, 0);    //0为不启用触发，1为启用
        node.param("TriggerSource", TriggerSource, 0);  //设置触发模式
        node.param("FrameRate", FrameRate, 10.0);       //无触发时的帧率
        node.param("SystemTime", SystemTime, false);
        node.param("publish_queue_size", publish_queue_size, 10);

        ROS_INFO("Loading camera info from ROS parameters...");
        std::vector<std::string> discovered_camera_names = discoverCameraNames(node);
        if (discovered_camera_names.empty())
        {
            ROS_ERROR("No valid camera parameters found in ROS parameter server!");
            exit(-1);
        }

        // Expose discovered camera keys for downstream dynamic nodelet loading.
        ros::param::set("/hikrobot_camera/active_camera_names", discovered_camera_names);
        ros::param::set("/hikrobot_camera/active_camera_names_ready", true);

        int configured_camera_count = static_cast<int>(discovered_camera_names.size());

        image_transport::ImageTransport main_cam_image(node);
        
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
            if (static_cast<int>(stDeviceList.nDeviceNum) != configured_camera_count)
            {
                ROS_ERROR("Configured %d camera(s) in YAML but found %d device(s). Strict matching requires exact equality.",
                          configured_camera_count, stDeviceList.nDeviceNum);
                exit(-1);
            }

            int active_device_count = configured_camera_count;
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
            printf("Find Devices: %d\n", stDeviceList.nDeviceNum);

            handles.clear();
            mutexs.clear();
            frames.clear();
            frame_emptys.clear();

            handles.resize(active_device_count, NULL);
            mutexs.resize(active_device_count);
            frames.resize(active_device_count);
            frame_emptys.resize(active_device_count, false);
            camera_device_indices.resize(active_device_count, -1);
            camera_yaml_names.resize(active_device_count);
            configured_camera_ips.resize(active_device_count);
            image_pubs.resize(active_device_count);
            rect_image_pubs.resize(active_device_count);
            image_msgs.resize(active_device_count);
            camera_info_msgs.resize(active_device_count);
            rect_camera_info_msgs.resize(active_device_count);
            rect_map_xs.resize(active_device_count);
            rect_map_ys.resize(active_device_count);
            rect_crop_rois.resize(active_device_count);
            rect_enabled_flags.resize(active_device_count, false);
            cv_ptrs.resize(active_device_count);

            for (int i = 0; i < active_device_count; i++)
            {
                pthread_mutex_init(&mutexs[i], NULL);  // 初始化互斥锁
            }

            std::vector<bool> used_devices(stDeviceList.nDeviceNum, false);
            for (int logical_index = 0; logical_index < active_device_count; ++logical_index)
            {
                const std::string &camera_name = discovered_camera_names[logical_index];
                std::string configured_ip;
                if (!node.getParam(camera_name + "/ip", configured_ip) || configured_ip.empty())
                {
                    ROS_ERROR("%s/ip is required and must not be empty", camera_name.c_str());
                    exit(-1);
                }

                int matched_index = findDeviceIndexByIp(configured_ip, used_devices);
                if (matched_index < 0)
                {
                    ROS_ERROR("Failed to match %s with configured ip %s", camera_name.c_str(), configured_ip.c_str());
                    exit(-1);
                }

                camera_device_indices[logical_index] = matched_index;
                camera_yaml_names[logical_index] = camera_name;
                configured_camera_ips[logical_index] = configured_ip;
                used_devices[matched_index] = true;
                ROS_INFO("%s matched to device index %d", camera_name.c_str(), matched_index);
            }

            for (int i = 0; i < active_device_count; ++i)
            {
                const std::string &camera_name = camera_yaml_names[i];
                std::string topic = "/hikrobot_camera/" + camera_name + "/image_raw";
                std::string rect_topic = "/hikrobot_camera/" + camera_name + "/rect/image_rect";
                image_pubs[i] = main_cam_image.advertiseCamera(topic, publish_queue_size);
                rect_image_pubs[i] = main_cam_image.advertiseCamera(rect_topic, publish_queue_size);

                cv_ptrs[i] = boost::make_shared<cv_bridge::CvImage>();
                cv_ptrs[i]->encoding = sensor_msgs::image_encodings::RGB8;

                loadCameraInfoFromParams(node, camera_name, camera_info_msgs[i]);

                const sensor_msgs::CameraInfo &camera_info = camera_info_msgs[i];
                cv::Size image_size(static_cast<int>(camera_info.width), static_cast<int>(camera_info.height));
                rect_enabled_flags[i] = buildRectifyArtifacts(camera_info,
                                                              image_size,
                                                              rect_map_xs[i],
                                                              rect_map_ys[i],
                                                              rect_crop_rois[i],
                                                              rect_camera_info_msgs[i]);

                if (rect_enabled_flags[i])
                {
                    ROS_INFO("Rectify enabled for %s, rect topic: %s", camera_name.c_str(), rect_topic.c_str());
                }
                else
                {
                    ROS_INFO("Rectify disabled for %s (roi/do_rectify=false or invalid roi)", camera_name.c_str());
                }

                ROS_INFO("Initialized publisher for %s on topic %s", camera_name.c_str(), topic.c_str());
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        // 选择设备初始设置并取流
        for (int i = 0; i < static_cast<int>(handles.size()); i++)
        {
            RunCamera(i, camera_device_indices[i], handles[i]);
        }
    }

    // 初始化和启动相机
    void Camera::RunCamera(int logical_index, int physical_device_index, void* &handle)
    {
        if (physical_device_index < 0 || physical_device_index >= static_cast<int>(stDeviceList.nDeviceNum))
        {
            ROS_ERROR("Invalid physical_device_index: %d", physical_device_index);
            exit(-1);
        }

        //选择设备并创建句柄
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[physical_device_index]);

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
        if (stDeviceList.pDeviceInfo[physical_device_index]->nTLayerType == MV_GIGE_DEVICE)
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

        const unsigned int desired_trigger_mode = (TriggerMode != 0) ? 1U : 0U;
        const char* desired_mode_name = (desired_trigger_mode == 1U) ? "On" : "Off";
        const char* fallback_mode_name = (desired_trigger_mode == 1U) ? "ON" : "OFF";
        
        // 1. 先设置 TriggerMode，这通常是解锁其他触发选项的前提
        if (!setEnumWithVerify(handle, "TriggerMode", desired_trigger_mode, desired_mode_name, fallback_mode_name))
        {
            ROS_ERROR("Failed to set TriggerMode=%d", TriggerMode);
            exit(-1);
        }

        if (TriggerMode)
        {
            // 设置 TriggerSource (触发源，例如 Line0)
            if (!setEnumWithVerify(handle,
                                   "TriggerSource",
                                   static_cast<unsigned int>(TriggerSource),
                                   triggerSourceToString(static_cast<unsigned int>(TriggerSource)).c_str(), 
                                   NULL))
            {
                ROS_ERROR("Failed to set TriggerSource=%d", TriggerSource);
                exit(-1);
            }
        }
        else
        {
            // TriggerMode == 0 (连续采图模式) 时，通过设置帧率控制获取帧率
            // MVS客户端中叫 Acquisition Frame Rate Control Enable 或 Acquisition Frame Rate Enable
            int ret_enable = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateControlEnable", true);
            if (ret_enable != MV_OK) {
                ret_enable = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
                if (ret_enable != MV_OK) {
                    ROS_WARN("Failed to enable AcquisitionFrameRate Control.");
                } else {
                    ROS_INFO("Enabled AcquisitionFrameRateEnable via fallback name.");
                }
            } else {
                ROS_INFO("Enabled AcquisitionFrameRateControlEnable.");
            }

            int ret_fps = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", FrameRate);
            if (ret_fps != MV_OK) {
                ROS_WARN("Failed to set AcquisitionFrameRate to %f", FrameRate);
            } else {
                ROS_INFO("Set AcquisitionFrameRate to %f", FrameRate);
            }
        }

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

        // 输出一次当前生效配置（帧率、分辨率、触发模式等）
        std::string yaml_camera_name = "unknown_camera";
        if (logical_index >= 0 && logical_index < static_cast<int>(camera_yaml_names.size()) && !camera_yaml_names[logical_index].empty())
        {
            yaml_camera_name = camera_yaml_names[logical_index];
        }
        logCurrentConfig(logical_index, physical_device_index, yaml_camera_name, handle);

        // 获取时间戳频率 (GevTimestampTickFrequency)
        MVCC_INTVALUE stTickFreq = {0};
        nRet = MV_CC_GetIntValue(handle, "GevTimestampTickFrequency", &stTickFreq);
        int64_t current_freq = 100000000; // 默认 100MHz (10ns)
        if (nRet == MV_OK && stTickFreq.nCurValue > 0)
        {
            current_freq = stTickFreq.nCurValue;
            ROS_INFO("Camera [%d] Timestamp Tick Frequency: %ld Hz", logical_index, current_freq);
        }
        else
        {
            ROS_WARN("Failed to get Tick Frequency, using default 100MHz");
        }

        // 创建工作线程
        ThreadData* data = new ThreadData;
        data->ndevice = logical_index;
        data->handle = handle;
        data->stParam = stParam;
        data->trigger_enabled = (TriggerMode != 0);
        data->trigger_source = TriggerSource;
        data->nTimestampFreq = current_freq;

        pthread_t nThreadID;
        threads.push_back(nThreadID);
        nRet = pthread_create(&threads[logical_index], NULL, WorkThread, static_cast<void*>(data));
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
        unsigned int nDataSizeForRGB = 0;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

        ThreadData* data = static_cast<ThreadData*>(p_handle);
        int ndevice = data->ndevice;
        void* handle = data->handle;
        MVCC_INTVALUE stParam = data->stParam;
        bool trigger_enabled = data->trigger_enabled;
        int trigger_source = data->trigger_source;

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
                if (trigger_enabled && nRet == MV_E_NODATA)
                {
                    ++image_empty_count;
                    ROS_WARN_THROTTLE(5.0,
                                      "Device %d in trigger mode (source=%d), waiting for trigger signal. Consecutive empty reads: %d",
                                      ndevice,
                                      trigger_source,
                                      image_empty_count);
                    continue;
                }

                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    // Clean up before exiting
                    if (pDataForRGB != NULL)
                    {
                        free(pDataForRGB);
                    }
                    free(pData);
                    delete static_cast<ThreadData*>(p_handle);
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数

            unsigned int requiredRgbBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
            if (pDataForRGB == NULL || nDataSizeForRGB < requiredRgbBufferSize)
            {
                if (pDataForRGB != NULL)
                {
                    free(pDataForRGB);
                    pDataForRGB = NULL;
                }

                pDataForRGB = (unsigned char*)malloc(requiredRgbBufferSize);
                if (NULL == pDataForRGB)
                {
                    printf("pDataForRGB is null\n");
                    free(pData);  // 释放pData
                    // Clean up before exiting
                    delete static_cast<ThreadData*>(p_handle);
                    exit(-1);
                }
                nDataSizeForRGB = requiredRgbBufferSize;
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
            stConvertParam.nDstBufferSize = nDataSizeForRGB;
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
            if (ndevice >= 0 &&
                ndevice < static_cast<int>(cv_ptrs.size()) &&
                ndevice < static_cast<int>(image_msgs.size()) &&
                ndevice < static_cast<int>(camera_info_msgs.size()) &&
                ndevice < static_cast<int>(camera_yaml_names.size()))
            {
                const std::string &camera_name = camera_yaml_names[ndevice];
                cv_ptrs[ndevice]->image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB);
                image_msgs[ndevice] = *(cv_ptrs[ndevice]->toImageMsg());

                if (SystemTime)
                {
                    image_msgs[ndevice].header.stamp = ros::Time::now();
                }
                else
                {
                    image_msgs[ndevice].header.stamp = ConvertToROSTime(stImageInfo.nDevTimeStampHigh, 
                                                                            stImageInfo.nDevTimeStampLow, 
                                                                            data->nTimestampFreq);
                }

                image_msgs[ndevice].header.frame_id = camera_name;
                camera_info_msgs[ndevice].header.stamp = image_msgs[ndevice].header.stamp;
                camera_info_msgs[ndevice].header.frame_id = camera_name;

                image_pubs[ndevice].publish(image_msgs[ndevice], camera_info_msgs[ndevice]);

                if (ndevice < static_cast<int>(rect_enabled_flags.size()) &&
                    rect_enabled_flags[ndevice] &&
                    ndevice < static_cast<int>(rect_map_xs.size()) &&
                    ndevice < static_cast<int>(rect_map_ys.size()) &&
                    ndevice < static_cast<int>(rect_crop_rois.size()) &&
                    ndevice < static_cast<int>(rect_camera_info_msgs.size()) &&
                    ndevice < static_cast<int>(rect_image_pubs.size()))
                {
                    cv::Mat rectified_full;
                    cv::remap(cv_ptrs[ndevice]->image,
                              rectified_full,
                              rect_map_xs[ndevice],
                              rect_map_ys[ndevice],
                              cv::INTER_LINEAR);

                    const cv::Rect &crop_roi = rect_crop_rois[ndevice];
                    if (crop_roi.width > 0 && crop_roi.height > 0 &&
                        crop_roi.x >= 0 && crop_roi.y >= 0 &&
                        crop_roi.x + crop_roi.width <= rectified_full.cols &&
                        crop_roi.y + crop_roi.height <= rectified_full.rows)
                    {
                        cv::Mat rect_cropped = rectified_full(crop_roi);
                        sensor_msgs::Image rect_image_msg = *(cv_bridge::CvImage(image_msgs[ndevice].header,
                                                                                 sensor_msgs::image_encodings::RGB8,
                                                                                 rect_cropped).toImageMsg());

                        rect_camera_info_msgs[ndevice].header = image_msgs[ndevice].header;
                        rect_image_pubs[ndevice].publish(rect_image_msg, rect_camera_info_msgs[ndevice]);
                    }
                }
            }
            else
            {
                ROS_WARN_THROTTLE(5.0, "Device %d has no valid camera publish mapping, frame skipped.", ndevice);
            }
            pthread_mutex_unlock(&mutexs[ndevice]);
        }

        if (pDataForRGB != NULL)
        {
            free(pDataForRGB);
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

    ros::Time ConvertToROSTime(uint32_t nDevTimeStampHigh, uint32_t nDevTimeStampLow, int64_t freq)
    {
        uint64_t ticks = (static_cast<uint64_t>(nDevTimeStampHigh) << 32) | nDevTimeStampLow;
        
        // 秒 = 总 ticks / 每秒 ticks 数
        uint64_t seconds = ticks / freq;
        // 剩余 ticks 转换为纳秒：(剩余 ticks * 1e9) / freq
        uint64_t nanoseconds = ((ticks % freq) * 1000000000UL) / freq;
        
        return ros::Time(static_cast<uint32_t>(seconds), static_cast<uint32_t>(nanoseconds));
    }

    std::string Camera::ipToString(uint32_t raw_ip)
    {
        std::ostringstream stream;
        stream << ((raw_ip >> 24) & 0xFF) << "."
               << ((raw_ip >> 16) & 0xFF) << "."
               << ((raw_ip >> 8) & 0xFF) << "."
               << (raw_ip & 0xFF);
        return stream.str();
    }

    bool Camera::ipMatches(const std::string &configured_ip, uint32_t device_raw_ip)
    {
        std::string big_endian = ipToString(device_raw_ip);
        uint32_t byte_swapped = ((device_raw_ip & 0x000000FFU) << 24) |
                                ((device_raw_ip & 0x0000FF00U) << 8)  |
                                ((device_raw_ip & 0x00FF0000U) >> 8)  |
                                ((device_raw_ip & 0xFF000000U) >> 24);
        std::string little_endian = ipToString(byte_swapped);
        return configured_ip == big_endian || configured_ip == little_endian;
    }

    int Camera::findDeviceIndexByIp(const std::string &configured_ip, std::vector<bool> &used_devices)
    {
        for (int i = 0; i < static_cast<int>(stDeviceList.nDeviceNum); ++i)
        {
            if (used_devices[i])
            {
                continue;
            }

            MV_CC_DEVICE_INFO *device_info = stDeviceList.pDeviceInfo[i];
            if (device_info == NULL)
            {
                continue;
            }

            if (device_info->nTLayerType != MV_GIGE_DEVICE)
            {
                ROS_ERROR("Device index %d is not a GigE camera. IP-only matching is enabled.", i);
                continue;
            }

            uint32_t device_ip_raw = device_info->SpecialInfo.stGigEInfo.nCurrentIp;
            if (ipMatches(configured_ip, device_ip_raw))
            {
                return i;
            }
        }
        return -1;
    }

    std::string Camera::triggerSourceToString(unsigned int trigger_source)
    {
        switch (trigger_source)
        {
            case 0:
                return "LINE0";
            case 1:
                return "LINE1";
            case 2:
                return "LINE2";
            case 3:
                return "LINE3";
            case 4:
                return "COUNTER0";
            case 7:
                return "SOFTWARE";
            case 8:
                return "FREQUENCY_CONVERTER";
            default:
                return "UNKNOWN";
        }
    }

    bool buildRectifyArtifacts(const sensor_msgs::CameraInfo &raw_info,
                               cv::Size image_size,
                               cv::Mat &map_x,
                               cv::Mat &map_y,
                               cv::Rect &crop_roi,
                               sensor_msgs::CameraInfo &rect_info)
    {
        if (!raw_info.roi.do_rectify)
        {
            return false;
        }

        if (image_size.width <= 0 || image_size.height <= 0)
        {
            return false;
        }

        cv::Mat K(3, 3, CV_64F);
        cv::Mat R(3, 3, CV_64F);
        cv::Mat P3x3(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                K.at<double>(r, c) = raw_info.K[r * 3 + c];
                R.at<double>(r, c) = raw_info.R[r * 3 + c];
                P3x3.at<double>(r, c) = raw_info.P[r * 4 + c];
            }
        }

        bool p3x3_valid = (P3x3.at<double>(0, 0) != 0.0 && P3x3.at<double>(1, 1) != 0.0);
        if (!p3x3_valid)
        {
            P3x3 = K.clone();
        }

        cv::Mat D(raw_info.D.size(), 1, CV_64F);
        for (size_t i = 0; i < raw_info.D.size(); ++i)
        {
            D.at<double>(static_cast<int>(i), 0) = raw_info.D[i];
        }

        cv::initUndistortRectifyMap(K, D, R, P3x3, image_size, CV_32FC1, map_x, map_y);

        int binning_x = static_cast<int>(raw_info.binning_x);
        int binning_y = static_cast<int>(raw_info.binning_y);
        if (binning_x <= 0)
        {
            binning_x = 1;
        }
        if (binning_y <= 0)
        {
            binning_y = 1;
        }

        int crop_x = static_cast<int>(raw_info.roi.x_offset / static_cast<uint32_t>(binning_x));
        int crop_y = static_cast<int>(raw_info.roi.y_offset / static_cast<uint32_t>(binning_y));
        int crop_w = static_cast<int>(raw_info.roi.width / static_cast<uint32_t>(binning_x));
        int crop_h = static_cast<int>(raw_info.roi.height / static_cast<uint32_t>(binning_y));

        if (crop_w <= 0 || crop_h <= 0)
        {
            return false;
        }

        crop_x = std::max(0, crop_x);
        crop_y = std::max(0, crop_y);
        if (crop_x >= image_size.width || crop_y >= image_size.height)
        {
            return false;
        }

        if (crop_x + crop_w > image_size.width)
        {
            crop_w = image_size.width - crop_x;
        }
        if (crop_y + crop_h > image_size.height)
        {
            crop_h = image_size.height - crop_y;
        }

        if (crop_w <= 0 || crop_h <= 0)
        {
            return false;
        }

        crop_roi = cv::Rect(crop_x, crop_y, crop_w, crop_h);

        rect_info = raw_info;
        rect_info.width = static_cast<uint32_t>(crop_w);
        rect_info.height = static_cast<uint32_t>(crop_h);

        for (size_t i = 0; i < rect_info.D.size(); ++i)
        {
            rect_info.D[i] = 0.0;
        }

        rect_info.R = {1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0};

        rect_info.K[0] = P3x3.at<double>(0, 0);
        rect_info.K[1] = P3x3.at<double>(0, 1);
        rect_info.K[2] = P3x3.at<double>(0, 2) - static_cast<double>(crop_x);
        rect_info.K[3] = 0.0;
        rect_info.K[4] = P3x3.at<double>(1, 1);
        rect_info.K[5] = P3x3.at<double>(1, 2) - static_cast<double>(crop_y);
        rect_info.K[6] = 0.0;
        rect_info.K[7] = 0.0;
        rect_info.K[8] = 1.0;

        rect_info.P[0] = P3x3.at<double>(0, 0);
        rect_info.P[1] = P3x3.at<double>(0, 1);
        rect_info.P[2] = P3x3.at<double>(0, 2) - static_cast<double>(crop_x);
        rect_info.P[4] = 0.0;
        rect_info.P[5] = P3x3.at<double>(1, 1);
        rect_info.P[6] = P3x3.at<double>(1, 2) - static_cast<double>(crop_y);
        rect_info.P[8] = 0.0;
        rect_info.P[9] = 0.0;
        rect_info.P[10] = 1.0;

        rect_info.roi.x_offset = 0;
        rect_info.roi.y_offset = 0;
        rect_info.roi.width = 0;
        rect_info.roi.height = 0;
        rect_info.roi.do_rectify = false;

        return true;
    }

    void Camera::logCurrentConfig(int logical_index, int physical_device_index, const std::string &yaml_camera_name, void* handle)
    {
        MVCC_INTVALUE width = {0};
        MVCC_INTVALUE height = {0};
        MVCC_INTVALUE payload = {0};
        MVCC_ENUMVALUE trigger_mode = {0};
        MVCC_ENUMVALUE trigger_source = {0};
        MVCC_FLOATVALUE frame_rate = {0};

        int ret_width = MV_CC_GetIntValue(handle, "Width", &width);
        int ret_height = MV_CC_GetIntValue(handle, "Height", &height);
        int ret_payload = MV_CC_GetIntValue(handle, "PayloadSize", &payload);
        int ret_trigger_mode = MV_CC_GetEnumValue(handle, "TriggerMode", &trigger_mode);
        int ret_trigger_source = MV_CC_GetEnumValue(handle, "TriggerSource", &trigger_source);

        int ret_frame_rate = MV_CC_GetFloatValue(handle, "ResultingFrameRate", &frame_rate);
        if (ret_frame_rate != MV_OK)
        {
            ret_frame_rate = MV_CC_GetFloatValue(handle, "AcquisitionFrameRate", &frame_rate);
        }

        std::string configured_ip = "N/A";
        if (logical_index >= 0 && logical_index < static_cast<int>(configured_camera_ips.size()) && !configured_camera_ips[logical_index].empty())
        {
            configured_ip = configured_camera_ips[logical_index];
        }

        std::string device_ip = "N/A";
        if (physical_device_index >= 0 && physical_device_index < static_cast<int>(stDeviceList.nDeviceNum))
        {
            MV_CC_DEVICE_INFO *device_info = stDeviceList.pDeviceInfo[physical_device_index];
            if (device_info != NULL && device_info->nTLayerType == MV_GIGE_DEVICE)
            {
                device_ip = ipToString(device_info->SpecialInfo.stGigEInfo.nCurrentIp);
            }
        }

        std::ostringstream report;
        report << "========== Camera Runtime Config ==========" << "\n"
               << "logical_index: " << logical_index << "\n"
               << "physical_device_index: " << physical_device_index << "\n"
             << "yaml_camera_name: " << yaml_camera_name << "\n"
               << "configured_ip: " << configured_ip << "\n"
               << "device_ip: " << device_ip << "\n"
                             << "timestamp_mode: " << (SystemTime ? "system_time" : "device_time") << "\n";

        if (ret_width == MV_OK && ret_height == MV_OK)
        {
            report << "resolution: " << width.nCurValue << "x" << height.nCurValue << "\n";
        }
        else
        {
            report << "resolution: unavailable" << "\n";
        }

        if (ret_frame_rate == MV_OK)
        {
            report << std::fixed << std::setprecision(2)
                   << "frame_rate_fps: " << frame_rate.fCurValue << "\n";
        }
        else
        {
            report << "frame_rate_fps: unavailable" << "\n";
        }

        if (ret_payload == MV_OK)
        {
            report << "payload_size_bytes: " << payload.nCurValue << "\n";
        }
        else
        {
            report << "payload_size_bytes: unavailable" << "\n";
        }

        if (ret_trigger_mode == MV_OK)
        {
            bool trigger_on = (trigger_mode.nCurValue != 0U);
            report << "trigger_mode: " << (trigger_on ? "on" : "off")
                   << " (" << trigger_mode.nCurValue << ")" << "\n";
        }
        else
        {
            report << "trigger_mode: unavailable" << "\n";
        }

        if (ret_trigger_source == MV_OK)
        {
            report << "trigger_source: " << triggerSourceToString(trigger_source.nCurValue)
                   << " (" << trigger_source.nCurValue << ")" << "\n";
        }
        else
        {
            report << "trigger_source: unavailable" << "\n";
        }

        report << "yaml_trigger_mode_param: " << TriggerMode << "\n"
               << "yaml_trigger_source_param: " << TriggerSource << "\n"
               << "===========================================";

        ROS_INFO_STREAM(report.str());
    }

    // Load camera info from ROS parameters
    void Camera::loadCameraInfoFromParams(ros::NodeHandle &node, const std::string &camera_name, sensor_msgs::CameraInfo &camera_info)
    {
        std::string param_prefix = camera_name + "/";
        
        // Load basic camera info
        int width, height;
        std::string distortion_model;
        
        if (!node.getParam(param_prefix + "image_width", width) ||
            !node.getParam(param_prefix + "image_height", height) ||
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

        // Load optional binning and ROI fields for CameraInfo publishing.
        int binning_x = 0;
        int binning_y = 0;
        if (node.getParam(param_prefix + "binning_x", binning_x))
        {
            camera_info.binning_x = static_cast<uint32_t>(std::max(0, binning_x));
        }
        else
        {
            camera_info.binning_x = 0;
            ROS_WARN("%sbinning_x not found, using default 0", param_prefix.c_str());
        }

        if (node.getParam(param_prefix + "binning_y", binning_y))
        {
            camera_info.binning_y = static_cast<uint32_t>(std::max(0, binning_y));
        }
        else
        {
            camera_info.binning_y = 0;
            ROS_WARN("%sbinning_y not found, using default 0", param_prefix.c_str());
        }

        int roi_x_offset = 0;
        int roi_y_offset = 0;
        int roi_height = 0;
        int roi_width = 0;
        bool roi_do_rectify = false;

        bool has_roi_x = node.getParam(param_prefix + "roi/x_offset", roi_x_offset);
        bool has_roi_y = node.getParam(param_prefix + "roi/y_offset", roi_y_offset);
        bool has_roi_h = node.getParam(param_prefix + "roi/height", roi_height);
        bool has_roi_w = node.getParam(param_prefix + "roi/width", roi_width);
        bool has_roi_rectify = node.getParam(param_prefix + "roi/do_rectify", roi_do_rectify);

        if (has_roi_x && has_roi_y && has_roi_h && has_roi_w)
        {
            camera_info.roi.x_offset = static_cast<uint32_t>(std::max(0, roi_x_offset));
            camera_info.roi.y_offset = static_cast<uint32_t>(std::max(0, roi_y_offset));
            camera_info.roi.height = static_cast<uint32_t>(std::max(0, roi_height));
            camera_info.roi.width = static_cast<uint32_t>(std::max(0, roi_width));
            camera_info.roi.do_rectify = has_roi_rectify ? roi_do_rectify : false;
        }
        else
        {
            camera_info.roi.x_offset = 0;
            camera_info.roi.y_offset = 0;
            camera_info.roi.height = 0;
            camera_info.roi.width = 0;
            camera_info.roi.do_rectify = false;
            ROS_WARN("%sroi not fully configured, using default ROI (all zeros)", param_prefix.c_str());
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
            node.hasParam(param_prefix + "ip") &&
            node.hasParam(param_prefix + "camera_matrix/data") &&
            node.hasParam(param_prefix + "distortion_coefficients/data") &&
            node.hasParam(param_prefix + "rectification_matrix/data") &&
            node.hasParam(param_prefix + "projection_matrix/data"))
        {
            return true;
        }
        
        return false;
    }

    std::vector<std::string> Camera::discoverCameraNames(ros::NodeHandle &node)
    {
        std::vector<std::string> param_names;
        ros::param::getParamNames(param_names);

        std::set<std::string> camera_name_set;
        std::string namespace_prefix = node.getNamespace();
        if (namespace_prefix.empty())
        {
            namespace_prefix = "/";
        }

        if (namespace_prefix.back() != '/')
        {
            namespace_prefix += "/";
        }

        for (size_t i = 0; i < param_names.size(); ++i)
        {
            const std::string &full_name = param_names[i];

            if (full_name.find(namespace_prefix) != 0)
            {
                continue;
            }

            std::string relative = full_name.substr(namespace_prefix.size());
            if (relative.empty())
            {
                continue;
            }

            size_t slash_pos = relative.find('/');
            if (slash_pos == std::string::npos)
            {
                continue;
            }

            std::string camera_name = relative.substr(0, slash_pos);
            if (camera_name.empty())
            {
                continue;
            }

            if (checkCameraParamsExist(node, camera_name))
            {
                camera_name_set.insert(camera_name);
            }
        }

        std::vector<std::string> camera_names(camera_name_set.begin(), camera_name_set.end());
        ROS_INFO("Discovered %zu camera config(s) from YAML", camera_names.size());
        for (size_t i = 0; i < camera_names.size(); ++i)
        {
            ROS_INFO("  camera[%zu]: %s", i, camera_names[i].c_str());
        }
        return camera_names;
    }

    Camera::~Camera()
    {
        // 销毁线程
        for (int i = 0; i < threads.size(); i++)
        {
            pthread_join(threads[i], NULL);
        }

        for (int i = 0; i < static_cast<int>(handles.size()); i++)
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
