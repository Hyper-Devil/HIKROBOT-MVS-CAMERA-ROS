#ifndef RECT_UTILS_HPP
#define RECT_UTILS_HPP

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cstddef>

namespace camera
{
    // Build undistort-rectify maps and the corresponding rectified CameraInfo.
    // Mirrors WorkThread's rectification pipeline exactly:
    //   cv::initUndistortRectifyMap → cv::remap(INTER_LINEAR) → crop to roi.
    // Returns false (and leaves outputs unmodified) when roi.do_rectify is false
    // or the roi dimensions are invalid.
    inline bool buildRectifyArtifacts(const sensor_msgs::CameraInfo &raw_info,
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
                K.at<double>(r, c)    = raw_info.K[r * 3 + c];
                R.at<double>(r, c)    = raw_info.R[r * 3 + c];
                P3x3.at<double>(r, c) = raw_info.P[r * 4 + c];
            }
        }

        bool p3x3_valid = (P3x3.at<double>(0, 0) != 0.0 && P3x3.at<double>(1, 1) != 0.0);
        if (!p3x3_valid)
        {
            P3x3 = K.clone();
        }

        cv::Mat D(static_cast<int>(raw_info.D.size()), 1, CV_64F);
        for (size_t i = 0; i < raw_info.D.size(); ++i)
        {
            D.at<double>(static_cast<int>(i), 0) = raw_info.D[i];
        }

        cv::initUndistortRectifyMap(K, D, R, P3x3, image_size, CV_32FC1, map_x, map_y);

        int binning_x = static_cast<int>(raw_info.binning_x);
        int binning_y = static_cast<int>(raw_info.binning_y);
        if (binning_x <= 0) binning_x = 1;
        if (binning_y <= 0) binning_y = 1;

        int crop_x = static_cast<int>(raw_info.roi.x_offset / static_cast<uint32_t>(binning_x));
        int crop_y = static_cast<int>(raw_info.roi.y_offset / static_cast<uint32_t>(binning_y));
        int crop_w = static_cast<int>(raw_info.roi.width    / static_cast<uint32_t>(binning_x));
        int crop_h = static_cast<int>(raw_info.roi.height   / static_cast<uint32_t>(binning_y));

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

        if (crop_x + crop_w > image_size.width)  crop_w = image_size.width  - crop_x;
        if (crop_y + crop_h > image_size.height) crop_h = image_size.height - crop_y;

        if (crop_w <= 0 || crop_h <= 0)
        {
            return false;
        }

        crop_roi = cv::Rect(crop_x, crop_y, crop_w, crop_h);

        rect_info        = raw_info;
        rect_info.width  = static_cast<uint32_t>(crop_w);
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

        // P[3] (Tx) and P[7] (Ty) are intentionally NOT overwritten —
        // they inherit the baseline terms from raw_info for stereo correctness.
        rect_info.P[0]  = P3x3.at<double>(0, 0);
        rect_info.P[1]  = P3x3.at<double>(0, 1);
        rect_info.P[2]  = P3x3.at<double>(0, 2) - static_cast<double>(crop_x);
        rect_info.P[4]  = 0.0;
        rect_info.P[5]  = P3x3.at<double>(1, 1);
        rect_info.P[6]  = P3x3.at<double>(1, 2) - static_cast<double>(crop_y);
        rect_info.P[8]  = 0.0;
        rect_info.P[9]  = 0.0;
        rect_info.P[10] = 1.0;

        rect_info.roi.x_offset  = 0;
        rect_info.roi.y_offset  = 0;
        rect_info.roi.width     = 0;
        rect_info.roi.height    = 0;
        rect_info.roi.do_rectify = false;

        return true;
    }

} // namespace camera

#endif // RECT_UTILS_HPP
