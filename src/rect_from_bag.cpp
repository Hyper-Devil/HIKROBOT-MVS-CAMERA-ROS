#include <ros/ros.h>
#include <ros/master.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <mutex>

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rect_utils.hpp"

// ---------------------------------------------------------------------------
// Per-camera rectifier: subscribes to one image_raw/camera_info pair and
// publishes rect/image_rect + rect/camera_info, reusing buildRectifyArtifacts.
// ---------------------------------------------------------------------------
class CameraRectifier
{
public:
    CameraRectifier(ros::NodeHandle &nh,
                    image_transport::ImageTransport &it,
                    const std::string &base_topic,
                    int queue_size)
        : initialized_(false)
    {
        const std::string rect_base = base_topic + "/rect";
        img_pub_ = it.advertiseCamera(rect_base + "/image_rect", queue_size);

        // camera_info has no transport variants, plain subscriber is correct.
        info_sub_ = nh.subscribe(base_topic + "/camera_info", 1,
                                 &CameraRectifier::infoCb, this);
        // image_raw via image_transport so compressed topics in bags also work.
        img_sub_ = it.subscribe(base_topic + "/image_raw", queue_size,
                                &CameraRectifier::imageCb, this);

        ROS_INFO("[rect_from_bag] watching %s", base_topic.c_str());
    }

private:
    void infoCb(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (initialized_) return;

        cv::Size size(static_cast<int>(msg->width), static_cast<int>(msg->height));
        bool ok = camera::buildRectifyArtifacts(
            *msg, size, map_x_, map_y_, crop_roi_, rect_info_template_);

        if (!ok)
        {
            ROS_WARN_ONCE("[rect_from_bag] roi.do_rectify=false — "
                          "no rectified output will be published.");
            return;
        }
        initialized_ = true;
        ROS_INFO("[rect_from_bag] rectifier ready, output %dx%d+%d+%d",
                 crop_roi_.width, crop_roi_.height,
                 crop_roi_.x,     crop_roi_.y);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        // Take a snapshot of shared state to minimise lock hold time.
        cv::Mat map_x, map_y;
        cv::Rect crop_roi;
        sensor_msgs::CameraInfo rect_info;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!initialized_) return;
            map_x     = map_x_;
            map_y     = map_y_;
            crop_roi  = crop_roi_;
            rect_info = rect_info_template_;
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (const cv_bridge::Exception &e)
        {
            ROS_ERROR("[rect_from_bag] cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat rectified_full;
        cv::remap(cv_ptr->image, rectified_full, map_x, map_y, cv::INTER_LINEAR);

        if (crop_roi.width  <= 0 || crop_roi.height <= 0 ||
            crop_roi.x      <  0 || crop_roi.y      <  0 ||
            crop_roi.x + crop_roi.width  > rectified_full.cols ||
            crop_roi.y + crop_roi.height > rectified_full.rows)
        {
            ROS_WARN_THROTTLE(5.0, "[rect_from_bag] crop_roi out of bounds, skipping frame.");
            return;
        }

        cv::Mat cropped = rectified_full(crop_roi);

        sensor_msgs::ImagePtr rect_msg =
            cv_bridge::CvImage(msg->header, msg->encoding, cropped).toImageMsg();

        rect_info.header = msg->header;
        img_pub_.publish(*rect_msg, rect_info);
    }

    image_transport::CameraPublisher img_pub_;
    ros::Subscriber                  info_sub_;
    image_transport::Subscriber      img_sub_;

    std::mutex            mutex_;
    bool                    initialized_;
    cv::Mat                 map_x_;
    cv::Mat                 map_y_;
    cv::Rect                crop_roi_;
    sensor_msgs::CameraInfo rect_info_template_;
};

// ---------------------------------------------------------------------------
// Topic discovery: poll until at least one image_raw+camera_info pair appears.
// ---------------------------------------------------------------------------
static std::vector<std::string> discoverBaseTopics(const std::string &prefix,
                                                    double timeout_s)
{
    const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
    const std::string suffix = "/image_raw";

    while (ros::ok())
    {
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        std::set<std::string> names;
        for (const auto &t : topics)
            names.insert(t.name);

        std::vector<std::string> bases;
        for (const auto &t : topics)
        {
            if (t.name.find(prefix) != 0) continue;
            if (t.datatype != "sensor_msgs/Image") continue;
            if (t.name.size() < suffix.size()) continue;
            if (t.name.compare(t.name.size() - suffix.size(),
                               suffix.size(), suffix) != 0) continue;

            const std::string base = t.name.substr(0, t.name.size() - suffix.size());
            if (names.count(base + "/camera_info"))
                bases.push_back(base);
        }

        if (!bases.empty()) return bases;
        if (ros::Time::now() > deadline) break;
        ros::Duration(0.5).sleep();
    }
    return {};
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hikrobot_rect_from_bag");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string topic_prefix;
    double scan_timeout;
    int queue_size;

    pnh.param<std::string>("topic_prefix", topic_prefix, "/hikrobot_camera");
    pnh.param<double>("scan_timeout",      scan_timeout,  10.0);
    pnh.param<int>   ("queue_size",        queue_size,    10);

    ROS_INFO("[rect_from_bag] scanning under \"%s\" (timeout %.1fs)...",
             topic_prefix.c_str(), scan_timeout);

    std::vector<std::string> bases = discoverBaseTopics(topic_prefix, scan_timeout);
    if (bases.empty())
    {
        ROS_ERROR("[rect_from_bag] no image_raw+camera_info pairs found under \"%s\" "
                  "within %.1fs — is the bag playing?",
                  topic_prefix.c_str(), scan_timeout);
        return 1;
    }

    ROS_INFO("[rect_from_bag] found %zu pair(s):", bases.size());
    for (const auto &b : bases)
        ROS_INFO("  %s", b.c_str());

    image_transport::ImageTransport it(nh);

    // Keep rectifiers alive until shutdown.
    std::vector<std::unique_ptr<CameraRectifier>> rectifiers;
    rectifiers.reserve(bases.size());
    for (const auto &base : bases)
        rectifiers.emplace_back(new CameraRectifier(nh, it, base, queue_size));

    // AsyncSpinner lets multiple camera callbacks run in parallel.
    ros::AsyncSpinner spinner(static_cast<uint32_t>(bases.size()));
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
