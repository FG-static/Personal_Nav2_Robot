#ifndef NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR
#define NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace nav2_visual_processing {

    struct Frame {

        cv::Mat color;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors; // 描述子
        std::vector<Eigen::Vector3d> points_3d;
    };

    class VisualProcessorNode : public rclcpp::Node {

    public:

        VisualProcessorNode();
    private:

        // 异步回调函数
        void sync_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
        );
        
        // 提取特征点
        Frame extract_frame(
            const cv::Mat &color_img,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
        );

        // SVD解算位姿
        void estimate_motion(
            const Frame &f1,
            const Frame &f2
        );

        // 可视化匹配结果
        cv::Mat visualize_matches(
            const Frame &f1,
            const Frame &f2,
            const std::vector<cv::DMatch> &matches
        );

        // 消息同步
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        cv::Ptr<cv::ORB> orb_;
        cv::Ptr<cv::DescriptorMatcher> matcher_;

        Frame last_f_;
        bool is_first_f_ = true;
    };
} // nav2_visual_processing

#endif // NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR