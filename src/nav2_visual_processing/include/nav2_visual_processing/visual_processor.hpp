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
#include <pcl-1.14/pcl/point_cloud.h> // 点云基础类型
#include <pcl-1.14/pcl/point_types.h> // 点类型定义
#include <pcl_conversions/pcl_conversions.h> // ros2和pcl消息互转
#include <pcl-1.14/pcl/filters/voxel_grid.h> // 滤波器
#include <pcl-1.14/pcl/common/transforms.h> // 点云坐标变换
#include <small_gicp/pcl/pcl_registration.hpp> // pcl结合使用
#include <small_gicp/registration/registration.hpp> // 核心注册算法
// 导入头文件为点云计算协方差


#include <vector>

namespace nav2_visual_processing {

    // 状态机的状态定义
    enum TrackingState {
        
        TRACKING = 0,  // 使用GICP进行精确追踪
        RECOVERY,  // 使用ORB+SVD进行恢复，获取初始位姿估计
        STATEMAX   // 状态最大数目
    };
    
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

        // ==================== 状态机管理 ====================
        TrackingState current_state_;  // 当前状态

        // 状态转换函数
        void switch_to_recovery_state();
        void switch_to_tracking_state();

        // 检查是否需要进行状态转换的函数
        bool should_switch_to_recovery();

        // ==================== 异步回调函数 ====================
        void sync_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
        );
        
        // 提取特征点
        Frame extract_frame(
            const cv::Mat &color_img,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
        );
    
        // Recovery状态的处理函数框架
        // SVD解算位姿
        Eigen::Matrix4d estimate_motion(
            const Frame &f1,
            const Frame &f2
        );

        // GICP算法进行精确位姿估计（Tracking状态）
        // 返回位姿矩阵，通过引用参数返回对齐分数
        Eigen::Matrix4d estimate_motion_with_gicp(
            const Frame &f1,
            const Frame &f2,
            double &alignment_score
        );

        // 检查GICP对齐结果是否有效
        bool is_gicp_result_valid(double alignment_score) const;

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

        // ==================== GICP相关变量（Tracking状态） ====================
        // GICP对齐分数的阈值（用于判断是否需要切换到Recovery状态）
        double gicp_score_threshold_ = 0.5;
        
        // GICP最后一次的对齐分数
        double last_gicp_score_ = 0.0;
        
        // 当前的相机位姿估计（世界坐标系到相机坐标系的变换）
        Eigen::Matrix4d current_pose_ = Eigen::Matrix4d::Identity();
        
        // GICP初始化用的粗略位姿（来自Recovery状态）
        Eigen::Matrix4d initial_pose_for_gicp_ = Eigen::Matrix4d::Identity();

        // ==================== 特征检测与匹配相关 ====================
        cv::Ptr<cv::ORB> orb_; // ORB特征提取器
        cv::Ptr<cv::DescriptorMatcher> matcher_;

        // ==================== 帧管理 ====================
        Frame last_f_; // 上一帧
        bool is_first_f_ = true;

        // Recovery状态中用于位姿初始化的上一帧
        Frame recovery_reference_frame_;
    };
} // nav2_visual_processing

#endif // NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR