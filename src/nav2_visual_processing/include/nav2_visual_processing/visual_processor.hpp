#ifndef NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR
#define NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl-1.14/pcl/point_cloud.h> // 点云基础类型
#include <pcl-1.14/pcl/point_types.h> // 点类型定义
#include <pcl_conversions/pcl_conversions.h> // ros2和pcl消息互转
#include <pcl-1.14/pcl/filters/voxel_grid.h> // 滤波器
#include <pcl-1.14/pcl/common/transforms.h> // 点云坐标变换

#include <unordered_map>

// FLANN (used by small_gicp) expects a serialization implementation for
// std::unordered_map when saving/loading LSH indices. The system FLANN
// version shipped with Ubuntu does not provide this specialization, so we
// provide it here to avoid compilation errors.
namespace flann {
namespace serialization {

// Forward-declare the primary template so we can provide a specialization
// before the full FLANN serialization headers are included.
template<typename T>
struct Serializer;

template<typename K, typename V>
struct Serializer<std::unordered_map<K, V>>
{
  template<typename InputArchive>
  static inline void load(InputArchive &ar, std::unordered_map<K, V> &map_val)
  {
    size_t size;
    ar & size;
    for (size_t i = 0; i < size; ++i) {
      K key;
      V value;
      ar & key;
      ar & value;
      map_val.emplace(std::move(key), std::move(value));
    }
  }

  template<typename OutputArchive>
  static inline void save(OutputArchive &ar, const std::unordered_map<K, V> &map_val)
  {
    ar & map_val.size();
    for (const auto &kv : map_val) {
      ar & kv.first;
      ar & kv.second;
    }
  }
};

}  // namespace serialization
}  // namespace flann

#include <small_gicp/pcl/pcl_registration.hpp> // pcl结合使用
#include <small_gicp/registration/registration.hpp> // 核心注册算法
#include <small_gicp/factors/gicp_factor.hpp>

#include <vector>
#include <array>

namespace nav2_visual_processing {

    // 状态机的状态定义
    enum TrackingState {
        
        TRACKING = 0,  // 使用GICP进行精确追踪
        RECOVERY,  // 使用ORB+SVD进行恢复，获取初始位姿估计
        STATEMAX   // 状态最大数目
    };
    
    struct Frame {

        cv::Mat color;
        cv::Mat gray; // 灰度图
        std::vector<cv::Point2f> kpts_2d; // 当前帧特征点坐标
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors; // 描述子
        std::vector<Eigen::Vector3d> points_3d;
        // 原始点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud; 
    
        Frame() : raw_cloud(new pcl::PointCloud<pcl::PointXYZ>()) {}
    };

    class VisualProcessorNode : public rclcpp::Node {

    public:

        explicit VisualProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    private:

        // camera_link -> map
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void publish_tf(
            const Eigen::Matrix4d &pose,
            const rclcpp::Time &stamp
        );

        // ==================== 状态机管理 ====================
        using MotionEstimatorFn = Eigen::Matrix4d (VisualProcessorNode::*)(
            const Frame &f1,
            const Frame &f2,
            double &alignment_score
        );

        TrackingState current_state_ = RECOVERY;  // 当前状态

        // 状态转换函数
        void switch_to_recovery_state();
        void switch_to_tracking_state();

        // 检查是否需要进行状态转换的函数
        bool should_switch_to_recovery();
        bool should_switch_to_tracking(const Eigen::Matrix4d &motion);

        // 通用运动估计算法入口（根据当前状态选择对应算法）
        Eigen::Matrix4d estimate_motion_by_state(
            const Frame &f1,
            const Frame &f2,
            double &alignment_score
        );

        // 适配器：将各状态的运动估计算法统一为同一签名
        Eigen::Matrix4d estimate_motion_svd_adapter(
            const Frame &f1,
            const Frame &f2,
            double &alignment_score
        );
        Eigen::Matrix4d estimate_motion_gicp_adapter(
            const Frame &f1,
            const Frame &f2,
            double &alignment_score
        );

        // 状态对应的运动估计算法数组
        std::array<MotionEstimatorFn, STATEMAX> motion_estimators_;

        // ==================== 异步回调函数 ====================
        void sync_callback(
            const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
        );
        
        // 提取特征点
        Frame extract_frame_orb(
            const cv::Mat &color_img
        );
        Frame extract_frame_flow(
            const cv::Mat &color_img
        );
        Frame extract_frame(
            const cv::Mat &color_img,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
        );
    
        // Recovery状态的处理函数框架
        // SVD解算位姿
        Eigen::Matrix4d estimate_motion_svd(
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

        // 发布点云到rviz2
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pc_pub_;

        // 数据转换函数
        pcl::PointCloud<pcl::PointXYZ>::Ptr vectorToPcl(
            const std::vector<Eigen::Vector3d> &points
        );

        // 消息同步
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> img_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pc_sub_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        // ==================== GICP相关变量（Tracking状态） ====================
        // 开启滤波器
        bool enable_voxel_filter_ = true;
        double voxel_leaf_size_ = 0.05; // 滤波器的分辨率

        // GICP对齐分数的阈值以及显著运动阈值（用于判断是否需要切换状态）
        double 
            gicp_score_threshold_ = 0.5,
            move_threshold_ = 0.01,
            angle_threshold_ = 0.1;
        
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

        // ==================== 光流跟踪 ====================
        // LK光流
        cv::Ptr<cv::SparsePyrLKOpticalFlow> lk_flow_;
        cv::Mat prev_gray_img_; // 上一帧灰度图
        std::vector<cv::KeyPoint> prev_keypoints_; // 上一帧特征点
        std::vector<Eigen::Vector3d> prev_points_3d_; // 上一帧3D点
        bool is_lk_flow_ = false; // 是否开启LK光流跟踪
        int min_flow_points_ = 100; // 特征点少于这个数量则用orb重新匹配设置初始值
    };
} // nav2_visual_processing

#endif // NAV2_VISUAL_PROCESSING__VISUAL_PROCESSOR