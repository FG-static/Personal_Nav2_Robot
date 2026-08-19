#ifndef MY_NAV2_ROBOT__MID360_GZ_PLUGIN_HPP
#define MY_NAV2_ROBOT__MID360_GZ_PLUGIN_HPP

#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace my_nav2_robot {

class Mid360GzPlugin : public gazebo::ModelPlugin {
public:
    Mid360GzPlugin();
    ~Mid360GzPlugin() override;

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
    struct ScanRay {
        ignition::math::Vector3d direction;
    };

    struct PointSample {
        float x = 0.0F;
        float y = 0.0F;
        float z = 0.0F;
        float intensity = 0.0F;
        std::uint32_t time_offset_ns = 0U;
    };

    enum class PrimitiveType {
        kBox,
        kPlane,
        kSphere,
        kCylinder
    };

    struct CollisionPrimitive {
        PrimitiveType type = PrimitiveType::kBox;
        ignition::math::Pose3d pose;
        ignition::math::Vector3d size;
        ignition::math::Vector3d normal = ignition::math::Vector3d::UnitZ;
        double radius = 0.0;
        double length = 0.0;
        double bounding_radius = -1.0;
    };

    void OnUpdate(const gazebo::common::UpdateInfo &info);
    std::string resolveCsvPath(const std::string &file_name) const;
    bool loadScanPattern(const std::string &file_name);
    std::vector<CollisionPrimitive> collectCollisionPrimitives();
    bool raycast(
        const ignition::math::Vector3d &origin,
        const ignition::math::Vector3d &direction,
        const std::vector<CollisionPrimitive> &collisions,
        double &range) const;
    bool intersectBox(
        const CollisionPrimitive &box,
        const ignition::math::Vector3d &origin,
        const ignition::math::Vector3d &direction,
        double &distance) const;
    bool intersectPlane(
        const CollisionPrimitive &plane,
        const ignition::math::Vector3d &origin,
        const ignition::math::Vector3d &direction,
        double &distance) const;
    bool intersectSphere(
        const CollisionPrimitive &sphere,
        const ignition::math::Vector3d &origin,
        const ignition::math::Vector3d &direction,
        double &distance) const;
    bool intersectCylinder(
        const CollisionPrimitive &cylinder,
        const ignition::math::Vector3d &origin,
        const ignition::math::Vector3d &direction,
        double &distance) const;
    void publishPointCloud(
        const std::vector<PointSample> &points,
        std::int64_t scan_start_ns) const;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::physics::LinkPtr tracking_link_;
    gazebo::event::ConnectionPtr update_connection_;

    std::string tracking_link_name_ = "livox_frame";
    std::string frame_id_ = "livox_frame";
    std::string topic_ = "/livox/lidar";
    std::string csv_file_name_;
    ignition::math::Pose3d sensor_pose_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int samples_per_scan_ = 20000;
    int downsample_ = 1;
    double update_rate_ = 10.0;
    double min_range_ = 0.2;
    double max_range_ = 12.0;
    double range_noise_stddev_ = 0.005;
    double dropout_rate_ = 0.0;
    int noise_seed_ = 42;

    std::vector<ScanRay> scan_pattern_;
    std::int64_t pattern_start_index_ = 0;
    std::int64_t last_publish_ns_ = -1;
    ignition::math::Pose3d previous_sensor_pose_;
    bool have_previous_pose_ = false;
    bool warned_unsupported_geometry_ = false;
    bool warned_empty_scan_ = false;

    std::mt19937 random_engine_;
    std::normal_distribution<double> range_noise_{0.0, 0.005};
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};

    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};

}  // namespace my_nav2_robot

#endif  // MY_NAV2_ROBOT__MID360_GZ_PLUGIN_HPP
