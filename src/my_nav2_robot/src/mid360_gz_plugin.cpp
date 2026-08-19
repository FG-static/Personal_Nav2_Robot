#include "my_nav2_robot/mid360_gz_plugin.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/pointer_cast.hpp>
#include <gazebo/common/Console.hh>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace my_nav2_robot {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kIntersectionEpsilon = 1.0e-9;

template<typename T>
T getSdfValue(
    const sdf::ElementPtr &sdf,
    const std::string &name,
    const T &default_value)
{
    if (!sdf || !sdf->HasElement(name)) {
        return default_value;
    }
    return sdf->Get<T>(name);
}

double smallestPositiveRoot(double first, double second)
{
    double result = std::numeric_limits<double>::infinity();
    if (first > kIntersectionEpsilon) {
        result = first;
    }
    if (second > kIntersectionEpsilon) {
        result = std::min(result, second);
    }
    return result;
}

}  // namespace

Mid360GzPlugin::Mid360GzPlugin() = default;

Mid360GzPlugin::~Mid360GzPlugin() = default;

void Mid360GzPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    model_ = model;
    if (!model_) {
        gzerr << "[Mid360GzPlugin] Load() received a null model" << std::endl;
        return;
    }
    world_ = model_->GetWorld();

    tracking_link_name_ = getSdfValue(sdf, "tracking_link_name", tracking_link_name_);
    frame_id_ = getSdfValue(sdf, "frame_id", frame_id_);
    topic_ = getSdfValue(sdf, "topic", topic_);
    csv_file_name_ = getSdfValue(sdf, "csv_file_name", csv_file_name_);
    sensor_pose_ = getSdfValue(sdf, "sensor_pose", sensor_pose_);
    samples_per_scan_ = std::max(1, getSdfValue(sdf, "samples_per_scan", samples_per_scan_));
    downsample_ = std::max(1, getSdfValue(sdf, "downsample", downsample_));
    update_rate_ = getSdfValue(sdf, "update_rate", update_rate_);
    min_range_ = getSdfValue(sdf, "min_range", min_range_);
    max_range_ = getSdfValue(sdf, "max_range", max_range_);
    range_noise_stddev_ = std::max(
        0.0, getSdfValue(sdf, "range_noise_stddev", range_noise_stddev_));
    dropout_rate_ = std::clamp(
        getSdfValue(sdf, "dropout_rate", dropout_rate_), 0.0, 1.0);
    noise_seed_ = getSdfValue(sdf, "noise_seed", noise_seed_);

    if (!std::isfinite(update_rate_) || update_rate_ <= 0.0) {
        update_rate_ = 10.0;
    }
    if (!std::isfinite(min_range_) || !std::isfinite(max_range_) ||
        min_range_ < 0.0 || max_range_ <= min_range_)
    {
        min_range_ = 0.2;
        max_range_ = 12.0;
    }

    random_engine_.seed(static_cast<std::mt19937::result_type>(noise_seed_));
    range_noise_ = std::normal_distribution<double>(0.0, range_noise_stddev_);

    tracking_link_ = model_->GetLink(tracking_link_name_);
    if (!tracking_link_) {
        gzerr << "[Mid360GzPlugin] Cannot find tracking link '"
              << tracking_link_name_ << "'" << std::endl;
        return;
    }

    const std::string resolved_csv = resolveCsvPath(csv_file_name_);
    if (!loadScanPattern(resolved_csv)) {
        gzwarn << "[Mid360GzPlugin] Scan CSV '" << csv_file_name_
               << "' (resolved '" << resolved_csv
               << "') is missing; using a built-in Mid360 FOV fallback"
               << std::endl;
        if (!loadFallbackScanPattern()) {
            gzerr << "[Mid360GzPlugin] Cannot build a fallback scan pattern"
                  << std::endl;
            return;
        }
    }

    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) {
        gzerr << "[Mid360GzPlugin] gazebo_ros::Node::Get() returned null"
              << std::endl;
        return;
    }
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_, rclcpp::SensorDataQoS());

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&Mid360GzPlugin::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
        node_->get_logger(),
        "Loaded %zu Mid360 rays; publishing %s at %.1f Hz with frame %s",
        scan_pattern_.size(), topic_.c_str(), update_rate_, frame_id_.c_str());
}

std::string Mid360GzPlugin::resolveCsvPath(const std::string &file_name) const
{
    if (!file_name.empty()) {
        std::ifstream direct(file_name);
        if (direct.good()) {
            return file_name;
        }
    }

    const std::string package_uri = "package://my_nav2_robot/";
    if (file_name.rfind(package_uri, 0) == 0) {
        try {
            const std::string share =
                ament_index_cpp::get_package_share_directory("my_nav2_robot");
            return share + "/" + file_name.substr(package_uri.size());
        } catch (const std::exception &) {
        }
    }

    try {
        const std::string share =
            ament_index_cpp::get_package_share_directory("my_nav2_robot");
        const std::string candidate = share + "/config/mid360.csv";
        std::ifstream share_file(candidate);
        if (share_file.good()) {
            return candidate;
        }
    } catch (const std::exception &) {
    }

    return file_name;
}

bool Mid360GzPlugin::loadScanPattern(const std::string &file_name)
{
    if (file_name.empty()) {
        return false;
    }

    std::ifstream input(file_name);
    if (!input.is_open()) {
        return false;
    }

    scan_pattern_.clear();
    scan_pattern_.reserve(800000U);
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        std::istringstream stream(line);
        std::string time_text;
        std::string azimuth_text;
        std::string polar_angle_text;
        if (!std::getline(stream, time_text, ',') ||
            !std::getline(stream, azimuth_text, ',') ||
            !std::getline(stream, polar_angle_text, ','))
        {
            continue;
        }

        try {
            const double azimuth = std::stod(azimuth_text) * kDegreesToRadians;
            const double elevation =
                (90.0 - std::stod(polar_angle_text)) * kDegreesToRadians;
            const double horizontal_scale = std::cos(elevation);
            ScanRay ray;
            ray.direction.Set(
                horizontal_scale * std::cos(azimuth),
                horizontal_scale * std::sin(azimuth),
                std::sin(elevation));
            ray.direction.Normalize();
            scan_pattern_.push_back(ray);
        } catch (const std::exception &) {
            continue;
        }
    }

    return !scan_pattern_.empty();
}

bool Mid360GzPlugin::loadFallbackScanPattern()
{
    constexpr int kAzimuthBins = 400;
    constexpr int kElevationBins = 50;
    constexpr double kMinElevationDeg = -7.0;
    constexpr double kMaxElevationDeg = 52.0;
    scan_pattern_.clear();
    scan_pattern_.reserve(static_cast<std::size_t>(kAzimuthBins * kElevationBins));
    for (int elevation_index = 0; elevation_index < kElevationBins; ++elevation_index) {
        const double elevation = (kMinElevationDeg +
            (kMaxElevationDeg - kMinElevationDeg) *
            static_cast<double>(elevation_index) /
            static_cast<double>(kElevationBins - 1)) * kDegreesToRadians;
        const double horizontal_scale = std::cos(elevation);
        for (int azimuth_index = 0; azimuth_index < kAzimuthBins; ++azimuth_index) {
            const double azimuth = 2.0 * M_PI *
                static_cast<double>(azimuth_index) /
                static_cast<double>(kAzimuthBins);
            ScanRay ray;
            ray.direction.Set(
                horizontal_scale * std::cos(azimuth),
                horizontal_scale * std::sin(azimuth),
                std::sin(elevation));
            ray.direction.Normalize();
            scan_pattern_.push_back(ray);
        }
    }
    return !scan_pattern_.empty();
}

std::vector<Mid360GzPlugin::CollisionPrimitive>
Mid360GzPlugin::collectCollisionPrimitives()
{
    std::vector<CollisionPrimitive> collisions;
    std::size_t unsupported_geometry_count = 0U;
    if (!world_) {
        return collisions;
    }

    for (const gazebo::physics::ModelPtr &world_model : world_->Models()) {
        if (!world_model || world_model == model_) {
            continue;
        }

        for (const gazebo::physics::LinkPtr &link : world_model->GetLinks()) {
            if (!link) {
                continue;
            }
            for (const gazebo::physics::CollisionPtr &collision : link->GetCollisions()) {
                if (!collision) {
                    continue;
                }
                const gazebo::physics::ShapePtr shape = collision->GetShape();
                if (!shape) {
                    continue;
                }

                CollisionPrimitive primitive;
                primitive.pose = collision->WorldPose();

                if (const auto box =
                    boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape))
                {
                    primitive.type = PrimitiveType::kBox;
                    primitive.size = box->Size();
                    primitive.bounding_radius = 0.5 * primitive.size.Length();
                } else if (const auto plane =
                    boost::dynamic_pointer_cast<gazebo::physics::PlaneShape>(shape))
                {
                    primitive.type = PrimitiveType::kPlane;
                    primitive.normal = plane->Normal();
                } else if (const auto sphere =
                    boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape))
                {
                    primitive.type = PrimitiveType::kSphere;
                    primitive.radius = sphere->GetRadius();
                    primitive.bounding_radius = primitive.radius;
                } else if (const auto cylinder =
                    boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape))
                {
                    primitive.type = PrimitiveType::kCylinder;
                    primitive.radius = cylinder->GetRadius();
                    primitive.length = cylinder->GetLength();
                    primitive.bounding_radius = std::hypot(
                        primitive.radius, 0.5 * primitive.length);
                } else {
                    ++unsupported_geometry_count;
                    continue;
                }

                collisions.push_back(primitive);
            }
        }
    }

    if (unsupported_geometry_count > 0U && !warned_unsupported_geometry_ && node_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Mid360 raycaster ignored %zu unsupported collision geometries; "
            "use box, plane, sphere, or cylinder collisions for LIO-visible obstacles",
            unsupported_geometry_count);
        warned_unsupported_geometry_ = true;
    }
    return collisions;
}

bool Mid360GzPlugin::intersectBox(
    const CollisionPrimitive &box,
    const ignition::math::Vector3d &origin,
    const ignition::math::Vector3d &direction,
    double &distance) const
{
    const ignition::math::Vector3d local_origin =
        box.pose.Rot().RotateVectorReverse(origin - box.pose.Pos());
    const ignition::math::Vector3d local_direction =
        box.pose.Rot().RotateVectorReverse(direction);
    const ignition::math::Vector3d half_size = 0.5 * box.size;

    const std::array<double, 3> lower = {
        -half_size.X(), -half_size.Y(), -half_size.Z()};
    const std::array<double, 3> upper = {
        half_size.X(), half_size.Y(), half_size.Z()};
    const std::array<double, 3> ray_origin = {
        local_origin.X(), local_origin.Y(), local_origin.Z()};
    const std::array<double, 3> ray_direction = {
        local_direction.X(), local_direction.Y(), local_direction.Z()};

    double enter = -std::numeric_limits<double>::infinity();
    double exit = std::numeric_limits<double>::infinity();
    for (std::size_t axis = 0U; axis < ray_origin.size(); ++axis) {
        if (std::abs(ray_direction[axis]) < kIntersectionEpsilon) {
            if (ray_origin[axis] < lower[axis] || ray_origin[axis] > upper[axis]) {
                return false;
            }
            continue;
        }

        double first = (lower[axis] - ray_origin[axis]) / ray_direction[axis];
        double second = (upper[axis] - ray_origin[axis]) / ray_direction[axis];
        if (first > second) {
            std::swap(first, second);
        }
        enter = std::max(enter, first);
        exit = std::min(exit, second);
        if (enter > exit) {
            return false;
        }
    }

    distance = enter > kIntersectionEpsilon ? enter : exit;
    return distance > kIntersectionEpsilon;
}

bool Mid360GzPlugin::intersectPlane(
    const CollisionPrimitive &plane,
    const ignition::math::Vector3d &origin,
    const ignition::math::Vector3d &direction,
    double &distance) const
{
    const ignition::math::Vector3d normal =
        plane.pose.Rot().RotateVector(plane.normal).Normalized();
    const double denominator = normal.Dot(direction);
    if (std::abs(denominator) < kIntersectionEpsilon) {
        return false;
    }
    distance = normal.Dot(plane.pose.Pos() - origin) / denominator;
    return distance > kIntersectionEpsilon;
}

bool Mid360GzPlugin::intersectSphere(
    const CollisionPrimitive &sphere,
    const ignition::math::Vector3d &origin,
    const ignition::math::Vector3d &direction,
    double &distance) const
{
    const ignition::math::Vector3d relative_origin = origin - sphere.pose.Pos();
    const double projection = relative_origin.Dot(direction);
    const double constant = relative_origin.SquaredLength() -
        sphere.radius * sphere.radius;
    const double discriminant = projection * projection - constant;
    if (discriminant < 0.0) {
        return false;
    }

    const double root = std::sqrt(discriminant);
    distance = smallestPositiveRoot(-projection - root, -projection + root);
    return std::isfinite(distance);
}

bool Mid360GzPlugin::intersectCylinder(
    const CollisionPrimitive &cylinder,
    const ignition::math::Vector3d &origin,
    const ignition::math::Vector3d &direction,
    double &distance) const
{
    const ignition::math::Vector3d local_origin =
        cylinder.pose.Rot().RotateVectorReverse(origin - cylinder.pose.Pos());
    const ignition::math::Vector3d local_direction =
        cylinder.pose.Rot().RotateVectorReverse(direction);
    const double half_length = 0.5 * cylinder.length;
    double best = std::numeric_limits<double>::infinity();

    const double quadratic =
        local_direction.X() * local_direction.X() +
        local_direction.Y() * local_direction.Y();
    if (quadratic > kIntersectionEpsilon) {
        const double linear = 2.0 * (
            local_origin.X() * local_direction.X() +
            local_origin.Y() * local_direction.Y());
        const double constant =
            local_origin.X() * local_origin.X() +
            local_origin.Y() * local_origin.Y() -
            cylinder.radius * cylinder.radius;
        const double discriminant = linear * linear - 4.0 * quadratic * constant;
        if (discriminant >= 0.0) {
            const double root = std::sqrt(discriminant);
            const double candidates[2] = {
                (-linear - root) / (2.0 * quadratic),
                (-linear + root) / (2.0 * quadratic)};
            for (const double candidate : candidates) {
                const double z = local_origin.Z() + candidate * local_direction.Z();
                if (candidate > kIntersectionEpsilon && std::abs(z) <= half_length) {
                    best = std::min(best, candidate);
                }
            }
        }
    }

    if (std::abs(local_direction.Z()) > kIntersectionEpsilon) {
        for (const double cap_z : {-half_length, half_length}) {
            const double candidate =
                (cap_z - local_origin.Z()) / local_direction.Z();
            if (candidate <= kIntersectionEpsilon) {
                continue;
            }
            const double x = local_origin.X() + candidate * local_direction.X();
            const double y = local_origin.Y() + candidate * local_direction.Y();
            if (x * x + y * y <= cylinder.radius * cylinder.radius) {
                best = std::min(best, candidate);
            }
        }
    }

    distance = best;
    return std::isfinite(distance);
}

bool Mid360GzPlugin::raycast(
    const ignition::math::Vector3d &origin,
    const ignition::math::Vector3d &direction,
    const std::vector<CollisionPrimitive> &collisions,
    double &range) const
{
    double nearest = max_range_;
    bool hit = false;

    for (const CollisionPrimitive &collision : collisions) {
        if (collision.bounding_radius > 0.0) {
            const ignition::math::Vector3d center_offset = collision.pose.Pos() - origin;
            const double along_ray = center_offset.Dot(direction);
            if (along_ray + collision.bounding_radius < min_range_ ||
                along_ray - collision.bounding_radius > nearest)
            {
                continue;
            }
            const double perpendicular_squared =
                center_offset.SquaredLength() - along_ray * along_ray;
            if (perpendicular_squared >
                collision.bounding_radius * collision.bounding_radius)
            {
                continue;
            }
        }

        double candidate = 0.0;
        bool intersects = false;
        switch (collision.type) {
            case PrimitiveType::kBox:
                intersects = intersectBox(collision, origin, direction, candidate);
                break;
            case PrimitiveType::kPlane:
                intersects = intersectPlane(collision, origin, direction, candidate);
                break;
            case PrimitiveType::kSphere:
                intersects = intersectSphere(collision, origin, direction, candidate);
                break;
            case PrimitiveType::kCylinder:
                intersects = intersectCylinder(collision, origin, direction, candidate);
                break;
        }

        if (intersects && candidate >= min_range_ && candidate < nearest) {
            nearest = candidate;
            hit = true;
        }
    }

    range = nearest;
    return hit;
}

void Mid360GzPlugin::publishPointCloud(
    const std::vector<PointSample> &points,
    const std::int64_t scan_start_ns) const
{
    if (!pointcloud_publisher_ || points.empty()) {
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp.sec = static_cast<std::int32_t>(
        scan_start_ns / 1000000000LL);
    cloud.header.stamp.nanosec = static_cast<std::uint32_t>(
        scan_start_ns % 1000000000LL);
    cloud.height = 1U;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "t", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity_iterator(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> time_iterator(cloud, "t");
    for (const PointSample &point : points) {
        *x_iterator = point.x;
        *y_iterator = point.y;
        *z_iterator = point.z;
        *intensity_iterator = point.intensity;
        *time_iterator = point.time_offset_ns;
        ++x_iterator;
        ++y_iterator;
        ++z_iterator;
        ++intensity_iterator;
        ++time_iterator;
    }

    pointcloud_publisher_->publish(cloud);
}

void Mid360GzPlugin::OnUpdate(const gazebo::common::UpdateInfo &info)
{
    if ((world_ && world_->IsPaused()) || !pointcloud_publisher_ ||
        scan_pattern_.empty() || !tracking_link_)
    {
        return;
    }

    const std::int64_t now_ns =
        static_cast<std::int64_t>(info.simTime.sec) * 1000000000LL +
        static_cast<std::int64_t>(info.simTime.nsec);
    const std::int64_t publish_period_ns = static_cast<std::int64_t>(
        1000000000.0 / update_rate_);
    const ignition::math::Pose3d current_sensor_pose =
        tracking_link_->WorldPose() * sensor_pose_;

    if (last_publish_ns_ >= now_ns) {
        last_publish_ns_ = -1;
        have_previous_pose_ = false;
    }
    if (!have_previous_pose_) {
        previous_sensor_pose_ = current_sensor_pose;
        last_publish_ns_ = now_ns;
        have_previous_pose_ = true;
        return;
    }
    if (now_ns - last_publish_ns_ < publish_period_ns) {
        return;
    }

    const std::int64_t scan_duration_ns = now_ns - last_publish_ns_;
    const int scheduled_point_count =
        (samples_per_scan_ + downsample_ - 1) / downsample_;
    const std::vector<CollisionPrimitive> collisions = collectCollisionPrimitives();
    std::vector<PointSample> points;
    points.reserve(static_cast<std::size_t>(scheduled_point_count));

    int scheduled_index = 0;
    const std::int64_t pattern_end = pattern_start_index_ + samples_per_scan_;
    for (std::int64_t index = pattern_start_index_;
        index < pattern_end;
        index += downsample_, ++scheduled_index)
    {
        const double scan_fraction = scheduled_point_count > 1 ?
            static_cast<double>(scheduled_index) /
            static_cast<double>(scheduled_point_count - 1) : 0.0;
        const ignition::math::Vector3d position = previous_sensor_pose_.Pos() +
            (current_sensor_pose.Pos() - previous_sensor_pose_.Pos()) * scan_fraction;
        const ignition::math::Quaterniond orientation = ignition::math::Quaterniond::Slerp(
            scan_fraction, previous_sensor_pose_.Rot(), current_sensor_pose.Rot());
        const ignition::math::Pose3d measurement_pose(position, orientation);

        const ScanRay &ray = scan_pattern_[static_cast<std::size_t>(
            index % static_cast<std::int64_t>(scan_pattern_.size()))];
        const ignition::math::Vector3d world_direction =
            measurement_pose.Rot().RotateVector(ray.direction).Normalized();

        double range = 0.0;
        if (!raycast(measurement_pose.Pos(), world_direction, collisions, range) ||
            uniform_(random_engine_) < dropout_rate_)
        {
            continue;
        }
        if (range_noise_stddev_ > 0.0) {
            range += range_noise_(random_engine_);
        }
        if (!std::isfinite(range) || range < min_range_ || range >= max_range_) {
            continue;
        }

        const ignition::math::Vector3d local_point = ray.direction * range;
        const std::int64_t point_offset_ns = static_cast<std::int64_t>(
            std::llround(scan_fraction * static_cast<double>(scan_duration_ns)));
        PointSample point;
        point.x = static_cast<float>(local_point.X());
        point.y = static_cast<float>(local_point.Y());
        point.z = static_cast<float>(local_point.Z());
        point.intensity = static_cast<float>(
            std::clamp(255.0 * (1.0 - range / max_range_), 1.0, 255.0));
        point.time_offset_ns = static_cast<std::uint32_t>(std::clamp<std::int64_t>(
            point_offset_ns, 0, std::numeric_limits<std::uint32_t>::max()));
        points.push_back(point);
    }

    pattern_start_index_ = pattern_end %
        static_cast<std::int64_t>(scan_pattern_.size());
    if (points.empty() && !warned_empty_scan_ && node_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Mid360 scan has no valid collision returns; check world collision geometry");
        warned_empty_scan_ = true;
    }
    publishPointCloud(points, last_publish_ns_);

    previous_sensor_pose_ = current_sensor_pose;
    last_publish_ns_ = now_ns;
}

}  // namespace my_nav2_robot

GZ_REGISTER_MODEL_PLUGIN(my_nav2_robot::Mid360GzPlugin)
