#include "my_nav2_robot/mid360_gz_plugin.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace my_nav2_robot {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kIntersectionEpsilon = 1.0e-9;

template<typename T>
T getSdfValue(
    const std::shared_ptr<const sdf::Element> &sdf,
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

void Mid360GzPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager & /*event_manager*/)
{
    model_entity_ = entity;
    model_ = gz::sim::Model(model_entity_);

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

    tracking_link_entity_ = model_.LinkByName(ecm, tracking_link_name_);
    if (tracking_link_entity_ == gz::sim::kNullEntity) {
        gzerr << "[Mid360GzPlugin] Cannot find tracking link '"
              << tracking_link_name_ << "'" << std::endl;
        return;
    }

    if (!loadScanPattern(csv_file_name_)) {
        gzerr << "[Mid360GzPlugin] Cannot load scan pattern '"
              << csv_file_name_ << "'" << std::endl;
        return;
    }

    if (!rclcpp::ok()) {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>(
        "mid360_gz_plugin_" + std::to_string(model_entity_));
    pointcloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        node_->get_logger(),
        "Loaded %zu Mid360 rays; publishing %s at %.1f Hz with frame %s",
        scan_pattern_.size(), topic_.c_str(), update_rate_, frame_id_.c_str());
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

bool Mid360GzPlugin::isModelDescendant(
    gz::sim::Entity entity,
    const gz::sim::EntityComponentManager &ecm) const
{
    while (entity != gz::sim::kNullEntity) {
        if (entity == model_entity_) {
            return true;
        }
        const auto *parent = ecm.Component<gz::sim::components::ParentEntity>(entity);
        if (parent == nullptr) {
            return false;
        }
        entity = parent->Data();
    }
    return false;
}

std::vector<Mid360GzPlugin::CollisionPrimitive>
Mid360GzPlugin::collectCollisionPrimitives(
    const gz::sim::EntityComponentManager &ecm)
{
    std::vector<CollisionPrimitive> collisions;
    std::size_t unsupported_geometry_count = 0U;

    ecm.Each<gz::sim::components::Collision>(
        [&](const gz::sim::Entity &entity, const gz::sim::components::Collision *) {
            if (isModelDescendant(entity, ecm)) {
                return true;
            }

            const auto *geometry_component =
                ecm.Component<gz::sim::components::Geometry>(entity);
            if (geometry_component == nullptr) {
                return true;
            }

            const sdf::Geometry &geometry = geometry_component->Data();
            CollisionPrimitive primitive;
            primitive.pose = gz::sim::worldPose(entity, ecm);

            if (geometry.Type() == sdf::GeometryType::BOX && geometry.BoxShape()) {
                primitive.type = PrimitiveType::kBox;
                primitive.size = geometry.BoxShape()->Size();
                primitive.bounding_radius = 0.5 * primitive.size.Length();
            } else if (
                geometry.Type() == sdf::GeometryType::PLANE && geometry.PlaneShape())
            {
                primitive.type = PrimitiveType::kPlane;
                primitive.normal = geometry.PlaneShape()->Normal();
            } else if (
                geometry.Type() == sdf::GeometryType::SPHERE && geometry.SphereShape())
            {
                primitive.type = PrimitiveType::kSphere;
                primitive.radius = geometry.SphereShape()->Radius();
                primitive.bounding_radius = primitive.radius;
            } else if (
                geometry.Type() == sdf::GeometryType::CYLINDER && geometry.CylinderShape())
            {
                primitive.type = PrimitiveType::kCylinder;
                primitive.radius = geometry.CylinderShape()->Radius();
                primitive.length = geometry.CylinderShape()->Length();
                primitive.bounding_radius = std::hypot(
                    primitive.radius, 0.5 * primitive.length);
            } else {
                ++unsupported_geometry_count;
                return true;
            }

            collisions.push_back(primitive);
            return true;
        });

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
    const gz::math::Vector3d &origin,
    const gz::math::Vector3d &direction,
    double &distance) const
{
    const gz::math::Vector3d local_origin =
        box.pose.Rot().RotateVectorReverse(origin - box.pose.Pos());
    const gz::math::Vector3d local_direction =
        box.pose.Rot().RotateVectorReverse(direction);
    const gz::math::Vector3d half_size = 0.5 * box.size;

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
    const gz::math::Vector3d &origin,
    const gz::math::Vector3d &direction,
    double &distance) const
{
    const gz::math::Vector3d normal =
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
    const gz::math::Vector3d &origin,
    const gz::math::Vector3d &direction,
    double &distance) const
{
    const gz::math::Vector3d relative_origin = origin - sphere.pose.Pos();
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
    const gz::math::Vector3d &origin,
    const gz::math::Vector3d &direction,
    double &distance) const
{
    const gz::math::Vector3d local_origin =
        cylinder.pose.Rot().RotateVectorReverse(origin - cylinder.pose.Pos());
    const gz::math::Vector3d local_direction =
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
    const gz::math::Vector3d &origin,
    const gz::math::Vector3d &direction,
    const std::vector<CollisionPrimitive> &collisions,
    double &range) const
{
    double nearest = max_range_;
    bool hit = false;

    for (const CollisionPrimitive &collision : collisions) {
        if (collision.bounding_radius > 0.0) {
            const gz::math::Vector3d center_offset = collision.pose.Pos() - origin;
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

void Mid360GzPlugin::PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm)
{
    if (info.paused || !pointcloud_publisher_ || scan_pattern_.empty() ||
        tracking_link_entity_ == gz::sim::kNullEntity)
    {
        return;
    }

    const std::int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        info.simTime).count();
    const std::int64_t publish_period_ns = static_cast<std::int64_t>(
        1000000000.0 / update_rate_);
    const gz::math::Pose3d current_sensor_pose =
        gz::sim::worldPose(tracking_link_entity_, ecm) * sensor_pose_;

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
    const std::vector<CollisionPrimitive> collisions = collectCollisionPrimitives(ecm);
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
        const gz::math::Vector3d position = previous_sensor_pose_.Pos() +
            (current_sensor_pose.Pos() - previous_sensor_pose_.Pos()) * scan_fraction;
        const gz::math::Quaterniond orientation = gz::math::Quaterniond::Slerp(
            scan_fraction, previous_sensor_pose_.Rot(), current_sensor_pose.Rot());
        const gz::math::Pose3d measurement_pose(position, orientation);

        const ScanRay &ray = scan_pattern_[static_cast<std::size_t>(
            index % static_cast<std::int64_t>(scan_pattern_.size()))];
        const gz::math::Vector3d world_direction =
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

        const gz::math::Vector3d local_point = ray.direction * range;
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

GZ_ADD_PLUGIN(
    my_nav2_robot::Mid360GzPlugin,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPostUpdate)
