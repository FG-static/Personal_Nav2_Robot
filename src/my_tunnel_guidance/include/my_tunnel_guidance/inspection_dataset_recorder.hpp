#ifndef MY_TUNNEL_GUIDANCE__INSPECTION_DATASET_RECORDER_HPP_
#define MY_TUNNEL_GUIDANCE__INSPECTION_DATASET_RECORDER_HPP_

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace my_tunnel_guidance {

struct InspectionStationSummary {

    int id = -1;
    std::int64_t t_start_ns = 0;
    std::int64_t t_end_ns = 0;
    std::string frame_id;
    std::string cloud_relpath;
    std::size_t point_count = 0;
    std::size_t merged_point_count = 0;
    int scan_count = 0;
    double max_drift_m = 0.0;
    bool saved = false;
};

class InspectionDatasetRecorder {

public:

    bool openMission(
        const std::filesystem::path & root,
        const std::string & frame_id,
        double voxel_size);

    bool isOpen() const { return !root_.empty(); }
    bool stationActive() const { return station_active_; }
    int nextStationId() const { return next_id_; }

    bool beginStation(
        std::int64_t t_start_ns,
        const Eigen::Isometry3d & pose_map_base);

    void addScan(
        const std::vector<Eigen::Vector3d> & map_points,
        const Eigen::Isometry3d & pose_map_base);

    bool finishStation(
        std::int64_t t_end_ns,
        const Eigen::Isometry3d & pose_map_base,
        InspectionStationSummary & summary);

    void abortStation();

    std::size_t mergedPointCount() const { return merged_.size(); }
    const pcl::PointCloud<pcl::PointXYZ> & mergedCloud() const { return merged_; }
    static const char * mergedMapRelpath() { return "map.pcd"; }

private:

    static std::string poseToJson(const Eigen::Isometry3d & pose);
    static std::string cloudRelpath(int id);
    bool writeMissionFile() const;
    bool loadMergedMap();
    bool saveMergedMap();
    bool mergeStationCloud(const pcl::PointCloud<pcl::PointXYZ> & station_cloud);
    int countExistingStations() const;

    std::filesystem::path root_;
    std::string frame_id_;
    double voxel_size_ = 0.03;
    int next_id_ = 0;

    bool station_active_ = false;
    std::int64_t t_start_ns_ = 0;
    Eigen::Isometry3d pose_start_ = Eigen::Isometry3d::Identity();
    double max_drift_m_ = 0.0;
    int scan_count_ = 0;
    std::vector<Eigen::Vector3d> points_;
    pcl::PointCloud<pcl::PointXYZ> merged_;
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__INSPECTION_DATASET_RECORDER_HPP_
