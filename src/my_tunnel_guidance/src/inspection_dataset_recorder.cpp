#include "my_tunnel_guidance/inspection_dataset_recorder.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace my_tunnel_guidance {

namespace {

constexpr int kChunkSize = 1000;

std::string paddedId(int id, int width) {

    std::ostringstream out;
    out << std::setw(width) << std::setfill('0') << id;
    return out.str();
}

}  // namespace

bool InspectionDatasetRecorder::openMission(
    const std::filesystem::path & root,
    const std::string & frame_id,
    double voxel_size
) {

    abortStation();
    root_.clear();
    merged_.clear();
    if (root.empty() || frame_id.empty() ||
        !std::isfinite(voxel_size) || voxel_size <= 0.0
    ) return false;

    std::error_code error;
    std::filesystem::create_directories(root / "clouds", error);
    if (error) return false;

    root_ = root;
    frame_id_ = frame_id;
    voxel_size_ = voxel_size;
    next_id_ = countExistingStations();
    loadMergedMap();
    return writeMissionFile();
}

bool InspectionDatasetRecorder::beginStation(
    std::int64_t t_start_ns,
    const Eigen::Isometry3d & pose_map_base
) {

    if (!isOpen() || station_active_)
        return false;

    station_active_ = true;
    t_start_ns_ = t_start_ns;
    pose_start_ = pose_map_base;
    max_drift_m_ = 0.0;
    scan_count_ = 0;
    points_.clear();
    return true;
}

void InspectionDatasetRecorder::addScan(
    const std::vector<Eigen::Vector3d> & map_points,
    const Eigen::Isometry3d & pose_map_base
) {

    if (!station_active_)
        return;

    points_.insert(points_.end(), map_points.begin(), map_points.end());
    ++ scan_count_;
    const double drift =
        (pose_map_base.translation() - pose_start_.translation()).norm();
    if (drift > max_drift_m_) {

        max_drift_m_ = drift;
    }
}

bool InspectionDatasetRecorder::finishStation(
    std::int64_t t_end_ns,
    const Eigen::Isometry3d & pose_map_base,
    InspectionStationSummary & summary
) {

    summary = {};
    if (!isOpen() || !station_active_) {

        return false;
    }

    const int id = next_id_;
    const std::string relpath = cloudRelpath(id);
    const std::filesystem::path cloud_path = root_ / relpath;
    std::error_code error;
    std::filesystem::create_directories(cloud_path.parent_path(), error);
    if (error) {

        abortStation();
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ> dense;
    dense.resize(points_.size());
    for (std::size_t i = 0; i < points_.size(); ++ i) {

        dense[i].x = static_cast<float>(points_[i].x());
        dense[i].y = static_cast<float>(points_[i].y());
        dense[i].z = static_cast<float>(points_[i].z());
    }

    pcl::PointCloud<pcl::PointXYZ> filtered;
    if (!dense.empty() && voxel_size_ > 1e-4) {

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_),
            static_cast<float>(voxel_size_));
        voxel.setInputCloud(dense.makeShared());
        voxel.filter(filtered);
    } else {

        filtered = dense;
    }

    if (filtered.empty()) {

        std::ofstream empty_cloud(cloud_path);
        empty_cloud << "# .PCD v0.7 - Point Cloud Data file format\n"
                    << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                    << "COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                    << "POINTS 0\nDATA ascii\n";
        if (!empty_cloud) {

            abortStation();
            return false;
        }
    } else if (pcl::io::savePCDFileBinaryCompressed(
            cloud_path.string(), filtered) != 0
    ) {

        abortStation();
        return false;
    }

    if (!mergeStationCloud(filtered)) {

        abortStation();
        return false;
    }

    std::ofstream jsonl(root_ / "stations.jsonl", std::ios::app);
    if (!jsonl) {

        abortStation();
        return false;
    }
    jsonl << "{\"id\":" << id
          << ",\"t_start_ns\":" << t_start_ns_
          << ",\"t_end_ns\":" << t_end_ns
          << ",\"frame_id\":\"" << frame_id_ << '"'
          << ",\"cloud\":\"" << relpath << '"'
          << ",\"n_points\":" << filtered.size()
          << ",\"n_scans\":" << scan_count_
          << ",\"max_drift_m\":" << max_drift_m_
          << ",\"pose_start\":" << poseToJson(pose_start_)
          << ",\"pose_end\":" << poseToJson(pose_map_base)
          << ",\"map\":\"" << mergedMapRelpath() << '"'
          << ",\"map_points\":" << merged_.size()
          << "}\n";
    jsonl.flush();
    if (!jsonl) {

        abortStation();
        return false;
    }

    summary.id = id;
    summary.t_start_ns = t_start_ns_;
    summary.t_end_ns = t_end_ns;
    summary.frame_id = frame_id_;
    summary.cloud_relpath = relpath;
    summary.point_count = filtered.size();
    summary.merged_point_count = merged_.size();
    summary.scan_count = scan_count_;
    summary.max_drift_m = max_drift_m_;
    summary.saved = true;
    ++ next_id_;
    if (!writeMissionFile()) {

        abortStation();
        return false;
    }
    abortStation();
    return true;
}

void InspectionDatasetRecorder::abortStation() {

    station_active_ = false;
    t_start_ns_ = 0;
    pose_start_ = Eigen::Isometry3d::Identity();
    max_drift_m_ = 0.0;
    scan_count_ = 0;
    points_.clear();
}

std::string InspectionDatasetRecorder::poseToJson(const Eigen::Isometry3d & pose) {

    const Eigen::Quaterniond q(pose.linear());
    const Eigen::Vector3d t = pose.translation();
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out << std::setprecision(9)
        << "{\"px\":" << t.x()
        << ",\"py\":" << t.y()
        << ",\"pz\":" << t.z()
        << ",\"qx\":" << q.x()
        << ",\"qy\":" << q.y()
        << ",\"qz\":" << q.z()
        << ",\"qw\":" << q.w()
        << '}';
    return out.str();
}

std::string InspectionDatasetRecorder::cloudRelpath(int id) {

    const int chunk = id / kChunkSize;
    return "clouds/c" + paddedId(chunk, 3) + "/" + paddedId(id, 6) + ".pcd";
}

bool InspectionDatasetRecorder::writeMissionFile() const {

    std::ofstream out(root_ / "mission.json");
    if (!out) {

        return false;
    }
    out << "{\"frame_id\":\"" << frame_id_ << '"'
        << ",\"voxel_size\":" << voxel_size_
        << ",\"chunk_size\":" << kChunkSize
        << ",\"map\":\"" << mergedMapRelpath() << '"'
        << ",\"map_points\":" << merged_.size()
        << ",\"stations\":" << next_id_
        << "}\n";
    return static_cast<bool>(out);
}

bool InspectionDatasetRecorder::loadMergedMap() {

    merged_.clear();
    const std::filesystem::path path = root_ / mergedMapRelpath();
    if (!std::filesystem::exists(path)) {

        return true;
    }
    return pcl::io::loadPCDFile(path.string(), merged_) >= 0;
}

bool InspectionDatasetRecorder::saveMergedMap() {

    const std::filesystem::path path = root_ / mergedMapRelpath();
    if (merged_.empty()) {

        std::ofstream empty_cloud(path);
        empty_cloud << "# .PCD v0.7 - Point Cloud Data file format\n"
                    << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                    << "COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
                    << "POINTS 0\nDATA ascii\n";
        return static_cast<bool>(empty_cloud);
    }
    return pcl::io::savePCDFileBinaryCompressed(path.string(), merged_) == 0;
}

bool InspectionDatasetRecorder::mergeStationCloud(
    const pcl::PointCloud<pcl::PointXYZ> & station_cloud
) {

    if (!station_cloud.empty()) {

        merged_ += station_cloud;
        if (voxel_size_ > 1e-4) {

            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setLeafSize(
                static_cast<float>(voxel_size_),
                static_cast<float>(voxel_size_),
                static_cast<float>(voxel_size_));
            voxel.setInputCloud(merged_.makeShared());
            pcl::PointCloud<pcl::PointXYZ> filtered;
            voxel.filter(filtered);
            merged_.swap(filtered);
        }
    }
    if (!saveMergedMap()) {

        return false;
    }
    return true;
}

int InspectionDatasetRecorder::countExistingStations() const {

    std::ifstream in(root_ / "stations.jsonl");
    if (!in) {

        return 0;
    }
    int count = 0;
    std::string line;
    while (std::getline(in, line)) {

        if (!line.empty()) {

            ++ count;
        }
    }
    return count;
}

}  // namespace my_tunnel_guidance
