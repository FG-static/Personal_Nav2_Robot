#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "my_tunnel_guidance/inspection_dataset_recorder.hpp"

TEST(InspectionDatasetRecorder, WritesFlatIndexAndChunkedCloud)
{
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "inspection_dataset_recorder_test";
    std::filesystem::remove_all(root);

    my_tunnel_guidance::InspectionDatasetRecorder recorder;
    ASSERT_TRUE(recorder.openMission(root, "map", 0.05));

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    ASSERT_TRUE(recorder.beginStation(1000, pose));
    recorder.addScan(
        {Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(1.02, 0.0, 0.0)}, pose);
    pose.translation().x() = 0.03;
    recorder.addScan({Eigen::Vector3d(2.0, 0.1, 0.0)}, pose);

    my_tunnel_guidance::InspectionStationSummary summary;
    ASSERT_TRUE(recorder.finishStation(2000, pose, summary));
    EXPECT_EQ(summary.id, 0);
    EXPECT_EQ(summary.t_start_ns, 1000);
    EXPECT_EQ(summary.t_end_ns, 2000);
    EXPECT_EQ(summary.cloud_relpath, "clouds/c000/000000.pcd");
    EXPECT_GE(summary.point_count, 1U);
    EXPECT_EQ(summary.scan_count, 2);
    EXPECT_NEAR(summary.max_drift_m, 0.03, 1e-9);
    EXPECT_TRUE(std::filesystem::exists(root / summary.cloud_relpath));
    EXPECT_TRUE(std::filesystem::exists(root / "stations.jsonl"));
    EXPECT_TRUE(std::filesystem::exists(root / "mission.json"));
    EXPECT_TRUE(std::filesystem::exists(root / "map.pcd"));

    std::ifstream jsonl(root / "stations.jsonl");
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(jsonl, line)));
    EXPECT_NE(line.find("\"id\":0"), std::string::npos);
    EXPECT_NE(line.find("\"t_start_ns\":1000"), std::string::npos);

    ASSERT_TRUE(recorder.beginStation(3000, Eigen::Isometry3d::Identity()));
    recorder.addScan({Eigen::Vector3d(0.0, 0.0, 1.0)}, Eigen::Isometry3d::Identity());
    my_tunnel_guidance::InspectionStationSummary second;
    ASSERT_TRUE(recorder.finishStation(4000, Eigen::Isometry3d::Identity(), second));
    EXPECT_EQ(second.id, 1);
    EXPECT_EQ(second.cloud_relpath, "clouds/c000/000001.pcd");
    EXPECT_GE(second.merged_point_count, 2U);
    EXPECT_GE(recorder.mergedPointCount(), 2U);

    pcl::PointCloud<pcl::PointXYZ> merged;
    ASSERT_GE(pcl::io::loadPCDFile((root / "map.pcd").string(), merged), 0);
    EXPECT_GE(merged.size(), 2U);
    bool saw_first_station = false;
    bool saw_second_station = false;
    for (const auto & point : merged) {
        if (std::abs(point.x - 1.0F) < 0.1F) {
            saw_first_station = true;
        }
        if (std::abs(point.z - 1.0F) < 0.1F) {
            saw_second_station = true;
        }
    }
    EXPECT_TRUE(saw_first_station);
    EXPECT_TRUE(saw_second_station);

    std::filesystem::remove_all(root);
}

TEST(InspectionDatasetRecorder, ReopenContinuesStationIds)
{
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "inspection_dataset_recorder_reopen";
    std::filesystem::remove_all(root);

    {
        my_tunnel_guidance::InspectionDatasetRecorder recorder;
        ASSERT_TRUE(recorder.openMission(root, "map", 0.1));
        ASSERT_TRUE(recorder.beginStation(1, Eigen::Isometry3d::Identity()));
        my_tunnel_guidance::InspectionStationSummary summary;
        ASSERT_TRUE(recorder.finishStation(2, Eigen::Isometry3d::Identity(), summary));
        EXPECT_EQ(summary.id, 0);
    }

    my_tunnel_guidance::InspectionDatasetRecorder recorder;
    ASSERT_TRUE(recorder.openMission(root, "map", 0.1));
    EXPECT_EQ(recorder.nextStationId(), 1);
    std::filesystem::remove_all(root);
}
