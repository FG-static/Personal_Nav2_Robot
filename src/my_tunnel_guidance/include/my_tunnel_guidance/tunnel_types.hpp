#ifndef MY_TUNNEL_GUIDANCE__TUNNEL_TYPES_HPP_
#define MY_TUNNEL_GUIDANCE__TUNNEL_TYPES_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <limits>
#include <vector>

namespace my_tunnel_guidance {

// PCA 结果存储
struct PlaneEstimate {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero(); // 质心
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero(); // 特征值
    double offset = 0.0; // 平面方程中的常数项
    double rmse = std::numeric_limits<double>::infinity(); // 点云到拟合平面的均方根误差
    std::size_t point_count = 0;
    bool valid = false;
};

struct TunnelFrameEstimate {
    Eigen::Vector3d tangent = Eigen::Vector3d::Zero(); // 涵洞纵向方向，机器人前进方向
    Eigen::Vector3d lateral = Eigen::Vector3d::Zero(); // 左右墙间横向方向
    Eigen::Vector3d up = Eigen::Vector3d::Zero(); // 地面法向量方向
    Eigen::Vector3d center = Eigen::Vector3d::Zero(); // 在地面上横墙中点
    double width = 0.0;
    double confidence = 0.0; // 置信度，由特征值计算
    bool valid = false;
};

struct CenterlineEstimate {
    std::vector<Eigen::Vector3d> points; // 中心线上的一系列离散点
    std::vector<Eigen::Vector3d> tangents;
    TunnelFrameEstimate local_frame; // 隧道局部坐标系
    bool valid = false;
};

// 在 odom 坐标系下维护的直涵洞墙体模型。
// left_l/right_l 是沿 lateral 轴到 model.center 的横向坐标，
// 初始标定后保持不变，后续只做慢更新。
struct TunnelWallModel {

    bool initialized = false;
    Eigen::Vector3d tangent = Eigen::Vector3d::UnitX();
    Eigen::Vector3d lateral = Eigen::Vector3d::UnitY();
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double left_l = 2.0;
    double right_l = -2.0;
    double width = 4.0;
};

}  // namespace my_tunnel_guidance

#endif  // MY_TUNNEL_GUIDANCE__TUNNEL_TYPES_HPP_
