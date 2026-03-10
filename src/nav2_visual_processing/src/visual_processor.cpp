#include "nav2_visual_processing/visual_processor.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_visual_processing {
    
    VisualProcessorNode::VisualProcessorNode() : Node("visual_processor_node") {

        // 初始化
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        orb_ = cv::ORB::create();
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

        img_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/depth_camera/image");
        pc_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/depth_camera/points");

        aligned_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_points", 10);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *img_sub_, *pc_sub_);
        sync_->registerCallback(std::bind(&VisualProcessorNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 初始化状态机
        current_state_ = RECOVERY;
        motion_estimators_[TRACKING] = &VisualProcessorNode::estimate_motion_gicp_adapter;
        motion_estimators_[RECOVERY] = &VisualProcessorNode::estimate_motion_svd_adapter;

        RCLCPP_INFO(this->get_logger(), "视觉处理节点已启动，等待同步数据...");
    }

    void VisualProcessorNode::publish_tf(const Eigen::Matrix4d &pose, const rclcpp::Time &stamp) {

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = stamp;
        t.header.frame_id = "map";          // 父坐标系
        t.child_frame_id = "camera_link";   // 子坐标系

        // 从 Matrix4d 提取平移
        t.transform.translation.x = pose(0, 3);
        t.transform.translation.y = pose(1, 3);
        t.transform.translation.z = pose(2, 3);

        // 从 Matrix4d 提取旋转并转换为四元数
        Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // 广播
        tf_broadcaster_->sendTransform(t);
    }

    /**
     * @brief 同步回调函数，处理来自深度相机的彩色图像和点云数据
     * @param img_msg 彩色图像消息
     * @param pc_msg 点云消息
     */
    void VisualProcessorNode::sync_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
    ) {
        
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

        // 提取特征和3D点
        Frame cur_f = extract_frame(cv_ptr->image, pc_msg);

        if (!is_first_f_) {

            if (current_state_ == TRACKING) goto GICP; // 直接进入GICP
            if (last_f_.descriptors.empty() || cur_f.descriptors.empty()) { // 检查

                RCLCPP_WARN(this->get_logger(), "特征点不足，跳过此帧解算");
                last_f_ = cur_f; // 更新上一帧，防止卡死
                return;
            }
            GICP:

            double alignment_score = 0.0;
            Eigen::Matrix4d motion = estimate_motion_by_state(last_f_, cur_f, alignment_score);

            // 更新当前位姿（简单累积变换）
            current_pose_ = current_pose_ * motion;

            // 如果在 Tracking 状态且GICP结果变差，触发 Recovery
            if (current_state_ == TRACKING && should_switch_to_recovery()) {

                switch_to_recovery_state();
            } else if (current_state_ == RECOVERY && should_switch_to_tracking(motion)) {

                switch_to_tracking_state();
            }
        }

        last_f_ = cur_f;
        is_first_f_ = false;
    }

    /**
     * @brief 从彩色图像和点云数据中提取特征帧
     * @param color_img 彩色图像
     * @param pc_msg 点云数据
     * @return Frame 包含特征点、描述子和3D点的帧结构
     */
    Frame VisualProcessorNode::extract_frame(
        const cv::Mat &color_img,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg
    ) {
        
        Frame f;
        f.color = color_img;
        
        // 使用ORB算法检测图像中的特征点，并计算描述子-HammingCode
        // f.keypoints: 存储检测到的特征点位置
        // f.descriptors: 存储每个特征点的描述子向量
        if (current_state_ == RECOVERY) {

            orb_->detectAndCompute(color_img, cv::noArray(), f.keypoints, f.descriptors);
        } else {

            orb_->detect(color_img, f.keypoints); // 只检测关键点
            f.descriptors = cv::Mat(); // 空描述子
        }

        // 获取点云的宽高信息
        int width = pc_msg->width;       // 点云的宽度（列数）
        int height = pc_msg->height;     // 点云的高度（行数）

        // 创建点云坐标迭代器，用于按索引访问点云的x, y, z坐标
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pc_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pc_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pc_msg, "z");

        f.points_3d.reserve(f.keypoints.size());

        // 遍历每个检测到的特征点，建立2D-3D对应关系
        for (const auto &kp : f.keypoints) {

            // 将特征点的浮点坐标转换为整数坐标（四舍五入）
            int u = static_cast<int>(kp.pt.x + 0.5);
            int v = static_cast<int>(kp.pt.y + 0.5);

            // 检查特征点坐标是否在点云的有效范围内
            if (u >= 0 && u < static_cast<int>(width) && v >= 0 && v < static_cast<int>(height)) {

                // 计算特征点对应的点云索引（行优先存储）
                int index = v * width + u;
                
                // 从点云中读取对应位置的3D坐标
                float x = iter_x[index];
                float y = iter_y[index];
                float z = iter_z[index];

                // 检查3D坐标是否为有限值（不是NaN或无穷大）
                // 深度相机无法获取深度时通常会返回NaN或Inf
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {

                    // 有效3D点，转换为Eigen向量并添加到结果中
                    f.points_3d.push_back(Eigen::Vector3d(x, y, z));
                } else {

                    // 深度无效，添加零向量占位（保持与特征点一一对应）
                    f.points_3d.push_back(Eigen::Vector3d::Zero());
                }
            } else {

                // 特征点坐标超出点云范围，添加零向量占位
                f.points_3d.push_back(Eigen::Vector3d::Zero());
            }
        }

        return f;
    }

    /**
     * @brief 通过SVD分解估计两帧之间的相机运动
     * @param f1 上一帧
     * @param f2 当前帧
     */
    Eigen::Matrix4d VisualProcessorNode::estimate_motion_svd(
        const Frame &f1,
        const Frame &f2
    ) {
        
        // 存粗描述子匹配信息
        /**
         * @param queryIdx 特征点在第一帧 keypoints 数组中的索引
         * @param trainIdx 特征点在第二帧 keypoints 数组中的索引
         * @param distance 两个描述子之间的距离
         */
        std::vector<cv::DMatch> matches;
        matcher_->match(f1.descriptors, f2.descriptors, matches); // 用hammingcode比较两个描述子

        std::vector<Eigen::Vector3d> pts1, pts2;
        for (auto &m : matches) {

            // 向量模必须不为0,否则被视为无效向量
            if (f1.points_3d[m.queryIdx].norm() > 0 && f2.points_3d[m.trainIdx].norm() > 0) {

                pts1.push_back(f1.points_3d[m.queryIdx]);
                pts2.push_back(f2.points_3d[m.trainIdx]);
            }
        }

        if (pts1.size() < 4) {

            RCLCPP_WARN(this->get_logger(), "匹配点不足（%zu），无法进行SVD解算", pts1.size());
            return Eigen::Matrix4d::Identity();
        }

        // SVD解算
        // 1. 计算质心与去质心坐标
        Eigen::Vector3d p1(0, 0, 0), p2(0, 0, 0);
        for (size_t i = 0; i < pts1.size(); ++ i) {

            p1 += pts1[i];
            p2 += pts2[i];
        }
        p1 /= pts1.size();
        p2 /= pts2.size();

        Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < pts1.size(); ++ i) {

            W += (pts1[i] - p1) * (pts2[i] - p2).transpose(); // 同时存到W矩阵中
        }
        
        // 2. SVD分解
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

        // 负数取反
        if (R.determinant() < 0) R = -R;

        // 3. 计算t
        Eigen::Vector3d t = p1 - R * p2;

        RCLCPP_INFO(this->get_logger(), "SVD解算平移 t: x=%f, y=%f, z=%f", t.x(), t.y(), t.z());

        // 转换李群对应T返回
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;

        // 可视化匹配结果
        cv::Mat match_img = visualize_matches(f1, f2, matches);
        
        // 可选：显示图像（调试用）
        cv::imshow("Feature Matches", match_img);
        cv::waitKey(1);

        return T;
    }

    // 具体算法接口函数
    Eigen::Matrix4d VisualProcessorNode::estimate_motion_svd_adapter(
        const Frame &f1,
        const Frame &f2,
        double &alignment_score
    ) {

        // SVD解算不输出对齐得分，简单置为0
        alignment_score = 0.0;
        return estimate_motion_svd(f1, f2);
    }

    Eigen::Matrix4d VisualProcessorNode::estimate_motion_gicp_adapter(
        const Frame &f1,
        const Frame &f2,
        double &alignment_score
    ) {

        return estimate_motion_with_gicp(f1, f2, alignment_score);
    }

    // 通用状态机接口函数
    Eigen::Matrix4d VisualProcessorNode::estimate_motion_by_state(
        const Frame &f1,
        const Frame &f2,
        double &alignment_score
    ) {

        if (current_state_ < 0 || current_state_ >= STATEMAX) {
            
            alignment_score = 999;
            return Eigen::Matrix4d::Identity();
        }

        auto fn = motion_estimators_[current_state_];
        if (!fn) {

            alignment_score = 999;
            return Eigen::Matrix4d::Identity();
        }

        return (this->*fn)(f1, f2, alignment_score);
    }

    /**
     * @brief 可视化两帧之间的特征点匹配结果
     * @param f1 上一帧（包含图像、特征点、描述子）
     * @param f2 当前帧
     * @param matches 匹配对列表
     * @return cv::Mat 拼接后的匹配结果图像
     */
    cv::Mat VisualProcessorNode::visualize_matches(
        const Frame &f1,
        const Frame &f2,
        const std::vector<cv::DMatch> &matches
    ) {

        cv::Mat match_img;
        cv::drawMatches(
            f1.color, f1.keypoints,
            f2.color, f2.keypoints,
            matches,
            match_img, // 输出图像
            cv::Scalar::all(-1), // 匹配线随机颜色
            cv::Scalar::all(-1), // 单独特征点随机颜色
            std::vector<char>(), // 不屏蔽任何匹配
            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
        );

        return match_img;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr VisualProcessorNode::vectorToPcl(
        const std::vector<Eigen::Vector3d> &points
    ) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(points.size());
        for (const auto &p : points) {

            if (p.norm() > 0) {

                cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
            }
        }
        return cloud;
    }

    bool VisualProcessorNode::should_switch_to_recovery() {
    
        // 在TRACKING状态下，检查GICP对齐分数是否超过阈值
        if (current_state_ == TRACKING) {

            // 如果GICP对齐分数超过阈值，说明追踪效果差，需要切换到Recovery状态重新初始化
            if (last_gicp_score_ > gicp_score_threshold_) {

                RCLCPP_WARN(this->get_logger(), 
                    "GICP对齐分数 %f 超过阈值 %f，需要切换到Recovery状态", 
                    last_gicp_score_, gicp_score_threshold_);
                return true;
            }
        }
        return false;
    }

    bool VisualProcessorNode::should_switch_to_tracking(const Eigen::Matrix4d &motion) {

        // 在RECOVERY状态下，检查SVD结果是否有效
        if (current_state_ == RECOVERY) {

            double translation_mag = motion.block<3, 1>(0, 3).norm(); // 平移距离
            Eigen::AngleAxisd rotation_vector(motion.block<3, 3>(0, 0));
            double rotation_mag = rotation_vector.angle(); // 旋转角度
            if (translation_mag > move_threshold_ || rotation_mag > angle_threshold_) {
                
                RCLCPP_INFO(this->get_logger(), "SVD结果有效，切换到Tracking状态");
                return true;
            }
        }
        return false;
    }

    void VisualProcessorNode::switch_to_tracking_state() {

        // 从Recovery状态切换到Tracking状态
        if (current_state_ != TRACKING) {

            RCLCPP_INFO(this->get_logger(), "切换到TRACKING状态，使用GICP进行精确追踪...");
            
            // 更新状态
            current_state_ = TRACKING;
            
            // 用当前的位姿估计作为GICP的初始值
            initial_pose_for_gicp_ = current_pose_;
            
            // 重置GICP对齐分数
            last_gicp_score_ = 0.0;
        }
    }

    void VisualProcessorNode::switch_to_recovery_state() {

        // 从Tracking状态切换到Recovery状态
        if (current_state_ != RECOVERY) {

            RCLCPP_INFO(this->get_logger(), "切换到RECOVERY状态，使用ORB+SVD进行位姿初始化...");
            
            // 更新状态
            current_state_ = RECOVERY;
            
            // 保存当前帧作为Recovery初始化的参考帧
            recovery_reference_frame_ = last_f_;
            
            // 重置位姿相关变量，为新的位姿计算做准备
            current_pose_ = Eigen::Matrix4d::Identity();
            initial_pose_for_gicp_ = Eigen::Matrix4d::Identity();
            
            // 重置GICP分数
            last_gicp_score_ = 0.0;
            
            RCLCPP_INFO(this->get_logger(), "Recovery状态已激活，等待ORB+SVD计算初始位姿估计...");
        }
    }

    /**
     * @brief 检查GICP对齐结果是否有效
     * @param alignment_score GICP的对齐分数
     * @return bool 如果对齐分数低于阈值（结果有效）返回true，否则返回false
     */
    bool VisualProcessorNode::is_gicp_result_valid(double alignment_score) const {

        // 对齐分数越低说明配准效果越好
        // 如果分数低于阈值，说明GICP配准结果有效
        if (alignment_score < gicp_score_threshold_) {

            return true;
        }
        
        RCLCPP_WARN(this->get_logger(), 
            "GICP配准结果无效：对齐分数 %f 超过阈值 %f", 
            alignment_score, gicp_score_threshold_);
        return false;
    }

    /**
     * @brief 通过全局ICP算法估计两帧之间的相机运动
     * @param f1 上一帧
     * @param f2 当前帧
     * @param alignment_score 全局ICP算法的对齐得分
     * @return Eigen::Matrix4d 全局ICP算法估计的相机运动
     */
    Eigen::Matrix4d VisualProcessorNode::estimate_motion_with_gicp(
        const Frame &f1,
        const Frame &f2,
        double &alignment_score
    ) {

        // 创建源和目标点云
        auto 
            cloud_target = vectorToPcl(f1.points_3d),
            cloud_source = vectorToPcl(f2.points_3d);
        
        if (cloud_target->size() < 3 || cloud_source->size() < 3) {

            RCLCPP_WARN(this->get_logger(), "点云不足，跳过此帧解算");
            alignment_score = 999;
            return Eigen::Matrix4d::Identity();
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr 
                cloud_filtered_target,
                cloud_filtered_source;

        // 体素网格滤波，降采样
        if (enable_voxel_filter_) {

            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.setInputCloud(cloud_target);
            voxel_filter.filter(*cloud_filtered_target_ptr);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_source_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.setInputCloud(cloud_source);
            voxel_filter.filter(*cloud_filtered_source_ptr);

            cloud_filtered_target = cloud_filtered_target_ptr;
            cloud_filtered_source = cloud_filtered_source_ptr;
        } else {

            cloud_filtered_target = cloud_target;
            cloud_filtered_source = cloud_source;
        }

        // 配置RegistrationPCL
        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;

        // 设置参数
        reg.setNumThreads(4); // 配置多线程加速
        reg.setCorrespondenceRandomness(20);
        reg.setMaxCorrespondenceDistance(1.0);
        reg.setRegistrationType("GICP");

        reg.setInputTarget(cloud_filtered_target);
        reg.setInputSource(cloud_filtered_source);

        auto aligned = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // 设置初始值
        Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
        reg.align(*aligned, init_guess); // 开始对齐迭代

        // 获取结果
        if (!reg.hasConverged()) {

            RCLCPP_WARN(this->get_logger(), "GICP 未收敛");
        }

        alignment_score = reg.getFitnessScore(); // 计算align后的对应点间的欧式距离
        last_gicp_score_ = alignment_score; // 作为分数

        // 获取最终变换矩阵
        Eigen::Matrix4f T_f = reg.getFinalTransformation();

        // 发布tf
        auto T_d = T_f.cast<double>();
        publish_tf(T_d, this->get_clock()->now());

        // 发布点云
        sensor_msgs::msg::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned, aligned_msg);
        aligned_msg.header.frame_id = "camera_link"; // 确保与你的 TF 坐标系一致
        aligned_msg.header.stamp = this->now();
        aligned_pc_pub_->publish(aligned_msg);

        auto t = T_d.block<3, 1>(0, 3);
        RCLCPP_INFO(this->get_logger(), "GICP对齐解算平移 t: x=%f, y=%f, z=%f", t.x(), t.y(), t.z());
        return T_d;
    }
} // namespace nav2_visual_processing

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav2_visual_processing::VisualProcessorNode>();

    RCLCPP_INFO(node->get_logger(), "Visual Processor Node is spinning...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
