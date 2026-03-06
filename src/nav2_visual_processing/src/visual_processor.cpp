#include "nav2_visual_processing/visual_processor.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nav2_visual_processing {
    
    VisualProcessorNode::VisualProcessorNode() : Node("visual_processor_node") {

        // 初始化
        orb_ = cv::ORB::create();
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

        img_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/depth_camera/image");
        pc_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/depth_camera/points");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *img_sub_, *pc_sub_);
        sync_->registerCallback(std::bind(&VisualProcessorNode::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "视觉处理节点已启动，等待同步数据...");
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

            if (last_f_.descriptors.empty() || cur_f.descriptors.empty()) { // 检查

                RCLCPP_WARN(this->get_logger(), "特征点不足，跳过此帧解算");
                last_f_ = cur_f; // 更新上一帧，防止卡死
                return;
            }
            estimate_motion(last_f_, cur_f);
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
        orb_->detectAndCompute(color_img, cv::noArray(), f.keypoints, f.descriptors);

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
    void VisualProcessorNode::estimate_motion(
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

        if (pts1.size() < 4) return;

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

        RCLCPP_INFO(this->get_logger(), "解算平移 t: x=%f, y=%f, z=%f", t.x(), t.y(), t.z());

        // 可视化匹配结果
        cv::Mat match_img = visualize_matches(f1, f2, matches);
        
        // 可选：显示图像（调试用）
        cv::imshow("Feature Matches", match_img);
        cv::waitKey(1);
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
} // namespace nav2_visual_processing

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav2_visual_processing::VisualProcessorNode>();

    RCLCPP_INFO(node->get_logger(), "Visual Processor Node is spinning...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
