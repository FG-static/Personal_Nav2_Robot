import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.odom_x, self.odom_y, self.gt_x, self.gt_y = [], [], [], []

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/ground_truth', self.gt_cb, 10)
        self.timer = self.create_timer(10.0, self.status_report)
        self.get_logger().info('轨迹记录器启动')
    
    def odom_cb(self, msg):
        self.odom_x.append(msg.pose.pose.position.x)
        self.odom_y.append(msg.pose.pose.position.y)
    
    def gt_cb(self, msg):
        self.gt_x.append(msg.pose.pose.position.x)
        self.gt_y.append(msg.pose.pose.position.y)
    
    def status_report(self):
        self.get_logger().info(f'已记录点数: Odom={len(self.odom_x)}, GT={len(self.gt_x)}')

    def save_plot(self):
        plt.figure(figsize=(10, 10), facecolor='white')
        plt.plot(self.gt_x, self.gt_y, label='Ground Truth', color='green', linewidth=2)
        plt.plot(self.odom_x, self.odom_y, label='Odometry', color='red', linestyle='--', linewidth=1.5)

        plt.title('Robot Trajectory Comparison')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.legend()
        plt.grid(True, linestyle=':', alpha=0.6)
        plt.axis('equal') 

        save_path = 'trajectory_result.png'
        plt.savefig(save_path)
        print(f'\n图片已保存至: {save_path}')

def main():
    rclpy.init()
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()