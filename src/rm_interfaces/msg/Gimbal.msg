# 时间戳
std_msgs/Header header

# MCU 采样时间戳（单位 ms，用于 ESKF dt 计算，与 header.stamp 独立）
uint32 t_ms

# IMU数据
geometry_msgs/Vector3 angular_velocity    # 角速度
geometry_msgs/Vector3 linear_acceleration # 线加速度

# 轮速
geometry_msgs/Quaternion wheel_velocity # 轮分速度

# 轮子标签对应：
#         ^     ^
# 左前轮   1 > < 2   右前轮
#
#         ^     ^
# 左后轮 < 3     4 > 右后轮
#

# 测试消息
string log