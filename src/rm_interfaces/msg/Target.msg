# 发给电控的底盘速度指令（车体坐标系）
# 由 serial_driver 从 /cmd_vel 填入，也可直接往 /chassis_cmd 发本消息做手动测试
float32 vx    # 线速度 m/s，前进为正
float32 wz    # 角速度 rad/s，逆时针为正

# 巡检到位停留开始时为 true，通知电控开灯拍照；1s 后为 false
bool capture_enable
