# 电控上行
std_msgs/Header header

# 拍照/开灯完成后为 true，上位机据此才能发下一个巡检点
bool capture_done

# 电控上报的实际底盘速度（车体坐标系）
float32 vx    # 线速度 m/s，前进为正
float32 wz    # 角速度 rad/s，逆时针为正
