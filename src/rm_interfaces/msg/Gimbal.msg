# 电控上行
std_msgs/Header header

# MCU 发来的 uint32 Temp（测试字段，后续可替换）
uint32 temp

# 拍照/开灯完成后为 true，上位机据此才能发下一个巡检点
bool capture_done
