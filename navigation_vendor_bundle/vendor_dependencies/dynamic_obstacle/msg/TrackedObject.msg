# 单个追踪障碍物
uint32 id                     # 唯一追踪ID
float64 x                     # 当前位置 x (map frame)
float64 y                     # 当前位置 y (map frame)
float64 vx                    # 估计速度 x (m/s)
float64 vy                    # 估计速度 y (m/s)
float64 heading               # 运动方向 (rad)
float64 size_x                # 包围盒宽度 (m)
float64 size_y                # 包围盒长度 (m)
bool is_confirmed             # 是否已确认 (连续多帧检测到)
uint32 track_length           # 已追踪帧数
geometry_msgs/Point[] predicted_path  # 预测轨迹点 (按时间升序, 0.1s步长)
