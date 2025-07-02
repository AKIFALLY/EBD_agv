# AGV 基本資訊
string agv_id
string agv_id1
float32 power

# AGV 速度資訊
int32 x_speed
int32 y_speed
int32 theta_speed

# PGV 感測資訊
int32 front_pgv
int32 back_pgv

# 任務與路線資訊
int32 start_point
int32 end_point
int32 action
int32 zone

# SLAM 位置資訊
int32 slam_x
int32 slam_y
int32 slam_theta

# 狀態資訊
int32 status1
int32 status2
int32 status3

# 警報資訊
int32 alarm1
int32 alarm2
int32 alarm3
int32 alarm4
int32 alarm5
int32 alarm6

# 樓層與雜項
int32 layer
int32 magic
