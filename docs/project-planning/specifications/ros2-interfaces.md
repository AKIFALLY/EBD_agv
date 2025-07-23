# ROS 2 介面規格文檔

## 📋 概述

本文檔定義 RosAGV 系統中所有 ROS 2 服務、訊息和動作介面的完整規格，基於實際程式碼實作。

## 🔧 PLC 代理服務介面 (plc_proxy_ws)

### 服務節點資訊
- **節點名稱**: `plc_service`
- **命名空間**: `/agvc` (可配置)
- **QoS 設定**: `depth=100`, `RELIABLE`
- **回調群組**: `ReentrantCallbackGroup` (支援並發)

### 1. 基本讀寫服務

#### ReadData.srv
```yaml
# 讀取單一 PLC 資料
# Request
string device_type    # "DM" (Data Memory) 或 "MR" (Memory Relay)
string address        # PLC 位址，如 "7600"

---
# Response
bool success          # 操作是否成功
string value          # 讀取的值 (字串格式)
string message        # 錯誤訊息或狀態說明
```

#### WriteData.srv
```yaml
# 寫入單一 PLC 資料
# Request
string device_type    # "DM" 或 "MR"
string address        # PLC 位址，如 "1000"
string value          # 要寫入的值 (字串格式)

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 2. 連續讀寫服務

#### ReadContinuousData.srv
```yaml
# 連續讀取多個 PLC 資料
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址，如 "7600"
int32 count          # 讀取數量

---
# Response
bool success          # 操作是否成功
string[] values       # 讀取的值陣列
string message        # 錯誤訊息或狀態說明
```

#### WriteContinuousData.srv
```yaml
# 連續寫入多個 PLC 資料
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
string[] values       # 要寫入的值陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 3. 位元組操作服務

#### ReadContinuousByte.srv
```yaml
# 連續讀取位元組資料
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
int32 count          # 讀取位元組數量

---
# Response
bool success          # 操作是否成功
uint8[] values        # 讀取的位元組陣列
string message        # 錯誤訊息或狀態說明
```

#### WriteContinuousByte.srv
```yaml
# 連續寫入位元組資料
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
uint8[] values        # 要寫入的位元組陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 4. 強制控制服務

#### ForceOn.srv
```yaml
# 強制開啟 MR 位元
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

#### ForceOff.srv
```yaml
# 強制關閉 MR 位元
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

## 🎮 搖桿控制介面 (joystick_ws)

### 標準 ROS 2 Joy 訊息
```yaml
# sensor_msgs/msg/Joy
Header header
float32[] axes      # 搖桿軸值 (-1.0 到 1.0)
int32[] buttons     # 按鈕狀態 (0 或 1)
```

### 搖桿配置
- **發布主題**: `/joy`
- **發布頻率**: 50 Hz
- **裝置路徑**: `/dev/input/js0`
- **支援型號**: 標準 USB 遊戲手把

## 🚗 AGV 控制介面 (agv_ws)

### AGV 狀態發布
```yaml
# 自訂 AGV 狀態訊息 (概念性)
Header header
string agv_id           # AGV 識別碼
string current_state    # 當前狀態
geometry_msgs/Pose current_pose    # 當前位置
float32 battery_level   # 電池電量 (0.0-1.0)
bool is_emergency_stop  # 緊急停止狀態
string[] active_tasks   # 活躍任務列表
```

### AGV 命令介面
```yaml
# 自訂 AGV 命令訊息 (概念性)
Header header
string command_type     # 命令類型 (MOVE, STOP, PAUSE, RESUME)
geometry_msgs/Pose target_pose     # 目標位置
string task_id         # 任務識別碼
float32 max_velocity   # 最大速度限制
```

## 📊 感測器資料介面 (sensorpart_ws)

### 感測器資料發布
```yaml
# 標準感測器訊息類型
sensor_msgs/msg/LaserScan     # 雷射掃描資料
sensor_msgs/msg/PointCloud2  # 點雲資料
sensor_msgs/msg/Imu          # IMU 資料
nav_msgs/msg/Odometry        # 里程計資料
```

### 感測器狀態監控
```yaml
# 自訂感測器狀態訊息 (概念性)
Header header
string sensor_name      # 感測器名稱
bool is_active         # 是否活躍
float32 data_rate      # 資料更新率
string status_message  # 狀態訊息
```

## 🔧 服務端點總覽

### PLC 代理服務端點
```bash
# 基本讀寫
/agvc/read_data                    # ReadData
/agvc/write_data                   # WriteData

# 連續讀寫
/agvc/read_continuous_data         # ReadContinuousData
/agvc/write_continuous_data        # WriteContinuousData

# 位元組操作
/agvc/read_continuous_byte         # ReadContinuousByte
/agvc/write_continuous_byte        # WriteContinuousByte

# 強制控制
/agvc/force_on                     # ForceOn
/agvc/force_off                    # ForceOff
```

### 主題總覽
```bash
# 搖桿控制
/joy                              # sensor_msgs/Joy

# AGV 狀態 (概念性)
/agvc/agv_status                  # 自訂 AGV 狀態
/agvc/agv_command                 # 自訂 AGV 命令

# 感測器資料
/scan                             # sensor_msgs/LaserScan
/points                           # sensor_msgs/PointCloud2
/imu                              # sensor_msgs/Imu
/odom                             # nav_msgs/Odometry
```

## 🎯 介面使用範例

### PLC 服務調用範例
```bash
# 讀取 DM 資料
ros2 service call /agvc/read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '7600'}"

# 寫入 DM 資料
ros2 service call /agvc/write_data plc_interfaces/srv/WriteData \
  "{device_type: 'DM', address: '1000', value: '123'}"

# 連續讀取
ros2 service call /agvc/read_continuous_data plc_interfaces/srv/ReadContinuousData \
  "{device_type: 'DM', start_address: '7600', count: 10}"
```

### 主題訂閱範例
```bash
# 監聽搖桿輸入
ros2 topic echo /joy

# 監聽 AGV 狀態
ros2 topic echo /agvc/agv_status

# 監聽雷射掃描
ros2 topic echo /scan
```

## 📝 介面設計原則

### 1. 一致性
- 統一的命名規範
- 標準的回應格式
- 一致的錯誤處理

### 2. 可擴展性
- 支援新的 PLC 指令類型
- 可添加新的感測器類型
- 靈活的參數配置

### 3. 可靠性
- 完整的錯誤處理
- 超時機制
- 重試邏輯

### 4. 效能
- 非阻塞式服務調用
- 批次操作支援
- 資料快取機制

## 📝 相關文檔

- [資料庫結構設計](./database-schema.md)
- [Web API 規格](./web-api-specification.md)
- [PLC 通訊協定](./plc-communication.md)
- [資料格式規範](./data-formats.md)

---

**最後更新**: 2025-01-17  
**維護責任**: ROS 2 架構師  
**版本**: v1.0.0
