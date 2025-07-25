# sensorpart_ws CLAUDE.md

## 模組概述
感測器數據處理系統，負責AGV上各類感測器的數據採集、處理與融合

## 專案結構
```
src/
└── sensorpart/         # 感測器處理核心
    ├── sensorpart/     # 感測器數據處理邏輯
    ├── drivers/        # 感測器驅動程式
    ├── filters/        # 數據濾波器
    └── fusion/         # 感測器融合演算法
```

## 核心功能

### 感測器支援
- **雷射掃描器**: Hokuyo等LiDAR感測器
- **相機系統**: RGB相機與深度相機
- **慣性感測器**: IMU加速度計與陀螺儀
- **超聲波感測器**: 近距離障礙物檢測

### 數據處理
- **噪音濾波**: 卡爾曼濾波與粒子濾波
- **數據融合**: 多感測器數據融合
- **校準補償**: 感測器偏移與畸變校正
- **異常檢測**: 感測器故障與異常數據檢測

## 開發指令

### 環境設定 (AGV容器內)
```bash
source /app/setup.bash && all_source
cd /app/sensorpart_ws
```

### 服務啟動
```bash
# 啟動感測器處理節點
ros2 run sensorpart sensorpart_node

# 啟動感測器校準工具
ros2 run sensorpart sensor_calibration

# 測試感測器連線
ros2 run sensorpart test_sensors
```

### 構建與測試
```bash
build_ws sensorpart_ws
ros2 test sensorpart  # 感測器系統測試
```

## 感測器驅動開發

### LiDAR驅動
```python
# drivers/lidar_driver.py
class LiDARDriver:
    def __init__(self, config):
        self.device_path = config['device_path']
        self.frame_id = config['frame_id']
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        
    def initialize_lidar(self):
        """初始化LiDAR設備"""
        try:
            self.lidar_device = self.connect_to_device(self.device_path)
            self.configure_scan_parameters()
            return True
        except Exception as e:
            self.get_logger().error(f"LiDAR初始化失敗: {e}")
            return False
            
    def process_scan_data(self, raw_data):
        """處理掃描數據並發布"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        
        # 設定掃描參數
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        
        # 轉換原始數據為距離數組
        scan_msg.ranges = self.convert_to_ranges(raw_data)
        scan_msg.intensities = self.extract_intensities(raw_data)
        
        self.scan_publisher.publish(scan_msg)
```

### IMU驅動
```python
# drivers/imu_driver.py
class IMUDriver:
    def __init__(self, config):
        self.device_path = config['device_path']
        self.frame_id = config['frame_id']
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        
    def read_imu_data(self):
        """讀取IMU數據"""
        try:
            raw_data = self.imu_device.read_data()
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # 角速度數據
            imu_msg.angular_velocity.x = raw_data['gyro_x']
            imu_msg.angular_velocity.y = raw_data['gyro_y']
            imu_msg.angular_velocity.z = raw_data['gyro_z']
            
            # 線性加速度數據
            imu_msg.linear_acceleration.x = raw_data['accel_x']
            imu_msg.linear_acceleration.y = raw_data['accel_y']
            imu_msg.linear_acceleration.z = raw_data['accel_z']
            
            # 方向估計(如果可用)
            if self.has_magnetometer:
                orientation = self.calculate_orientation(raw_data)
                imu_msg.orientation = orientation
                
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().warn(f"IMU數據讀取錯誤: {e}")
```

### 相機驅動
```python
# drivers/camera_driver.py
class CameraDriver:
    def __init__(self, config):
        self.camera_id = config['camera_id']
        self.resolution = config['resolution']
        self.frame_rate = config['frame_rate']
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
    def capture_and_publish(self):
        """擷取並發布影像"""
        try:
            frame = self.camera.read()
            if frame is not None:
                # 轉換為ROS 2影像訊息
                image_msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.stamp = self.get_clock().now().to_msg()
                image_msg.header.frame_id = self.frame_id
                
                self.image_publisher.publish(image_msg)
                
        except Exception as e:
            self.get_logger().warn(f"相機擷取錯誤: {e}")
```

## 數據濾波與處理

### 卡爾曼濾波器
```python
# filters/kalman_filter.py
class KalmanFilter:
    def __init__(self, initial_state, initial_covariance):
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_noise = None
        self.measurement_noise = None
        
    def predict(self, control_input=None):
        """預測步驟"""
        # 狀態預測
        self.state = self.state_transition @ self.state
        if control_input is not None:
            self.state += self.control_matrix @ control_input
            
        # 協方差預測
        self.covariance = (self.state_transition @ self.covariance @ 
                          self.state_transition.T + self.process_noise)
    
    def update(self, measurement):
        """更新步驟"""
        # 卡爾曼增益
        innovation_covariance = (self.measurement_matrix @ self.covariance @ 
                               self.measurement_matrix.T + self.measurement_noise)
        kalman_gain = (self.covariance @ self.measurement_matrix.T @ 
                      np.linalg.inv(innovation_covariance))
        
        # 狀態更新
        innovation = measurement - self.measurement_matrix @ self.state
        self.state += kalman_gain @ innovation
        
        # 協方差更新
        self.covariance = ((np.eye(len(self.state)) - kalman_gain @ 
                           self.measurement_matrix) @ self.covariance)
```

### 移動平均濾波器
```python
# filters/moving_average_filter.py
class MovingAverageFilter:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data_buffer = deque(maxlen=window_size)
        
    def add_sample(self, sample):
        """添加新樣本"""
        self.data_buffer.append(sample)
        
    def get_filtered_value(self):
        """獲取濾波後的值"""
        if len(self.data_buffer) == 0:
            return None
        return sum(self.data_buffer) / len(self.data_buffer)
        
    def reset(self):
        """重置濾波器"""
        self.data_buffer.clear()
```

## 感測器融合

### 多感測器融合
```python
# fusion/sensor_fusion.py
class SensorFusion:
    def __init__(self):
        self.lidar_data = None
        self.imu_data = None
        self.camera_data = None
        self.fused_publisher = self.create_publisher(
            PointCloud2, '/fused_sensor_data', 10
        )
        
    def fuse_lidar_imu(self, lidar_scan, imu_data):
        """融合LiDAR與IMU數據"""
        # 使用IMU數據校正LiDAR掃描
        corrected_scan = self.motion_compensation(lidar_scan, imu_data)
        
        # 轉換為點雲格式
        point_cloud = self.scan_to_pointcloud(corrected_scan)
        
        return point_cloud
        
    def motion_compensation(self, scan, imu_data):
        """運動補償"""
        # 計算掃描期間的運動
        angular_velocity = imu_data.angular_velocity.z
        scan_time = scan.time_increment * len(scan.ranges)
        
        # 補償角度偏移
        compensated_ranges = []
        for i, range_value in enumerate(scan.ranges):
            time_offset = i * scan.time_increment
            angle_offset = angular_velocity * time_offset
            
            # 校正該點的角度
            corrected_angle = scan.angle_min + i * scan.angle_increment - angle_offset
            compensated_ranges.append((range_value, corrected_angle))
            
        return compensated_ranges
```

### 障礙物檢測
```python
# fusion/obstacle_detection.py
class ObstacleDetection:
    def __init__(self):
        self.min_obstacle_size = 0.1  # 最小障礙物尺寸(m)
        self.max_detection_range = 10.0  # 最大檢測範圍(m)
        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, '/obstacles', 10
        )
        
    def detect_obstacles(self, sensor_data):
        """檢測障礙物"""
        obstacles = []
        
        # 從點雲數據中檢測障礙物
        if 'pointcloud' in sensor_data:
            cloud_obstacles = self.detect_from_pointcloud(
                sensor_data['pointcloud']
            )
            obstacles.extend(cloud_obstacles)
            
        # 從超聲波數據中檢測近距離障礙物
        if 'ultrasonic' in sensor_data:
            ultrasonic_obstacles = self.detect_from_ultrasonic(
                sensor_data['ultrasonic']
            )
            obstacles.extend(ultrasonic_obstacles)
            
        # 融合與去重
        fused_obstacles = self.merge_obstacles(obstacles)
        
        return fused_obstacles
```

## 感測器配置

### 感測器參數配置
```yaml
# /app/config/agv/sensor_config.yaml
sensors:
  lidar:
    - name: "front_lidar"
      type: "hokuyo_urg"
      device_path: "/dev/ttyACM0"
      frame_id: "front_lidar_link"
      angle_min: -2.0944      # -120度
      angle_max: 2.0944       # 120度
      range_min: 0.1          # 0.1m
      range_max: 30.0         # 30m
      scan_rate: 10           # 10Hz
      
    - name: "rear_lidar"
      type: "hokuyo_urg"
      device_path: "/dev/ttyACM1"
      frame_id: "rear_lidar_link"
      angle_min: -2.0944
      angle_max: 2.0944
      range_min: 0.1
      range_max: 30.0
      scan_rate: 10
      
  imu:
    name: "base_imu"
    type: "xsens_mti"
    device_path: "/dev/ttyUSB0"
    frame_id: "imu_link"
    sample_rate: 100        # 100Hz
    
  camera:
    - name: "front_camera"
      type: "usb_camera"
      device_id: 0
      frame_id: "front_camera_link"
      resolution: [1280, 720]
      frame_rate: 30
      
  ultrasonic:
    - name: "front_ultrasonic"
      type: "mb1040"
      pin: 18
      frame_id: "front_ultrasonic_link"
      max_range: 7.65
      min_range: 0.20
      field_of_view: 0.7854   # 45度
```

### 濾波器配置
```yaml
# 濾波器參數
filters:
  lidar_filter:
    type: "median"
    window_size: 5
    range_threshold: 0.05   # 5cm
    
  imu_filter:
    type: "kalman"
    process_noise: 0.01
    measurement_noise: 0.1
    
  camera_filter:
    type: "gaussian"
    kernel_size: 5
    sigma: 1.0
```

## 測試與調試

### 感測器測試
```bash
# 測試LiDAR
ros2 topic echo /scan

# 測試IMU
ros2 topic echo /imu/data

# 測試相機
ros2 topic echo /camera/image_raw

# 查看感測器TF樹
ros2 run tf2_tools view_frames.py
```

### 視覺化工具
```bash
# 啟動RViz查看感測器數據
ros2 run rviz2 rviz2 -d /app/config/agv/sensor_visualization.rviz

# 查看點雲數據
ros2 run rqt_plot rqt_plot /scan/ranges[0]

# 相機影像顯示
ros2 run rqt_image_view rqt_image_view
```

## 校準程序

### LiDAR校準
```python
# calibration/lidar_calibration.py
class LiDARCalibration:
    def calibrate_lidar_mounting(self):
        """校準LiDAR安裝角度"""
        # 掃描平面參考物件
        reference_scans = self.collect_reference_scans()
        
        # 計算安裝角度偏移
        mounting_offset = self.calculate_mounting_offset(reference_scans)
        
        # 更新轉換矩陣
        self.update_transform_matrix(mounting_offset)
        
        return mounting_offset
```

### IMU校準
```bash
# IMU校準程序
ros2 run sensorpart imu_calibration --duration 60

# 保存校準數據
ros2 service call /sensorpart/save_imu_calibration

# 載入校準數據
ros2 service call /sensorpart/load_imu_calibration
```

## 故障排除

### 常見問題
1. **感測器無法連線**: 檢查設備路徑與權限
2. **數據噪音過大**: 調整濾波器參數
3. **校準偏移**: 重新執行校準程序
4. **數據延遲**: 檢查系統負載與通訊頻寬

### 診斷工具
```bash
# 感測器狀態診斷
ros2 run sensorpart sensor_diagnostics

# 檢查設備連線
ls -la /dev/tty* /dev/video*

# 查看感測器統計
ros2 topic hz /scan
ros2 topic bw /camera/image_raw
```

## 性能最佳化

### 數據處理最佳化
- **多執行緒處理**: 並行處理多個感測器數據
- **數據快取**: 減少重複計算
- **適應性濾波**: 根據環境動態調整濾波參數
- **硬體加速**: 利用GPU加速影像處理

### 頻寬管理
- **數據壓縮**: 壓縮影像與點雲數據
- **選擇性發布**: 根據需求調整發布頻率
- **品質調整**: 動態調整感測器解析度

## 重要提醒
- 感測器數據直接影響AGV安全，校準與維護至關重要
- 環境因素會影響感測器效能，需定期檢查與調整
- 多感測器融合可提高可靠性，但也增加計算負載
- 僅適用於AGV車載系統，確保在正確容器環境中運行