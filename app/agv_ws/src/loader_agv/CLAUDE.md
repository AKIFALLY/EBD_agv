# loader_agv - 裝載車AGV控制系統

## 專案概述
loader_agv是RosAGV系統中專門負責裝載作業的AGV控制套件，實現Loader AGV的完整控制邏輯。支援從傳送箱取料、多工位操作（清潔機、浸潤機、預乾燥機）、AGV端口管理、視覺定位等複雜裝載流程。

## 核心模組

### 主要類別
- **LoaderAGVCoreNode** (`agv_core_node.py`): Loader AGV核心控制節點
- **LoaderContext** (`loader_context.py`): Loader特定狀態上下文管理
- **RobotContext** (`robot_context.py`): 機器人狀態管理上下文

### 機器人狀態架構
```
robot_states/
├── base_robot_state.py        # 機器人狀態基類
├── idle_state.py              # 機器人待機狀態
├── complete_state.py          # 完成狀態
├── loader_robot_parameter.py  # Loader機器人參數

# Take Transfer 流程 (從傳送箱取料)
├── take_transfer/
│   ├── agv_port_check_empty_state.py    # AGV端口空位檢查
│   ├── transfer_check_have_state.py     # 傳送箱載具檢查
│   ├── transfer_vision_position_state.py # 傳送箱視覺定位
│   ├── take_transfer_state.py           # 傳送箱取料動作
│   └── put_agv_state.py                 # AGV放置動作

# Put Cleaner 流程 (送至清潔機)
├── put_cleaner/
│   ├── agv_port_check_have_state.py     # AGV端口載具檢查
│   ├── cleaner_check_have_state.py      # 清潔機狀態檢查
│   ├── cleaner_vision_position_state.py # 清潔機視覺定位
│   ├── take_agv_state.py                # AGV取料動作
│   └── put_cleaner_state.py             # 清潔機放置動作

# Put Soaker 流程 (送至浸潤機)
├── put_soaker/
│   ├── agv_port_check_have_state.py     # AGV端口載具檢查
│   ├── soaker_check_have_state.py       # 浸潤機狀態檢查
│   ├── soaker_vision_position_state.py  # 浸潤機視覺定位
│   ├── take_agv_state.py                # AGV取料動作
│   └── put_soaker_state.py              # 浸潤機放置動作

# Put Pre-dryer 流程 (送至預乾燥機)
├── put_pre_dryer/
│   ├── agv_port_check_have_state.py     # AGV端口載具檢查
│   ├── pre_dryer_check_have_state.py    # 預乾燥機狀態檢查
│   ├── pre_dryer_vision_position_state.py # 預乾燥機視覺定位
│   ├── take_agv_state.py                # AGV取料動作
│   └── put_pre_dryer_state.py           # 預乾燥機放置動作

# Take Cleaner 流程 (從清潔機取料)
├── take_cleaner/
│   ├── agv_port_check_empty_state.py    # AGV端口空位檢查
│   ├── cleaner_check_have_state.py      # 清潔機載具檢查
│   ├── cleaner_vision_position_state.py # 清潔機視覺定位
│   ├── take_cleaner_state.py            # 清潔機取料動作
│   └── put_agv_state.py                 # AGV放置動作

# Take Soaker 流程 (從浸潤機取料)
└── take_soaker/
    ├── agv_port_check_empty_state.py    # AGV端口空位檢查
    ├── soaker_check_have_state.py       # 浸潤機載具檢查
    ├── soaker_vision_position_state.py  # 浸潤機視覺定位
    ├── take_soaker_state.py             # 浸潤機取料動作
    └── put_agv_state.py                 # AGV放置動作
```

## 關鍵檔案

### 核心檔案
- `/loader_agv/agv_core_node.py` - Loader AGV核心控制節點
- `/loader_agv/loader_context.py` - Loader狀態管理上下文
- `/loader_agv/robot_context.py` - 機器人狀態控制
- `/launch/launch.py` - ROS 2啟動配置

### 測試檔案 (完整測試套件)
- `/test/TEST_REPORT.md` - 詳細測試報告
- `/test/conftest.py` - 測試配置和fixtures
- `/test/run_tests.py` - 測試運行器
- `/test/test_take_transfer_integration.py` - Take Transfer完整流程測試

### 狀態參數
- `/robot_states/loader_robot_parameter.py` - Loader機器人專用參數配置

## 開發指令

### 基本操作
```bash
# 進入AGV容器
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# 構建loader_agv
build_ws agv_ws
# 或單獨構建
colcon build --packages-select loader_agv

# 啟動Loader AGV
export AGV_ID="loader01"
export ROS_NAMESPACE="/loader01"
ros2 launch loader_agv launch.py
```

### 測試指令
```bash
# 執行完整測試套件
cd /app/agv_ws/src/loader_agv
python3 test/run_tests.py

# 執行特定測試
python3 -m pytest test/test_take_transfer_integration.py -v
python3 -m pytest test/test_agv_port_check_empty_state.py -v

# 執行簡化測試
python3 test/simple_test_runner.py

# Demo測試 (可直接運行)
python3 test/test_demo.py
```

## 配置設定

### AGV配置
- `/app/config/agv/loader01_config.yaml` - Loader01配置
- `/app/config/agv/loader02_config.yaml` - Loader02配置

### 重要參數
```yaml
# loader_config.yaml
agv_id: "loader01"
agv_type: "loader"
robot_arm_enabled: true
vision_system_enabled: true

# 端口配置
agv_ports:
  port_1: {x: 0.5, y: 0.3, z: 0.1}
  port_2: {x: 0.5, y: -0.3, z: 0.1}

# 工位配置
stations:
  transfer_box: {x: 5.0, y: 2.0, approach_angle: 0.0}
  cleaner: {x: 8.0, y: 1.5, approach_angle: 1.57}
  soaker: {x: 11.0, y: 1.5, approach_angle: 1.57}
  pre_dryer: {x: 14.0, y: 1.5, approach_angle: 1.57}
```

### 環境變數
```bash
export AGV_ID="loader01"                    # AGV識別碼
export ROS_NAMESPACE="/loader01"            # ROS命名空間
export DEVICE_CONFIG_FILE="/app/config/agv/loader01_config.yaml"
```

## 整合點

### 與其他專案整合
- **agv_base**: 繼承AgvNodebase和BaseContext
- **agv_interfaces**: 發布AgvStatus和AgvStateChange
- **plc_proxy_ws**: 機器人手臂PLC控制
- **keyence_plc_ws**: 設備狀態檢查和控制
- **db_proxy_ws**: 任務狀態和歷史記錄
- **sensorpart_ws**: 視覺定位和感測器整合

### ROS 2話題
```bash
# 發布話題
/<agv_id>/status              # Loader AGV狀態
/<agv_id>/robot_state         # 機器人狀態
/<agv_id>/vision_result       # 視覺定位結果

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/stations_status       # 工位狀態
```

## 測試方法

### 完整測試流程
```bash
# 1. 環境準備
cd /app/agv_ws/src/loader_agv
source /app/setup.bash && all_source

# 2. 執行全部測試
python3 test/run_tests.py

# 3. 檢查測試報告
cat test/TEST_REPORT.md
```

### 個別功能測試
```bash
# AGV端口檢查測試
python3 -m pytest test/test_agv_port_check_empty_state.py::test_agv_port_empty_detection -v

# 傳送箱檢查測試
python3 -m pytest test/test_transfer_check_have_state.py::test_transfer_have_detection -v

# 視覺定位測試
python3 -m pytest test/test_transfer_vision_position_state.py::test_vision_positioning -v

# 取料動作測試
python3 -m pytest test/test_take_transfer_state.py::test_take_transfer_action -v

# 整合流程測試
python3 -m pytest test/test_take_transfer_integration.py::test_complete_take_transfer_flow -v
```

### 實際硬體測試
```bash
# 啟動Loader AGV
ros2 launch loader_agv launch.py

# 監控狀態
ros2 topic echo /<agv_id>/status

# 手動觸發Take Transfer流程
ros2 service call /<agv_id>/start_take_transfer
```

## 故障排除

### 常見問題

#### 視覺定位失敗
```bash
# 檢查視覺系統狀態
ros2 topic echo /<agv_id>/vision_result

# 確認相機連接
ros2 service call /vision/camera_status

# 重新校準視覺系統
ros2 service call /vision/recalibrate
```

#### AGV端口檢查異常
```bash
# 檢查端口感測器
ros2 topic echo /<agv_id>/port_sensors

# 重新初始化端口狀態
ros2 service call /<agv_id>/reset_ports

# 手動設定端口狀態
ros2 service call /<agv_id>/set_port_status "{port_id: 1, has_carrier: false}"
```

#### 機器人手臂控制問題
```bash
# 檢查PLC連接
ros2 service call /plc/connection_status

# 重置機器人狀態
ros2 service call /<agv_id>/reset_robot

# 檢查機器人參數
python3 robot_states/loader_robot_parameter.py
```

#### 狀態機卡住
```bash
# 檢查當前狀態
ros2 topic echo /<agv_id>/status

# 強制重置狀態
ros2 service call /<agv_id>/force_reset

# 檢查狀態轉換日誌
ros2 topic echo /<agv_id>/state_change
```

### 除錯技巧
- 使用完整測試套件驗證功能
- 監控`/<agv_id>/status`話題掌握實時狀態
- 檢查`TEST_REPORT.md`了解測試覆蓋情況
- 使用`conftest.py`中的fixtures進行模組測試
- 透過視覺化工具監控機器人軌跡

### 效能監控
- Take Transfer完整流程執行時間
- 視覺定位精度和速度
- AGV端口切換效率
- 機器人手臂動作平滑度
- 多工位協調效率分析