# cargo_mover_agv - 貨物搬運車AGV控制系統

## 專案概述
cargo_mover_agv是RosAGV系統中專門負責貨物搬運作業的AGV控制套件，實現Cargo Mover AGV的完整控制邏輯。支援雙Hokuyo設備管理、架台搬運操作、入口/出口流程控制、視覺定位、非同步任務處理等複雜搬運功能。

## 核心模組

### 主要類別
- **CargoMoverAGVCoreNode** (`agv_core_node.py`): Cargo AGV核心控制節點
- **CargoContext** (`cargo_context.py`): Cargo特定狀態上下文管理
- **RobotContext** (`robot_context.py`): 機器人狀態管理上下文

### 機器人狀態架構
```
robot_states/
├── base_robot_state.py           # 機器人狀態基類
├── idle_state.py                 # 機器人待機狀態
├── complete_state.py             # 完成狀態(含延遲重置)
├── cargo_robot_parameter.py      # Cargo機器人參數

# ENTRANCE 流程 (入口操作)
├── entrance/
│   ├── check_rack_side_state.py         # 架台側邊檢查
│   ├── select_rack_port_state.py        # 架台端口選擇
│   ├── rack_vision_position_state.py    # 架台視覺定位
│   ├── take_rack_port_state.py          # 架台端口取料
│   ├── wait_rotation_state.py           # 等待旋轉(含非同步任務)
│   ├── transfer_check_empty_state.py    # 傳送箱空位檢查
│   ├── transfer_vision_position_state.py # 傳送箱視覺定位
│   └── put_tranfer_state.py             # 傳送箱放置

# EXIT 流程 (出口操作)  
└── exit/
    ├── check_rack_side_state.py         # 架台側邊檢查
    ├── select_rack_port_state.py        # 架台端口選擇
    ├── rack_vision_position_state.py    # 架台視覺定位
    ├── put_rack_port_state.py           # 架台端口放置
    ├── wait_rotation_state.py           # 等待旋轉(含非同步任務)
    ├── transfer_check_have_state.py     # 傳送箱載具檢查
    ├── transfer_vision_position_state.py # 傳送箱視覺定位
    └── take_transfer_state.py           # 傳送箱取料
```

### Cargo特定狀態
```
cargo_states/
└── idle_state.py                # Cargo待機狀態(含Hokuyo管理)
```

## 關鍵檔案

### 核心檔案
- `/cargo_mover_agv/agv_core_node.py` - Cargo AGV核心控制節點
- `/cargo_mover_agv/cargo_context.py` - Cargo狀態管理上下文
- `/cargo_mover_agv/robot_context.py` - 機器人狀態控制
- `/launch/launch.py` - ROS 2啟動配置

### 測試檔案 (完整測試套件)
- `/test/FINAL_TEST_REPORT.md` - 完整測試報告
- `/test/async_update_task_analysis_report.md` - 非同步任務分析報告
- `/test/test_idle_state_hokuyo.py` - Idle狀態Hokuyo測試
- `/test/test_complete_state_hokuyo.py` - Complete狀態Hokuyo測試
- `/test/test_complete_state_delayed_reset.py` - 延遲重置測試
- `/test/test_hokuyo_busy_states.py` - Hokuyo忙碌狀態測試
- `/test/test_wait_rotation_async_update_task.py` - 等待旋轉非同步測試
- `/test/test_fixed_wait_rotation_async_update_task.py` - 修復版非同步測試

### 特殊功能檔案
- `/robot_states/cargo_robot_parameter.py` - Cargo機器人專用參數
- `/robot_states/test/test_task_sorting.py` - 任務排序優化測試

## 開發指令

### 基本操作
```bash
# 進入AGV容器
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# 構建cargo_mover_agv
build_ws agv_ws
# 或單獨構建
colcon build --packages-select cargo_mover_agv

# 啟動Cargo AGV
export AGV_ID="cargo01"
export ROS_NAMESPACE="/cargo01"
ros2 launch cargo_mover_agv launch.py
```

### 測試指令
```bash
# 執行完整測試套件
cd /app/agv_ws/src/cargo_mover_agv
python3 -m pytest test/ -v

# 執行特定功能測試
python3 -m pytest test/test_idle_state_hokuyo.py -v              # Hokuyo初始化測試
python3 -m pytest test/test_complete_state_delayed_reset.py -v   # 延遲重置測試
python3 -m pytest test/test_wait_rotation_async_update_task.py -v # 非同步任務測試
python3 -m pytest test/test_hokuyo_busy_states.py -v             # Hokuyo忙碌狀態測試

# 任務排序優化測試
python3 robot_states/test/test_task_sorting.py
```

### Hokuyo設備測試
```bash
# 測試雙Hokuyo設備管理
python3 test/test_idle_state_hokuyo.py

# 檢查Hokuyo狀態
ros2 topic echo /<agv_id>/hokuyo_status

# 重新初始化Hokuyo
ros2 service call /<agv_id>/reset_hokuyo
```

## 配置設定

### AGV配置
- `/app/config/agv/cargo01_config.yaml` - Cargo01配置
- `/app/config/agv/cargo02_config.yaml` - Cargo02配置

### 重要參數
```yaml
# cargo_config.yaml
agv_id: "cargo01"
agv_type: "cargo"
hokuyo_devices:
  hokuyo_1: {ip: "192.168.1.101", port: 8000}
  hokuyo_2: {ip: "192.168.1.102", port: 8000}

# 架台配置
rack_positions:
  rack_a: {x: 10.0, y: 5.0, ports: 8}
  rack_b: {x: 15.0, y: 5.0, ports: 8}

# 傳送箱配置
transfer_box:
  position: {x: 20.0, y: 10.0}
  capacity: 4

# 旋轉參數
rotation_params:
  wait_timeout_ms: 30000      # 等待旋轉逾時
  async_task_interval_ms: 500 # 非同步任務間隔
```

### 環境變數
```bash
export AGV_ID="cargo01"                     # AGV識別碼
export ROS_NAMESPACE="/cargo01"             # ROS命名空間
export DEVICE_CONFIG_FILE="/app/config/agv/cargo01_config.yaml"
export HOKUYO_ENABLE="true"                 # 啟用Hokuyo設備
```

## 整合點

### 與其他專案整合
- **agv_base**: 繼承AgvNodebase和BaseContext，使用Hokuyo DMS 8-bit
- **agv_interfaces**: 發布AgvStatus和AgvStateChange
- **plc_proxy_ws**: 架台和傳送箱PLC控制
- **keyence_plc_ws**: 端口狀態檢查和控制
- **db_proxy_ws**: 任務狀態和架台資訊查詢
- **sensorpart_ws**: 視覺定位和Hokuyo感測器整合

### ROS 2話題
```bash
# 發布話題
/<agv_id>/status              # Cargo AGV狀態
/<agv_id>/robot_state         # 機器人狀態
/<agv_id>/hokuyo_status       # 雙Hokuyo設備狀態
/<agv_id>/vision_result       # 視覺定位結果
/<agv_id>/rack_info           # 架台資訊

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/rack_status           # 架台狀態
/system/transfer_status       # 傳送箱狀態
```

## 測試方法

### 完整測試流程
```bash
# 1. 環境準備
cd /app/agv_ws/src/cargo_mover_agv
source /app/setup.bash && all_source

# 2. 執行全部測試
python3 -m pytest test/ -v --tb=short

# 3. 檢查測試報告
cat test/FINAL_TEST_REPORT.md
cat test/async_update_task_analysis_report.md
```

### Hokuyo設備測試
```bash
# Idle狀態Hokuyo初始化測試
python3 -m pytest test/test_idle_state_hokuyo.py::test_hokuyo_initialization -v

# Complete狀態Hokuyo管理測試
python3 -m pytest test/test_complete_state_hokuyo.py::test_hokuyo_finalization -v

# Hokuyo忙碌狀態測試
python3 -m pytest test/test_hokuyo_busy_states.py::test_hokuyo_busy_handling -v
```

### 非同步任務測試
```bash
# 等待旋轉非同步任務測試
python3 -m pytest test/test_wait_rotation_async_update_task.py::test_async_task_handling -v

# 修復版非同步測試
python3 -m pytest test/test_fixed_wait_rotation_async_update_task.py::test_fixed_async_logic -v
```

### 實際硬體測試
```bash
# 啟動Cargo AGV
ros2 launch cargo_mover_agv launch.py

# 監控狀態
ros2 topic echo /<agv_id>/status

# 測試ENTRANCE流程
ros2 service call /<agv_id>/start_entrance_flow

# 測試EXIT流程
ros2 service call /<agv_id>/start_exit_flow
```

## 故障排除

### 常見問題

#### Hokuyo設備連接失敗
```bash
# 檢查Hokuyo網路連接  
ping 192.168.1.101
ping 192.168.1.102

# 重新初始化Hokuyo設備
ros2 service call /<agv_id>/reset_hokuyo

# 檢查Hokuyo狀態
ros2 topic echo /<agv_id>/hokuyo_status
```

#### 非同步任務更新問題
```bash
# 檢查非同步任務狀態
ros2 topic echo /<agv_id>/async_task_status

# 重新啟動非同步任務處理
ros2 service call /<agv_id>/restart_async_tasks

# 查看非同步任務日誌
ros2 topic echo /<agv_id>/async_task_log
```

#### 延遲重置機制異常
```bash
# 檢查Complete狀態重置邏輯
python3 test/test_complete_state_delayed_reset.py

# 手動觸發重置
ros2 service call /<agv_id>/force_reset

# 檢查重置計時器
ros2 param get /<agv_id> reset_delay_ms
```

#### 架台端口選擇錯誤
```bash
# 檢查架台資訊
ros2 topic echo /system/rack_status

# 重新掃描架台端口
ros2 service call /<agv_id>/rescan_rack_ports

# 手動設定端口狀態
ros2 service call /<agv_id>/set_rack_port_status
```

### 除錯技巧
- 使用完整測試報告分析問題模式
- 監控`/<agv_id>/hokuyo_status`掌握設備狀態
- 檢查非同步任務分析報告了解效能瓶頸
- 使用`pytest -v --tb=long`獲得詳細錯誤信息
- 透過視覺化工具監控AGV軌跡和架台交互

### 效能監控
- 雙Hokuyo設備同步效能
- ENTRANCE/EXIT流程完整執行時間
- 非同步任務更新頻率和延遲
- 架台端口切換效率
- 視覺定位精度和速度
- 延遲重置機制響應時間