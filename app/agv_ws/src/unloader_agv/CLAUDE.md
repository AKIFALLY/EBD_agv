# unloader_agv - 卸載車AGV控制系統

## 專案概述
unloader_agv是RosAGV系統中專門負責卸載作業的AGV控制套件，實現Unloader AGV的完整控制邏輯。支援從烘箱取料、預乾燥機操作、出料傳送帶操作、視覺定位、量化計算等複雜卸載流程。

## 核心模組

### 主要類別
- **UnloaderAGVCoreNode** (`agv_core_node.py`): Unloader AGV核心控制節點
- **UnloaderContext** (`unloader_context.py`): Unloader特定狀態上下文管理
- **RobotContext** (`robot_context.py`): 機器人狀態管理上下文

### 機器人狀態架構
```
robot_states/
├── base_robot_state.py            # 機器人狀態基類
├── idle_state.py                  # 機器人待機狀態
├── complete_state.py              # 完成狀態
├── unloader_robot_parameter.py    # Unloader機器人參數

# Take Oven 流程 (從烘箱取料)
├── take_oven/
│   ├── agv_port_check_empty_state.py    # AGV端口空位檢查
│   ├── oven_check_have_state.py         # 烘箱載具檢查
│   ├── oven_vision_position_state.py    # 烘箱視覺定位
│   ├── take_oven_state.py               # 烘箱取料動作
│   └── put_agv_state.py                 # AGV放置動作

# Put Oven 流程 (送至烘箱)
├── put_oven/
│   ├── agv_port_check_have_state.py     # AGV端口載具檢查
│   ├── oven_check_empty_state.py        # 烘箱空位檢查
│   ├── oven_vision_position_state.py    # 烘箱視覺定位
│   ├── take_agv_state.py                # AGV取料動作
│   └── put_oven_state.py                # 烘箱放置動作

# Take Pre-dryer 流程 (從預乾燥機取料)
├── take_pre_dryer/
│   ├── agv_port_check_empty_state.py    # AGV端口空位檢查
│   ├── pre_dryer_check_have_state.py    # 預乾燥機載具檢查
│   ├── pre_dryer_vision_position_state.py # 預乾燥機視覺定位
│   ├── take_pre_dryer_state.py          # 預乾燥機取料動作
│   └── put_agv_state.py                 # AGV放置動作

# Put Boxout Transfer 流程 (送至出料傳送帶)
└── put_boxout_transfer/
    ├── agv_port_check_have_state.py     # AGV端口載具檢查
    ├── boxout_transfer_check_empty_state.py # 出料傳送帶空位檢查
    ├── boxout_transfer_vision_position_state.py # 出料傳送帶視覺定位
    ├── take_agv_state.py                # AGV取料動作
    └── put_boxout_transfer_state.py     # 出料傳送帶放置動作
```

## 關鍵檔案

### 核心檔案
- `/unloader_agv/agv_core_node.py` - Unloader AGV核心控制節點(未顯示具體實現)
- `/unloader_agv/unloader_context.py` - Unloader狀態管理上下文
- `/unloader_agv/robot_context.py` - 機器人狀態控制
- `/launch/launch.py` - ROS 2啟動配置

### 測試檔案
- `/test/README.md` - 測試說明文檔
- `/test/test_pre_dryer_calculation.py` - 預乾燥機計算邏輯測試
- `/test/test_take_quantity.py` - 取料數量計算測試

### 參數配置
- `/robot_states/unloader_robot_parameter.py` - Unloader機器人專用參數配置

## 開發指令

### 基本操作
```bash
# 進入AGV容器
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# 構建unloader_agv
build_ws agv_ws
# 或單獨構建
colcon build --packages-select unloader_agv

# 啟動Unloader AGV
export AGV_ID="unloader01"
export ROS_NAMESPACE="/unloader01"
ros2 launch unloader_agv launch.py
```

### 測試指令
```bash
# 執行測試套件
cd /app/agv_ws/src/unloader_agv
python3 -m pytest test/ -v

# 執行特定測試
python3 -m pytest test/test_pre_dryer_calculation.py -v    # 預乾燥機計算測試
python3 -m pytest test/test_take_quantity.py -v           # 取料數量測試

# 直接執行測試文件
python3 test/test_pre_dryer_calculation.py
python3 test/test_take_quantity.py
```

### 核心節點測試
```bash
# 整合測試
python3 unloader_agv/test_agv_core_node.py
```

## 配置設定

### AGV配置
- `/app/config/agv/unloader01_config.yaml` - Unloader01配置
- `/app/config/agv/unloader02_config.yaml` - Unloader02配置

### 重要參數
```yaml
# unloader_config.yaml
agv_id: "unloader01"
agv_type: "unloader"
robot_arm_enabled: true
vision_system_enabled: true

# 端口配置
agv_ports:
  port_1: {x: 0.5, y: 0.3, z: 0.1}
  port_2: {x: 0.5, y: -0.3, z: 0.1}

# 工位配置
stations:
  oven: {x: 5.0, y: 2.0, capacity: 12, approach_angle: 0.0}
  pre_dryer: {x: 8.0, y: 1.5, capacity: 8, approach_angle: 1.57}
  boxout_transfer: {x: 11.0, y: 3.0, approach_angle: 3.14}

# 計算參數
calculation_params:
  max_take_quantity: 4            # 最大取料數量
  pre_dryer_capacity: 8           # 預乾燥機容量
  quality_check_enabled: true    # 品質檢查
```

### 環境變數
```bash
export AGV_ID="unloader01"                  # AGV識別碼
export ROS_NAMESPACE="/unloader01"          # ROS命名空間
export DEVICE_CONFIG_FILE="/app/config/agv/unloader01_config.yaml"
```

## 整合點

### 與其他專案整合
- **agv_base**: 繼承AgvNodebase和BaseContext
- **agv_interfaces**: 發布AgvStatus和AgvStateChange
- **plc_proxy_ws**: 烘箱和預乾燥機PLC控制
- **keyence_plc_ws**: 設備狀態檢查和控制
- **db_proxy_ws**: 任務狀態和設備資訊查詢
- **sensorpart_ws**: 視覺定位和感測器整合

### ROS 2話題
```bash
# 發布話題
/<agv_id>/status              # Unloader AGV狀態
/<agv_id>/robot_state         # 機器人狀態
/<agv_id>/vision_result       # 視覺定位結果
/<agv_id>/quantity_calculation # 取料數量計算結果

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/oven_status          # 烘箱狀態
/system/pre_dryer_status     # 預乾燥機狀態
/system/boxout_transfer_status # 出料傳送帶狀態
```

## 測試方法

### 基本測試流程
```bash
# 1. 環境準備
cd /app/agv_ws/src/unloader_agv
source /app/setup.bash && all_source

# 2. 執行測試
python3 -m pytest test/ -v

# 3. 檢查測試結果
echo "檢查測試日誌和結果"
```

### 特定功能測試
```bash
# 預乾燥機計算邏輯測試
python3 -m pytest test/test_pre_dryer_calculation.py::test_pre_dryer_capacity_calculation -v

# 取料數量計算測試
python3 -m pytest test/test_take_quantity.py::test_optimal_quantity_calculation -v

# 核心節點整合測試
python3 unloader_agv/test_agv_core_node.py
```

### 實際硬體測試
```bash
# 啟動Unloader AGV
ros2 launch unloader_agv launch.py

# 監控狀態
ros2 topic echo /<agv_id>/status

# 手動觸發Take Oven流程
ros2 service call /<agv_id>/start_take_oven

# 手動觸發Put Boxout Transfer流程
ros2 service call /<agv_id>/start_put_boxout_transfer
```

## 故障排除

### 常見問題

#### 視覺定位失敗
```bash
# 檢查視覺系統狀態
ros2 topic echo /<agv_id>/vision_result

# 重新校準視覺系統
ros2 service call /vision/recalibrate

# 檢查相機連接
ros2 service call /vision/camera_status
```

#### 取料數量計算異常
```bash
# 檢查計算參數
ros2 param list /<agv_id> | grep quantity

# 重新計算取料數量
ros2 service call /<agv_id>/recalculate_quantity

# 驗證計算邏輯
python3 test/test_take_quantity.py
```

#### 預乾燥機操作問題
```bash
# 檢查預乾燥機狀態
ros2 topic echo /system/pre_dryer_status

# 重置預乾燥機連接
ros2 service call /<agv_id>/reset_pre_dryer

# 驗證計算邏輯
python3 test/test_pre_dryer_calculation.py
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

### 除錯技巧
- 使用pytest -v獲得詳細測試結果
- 監控`/<agv_id>/status`話題掌握實時狀態
- 檢查數量計算邏輯確保最佳化效率
- 使用視覺化工具監控機器人軌跡
- 驗證PLC通訊確保設備控制正確

### 效能監控
- Take Oven完整流程執行時間
- Put Boxout Transfer效率分析
- 預乾燥機操作週期時間
- 視覺定位精度和速度
- 取料數量優化效果
- AGV端口切換效率
- 機器人手臂動作平滑度