# agv_base - AGV基礎核心框架

## 專案概述
agv_base是RosAGV系統的核心基礎框架，提供AGV狀態機的抽象基類和通用邏輯。實現3層架構的基礎層(Base層)，為所有AGV車型提供統一的狀態管理、事件處理和硬體控制介面。

## 核心模組

### 主要類別
- **AgvNodebase** (`agv_node_base.py`): AGV節點基類，統一的ROS 2節點框架
- **BaseContext** (`base_context.py`): 狀態機上下文管理，處理狀態轉換
- **AgvStatus** (`agv_status.py`): AGV狀態數據結構定義
- **Robot** (`robot.py`): 機器人硬體抽象層
- **Event** (`event.py`): 事件系統，支援狀態間通訊

### 狀態機架構
```
agv_base/states/          # Base層狀態
├── idle_state.py        # 待機狀態
├── auto_state.py        # 自動狀態  
├── manual_state.py      # 手動狀態
├── error_state.py       # 錯誤狀態
└── state.py            # 狀態基類

agv_base/agv_states/     # AGV層狀態
├── idle_state.py        # AGV待機狀態
├── mission_select_state.py  # 任務選擇狀態
├── Running_state.py     # 運行狀態
├── wait_robot_state.py  # 等待機器人狀態
└── write_path_state.py  # 路徑寫入狀態
```

## 關鍵檔案

### 核心檔案
- `/agv_base/agv_node_base.py` - AGV節點基類，50ms主循環，PLC通訊
- `/agv_base/base_context.py` - 狀態機上下文，狀態轉換邏輯
- `/agv_base/context_abc.py` - 上下文抽象基類
- `/agv_base/robot.py` - 機器人硬體控制抽象

### 狀態管理
- `/agv_base/states/state.py` - 狀態基類，定義狀態接口
- `/agv_base/agv_status.py` - AGV狀態數據模型
- `/agv_base/event.py` - 事件系統實現

### 感測器整合
- `/agv_base/hokuyo_dms_8bit.py` - Hokuyo雷射感測器8位元資料處理

## 開發指令

### 基本構建
```bash
# 進入AGV容器
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# 構建agv_ws
build_ws agv_ws

# 測試基礎功能
cd /app/agv_ws/src/agv_base
python3 -m pytest test/
```

### 狀態機測試
```bash
# 測試AGV節點基礎功能
python3 agv_base/test_agv_node.py

# 測試事件系統
python3 agv_base/test_agv_node_event.py
```

## 配置設定

### AGV基礎配置
- `/app/config/agv/base_config.yaml` - AGV基礎參數配置
- `/app/config/hardware_mapping.yaml` - 硬體設備映射

### 重要參數
```yaml
# base_config.yaml
agv_id: "agv01"
plc_ip: "192.168.1.100"
cycle_time_ms: 50          # 主循環週期
status_publish_rate: 1.5   # 狀態發布頻率
```

## 整合點

### 與其他專案整合
- **車型專案** (`cargo_mover_agv`, `loader_agv`, `unloader_agv`): 繼承BaseContext和AgvNodebase
- **plc_proxy_ws**: 透過PlcClient進行PLC通訊
- **agv_interfaces**: 使用AgvStatus和AgvStateChange訊息
- **db_proxy_ws**: 狀態資料持久化

### ROS 2 話題
```bash
# 發布話題
/agv/status                # AgvStatus訊息

# 訂閱話題  
/agv/state_change         # 狀態變更事件
```

## 測試方法

### 單元測試
```bash
# 執行所有測試
python3 -m pytest agv_base/test_*

# 測試特定功能
python3 agv_base/test_agv_node.py        # 節點基礎功能
python3 agv_base/test_agv_node_event.py  # 事件系統
```

### 狀態機測試
```bash
# 狀態轉換測試
python3 agv_base/states/practise_test.py

# 特定狀態測試
python3 agv_base/states/test_a_state.py
python3 agv_base/states/test_b_state.py
```

### 整合測試
```bash
# 完整AGV節點測試
ros2 run agv_base test_agv_node

# 狀態機循環測試
ros2 topic echo /agv/status
```

## 故障排除

### 常見問題

#### PLC連線失敗
```bash
# 檢查PLC客戶端狀態
ros2 service call /plc/read_data plc_interfaces/srv/ReadData

# 確認網路連通性
ping <PLC_IP>
```

#### 狀態機卡住
```bash
# 檢查當前狀態
ros2 topic echo /agv/status

# 重置狀態機
ros2 service call /agv/reset_state
```

#### 感測器數據異常
```bash
# 檢查Hokuyo感測器
python3 agv_base/hokuyo_dms_8bit.py

# 查看感測器日誌
ros2 topic echo /agv/sensor_data
```

### 除錯技巧
- 使用`self.get_logger().info()`記錄狀態轉換
- 監控`/agv/status`話題掌握AGV當前狀態
- 檢查PLC記憶體映射是否正確
- 確認狀態轉換條件邏輯

### 效能監控
- 主循環執行時間應保持在50ms內
- PLC通訊延遲監控
- 狀態轉換頻率分析
- 記憶體使用情況檢查