# ecs_ws CLAUDE.md

## 模組概述
設備控制系統(Equipment Control System)，專注於PLC數據收集、門控制和設備信號管理，為AGVC系統提供核心的工業設備控制功能

## 專案結構 (實際驗證)
```
src/
└── ecs/                          # 設備控制系統
    ├── ecs_core.py              # PLC數據讀取和資料庫信號更新
    ├── door_logic.py            # 門控制邏輯實現
    ├── door_controller_config.py # 門控制器配置管理
    └── door_controller_node_mqtt.py # MQTT門控制節點(KUKA整合)
```

## 核心功能 (基於實際實現)

### PLC數據管理 (ecs_core.py)
- **連續數據讀取**: 0.1秒週期讀取PLC指定範圍數據
- **記憶體映射**: 使用PlcMemory管理PLC數據緩存
- **資料庫同步**: 自動更新設備信號值到PostgreSQL
- **信號變更檢測**: 只在數值變動時更新資料庫

### 門控制系統 (door_logic.py)
- **門狀態查詢**: 透過PLC proxy讀取門開關狀態
- **門控制指令**: 強制開門/關門操作
- **異步操作**: 支援異步和同步門控制
- **批次控制**: 支援多門同時控制

### MQTT門控制器 (door_controller_node_mqtt.py)
- **MQTT通訊**: 透過MQTT與外部系統(如KUKA ECS)通訊
- **狀態監控**: 定期檢查門狀態變化並發布
- **外部整合**: 接收外部門控制請求並執行

## 關鍵檔案

### 核心檔案
- `/ecs/ecs_core.py` - PLC數據讀取核心，0.1秒週期更新資料庫信號
- `/ecs/door_logic.py` - 門控制邏輯實現，支援同步/異步操作
- `/ecs/door_controller_config.py` - 門配置管理，支援YAML和字串配置
- `/ecs/door_controller_node_mqtt.py` - MQTT門控制節點，KUKA ECS整合

### 配置檔案
- `/app/config/door_config.yaml` - 門控制配置
- `setup.py` - 包含兩個ROS節點入口點

## 實際技術棧
- **ROS 2節點**: ecs_core, door_controller_node_mqtt
- **PLC通訊**: 透過plc_proxy.PlcClient
- **資料庫**: PostgreSQL (透過db_proxy.ConnectionPoolManager)
- **MQTT**: paho-mqtt (外部系統整合)
- **記憶體管理**: keyence_plc.PlcMemory

## 🔧 開發工具指南

### 宿主機操作 (推薦用於診斷和管理)

#### ECS 系統診斷工具
```bash
# AGVC 系統健康檢查 (含 ECS)
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查
agvc_services                        # 所有服務狀態檢查

# ECS 日誌分析
scripts/log-tools/log-analyzer.sh agvc | grep -i "ecs\|equipment"  # ECS 相關日誌
scripts/log-tools/log-analyzer.sh agvc --stats --filter "ecs"

# 設備連接診斷
scripts/network-tools/port-check.sh system    # 檢查設備端口
quick_agvc "check_agvc_status"         # 檢查 ECS 狀態
```

#### 開發工作流工具
```bash
# 建置和測試
source scripts/dev-tools/dev-tools.sh
dev_build --workspace ecs_ws
dev_test --workspace ecs_ws
dev_check --workspace ecs_ws --severity warning
```

### 容器內操作 (ROS 2 開發)

#### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間 (或使用 all_source 自動檢測)
cd /app/ecs_ws
```

#### 服務管理 (基於實際entry_points)
```bash
# 【方法1: 透過宿主機工具】(推薦)
source scripts/docker-tools/docker-tools.sh
quick_agvc "ros2 run ecs ecs_core"               # 啟動 ECS 核心節點
quick_agvc "ros2 run ecs door_controller_node_mqtt"  # 啟動 MQTT 門控制節點

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
ros2 run ecs ecs_core                        # 啟動ECS核心節點
ros2 run ecs door_controller_node_mqtt       # 啟動MQTT門控制節點
check_agvc_status                           # 檢查ECS狀態信息
```

### 構建與測試
```bash
build_ws ecs_ws
colcon test --packages-select ecs    # ECS包測試
```

## 開發指南 (基於實際實現)

### ECS核心節點開發 (ecs_core.py)
```python
# 新增PLC數據讀取範圍
self.declare_parameter('read_ranges', ["DM,7600,20", "DM,5000,200"])

# 擴展信號處理邏輯
def write_signals_to_db(self):
    # 只在信號值變動時更新資料庫
    if signal.value != str(value):
        self.get_logger().info(f"變動[{signal.value}] to {str(value)} for {signal.name}")
        signal.value = str(value)
        session.merge(signal)
```

### 門控制開發 (door_logic.py)
```python
# 異步門控制實現
def async_control_door(self, door_id: int, is_open: bool) -> Dict:
    cfg = self.config.get_config(door_id)
    if is_open:
        response = self.plc_client.async_force_on(
            cfg["mr_type"], cfg["mr_address"], self.force_callback)
    else:
        response = self.plc_client.async_force_off(
            cfg["mr_type"], cfg["mr_address"], self.force_callback)

# 門狀態查詢
def state_door(self, door_id: int) -> Dict:
    cfg = self.config.get_config(door_id)
    response = self.plc_client.read_continuous_byte(
        cfg["dm_type"], cfg["dm_address"], 1)
    bit = bool(response.values[0])
    door_state = "OPEN" if bit else "CLOSE"
```

### MQTT門控制器開發 (door_controller_node_mqtt.py)
```python
# MQTT配置參數
self.declare_parameter('broker_host', '192.168.11.206')
self.declare_parameter('broker_port', 2883)
self.declare_parameter('sub_topic', 'request/to/agvc/door')
self.declare_parameter('pub_topic', 'response/to/kukaecs/door')

# 門狀態監控
def check_door_status(self):
    # 定期檢查門狀態變化並透過MQTT發布
```

### 新增門控制功能
1. **擴展門配置**: 在door_config.yaml添加新門定義
2. **修改DoorLogic**: 實現新的門控制邏輯
3. **更新MQTT**: 添加新的MQTT事件處理
4. **測試整合**: 確保PLC通訊正常

## ECS整合架構 (基於實際實現)

### 系統整合
```
ECS (設備控制系統)
├─ ecs_core → plc_proxy → keyence_plc → PLC硬體
├─ door_logic → plc_proxy → 門控制PLC
├─ door_controller_node_mqtt → MQTT → KUKA ECS
└─ db_proxy → PostgreSQL (設備信號資料庫)
```

### 實際通訊棧
- **ecs_core**: 0.1秒週期讀取PLC數據，更新資料庫信號
- **door_logic**: 門控制邏輯，支援同步/異步操作  
- **plc_proxy**: ROS 2 PLC客戶端代理
- **keyence_plc**: Keyence PLC通訊協議和記憶體管理
- **MQTT**: 外部系統(KUKA ECS)整合

## 配置管理 (實際檔案)

### 門控制配置
```yaml
# /app/config/door_config.yaml
doors:
  - "1,MR,100,DM,5000"    # 門ID,控制類型,控制地址,狀態類型,狀態地址
  - "2,MR,101,DM,5001"
```

### ECS核心參數 (ROS 2參數)
```python
# ecs_core.py 預設參數
read_ranges: ["DM,7600,20", "DM,5000,200"]  # PLC讀取範圍
db_url_agvc: 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
```

### MQTT參數 (ROS 2參數)
```python
# door_controller_node_mqtt.py 預設參數
broker_host: '192.168.11.206'
broker_port: 2883
username: 'DsH8vSx2uhTao1hlc9vx'
sub_topic: 'request/to/agvc/door'
pub_topic: 'response/to/kukaecs/door'
```

## 狀態監控 (實際實現)

### PLC數據監控 (ecs_core.py)
```python
# 0.1秒週期監控
def main_loop_timer(self):
    self.read_plc_data()        # 讀取PLC數據
    self.write_signals_to_db()  # 更新資料庫信號

# 信號變動檢測
if signal.value != str(value):
    self.get_logger().info(f"變動[{signal.value}] to {str(value)} for {signal.name}")
    signal.value = str(value)
    session.merge(signal)
```

### 門狀態監控 (door_controller_node_mqtt.py)
```python
# 1秒週期門狀態檢查
self.timer = self.create_timer(1.0, self.check_door_status)

# 初始化門狀態
for door_id in self.config.doors.keys():
    state_info = self.door_logic.state_door(door_id)
    self.door_status[door_id] = state_info["state"]
```

### 異常處理
- **PLC通訊失敗**: 記錄錯誤並繼續下次讀取
- **資料庫連接錯誤**: ConnectionPoolManager自動重連
- **MQTT連接中斷**: 自動重連機制
- **門控制失敗**: 錯誤回調和狀態記錄

## 測試與調試 (基於實際節點)

### ECS節點測試
```bash
# 啟動ECS核心節點
ros2 run ecs ecs_core --ros-args -p read_ranges:="['DM,7600,20']"

# 啟動MQTT門控制器
ros2 run ecs door_controller_node_mqtt --ros-args -p broker_host:="192.168.11.206"

# 檢查節點狀態
ros2 node list | grep ecs
ros2 node info /ecs_core
```

### 門控制測試 (透過Web API)
```bash
# 透過web_api測試門控制
curl -X POST http://localhost:8000/door/open -d '{"door_id": 1}'
curl -X GET http://localhost:8000/door/status/1
```

### 資料庫信號檢查
```bash
# 檢查設備信號更新
quick_agvc "psql -U agvc -d agvc -c \"SELECT name, value FROM eqp_signal WHERE dm_address IS NOT NULL;\""
```

## 故障排除 (基於實際實現)

### 常見問題
1. **ECS核心節點無法啟動**: 檢查資料庫連接和PLC proxy狀態
2. **PLC數據讀取失敗**: 驗證Zenoh連接和PLC proxy服務
3. **門控制無回應**: 檢查門配置和PLC連接
4. **MQTT連接失敗**: 驗證MQTT broker設定和網路連通性

### 診斷工具
```bash
# 檢查ECS節點狀態
ros2 node list | grep ecs
ros2 node info /ecs_core

# 檢查PLC通訊
quick_agvc "check_zenoh_status"  # 檢查Zenoh連線
quick_agvc "check_agvc_status"   # 檢查AGVC狀態

# 資料庫連接檢查
quick_agvc "start_db"            # 檢查資料庫連接
```

### 日誌分析
```bash
# ECS節點日誌
agvc_logs | grep -i "ecs_core\|door_controller"

# PLC通訊日誌分析
scripts/log-tools/log-analyzer.sh agvc --filter "plc\|ecs"

# 資料庫操作日誌
scripts/log-tools/log-analyzer.sh agvc --filter "database\|postgresql"
```

### 性能監控
- **PLC讀取頻率**: 0.1秒週期 (可調整read_ranges參數)
- **資料庫更新效率**: 只在信號變動時更新
- **MQTT回應時間**: 門狀態變化實時發布
- **記憶體使用**: PlcMemory緩存管理

### 重要提醒
- **必須在AGVC容器內運行**: 所有ROS 2節點需要正確的環境
- **PLC連線依賴**: 需要確保plc_proxy服務正常運行
- **資料庫依賴**: 需要PostgreSQL連接用於信號存儲
- **配置檔案**: door_config.yaml必須存在且格式正確