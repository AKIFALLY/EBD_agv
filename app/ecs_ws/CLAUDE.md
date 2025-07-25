# ecs_ws CLAUDE.md

## 模組概述
設備控制系統(Equipment Control System)，管理AGV車隊中的所有設備與子系統，提供統一的設備控制介面

## 專案結構
```
src/
└── ecs/                # 設備控制系統核心
    ├── ecs/            # ECS主要邏輯
    ├── controllers/    # 設備控制器
    ├── managers/       # 設備管理器
    └── interfaces/     # 設備介面定義
```

## 核心功能

### 設備管理
- **設備註冊**: 自動發現與註冊設備
- **狀態監控**: 即時設備狀態監控
- **故障管理**: 設備故障檢測與處理
- **配置管理**: 設備參數配置與更新

### 控制架構
- **統一介面**: 標準化設備控制API
- **PLC整合**: 透過PLC proxy控制工業設備
- **即時響應**: 低延遲設備控制回應
- **安全控制**: 設備安全狀態檢查

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

#### 服務管理
```bash
# 【方法1: 透過宿主機工具】(推薦)
source scripts/docker-tools/docker-tools.sh
quick_agvc "start_ecs"               # 啟動 ECS 服務
quick_agvc "ros2 run ecs ecs_node"   # 手動啟動 ECS 節點

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
start_ecs                            # 啟動ECS服務
ros2 run ecs ecs_node               # 手動啟動ECS節點
check_agvc_status                   # 檢查ECS狀態信息
```

### 構建與測試
```bash
build_ws ecs_ws
ros2 test ecs  # ECS系統測試
```

## 設備控制開發

### 設備控制器
```python
# controllers/agv_controller.py
class AGVController:
    def __init__(self, agv_id: str):
        self.agv_id = agv_id
        self.plc_client = PLCProxyClient()
        
    async def move_to_position(self, x: float, y: float):
        """控制AGV移動到指定位置"""
        command = MoveCommand(target_x=x, target_y=y)
        return await self.plc_client.send_command(command)
        
    async def get_status(self) -> AGVStatus:
        """獲取AGV當前狀態"""
        status_data = await self.plc_client.read_status()
        return AGVStatus.parse(status_data)
```

### 設備管理器
```python
# managers/equipment_manager.py
class EquipmentManager:
    def __init__(self):
        self.controllers = {}
        self.status_monitor = StatusMonitor()
        
    def register_device(self, device_id: str, controller):
        """註冊新設備控制器"""
        self.controllers[device_id] = controller
        self.status_monitor.add_device(device_id)
        
    async def execute_command(self, device_id: str, command):
        """執行設備控制指令"""
        if device_id in self.controllers:
            return await self.controllers[device_id].execute(command)
```

### 新增設備類型
1. **創建控制器**: 在`controllers/`下實現設備特定控制邏輯
2. **定義介面**: 在`interfaces/`下定義標準化介面
3. **註冊設備**: 在設備管理器中註冊新設備類型
4. **配置映射**: 更新硬體映射配置檔案

## ECS整合架構

### 系統整合
```
ECS (設備控制)
├─ plc_proxy_ws → keyence_plc_ws → PLC硬體
├─ agv_ws → AGV狀態機
├─ rcs_ws → 機器人控制
└─ wcs_ws → 倉庫管理
```

### PLC通訊棧
- **ECS**: 發送高階設備控制指令
- **plc_proxy**: ROS 2服務代理層
- **keyence_plc**: Keyence PLC通訊協議
- **PLC硬體**: 實際設備控制執行

## 設備配置

### 硬體映射
```yaml
# /app/config/agvc/equipment.yaml
equipment:
  agvs:
    - id: "AGV001"
      type: "cargo_mover"
      plc_address: "192.168.1.101"
      controllers: ["movement", "sensor", "safety"]
      
    - id: "AGV002"  
      type: "loader"
      plc_address: "192.168.1.102"
      controllers: ["movement", "robot_arm", "conveyor"]
      
  stations:
    - id: "CHARGE_01"
      type: "charging_station"
      plc_address: "192.168.1.201"
      controllers: ["charger", "positioning"]
```

### 控制參數
- **響應超時**: 設備控制指令超時時間
- **重試機制**: 控制失敗重試策略
- **狀態更新頻率**: 設備狀態監控週期
- **安全檢查間隔**: 設備安全狀態檢查頻率

## 狀態監控

### 即時監控
```python
# managers/status_monitor.py
class StatusMonitor:
    async def monitor_equipment(self):
        """持續監控所有設備狀態"""
        while True:
            for device_id in self.monitored_devices:
                status = await self.get_device_status(device_id)
                await self.process_status_update(device_id, status)
            await asyncio.sleep(self.update_interval)
```

### 異常處理
- **設備離線**: 自動重連與故障恢復
- **狀態異常**: 安全停止與警報處理
- **通訊錯誤**: PLC通訊故障處理
- **參數異常**: 設備參數範圍檢查

## 測試與調試

### 設備測試
```bash
# 測試AGV控制
ros2 service call /ecs/agv/move ecs_msgs/srv/MoveAGV "{agv_id: 'AGV001', target_x: 10.0, target_y: 5.0}"

# 查看設備狀態
ros2 topic echo /ecs/equipment_status

# 設備診斷
ros2 run ecs equipment_diagnostics --device AGV001
```

### 系統整合測試
- ECS與PLC proxy通訊測試
- 設備控制指令回應測試
- 故障恢復機制測試
- 多設備協調控制測試

## 故障排除

### 常見問題
1. **設備無回應**: 檢查PLC連線與設備狀態
2. **控制指令失敗**: 驗證PLC proxy服務狀態
3. **狀態更新延遲**: 檢查網路延遲與系統負載
4. **設備衝突**: 確認設備排他性控制邏輯

### 診斷工具
```bash
# ECS服務狀態
ros2 service call /ecs/get_system_status

# 設備連線狀態
ros2 topic echo /ecs/device_connections

# PLC通訊診斷
check_zenoh_status  # 檢查Zenoh連線
```

### 日誌分析
- ECS日誌: ROS 2節點輸出
- PLC通訊日誌: 透過plc_proxy查看
- 設備狀態日誌: `/tmp/ecs_equipment.log`

## 性能最佳化

### 控制最佳化
- **批次指令**: 合併多個設備控制指令
- **狀態快取**: 減少重複的狀態查詢
- **優先權管理**: 緊急指令優先處理
- **負載均衡**: 分散設備控制負載

### 資源管理
- 設備控制器執行緒池管理
- 記憶體使用最佳化
- 網路頻寬管理
- CPU資源分配

## 安全機制

### 設備安全
- **安全狀態檢查**: 持續監控設備安全狀態
- **緊急停止**: 實施緊急停止機制
- **防護檢查**: 設備運行前安全檢查
- **故障隔離**: 故障設備自動隔離

### 系統安全
- 設備控制權限管理
- 指令合法性驗證
- 異常行為檢測
- 安全日誌記錄

## 重要提醒
- ECS控制影響整個系統安全，變更需謹慎
- 設備控制指令需包含完整安全檢查
- 必須在AGVC容器內運行
- 與PLC通訊需考慮工業環境特性