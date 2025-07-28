# rcs_ws CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agvc-workspaces.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md
@docs-ai/operations/tools/unified-tools.md

## 📋 模組概述

**RCS (Robot Control System) 車隊控制系統** - 負責 AGV 車隊的任務分派和交通管理，整合 CT 車隊管理、KUKA 車隊管理和交通區域控制功能，是 AGVC 管理系統的車隊調度核心。

### 核心定位
- **車隊調度中心**: 統一管理 CT 和 KUKA 兩套車隊系統
- **任務分派引擎**: 1秒定時器協調的任務派發機制
- **交通管制**: 交通區域控制和衝突避免
- **狀態監控**: 即時監控 AGV 狀態變更和同步

詳細系統架構請參考: @docs-ai/context/workspaces/agvc-workspaces.md

## 專案結構 (實際驗證)
```
src/
├── rcs/                          # RCS 車隊控制系統
│   ├── rcs_core.py              # RCS 核心節點 - 1秒定時器協調中心
│   ├── ct_manager.py            # CT 車隊管理器
│   ├── kuka_manager.py          # KUKA 車隊管理器
│   ├── kuka_dispatcher.py       # KUKA 任務派發器
│   ├── kuka_dispatcher_v2.py    # KUKA 派發器 V2
│   ├── kuka_robot.py            # KUKA 機器人控制
│   ├── kuka_container.py        # KUKA 容器管理
│   ├── kuka_config_manager.py   # KUKA 配置管理
│   ├── kuka_config_cli.py       # 配置管理 CLI 工具
│   ├── kuka_monitor.py          # KUKA 監控模組
│   ├── task_status_simulator.py # 任務狀態模擬器
│   ├── rack_state_manager.py    # 架台狀態管理
│   ├── wcs_priority_scheduler.py # WCS 優先級調度器
│   └── wcs_task_adapter.py      # WCS 任務適配器
├── rcs_interfaces/              # RCS 介面定義 (CMake 專案)
├── traffic_manager/             # 交通管理模組
│   └── traffic_controller.py   # 交通區域控制器
├── test/                        # 測試套件
├── test_config_manager.py      # 配置管理器測試腳本
└── test_ct_dispatch.py         # CT 派發測試腳本
```

## 核心功能 (基於實際實現)

### RCS 核心系統 (rcs_core.py)
- **系統協調**: 1秒定時器主迴圈協調所有車隊管理器
- **資料庫整合**: ConnectionPoolManager 連接 PostgreSQL
- **車隊管理**: 整合 KukaManager 和 CtManager
- **任務模擬**: TaskStatusSimulator 處理任務狀態轉換

### CT 車隊管理 (ct_manager.py)
- **AGV 監控**: 訂閱 `/agv/state_change` 和 `/agv/status` 主題
- **狀態同步**: 監控 AGV 狀態變更並同步至資料庫
- **車隊載入**: 從資料庫載入 CT AGV 資訊
- **分派邏輯**: CT 車隊任務分派處理

### KUKA 車隊系統
- **KUKA Manager**: KUKA 車隊總體管理
- **KUKA Dispatcher**: KUKA 任務派發邏輯 (V1 和 V2)
- **KUKA Robot**: 個別機器人控制
- **KUKA Container**: 容器管理功能
- **配置管理**: KUKA 配置和 CLI 工具

### 交通管理 (traffic_manager)
- **TrafficController**: 交通區域控制器
- **區域管理**: 交通區域狀態管理

## 關鍵檔案

### 核心檔案
- `/rcs/rcs_core.py` - RCS 核心節點，1秒定時器協調中心
- `/rcs/ct_manager.py` - CT 車隊管理器，AGV 狀態監控
- `/rcs/kuka_manager.py` - KUKA 車隊管理器
- `/rcs/task_status_simulator.py` - 任務狀態模擬器

### 配置檔案
- `setup.py` - 僅包含 rcs_core 節點入口點
- `package.xml` - ROS 2 包配置

## 🚀 技術棧特性

詳細技術棧說明請參考: @docs-ai/context/system/technology-stack.md

### 核心技術
- **ROS 2 Jazzy**: 基於最新 ROS 2 發行版
- **PostgreSQL**: 資料庫連接透過 db_proxy.ConnectionPoolManager
- **Zenoh RMW**: 跨容器通訊機制
- **Python 3.12**: 主要開發語言

### RCS 特定架構
- **ROS 2 節點**: rcs_core (唯一 entry_point)
- **AGV 通訊**: agv_interfaces.msg (AgvStateChange, AgvStatus)
- **主題訂閱**: `/agv/state_change`, `/agv/status`
- **定時協調**: 1秒定時器主迴圈

## 🔧 開發環境

### 容器環境要求
**⚠️ 重要**: 所有 ROS 2 程式必須在 AGVC Docker 容器內執行

詳細開發環境設定請參考:
- @docs-ai/context/system/dual-environment.md - 雙環境架構說明
- @docs-ai/operations/development/docker-development.md - 容器開發指導
- @docs-ai/operations/development/ros2-development.md - ROS 2 開發指導
- @docs-ai/operations/tools/unified-tools.md - 統一工具系統

### 服務啟動 (基於實際entry_points)
```bash
# 啟動 RCS 核心節點 (包含所有管理器)
ros2 run rcs rcs_core

# 啟動交通控制器 (獨立包)
ros2 run traffic_manager traffic_controller
```

### RCS 特定測試
```bash
# 測試 CT 任務分派
cd /app/rcs_ws/src/rcs
python3 test_ct_dispatch.py

# 測試配置管理器
python3 test_config_manager.py
```

### 配置管理工具
```bash
# KUKA 配置 CLI 工具
cd /app/rcs_ws/src/rcs
python3 kuka_config_cli.py --help

# 其他配置管理功能
python3 kuka_config_manager.py
```

## 開發指南 (基於實際實現)

### RCS 核心開發 (rcs_core.py)
```python
# 實際實現結構
class RcsCore(Node):
    def __init__(self):
        # 資料庫連線池
        self.db_pool = ConnectionPoolManager(
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
        
        # 車隊管理器初始化
        self.kuka_manager = KukaManager(self)
        self.ct_manager = CtManager(self)
        
        # 任務狀態模擬器
        self.task_status_simulator = TaskStatusSimulator(self.db_pool, self.get_logger())
        
        # 1秒定時器主迴圈
        self.timer_1s = self.create_timer(1.0, self.main_loop)

    def main_loop(self):
        # 任務狀態模擬處理
        self.task_status_simulator.process_task_status_transitions()
        
        # KUKA 和 CT 車隊任務派發
        self.kuka_manager.dispatch()
        self.ct_manager.dispatch()
```

### CT 管理器開發 (ct_manager.py)
```python
# AGV 狀態監控設定
def _setup_agv_monitoring(self):
    # 訂閱 AGV 狀態變更
    self.agv_state_monitor_sub = self.rcs_core.create_subscription(
        AgvStateChange, "/agv/state_change", self.handle_state_change, 10)
    
    # 訂閱 AGV 狀態
    self.agv_status_monitor_sub = self.rcs_core.create_subscription(
        AgvStateChange, "/agv/status", self.agv_status_monitor_callback, 10)
```

## 整合點

### 與其他專案整合
- **db_proxy_ws**: 使用 ConnectionPoolManager 查詢 AGV 狀態和任務資訊
- **agv_ws**: 透過 `/agv/state_change` 和 `/agv/status` 主題接收 AGV 狀態
- **wcs_ws**: 與 WCS 系統進行任務協調
- **kuka_fleet_ws**: KUKA 車隊整合 (如果使用)

### ROS 2 主題整合 (實際訂閱)
```bash
# CT Manager 訂閱的主題
/agv/state_change          # AGV 狀態變更監控
/agv/status               # AGV 狀態監控
```

## 🚨 故障排除

詳細故障排除指導請參考:
- @docs-ai/operations/maintenance/troubleshooting.md - 故障排除流程
- @docs-ai/operations/maintenance/system-diagnostics.md - 系統診斷工具
- @docs-ai/operations/tools/unified-tools.md - 統一工具系統

### RCS 特定問題檢查

#### RCS 核心節點診斷
```bash
# 檢查 RCS 節點狀態
ros2 node list | grep rcs_core
ros2 node info /rcs_core

# 查看節點日誌
ros2 run rcs rcs_core
```

#### AGV 狀態監控檢查
```bash
# 檢查 AGV 主題
ros2 topic list | grep agv
ros2 topic echo /agv/state_change
ros2 topic echo /agv/status
```

#### 任務分派測試
```bash
# 執行 CT 分派測試
cd /app/rcs_ws/src/rcs
python3 test_ct_dispatch.py
python3 test_config_manager.py
```

### 重要依賴檢查
- **資料庫連接**: 需要 PostgreSQL 和 db_proxy 服務正常
- **AGV 主題**: 需要 AGV 系統發布狀態訊息
- **定時器運行**: 檢查 1秒定時器是否正常執行

## 🔗 交叉引用
- 系統概覽: @docs-ai/context/system/rosagv-overview.md
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- AGVC 工作空間: @docs-ai/context/workspaces/agvc-workspaces.md
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- 容器開發: @docs-ai/operations/development/docker-development.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 故障排除: @docs-ai/operations/maintenance/troubleshooting.md
- 統一工具: @docs-ai/operations/tools/unified-tools.md