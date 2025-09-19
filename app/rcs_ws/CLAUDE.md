# rcs_ws - 機器人控制系統工作空間

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档
@docs-ai/knowledge/business/eyewear-production-process.md
@docs-ai/knowledge/system/rack-rotation-logic.md
@docs-ai/knowledge/system/rack-management-architecture.md
@docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md
@docs-ai/knowledge/agv-domain/wcs-database-design.md
@docs-ai/knowledge/agv-domain/wcs-workid-system.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/development/testing/testing-standards.md
@docs-ai/operations/guides/system-diagnostics.md
@docs-ai/operations/guides/troubleshooting.md
@docs-ai/operations/tools/unified-tools.md

## 📋 工作空間概述

**機器人控制系統工作空間** 專注於 AGV 車隊的基本任務分派，負責 CT 車隊管理和 KUKA 車隊管理的核心功能，採用簡化設計理念。

### 機器人控制系統工作空間特有功能
- **🚗 雙車隊管理**: CT 車隊和 KUKA 車隊的統一控制
- **📋 任務分派**: 基本的任務查詢和派發功能
- **🚦 交通管制**: 交通區域的佔用和釋放管理
- **⚙️ 簡化設計**: 回歸簡單易懂的車隊控制邏輯

### 簡化設計理念
- **回歸簡單**: 移除複雜的 WCS 適配器和優先度調度器
- **專注核心**: 專注於基本的任務查詢和派發功能
- **易於理解**: 清晰的邏輯，便於維護和擴展
- **統一參數**: 使用一致的 `parameters["model"]` 格式 (小寫)

## ⚠️ 重構警告與教訓 (2025-07-29 事件)

### 重構歷史
- **原始系統**: 包含 7+ 個 KUKA 相關模組，總計超過 3000 行程式碼
  - `kuka_manager.py` (1517 行)、`kuka_dispatcher.py`、`kuka_robot.py`、`kuka_container.py` 等
- **簡化後**: 合併為單一 `simple_kuka_manager.py` (481 行)
- **問題發生**: 重構時誤刪關鍵監控功能，導致前端無法顯示即時狀態
- **修復**: 2025-09-18 透過 commit d77f8275 恢復遺漏功能

### 🔴 絕對不可刪除的功能清單
1. **機器人位置更新** (`on_robot_update`)
   - 同步 KUKA 機器人位置到資料庫
   - 座標轉換: KUKA mm → 像素 (12.5mm = 1px)
   - 觸發 `ModifyLog.mark(session, "agv")` 通知前端

2. **容器狀態管理** (`on_container_update`)
   - 同步 KUKA 容器狀態到 Rack 表
   - 更新 `is_carry` 和 `is_in_map` 狀態
   - 觸發 `ModifyLog.mark(session, "rack")` 通知前端

3. **監控機制** (KukaFleetAdapter)
   - 每 0.05 秒查詢一次機器人和容器狀態
   - 自動啟動監控 (`start_monitoring`)
   - 回調機制連接資料庫更新

4. **ModifyLog 機制**
   - 前後端即時同步的核心
   - `agvc_ui_socket.py` 依賴此機制更新前端顯示
   - 絕對不可移除或註解

### 重構教訓
- ❌ **錯誤**: 將「監控更新」判斷為非核心功能
- ❌ **錯誤**: 沒理解前端對即時資料的依賴
- ❌ **錯誤**: 過度簡化，刪除了必要功能
- ✅ **正確**: 簡化應保留所有核心功能
- ✅ **正確**: 重構前應列出所有功能清單
- ✅ **正確**: 必須有端到端測試驗證

### 核心定位
- **簡化車隊調度**: 管理 CT 和 KUKA 兩套車隊系統
- **基本任務分派**: 1秒定時器的簡單任務派發機制
- **狀態監控**: AGV 狀態變更監控

詳細系統架構請參考: @docs-ai/context/workspaces/agvc-workspaces.md

## 專案結構 (實際驗證 - 整理後)
```
src/
├── rcs/                          # RCS 車隊控制系統 (簡化版本)
│   ├── __init__.py              # Python 包初始化
│   ├── rcs_core.py              # RCS 核心節點 - 1秒定時器協調中心
│   ├── simple_ct_manager.py     # CT 車隊管理器 (簡化版本)
│   ├── simple_kuka_manager.py   # KUKA 車隊管理器 (簡化版本)
│   └── test/                    # 測試套件 (整理後)
│       ├── __init__.py          # Python 包初始化
│       ├── conftest.py          # pytest fixtures 配置
│       ├── pytest.ini           # pytest 配置檔案
│       └── test_rcs_pytest.py   # 主要測試檔案 (pytest 標準)
├── rcs_interfaces/              # RCS 介面定義 (CMake 專案)
└── traffic_manager/             # 交通管理模組
    └── traffic_controller.py   # 交通區域控制器
```

## 核心功能 (基於實際實現)

### RCS 核心系統 (rcs_core.py)
- **系統協調**: 1秒定時器主迴圈協調所有車隊管理器
- **資料庫整合**: ConnectionPoolManager 連接 PostgreSQL
- **車隊管理**: 整合 KukaManager 和 CtManager

### CT 車隊管理 (simple_ct_manager.py)
- **AGV 監控**: 訂閱 `/agv/state_change` 和 `/agv/status` 主題
- **狀態同步**: 監控 AGV 狀態變更並同步至資料庫
- **車隊載入**: 從資料庫載入 CT AGV 資訊
- **分派邏輯**: CT 車隊任務分派處理 (簡化版本)

### KUKA 車隊系統 (simple_kuka_manager.py)
- **KUKA Manager**: KUKA 車隊總體管理 (簡化版本)
- **基本任務派發**: 簡單的 KUKA400i AGV 任務派發邏輯
- **工作 ID 路由**: 210001 (move), 220001 (rack_move), 其他 (workflow)
- **參數統一**: 使用一致的 `parameters["model"]` 格式

### 交通管理 (traffic_manager)
- **TrafficController**: 交通區域控制器
- **區域管理**: 交通區域狀態管理

## 關鍵檔案

### 核心檔案
- `/rcs/rcs_core.py` - RCS 核心節點，1秒定時器協調中心
- `/rcs/simple_ct_manager.py` - CT 車隊管理器，AGV 狀態監控 (簡化版本)
- `/rcs/simple_kuka_manager.py` - KUKA 車隊管理器 (簡化版本)

### 配置檔案
- `setup.py` - 僅包含 rcs_core 節點入口點
- `package.xml` - ROS 2 包配置

## 🚀 RCS 專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

### RCS 系統特定技術棧
- **ROS 2 節點**: rcs_core (唯一 entry_point)
- **AGV 通訊**: agv_interfaces.msg (AgvStateChange, AgvStatus)
- **主題訂閱**: `/agv/state_change`, `/agv/status`
- **定時協調**: 1秒定時器主迴圈

### 服務啟動 (基於實際entry_points)
```bash
# 啟動 RCS 核心節點 (包含所有管理器)
ros2 run rcs rcs_core

# 啟動交通控制器 (獨立包)
ros2 run traffic_manager traffic_controller
```

### RCS 簡化版本測試
```bash
# 使用完整測試套件 (推薦)
bash /app/rcs_ws/run_rcs_tests.sh

# 或執行個別測試
python3 /app/rcs_ws/test_rcs_system.py           # 完整系統測試
python3 /app/rcs_ws/test_database_integration.py # 資料庫整合測試
python3 /app/rcs_ws/test_dispatch_logic.py       # 派發邏輯測試

# 直接啟動 RCS 核心節點
ros2 run rcs rcs_core

# 檢查 RCS 運行狀態
ros2 node info /rcs_core
ros2 topic echo /agv/state_change
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
        
        # 車隊管理器初始化 (簡化版本)
        self.kuka_manager = KukaManager(self)
        self.ct_manager = CtManager(self)
        
        # 1秒定時器主迴圈
        self.timer_1s = self.create_timer(1.0, self.main_loop)

    def main_loop(self):
        # KUKA 和 CT 車隊任務派發
        self.kuka_manager.dispatch()
        self.ct_manager.dispatch()
```

### CT 管理器開發 (simple_ct_manager.py)
```python
# 簡化版本的 CT AGV 任務派發
def dispatch(self):
    """CT AGV 簡單任務派發邏輯"""
    with self.rcs_core.db_pool.session() as session:
        # 查詢未分派的 CT 任務 (非 KUKA400i)
        ct_tasks = session.exec(
            select(Task).where(
                Task.status_id == 1,
                Task.mission_code == None,
                Task.parameters["model"].as_string() != "KUKA400i"
            )
        ).all()
        
        # 基本分派邏輯
        if ct_tasks:
            self.rcs_core.get_logger().info(f"發現 {len(ct_tasks)} 個 CT 任務待分派")
```

### KUKA 管理器開發 (simple_kuka_manager.py)
```python
# 簡化版本的 KUKA400i 任務派發
def dispatch(self):
    """KUKA400i AGV 簡單任務派發"""
    idle_kuka400i_agvs = self.kuka_fleet.select_agv(KukaFleetAdapter.STATUS_IDLE)
    if not idle_kuka400i_agvs:
        return

    with self.rcs_core.db_pool.session() as session:
        # 查詢 KUKA400i 任務 (統一使用小寫 model)
        kuka_tasks = session.exec(
            select(Task).where(
                Task.status_id == 1,
                Task.mission_code == None,
                Task.parameters["model"].as_string() == "KUKA400i"
            )
        ).all()
        
        # 基本工作 ID 路由
        for task in kuka_tasks:
            work_id = int(task.work_id)
            if work_id == 210001:  # move
                self._dispatch_move_task(task, idle_kuka400i_agvs[0])
            elif work_id == 220001:  # rack_move
                self._dispatch_rack_move_task(task, idle_kuka400i_agvs[0])
            else:  # workflow
                self._dispatch_workflow_task(task, idle_kuka400i_agvs[0])
```

## 整合點

### 與其他專案整合 (簡化版本)
- **db_proxy_ws**: 使用 ConnectionPoolManager 查詢 AGV 狀態和任務資訊
- **agv_ws**: 透過 `/agv/state_change` 和 `/agv/status` 主題接收 AGV 狀態
- **tafl_wcs_ws**: 整合 TAFL WCS (Task Automation Flow Language) 的任務自動化流程
- **kuka_fleet_ws**: KUKA 車隊整合 (KukaFleetAdapter)

### WCS 系統整合 (基於實際實現)
基於 @docs-ai/knowledge/agv-domain/wcs-database-design.md 的資料模型：

- **車隊調度協調**: 讀取 WCS 產生的 Task 記錄，根據 work_id 和 priority 進行任務分派
- **狀態回報機制**: 將 AGV 執行狀態同步回 task.status_id，支援 RCS_CANCELLING (52) 等 RCS 特定狀態
- **實時任務處理**: KUKA Manager 直接處理任務狀態轉換和 AGV 派發

### ROS 2 主題整合 (實際訂閱)
```bash
# CT Manager 訂閱的主題
/agv/state_change          # AGV 狀態變更監控
/agv/status               # AGV 狀態監控
```

## 🧪 RCS 專項測試

**⚠️ 通用測試指導請參考**: ../../CLAUDE.md 測試章節

### RCS 專用測試特性
- **pytest 框架**: 遵循統一測試標準
- **測試分類**: unit, integration, functional, database
- **跨工作空間依賴**: 需要 agv_interfaces, db_proxy, kuka_fleet_adapter

### RCS 特定測試執行
```bash
# 日常開發測試 (推薦)
cd /app/rcs_ws
python3 -m pytest src/rcs/test/test_rcs_pytest.py -v

# ROS 2 標準測試
colcon test --packages-select rcs
```

## 🚨 RCS 專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### RCS 系統特定診斷
```bash
# RCS 核心節點診斷
ros2 node list | grep rcs_core
ros2 node info /rcs_core

# AGV 狀態監控檢查
ros2 topic echo /agv/state_change
ros2 topic echo /agv/status

# 任務分派測試
ros2 run rcs rcs_core  # 觀察日誌輸出
```

### RCS 關鍵依賴
- **資料庫連接**: PostgreSQL 和 db_proxy 服務
- **AGV 主題**: AGV 狀態訊息發布
- **定時器運行**: 1秒定時器正常執行

## 🔗 交叉引用

### 相關模組
- **db_proxy_ws**: `../db_proxy_ws/CLAUDE.md` - 資料庫連接池管理
- **agv_ws**: `../agv_ws/CLAUDE.md` - AGV 狀態監控整合
- **tafl_wcs_ws**: `../tafl_wcs_ws/CLAUDE.md` - TAFL WCS 任務自動化流程
- **kuka_fleet_ws**: `../kuka_fleet_ws/CLAUDE.md` - KUKA 車隊整合

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節