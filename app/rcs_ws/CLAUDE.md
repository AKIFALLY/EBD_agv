# rcs_ws CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agvc-workspaces.md
@docs-ai/knowledge/agv-domain/wcs-system-design.md
@docs-ai/knowledge/agv-domain/wcs-database-design.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md
@docs-ai/operations/tools/unified-tools.md

## 📋 模組概述

**RCS (Robot Control System) 簡化車隊控制系統** - 負責 AGV 車隊的基本任務分派，專注於 CT 車隊管理和 KUKA 車隊管理的核心功能，回歸簡單易懂的設計。

### 簡化設計理念
- **回歸簡單**: 移除複雜的 WCS 適配器和優先度調度器
- **專注核心**: 專注於基本的任務查詢和派發功能
- **易於理解**: 清晰的邏輯，便於維護和擴展
- **統一參數**: 使用一致的 `parameters["model"]` 格式 (小寫)

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
- **ai_wcs_ws**: 整合 AI WCS 決策引擎的任務分派
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

## 🧪 測試腳本 (基於 pytest 標準)

### 測試框架政策
**⚠️ 重要**: 自 2025-07-29 起，RCS 模組使用 **pytest 測試框架**，遵循 @docs-ai/operations/development/testing-standards.md 規範。

### pytest 標準測試 (`test_rcs_pytest.py`)
基於 docs-ai 測試標準規範實作的正式測試檔案：

- **測試分類**: 
  - `@pytest.mark.unit` - 單元測試
  - `@pytest.mark.integration` - 整合測試  
  - `@pytest.mark.functional` - 功能測試
  - `@pytest.mark.database` - 資料庫測試

- **測試範圍**:
  - 資料庫連接和查詢功能
  - 任務和 AGV 資料結構驗證
  - 工作 ID 分類和路由邏輯
  - CT/KUKA Manager 初始化
  - 參數格式一致性

### 推薦測試執行方式

#### 1. 日常開發調試 (最簡單)
```bash
# 進入容器並載入環境
agvc_enter && all_source

# 直接執行 pytest 測試 (推薦)
cd /app/rcs_ws
python3 -m pytest src/rcs/test/test_rcs_pytest.py -v

# 執行特定標記的測試
python3 -m pytest src/rcs/test/test_rcs_pytest.py -m unit -v
python3 -m pytest src/rcs/test/test_rcs_pytest.py -m database -v
```

#### 2. 正式提交前 (ROS 2 標準方式)
```bash
# 進入容器並載入環境
agvc_enter && all_source
cd /app/rcs_ws

# ROS 2 標準測試
colcon test --packages-select rcs
colcon test-result --verbose
```

#### 3. 測試覆蓋率檢查
```bash
# 生成測試覆蓋率報告
python3 -m pytest src/rcs/test/test_rcs_pytest.py --cov=rcs --cov-report=html
python3 -m pytest src/rcs/test/test_rcs_pytest.py --cov=rcs --cov-report=term-missing
```

### 測試檔案結構 (已整理)
RCS 測試已整理為標準的 pytest 結構：
- `src/rcs/test/test_rcs_pytest.py` - 主要測試檔案 (pytest 標準)
- `src/rcs/test/conftest.py` - pytest fixtures 配置
- `src/rcs/test/pytest.ini` - pytest 配置檔案

### 測試配置
- **pytest.ini**: pytest 配置檔案，定義測試路徑和標記
- **測試路徑**: `src/rcs/test/`
- **測試標記**: unit, integration, functional, database, slow

### 測試前置條件
- **環境**: 必須在 AGVC 容器內執行
- **工作空間**: 需要載入 AGVC 工作空間 (`all_source`)
- **服務**: PostgreSQL 容器需要正常運行
- **測試框架**: 使用 pytest 框架 (符合最新標準)
- **依賴套件**: 某些測試需要 agv_interfaces 等跨工作空間依賴

### 依賴問題解決 (實現 100% 測試通過)
```bash
# 完整的依賴建置程序 - 從 15 passed, 4 skipped 提升到 19 passed, 0 skipped

# 1. 安裝 Python 依賴
agvc_enter && pip3 install PyYAML

# 2. 建置跨工作空間依賴
# 建置 agv_interfaces
cd /app/agv_ws && colcon build --packages-select agv_interfaces

# 建置 db_proxy (資料庫代理)
cd /app/db_proxy_ws && colcon build --packages-select db_proxy

# 建置 kuka_fleet_adapter (KUKA 車隊整合)
cd /app/kuka_fleet_ws && colcon build --packages-select kuka_fleet_adapter

# 3. 載入完整環境
cd /app/rcs_ws
source /app/agv_ws/install/setup.bash
source /app/db_proxy_ws/install/setup.bash 
source /app/kuka_fleet_ws/install/setup.bash

# 4. 執行測試 (顯著改善測試通過率!)
colcon test --packages-select rcs --event-handlers console_direct+
# 期望結果: 18-19 passed, 0-1 skipped (從 15 passed, 4 skipped 大幅改善)
```

### 測試結果解讀
- **PASSED**: 測試通過
- **FAILED**: 測試失敗，需要檢查
- **SKIPPED**: 測試跳過 (通常因為依賴不可用)
- **覆蓋率**: 顯示程式碼測試覆蓋率

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
# 使用專用測試腳本 (推薦)
python3 /app/rcs_ws/test_dispatch_logic.py

# 或檢查簡化版本的任務分派
ros2 run rcs rcs_core
# 觀察日誌輸出的任務分派訊息
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