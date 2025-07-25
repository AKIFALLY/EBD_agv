# wcs_ws CLAUDE.md

## 模組概述
倉庫控制系統(Warehouse Control System)，包含KUKA WCS、通用WCS模組和WCS基礎架構，提供任務判斷和條件檢查功能

## 專案結構 (實際驗證)
```
src/
├── kuka_wcs/        # KUKA WCS 核心節點
│   ├── kuka_wcs_node.py           # 主節點
│   ├── task_decision_engine.py    # 任務判斷引擎
│   └── task_handler/              # 任務處理器
├── wcs/             # WCS 通用模組
│   ├── wcs_core.py               # WCS 核心
│   ├── task_manager.py           # 任務管理器
│   └── services/                 # WCS 服務
└── wcs_base/        # WCS 基礎架構
    ├── wcs_base_node.py          # 基礎節點
    ├── task_condition_query_node.py # 任務條件查詢節點
    ├── task_condition_checker.py    # 任務條件檢查器
    └── database_manager.py          # 資料庫管理器
```

## 核心功能 (基於實際實現)

### KUKA WCS 系統 (kuka_wcs)
- **任務判斷引擎**: TaskDecisionEngine 提供智能任務判斷
- **KUKA Fleet 整合**: 與 KukaFleetAdapter 協作
- **資料庫整合**: 透過 AGVCDatabaseClient 存取資料庫
- **任務處理器**: 專門的任務處理邏輯

### WCS 通用模組 (wcs)
- **WCS 核心**: wcs_core.py 主要系統邏輯
- **任務管理**: task_manager.py 任務管理功能
- **服務層**: services/ 目錄下的 WCS 服務

### WCS 基礎架構 (wcs_base)
- **條件檢查**: task_condition_checker.py 任務條件驗證
- **資料庫管理**: database_manager.py 資料庫操作
- **查詢服務**: task_condition_query_node.py 條件查詢服務

## 關鍵檔案

### 核心檔案
- `/kuka_wcs/kuka_wcs_node.py` - KUKA WCS 主節點
- `/kuka_wcs/task_decision_engine.py` - 任務判斷引擎
- `/wcs/wcs_core.py` - WCS 核心節點
- `/wcs_base/task_condition_query_node.py` - 任務條件查詢節點

### 配置檔案
- `/kuka_wcs/config/kuka_wcs_config.yaml` - KUKA WCS 配置
- 各 `setup.py` 檔案定義節點入口點

## 實際技術棧
- **ROS 2節點**: kuka_wcs_node, wcs_core, wcs_base_node, task_condition_query_node
- **任務處理**: task_handler/ 模組化任務處理器
- **資料庫**: 透過 AGVCDatabaseClient 和 database_manager
- **外部整合**: KukaFleetAdapter 整合

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間 (或使用 all_source 自動檢測)
cd /app/wcs_ws
```

### 服務啟動 (基於實際entry_points)
```bash
# KUKA WCS 節點
ros2 run kuka_wcs kuka_wcs_node

# WCS 核心節點
ros2 run wcs wcs_core

# WCS 基礎節點
ros2 run wcs_base wcs_base_node

# 任務條件查詢節點
ros2 run wcs_base task_condition_query_node

# 任務條件查詢 CLI
ros2 run wcs_base task_condition_query_cli
```

### 構建與測試
```bash
# 構建整個工作空間
build_ws wcs_ws

# 單獨構建各包
colcon build --packages-select kuka_wcs
colcon build --packages-select wcs
colcon build --packages-select wcs_base

# 執行測試
python3 -m pytest test/ -v

# KUKA WCS 測試
cd /app/wcs_ws/src/kuka_wcs
python3 test_empty_rack_condition.py
```

## 開發指南 (基於實際實現)

### KUKA WCS 節點開發 (kuka_wcs_node.py)
```python
# 實際實現結構
class KukaWCSNode(Node):
    def __init__(self):
        # KUKA Fleet Adapter 整合
        from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
        
        # 資料庫客戶端整合
        from db_proxy.agvc_database_client import AGVCDatabaseClient
        
        # 任務判斷引擎
        from .task_decision_engine import TaskDecisionEngine
```

### 任務條件檢查開發 (task_condition_checker.py)
```python
# 任務條件驗證系統
# 基於 wcs_base 模組實現
```

## 整合點

### 與其他專案整合
- **kuka_fleet_ws**: 透過 KukaFleetAdapter 整合 KUKA Fleet 系統
- **db_proxy_ws**: 使用 AGVCDatabaseClient 存取資料庫
- **rcs_ws**: 與 RCS 系統協作進行任務管理

### 任務處理器 (實際存在)
```bash
# 任務處理器模組
/kuka_wcs/task_handler/
├── empty_rack_to_boxout.py
├── full_rack_to_manual_receive.py
├── rack_rotate_180.py
└── ready_rack_to_boxin.py
```

## 故障排除

### 常見問題

#### KUKA WCS 節點無法啟動
```bash
# 檢查節點狀態
ros2 node list | grep kuka_wcs

# 檢查 KUKA Fleet Adapter 連接
# (需要 kuka_fleet_ws 正常運行)

# 檢查資料庫連接
quick_agvc "start_db"
```

#### 任務條件檢查失敗
```bash
# 測試任務條件查詢
ros2 run wcs_base task_condition_query_cli

# 檢查資料庫管理器
# (database_manager.py)
```

#### WCS 核心問題
```bash
# 檢查 WCS 核心節點
ros2 run wcs wcs_core

# 檢查任務管理器狀態
# (task_manager.py)
```

### 除錯技巧
- 檢查 KUKA Fleet Adapter 是否可用
- 驗證資料庫連接狀態
- 使用任務條件查詢 CLI 工具
- 檢查各節點的日誌輸出

### 重要提醒
- **必須在AGVC容器內運行**: 需要正確的 ROS 2 環境
- **依賴外部系統**: 需要 kuka_fleet_ws 和 db_proxy_ws
- **模組化設計**: 三個獨立包提供不同功能層級
- **任務處理器**: kuka_wcs 包含專門的任務處理邏輯