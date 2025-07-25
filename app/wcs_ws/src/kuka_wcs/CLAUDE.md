# kuka_wcs - KUKA 倉庫控制系統

## 專案概述
KUKA WCS (Warehouse Control System) 主節點，整合 KukaFleetAdapter 和任務判斷引擎，提供 AGV 車隊管理功能。

## 實際程式結構
```
kuka_wcs/
├── kuka_wcs_node.py           # 主節點 (實際存在)
├── task_decision_engine.py    # 任務判斷引擎 (實際存在)
├── __init__.py
└── task_handler/              # 任務處理器模組
    ├── __init__.py
    ├── empty_rack_to_boxout.py      # 空架到出口傳送箱
    ├── full_rack_to_manual_receive.py  # 滿架到手動接收
    ├── rack_rotate_180.py           # 料架旋轉180度
    └── ready_rack_to_boxin.py       # 準備架到入口箱
```

## 核心功能 (基於實際實現)

### KUKA WCS 主節點 (kuka_wcs_node.py)
- **整合外部系統**: 整合 KukaFleetAdapter 和 AGVCDatabaseClient
- **任務訂閱**: 訂閱 `/agvc/tasks` 主題接收任務列表
- **定時處理**: 每1秒執行一次任務處理邏輯
- **任務決策**: 使用 TaskDecisionEngine 進行任務分析

### 任務判斷引擎 (task_decision_engine.py)
- **機器人狀態管理**: 追蹤機器人電量、狀態、位置
- **容器狀態管理**: 管理容器的空滿狀態和位置
- **任務優先級**: 支援5級優先級 (LOW=1, NORMAL=5, HIGH=10, URGENT=20, EMERGENCY=50)
- **智能分配**: 基於電量和可用性選擇最佳機器人

### 任務處理器 (task_handler/)
- **EmptyRackToBoxoutHandler**: 空架到出口傳送箱處理器
- **條件檢查**: 使用 TaskConditionChecker 進行即時條件驗證
- **資料收集**: 從條件檢查中收集任務所需資料
- **任務創建**: 創建 KUKA 搬運任務到資料庫

## 關鍵檔案

### 實際存在的檔案
- `/kuka_wcs/kuka_wcs_node.py` - WCS 主節點實現
- `/kuka_wcs/task_decision_engine.py` - 任務判斷引擎
- `/kuka_wcs/task_handler/empty_rack_to_boxout.py` - 空架搬運處理器
- `/setup.py` - 定義 entry_points: `kuka_wcs_node`

### 設定與配置
- `setup.py` 中定義的 entry_points
- 無實際配置檔案 (YAML 配置為虛構內容)

## 開發指令 (基於實際 entry_points)

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間 (或使用 all_source 自動檢測)
cd /app/wcs_ws
```

### 服務啟動
```bash
# KUKA WCS 主節點 (實際可用)
ros2 run kuka_wcs kuka_wcs_node
```

### 構建與測試
```bash
# 構建 kuka_wcs 包
colcon build --packages-select kuka_wcs

# 構建整個工作空間
build_ws wcs_ws

# 測試空架條件檢查
cd /app/wcs_ws/src/kuka_wcs
python3 test_empty_rack_condition.py

# 執行單元測試
python3 -m pytest tests/ -v
```

## 實際技術實現

### KUKA WCS 節點架構
```python
class KukaWCSNode(Node):
    def __init__(self):
        # 整合 KukaFleetAdapter
        self.kuka_adapter = KukaFleetAdapter(self)
        
        # 整合資料庫客戶端
        self.db_client = AGVCDatabaseClient(self)
        
        # 任務決策引擎
        self.decision_engine = TaskDecisionEngine(self.get_logger())
        
        # 訂閱任務主題
        self.tasks_subscription = self.create_subscription(
            Tasks, '/agvc/tasks', self.tasks_callback, 10)
        
        # 定時處理任務
        self.task_timer = self.create_timer(1.0, self.process_tasks)
```

### 任務判斷引擎關鍵類別
```python
# 實際存在的資料類別
@dataclass
class RobotInfo:
    robot_id: str
    status: str
    battery_level: int
    current_position: Optional[str] = None
    assigned_task: Optional[str] = None

@dataclass
class ContainerInfo:
    container_code: str
    container_model_code: str
    node_code: str
    empty_full_status: int
    in_map_status: int
    is_carry: int

# 任務處理方法
def make_task_decisions(self) -> List[Dict[str, Any]]:
    # 按優先級排序任務
    # 選擇最佳機器人
    # 返回任務分配建議
```

## 整合點

### 外部依賴
- **kuka_fleet_ws**: 透過 KukaFleetAdapter 整合
- **db_proxy_ws**: 使用 AGVCDatabaseClient 存取資料庫
- **wcs_base**: 使用 BaseTaskHandler 和 TaskConditionChecker

### ROS 2 主題 (實際使用)
```bash
# 訂閱主題
/agvc/tasks                    # 任務列表 (Tasks 消息類型)

# 任務狀態
# status_id == 0: 未執行
# status_id == 1: 已選擇
# status_id == 2: 執行中
```

## 故障排除

### 常見問題

#### KUKA WCS 節點無法啟動
```bash
# 檢查節點狀態
ros2 node list | grep kuka_wcs

# 檢查依賴模組
# 確保 kuka_fleet_ws 和 db_proxy_ws 正常運行

# 檢查 Python 路徑
# 節點會自動添加相關路徑到 sys.path
```

#### 任務條件檢查失敗
```bash
# 檢查 wcs_base 模組
ros2 run wcs_base task_condition_query_node

# 測試任務處理器
cd /app/wcs_ws/src/kuka_wcs
python3 test_empty_rack_condition.py
```

#### 導入錯誤
```bash
# 檢查警告訊息
# 節點使用 try-except 處理導入失敗
# Warning: Could not import KukaFleetAdapter
# Warning: Could not import AGVCDatabaseClient
```

### 除錯技巧
- 檢查 `sys.path` 設定是否正確
- 使用 `self.get_logger().info()` 追蹤執行狀態
- 監控任務接收和處理流程
- 驗證條件檢查器的即時查詢功能

### 重要提醒
- **必須在AGVC容器內運行**: 需要正確的 ROS 2 環境和資料庫連接
- **依賴外部系統**: 需要 kuka_fleet_ws 和 db_proxy_ws 正常運行
- **任務處理器**: 使用基於條件表格的檢查機制
- **即時模式**: TaskConditionChecker 啟用即時查詢模式