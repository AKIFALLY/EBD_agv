# ⚠️ 已棄用並歸檔 (DEPRECATED & ARCHIVED)

**棄用日期**: 2025-11-18
**歸檔原因**: TAFL WCS 系統已被 KUKA WCS 完全取代
**替代方案**: 使用 `kuka_wcs_ws` 進行倉儲控制系統開發
**遷移指南**: 參見 [TAFL 到 KUKA WCS 遷移指南](/home/ct/EBD_agv/docs-ai/guides/migration-from-tafl-to-kuka-wcs.md)

本文檔已移至 archived 目錄，僅供歷史參考。不應再用於新的開發工作。

---

# TAFL WCS 流程執行系統

## 🎯 TAFL WCS 概述

本文檔說明 RosAGV 的 TAFL WCS (Task Automation Flow Language - Warehouse Control System) 流程執行系統。這是基於 TAFL v1.1 語言的倉庫控制流程執行器，提供結構化的流程定義和自動執行能力。

## ⚠️ 重要說明
- **TAFL 核心**: `tafl_ws` - TAFL 語言核心實作（解析器、執行器、驗證器）
- **現行系統**: `tafl_wcs_ws` - 基於 TAFL 語言的 WCS 流程執行器

## 📋 系統功能

### 實際定位
```
TAFL WCS 在 RosAGV 生態中的角色
├── 📝 流程執行引擎
│   ├── TAFL YAML 檔案載入
│   ├── 週期性流程執行
│   └── 執行進度追蹤
├── 🗄️ 資料庫操作
│   ├── 查詢 (locations, racks, tasks, carriers)
│   ├── 建立 (tasks, racks)
│   └── 更新 (task status, rack status)
└── 🔄 自動化控制
    ├── 條件判斷執行
    ├── 循環處理
    └── 規則觸發
```

### 系統架構
```
TAFL WCS 實際架構
┌─────────────────────────────────────┐
│        TAFL WCS Node (ROS 2)       │
├─────────────────────────────────────┤
│  TAFLWCSManager                     │
│  ├── 載入 YAML 流程檔案             │
│  ├── 管理執行排程                   │
│  └── 追蹤執行狀態                   │
├─────────────────────────────────────┤
│  TAFLExecutorWrapper                │
│  ├── TAFL Parser                   │
│  ├── TAFL Executor                 │
│  └── TAFL Validator                │
├─────────────────────────────────────┤
│  TAFLDatabaseBridge                 │
│  └── PostgreSQL 連接                │
└─────────────────────────────────────┘
```

## 📝 TAFL 流程執行

### 流程檔案位置
所有 TAFL 流程檔案儲存在：
```
/app/config/tafl/flows/
├── rack_rotation_room_outlet_afull_bempty.yaml  # 架台翻轉流程
├── rack_rotation_room_inlet_aempty_bwork.yaml   # 入口架台處理
├── generate_agv_tasks.yaml                      # AGV 任務生成
├── test_query_verb.yaml                         # 查詢測試
└── test_create_verb.yaml                        # 建立測試
```

### 實際流程範例
```yaml
# rack_rotation_room_outlet_afull_bempty.yaml
metadata:
  id: rack_rotation_room_outlet_afull_bempty
  name: 房間出口架台翻轉（A面滿B面空）
  enabled: true
  version: "1.0.0"

settings:
  execution_interval: 9  # 每 9 秒執行一次

variables:
  work_id: 220001
  rotation_angle: 180
  priority: 5

flow:
  # 查詢所有房間出口位置
  - query:
      target: locations
      where:
        type: room_outlet
      as: outlet_locations

  # 遍歷每個出口位置
  - for:
      in: ${outlet_locations}
      as: location
      do:
        # 查詢該位置的架台
        - query:
            target: racks
            where:
              location_id: ${location.id}
            as: racks_at_location

        # 如果有架台，檢查載具狀況
        - if:
            condition: ${racks_at_location}
            then:
              # 查詢 A 面和 B 面載具
              # 判斷是否需要翻轉
              # 創建翻轉任務
```

### 執行流程
1. **掃描階段**: 每 3 秒掃描 `/app/config/tafl/flows/` 目錄
2. **載入階段**: 載入或更新已變更的 YAML 檔案
3. **驗證階段**: 使用 TAFLValidator 驗證流程結構
4. **執行階段**: 根據 execution_interval 設定週期執行
5. **進度回報**: 透過 ROS 2 主題發布執行進度

## 🔧 核心元件

### TAFLWCSNode (ROS 2 節點)
```python
class EnhancedTAFLWCSNode(Node):
    def __init__(self):
        # ROS 2 參數
        self.declare_parameter('flows_dir', '/app/config/tafl/flows')
        self.declare_parameter('auto_execute', True)
        self.declare_parameter('database_url',
            'postgresql://agvc:password@192.168.100.254:5432/agvc')

        # 初始化管理器
        self.manager = TAFLWCSManager(
            flows_dir=flows_dir,
            database_url=database_url
        )

        # 建立發布者和訂閱者
        self.progress_publisher = self.create_publisher(
            String, '/tafl/execution_progress', 10)
        self.result_publisher = self.create_publisher(
            String, '/tafl/execution_result', 10)
```

### TAFLWCSManager (流程管理器)
```python
class TAFLWCSManager:
    def scan_flows(self, only_enabled=True):
        """掃描並載入 TAFL 流程檔案"""

    def check_and_execute_flows(self, current_time, progress_reporter):
        """檢查並執行到期的流程"""

    def execute_flow_sync(self, flow_id):
        """同步執行單個流程"""
```

### TAFLExecutorWrapper (執行器封裝)
```python
class TAFLExecutorWrapper:
    def execute_flow_sync(self, flow_content, flow_id=None):
        """同步執行 TAFL 流程（類似 RCS 調度）"""
        # 建立新的事件迴圈
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            result = loop.run_until_complete(
                self.execute_flow(flow_content, flow_id))
            return result
        finally:
            loop.close()
```

### TAFLDatabaseBridge (資料庫橋接)
```python
class TAFLDatabaseBridge:
    # 查詢操作
    def query_locations(self, filters=None)
    def query_racks(self, filters=None)
    def query_tasks(self, filters=None)
    def query_carriers(self, filters=None)

    # 建立操作
    def create_task(self, task_data)
    def create_rack(self, rack_data)

    # 更新操作
    def update_task_status(self, task_id, status_id)
    def update_rack(self, rack_id, updates)
```

## 🗄️ 資料庫整合

### 支援的資料表
- **location**: 位置資訊（90 筆）
- **rack**: 架台資訊（8 筆）
- **task**: 任務資訊
- **carrier**: 載具資訊
- **work**: 工作定義（48 種）

### TAFL 查詢範例
```yaml
# 查詢房間出口的架台
- query:
    target: locations
    where:
      type: room_outlet
    as: outlet_locations

# 查詢特定架台的載具
- query:
    target: carriers
    where:
      rack_id: ${rack.id}
      rack_index_min: 1
      rack_index_max: 16
    as: a_side_carriers

# 建立任務
- create:
    target: task
    with:
      name: "架台翻轉任務"
      work_id: 220001
      rack_id: ${rack.id}
      priority: 5
      status_id: 1
```

## 🔄 執行監控

### ROS 2 主題
```bash
# 執行進度
/tafl/execution_progress
# 訊息格式：
{
  "flow_id": "rack_rotation_room_outlet",
  "current_step": 5,
  "total_steps": 10,
  "percentage": 50.0,
  "status": "EXECUTING",
  "message": "Executing step 5/10"
}

# 執行結果
/tafl/execution_result
# 訊息格式：
{
  "flow_id": "rack_rotation_room_outlet",
  "status": "completed",
  "execution_time": 2.5,
  "timestamp": "2025-09-18T10:30:00"
}
```

### 執行狀態
- **PENDING**: 等待執行
- **PARSING**: 解析流程
- **VALIDATING**: 驗證結構
- **EXECUTING**: 執行中
- **COMPLETED**: 完成
- **FAILED**: 失敗
- **SKIPPED**: 跳過（條件不滿足）

## 🚀 使用方式

### 啟動 TAFL WCS 節點
```bash
# 方式 1：直接執行
ros2 run tafl_wcs tafl_wcs_node

# 方式 2：使用 launch 檔案
ros2 launch tafl_wcs tafl_wcs.launch.py

# 方式 3：帶參數執行
ros2 run tafl_wcs tafl_wcs_node \
  --ros-args \
  -p flows_dir:=/app/config/tafl/flows \
  -p auto_execute:=true
```

### 監控執行狀態
```bash
# 查看執行進度
ros2 topic echo /tafl/execution_progress

# 查看執行結果
ros2 topic echo /tafl/execution_result

# 查看節點狀態
ros2 node info /tafl_wcs_node
```

## 🔧 配置管理

### 節點參數
```python
# 預設參數
flows_dir = '/app/config/tafl/flows'      # 流程檔案目錄
auto_execute = True                       # 自動執行
database_url = 'postgresql://agvc:password@192.168.100.254:5432/agvc'
scan_interval = 3.0                       # 掃描間隔（秒）
```

### 流程配置
每個 TAFL 流程檔案包含：
```yaml
metadata:
  id: flow_id               # 流程識別碼
  name: "流程名稱"          # 顯示名稱
  enabled: true            # 是否啟用
  version: "1.0.0"         # 版本號

settings:
  execution_interval: 9    # 執行間隔（秒）

variables:                 # 流程變數
  work_id: 220001
  priority: 5

flow:                      # 流程步驟
  - query: ...
  - if: ...
  - create: ...
```

## 📊 效能指標

### 執行統計
系統追蹤以下指標：
- **total_flows**: 總執行次數
- **successful_flows**: 成功次數
- **failed_flows**: 失敗次數
- **average_execution_time**: 平均執行時間
- **success_rate**: 成功率

### 歷史記錄
執行歷史儲存在：`/tmp/tafl_execution_history.json`

## 🚨 故障排除

### 常見問題

#### 流程未執行
```bash
# 檢查流程檔案
ls -la /app/config/tafl/flows/

# 檢查流程是否啟用
grep "enabled:" /app/config/tafl/flows/*.yaml

# 查看節點日誌
ros2 run tafl_wcs tafl_wcs_node --ros-args --log-level debug
```

#### 資料庫連接失敗
```bash
# 檢查資料庫連接
psql -h 192.168.100.254 -U agvc -d agvc

# 檢查網路
ping 192.168.100.254
```

#### 流程執行錯誤
```bash
# 查看詳細錯誤
ros2 topic echo /tafl/execution_result

# 檢查流程語法
python3 -c "
import yaml
with open('/app/config/tafl/flows/your_flow.yaml') as f:
    yaml.safe_load(f)
"
```

## 💡 最佳實踐

### 流程設計
1. **簡單明確**: 每個流程專注單一任務
2. **錯誤處理**: 使用 if 條件檢查異常情況
3. **效能考量**: 避免過於頻繁的資料庫查詢
4. **除錯友善**: 添加 description 說明每個步驟

### 執行間隔設定
- **即時任務**: 3-5 秒
- **常規任務**: 10-30 秒
- **背景任務**: 60 秒以上

### 資料庫查詢優化
- 使用索引欄位進行查詢
- 限制查詢結果數量
- 快取不常變動的資料

---

**相關文檔：**
- [TAFL 編輯器](../technical-details/tafl-editor.md) - TAFL 流程視覺化編輯
- [KUKA AGV Fleet 整合](../technical-details/kuka-integration.md) - 外部系統整合
- [技術架構](../system-architecture/dual-environment.md) - 整體系統架構
- [故障排除](../operations/troubleshooting.md) - 系統問題診斷