# TAFL WCS 工作空間 (tafl_wcs_ws)

## 📋 基本資訊

**啟動狀態**: ⚠️ 手動啟動 (需明確啟動 TAFL WCS 節點)
**運行環境**: 🖥️ AGVC 管理系統
**主要功能**: TAFL 倉庫控制系統 - 基於 TAFL v1.1 語言的 WCS 實作
**依賴狀態**: 依賴 tafl_ws (TAFL 解析器) 和 db_proxy_ws (資料庫操作)
**手動啟動**: 可使用 `ros2 run tafl_wcs tafl_wcs_node` 啟動

## 📋 專案概述

TAFL WCS (Task Automation Flow Language - Warehouse Control System) 是基於 TAFL v1.1 語言規格的倉庫控制系統實作，是RosAGV系統的核心流程執行器。該系統提供了結構化和標準化的流程定義與執行能力，完整支援 TAFL v1.1 的 6 段式結構和 5-Level 變數作用域。

作為 RosAGV 系統的核心流程控制組件，TAFL WCS 負責執行複雜的倉庫自動化流程，包括料架旋轉、任務建立、AGV 調度等關鍵業務邏輯。系統整合了 PostgreSQL 資料庫，提供完整的資料持久化和狀態管理功能。

**重要特點**:
- 完整支援 TAFL v1.1 規格（6 段式結構）
- 4-Phase 執行模型（Settings → Preload → Rules → Flow）
- 5-Level 變數作用域管理
- 同步執行模式，避免 asyncio 記憶體問題
- ROS 2 生命週期管理

## 🔗 依賴關係

### 依賴的工作空間
- **tafl_ws**: 提供 TAFL 解析器和執行引擎 (✅ 必要依賴)
  - TAFLParser: YAML 解析和 AST 構建
  - TAFLExecutor: 流程執行引擎
  - TAFLValidator: 語法驗證器
- **db_proxy_ws**: 提供資料庫連接池管理 (✅ 必要依賴)
  - ConnectionPoolManager: PostgreSQL 連接池
- **agv_interfaces**: 提供 ROS 2 訊息和服務定義
- **rclpy**: ROS 2 Python 客戶端庫

### 被依賴的工作空間
- **rcs_ws**: 機器人控制系統使用 TAFL WCS 執行流程
- **agvcui**: Web 界面可能調用 TAFL 流程
- **測試工具**: 流程測試和驗證工具

### 資料庫依賴
- **PostgreSQL 16**: 主資料庫 (192.168.100.254:5432)
- **資料庫名稱**: agvc
- **連接字串**: `postgresql://agvc:password@192.168.100.254:5432/agvc`

## 🏗️ 專案結構

```
tafl_wcs_ws/
├── src/
│   └── tafl_wcs/
│       ├── tafl_wcs/                    # 主程式碼目錄
│       │   ├── __init__.py
│       │   ├── tafl_db_bridge.py        # 資料庫橋接模組
│       │   ├── tafl_executor_wrapper.py # TAFL 執行器封裝
│       │   ├── tafl_functions.py        # TAFL 函數庫擴展
│       │   ├── tafl_wcs_manager.py      # 流程管理器
│       │   └── tafl_wcs_node.py         # ROS 2 主節點
│       ├── test/                        # 測試目錄
│       │   ├── __init__.py
│       │   ├── README.md                # 測試說明文檔
│       │   ├── run_all_tests.py         # 統一測試入口
│       │   ├── test_parking_flows.py    # 空料架停車區管理測試
│       │   ├── test_machine_to_prepare.py    # 射出機停車格→準備區測試
│       │   ├── test_full_rack_to_collection.py  # 完成料架→收料區測試
│       │   ├── test_rack_rotation.py    # 架台翻轉測試
│       │   ├── test_room_dispatch.py    # 房間投料調度測試
│       │   ├── test_duplicate_prevention.py  # 重複執行防護測試
│       │   ├── test_all_tafl_flows.py   # 所有 TAFL 流程測試
│       │   ├── test_db_connection.py    # 資料庫連接測試
│       │   └── test_tafl_v11_compliance.py   # TAFL v1.1 合規性測試
│       ├── launch/
│       │   └── tafl_wcs.launch.py       # ROS 2 Launch 檔案
│       ├── config/                      # 配置目錄
│       ├── package.xml                  # ROS 2 套件描述
│       └── setup.py                     # Python 套件設定
├── pytest.ini                           # pytest 配置
├── run_tests.sh                         # 測試執行腳本
├── CLAUDE.md                            # AI Agent 指導文件
└── README.md                            # 本文件
```

## ⚙️ 主要功能

### 1. TAFL v1.1 執行引擎
**核心功能**:
- **6 段式結構支援**: metadata, settings, preload, rules, variables, flow
- **4-Phase 執行模型**: 按順序執行 Settings → Preload → Rules → Flow
- **5-Level 變數作用域**: Loop → Flow → Global → Preload → Rules
- **同步執行**: 使用同步執行模式，避免記憶體洩漏問題

### 2. 資料庫整合 (TAFLDatabaseBridge)
**支援的操作**:

**查詢操作**:
- `query_locations()` - 查詢位置資訊
- `query_racks()` - 查詢料架資訊
- `query_tasks()` - 查詢任務資訊
- `query_works()` - 查詢工作定義
- `query_work_by_id(work_id)` - 查詢特定工作
- `query_parking_by_room(room)` - 查詢停車位

**建立操作**:
- `create_task(task_data)` - 建立新任務
- `create_rack(rack_data)` - 建立新料架

**更新操作**:
- `update_task_status(task_id, status)` - 更新任務狀態
- `update_rack(rack_id, data)` - 更新料架資訊
- `update_rack_side_completed(rack_id, side, completed)` - 更新料架面完成狀態
- `update_location_status(location_id, status)` - 更新位置狀態

### 3. TAFL 函數擴展 (TAFLFunctions)
**自訂函數**:
- `generate_task_id()` - 生成任務 ID
- `generate_rack_id()` - 生成料架 ID
- `calculate_priority(task_type)` - 計算優先級
- `validate_location(location_id)` - 驗證位置
- `check_rack_status(rack_id)` - 檢查料架狀態

### 4. ROS 2 服務介面
**提供的服務**:
- `/tafl_wcs/execute_flow` - 執行 TAFL 流程
- `/tafl_wcs/list_flows` - 列出可用流程
- `/tafl_wcs/get_flow_status` - 查詢流程狀態
- `/tafl_wcs/stop_flow` - 停止執行中的流程

### 5. 流程管理器 (TAFLWcsManager)
**核心功能**:
- **流程載入**: 從檔案或字串載入 TAFL 流程
- **流程驗證**: 執行前驗證流程語法
- **執行管理**: 管理流程執行生命週期
- **狀態追蹤**: 追蹤流程執行狀態
- **錯誤處理**: 優雅處理執行錯誤

## 🔧 核心 API

### TAFL 執行器包裝
```python
from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper

# 建立執行器
executor = TAFLExecutorWrapper(db_bridge)

# 執行流程
result = await executor.execute_flow(flow_content)

# 設定變數
executor.set_variable('room_id', 1)

# 查詢變數
value = executor.get_variable('rack_id')
```

### 資料庫橋接
```python
from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge

# 建立資料庫連接
db_bridge = TAFLDatabaseBridge(DATABASE_URL)

# 查詢操作
racks = db_bridge.query_racks({'room': 1})
tasks = db_bridge.query_tasks({'status': 'pending'})

# 建立操作
task_id = db_bridge.create_task({
    'work_id': 'WRK001',
    'priority': 5,
    'status': 'pending'
})

# 更新操作
db_bridge.update_task_status(task_id, 'completed')
```

### ROS 2 節點使用
```python
import rclpy
from tafl_wcs.tafl_wcs_node import TAFLWcsNode

# 初始化 ROS 2
rclpy.init()

# 建立節點
node = TAFLWcsNode()

# 執行節點
rclpy.spin(node)

# 清理
node.destroy_node()
rclpy.shutdown()
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
cd /app/tafl_wcs_ws
colcon build --packages-select tafl_wcs
source install/setup.bash

# 執行所有測試
./run_tests.sh

# 執行特定測試
python3 -m pytest src/tafl_wcs/test/test_db_connection.py -v
python3 -m pytest src/tafl_wcs/test/test_tafl_v11_compliance.py -v
```

### 2. 資料庫連接測試
```bash
# 測試資料庫連接
cd /app/tafl_wcs_ws/src/tafl_wcs
python3 -m pytest test/test_db_connection.py -v

# 驗證連接池
python3 -c "
from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
db = TAFLDatabaseBridge('postgresql://agvc:password@192.168.100.254:5432/agvc')
print('✅ 資料庫連接成功')
"
```

### 3. TAFL v1.1 合規性測試
```bash
# 測試 TAFL v1.1 合規性
python3 -m pytest test/test_tafl_v11_compliance.py -v

# 執行業務流程測試
cd src/tafl_wcs/test
python3 run_all_tests.py
```

## 🚀 啟動和使用

### 1. 環境準備
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入環境
source /app/setup.bash
agvc_source

# 進入工作空間
cd /app/tafl_wcs_ws
```

### 2. 建置工作空間
```bash
# 清理舊建置（如需要）
rm -rf build install log

# 建置
colcon build --packages-select tafl_wcs

# 載入安裝
source install/setup.bash
```

### 3. 啟動節點
```bash
# 方法 1: 直接啟動
ros2 run tafl_wcs tafl_wcs_node

# 方法 2: 使用 launch 檔案
ros2 launch tafl_wcs tafl_wcs.launch.py

# 方法 3: 在背景執行
ros2 run tafl_wcs tafl_wcs_node &
```

### 4. 執行流程
```bash
# 使用 ROS 2 服務執行流程
ros2 service call /tafl_wcs/execute_flow tafl_wcs_interfaces/srv/ExecuteFlow "{
  flow_path: '/app/config/tafl/rack_rotation_room_outlet.tafl.yaml'
}"

# 查詢流程狀態
ros2 service call /tafl_wcs/get_flow_status tafl_wcs_interfaces/srv/GetFlowStatus

# 列出可用流程
ros2 service call /tafl_wcs/list_flows tafl_wcs_interfaces/srv/ListFlows
```

## 📊 TAFL 流程範例

### 簡單流程範例
```yaml
metadata:
  id: simple_example
  name: Simple Example Flow
  version: 1.1.0

variables:
  room_id: 1
  rack_count: 0

flow:
  - query:
      target: racks
      where:
        room: ${room_id}
      as: available_racks

  - set:
      rack_count: ${#available_racks}

  - log:
      message: "Found ${rack_count} racks in room ${room_id}"
```

### 料架旋轉流程範例
```yaml
metadata:
  id: rack_rotation
  name: Rack Rotation at Room Outlet
  version: 1.1.0

settings:
  database:
    url: postgresql://agvc:password@192.168.100.254:5432/agvc

preload:
  - query:
      target: parking_locations
      where:
        room: ${room_id}
      as: parking_spots

variables:
  room_id: 1
  target_location: null

flow:
  - set:
      target_location: ${parking_spots[0].location_id}

  - create:
      target: task
      data:
        type: rack_rotation
        location: ${target_location}
        priority: 5
      as: rotation_task

  - log:
      message: "Created rotation task ${rotation_task.task_id}"
```

## 🎯 里程碑和開發歷程

### v1.0.0 (2024-08) - 初始實作
- 基本 TAFL v1.0 解析和執行
- 資料庫整合基礎
- ROS 2 服務介面

### v1.1.0 (2024-08) - TAFL v1.1 升級
- 6 段式結構支援
- 5-Level 變數作用域
- Preload 階段實作
- Rules 階段支援

### v1.1.2 (2024-09) - 語法標準化
- 統一使用 'as' 參數儲存結果
- 改進錯誤處理
- 增強測試覆蓋

## 🚨 故障排除

### 常見問題

#### 資料庫連接失敗
```bash
# 檢查資料庫狀態
docker compose -f docker-compose.agvc.yml ps postgres

# 測試連接
psql -h 192.168.100.254 -U agvc -d agvc

# 檢查連接池
ros2 service call /db_proxy/test_connection
```

#### TAFL 解析錯誤
```bash
# 驗證 TAFL 檔案
python3 -c "
from tafl.parser import TAFLParser
parser = TAFLParser()
with open('flow.yaml', 'r') as f:
    ast = parser.parse_string(f.read())
print('✅ TAFL 解析成功')
"
```

#### 節點無法啟動
```bash
# 檢查依賴
ros2 pkg list | grep tafl

# 檢查環境變數
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION

# 重新載入環境
source /app/setup.bash
agvc_source
```

## 🔗 相關文檔

- **TAFL Parser**: `/app/tafl_ws/README.md`
- **TAFL 語言規格**: `@docs-ai/knowledge/system/tafl/tafl-language-specification.md`
- **資料庫設計**: `@docs-ai/knowledge/agv-domain/wcs-database-design.md`
- **WCS 系統設計**: `@docs-ai/knowledge/agv-domain/wcs-system-design.md`
- **測試標準**: `@docs-ai/operations/development/testing/testing-standards.md`