# 資料庫代理工作空間 (db_proxy_ws)

## 📋 基本資訊

**啟動狀態**: ⚠️ 手動啟動 (未在容器啟動腳本中自動啟動)
**運行環境**: 🖥️ AGVC 管理系統 (主要)
**主要功能**: PostgreSQL 資料庫代理服務和 ORM 管理
**依賴狀態**: 使用虛擬環境套件 (sqlalchemy, psycopg2, sqlmodel)，提供資料庫核心功能
**手動啟動**: 可使用 `ros2 run db_proxy agvc_database_node` 啟動

## 📋 專案概述

資料庫代理工作空間提供 RosAGV 系統與 PostgreSQL 資料庫之間的橋接服務。該工作空間使用 SQLModel 和 SQLAlchemy 實現了完整的 ORM 資料庫操作介面，包括任務管理、載具追蹤、貨架狀態、設備監控、交通區域管理等核心功能，並提供 ROS 2 服務介面供其他模組使用。作為 AGVC 管理系統的核心組件，它負責所有資料的持久化和管理。

## 🔗 依賴關係

### 虛擬環境套件依賴
- **sqlalchemy**: Python ORM 框架，提供資料庫抽象層
- **psycopg2**: PostgreSQL 資料庫適配器，提供 Python-PostgreSQL 連線
- **sqlmodel**: 基於 SQLAlchemy 的現代 ORM，結合 Pydantic 驗證

### 系統套件依賴
- **PostgreSQL**: 資料庫系統 (192.168.100.254:5432)

### 被依賴的工作空間
- **web_api_ws**: 使用 `db_proxy.crud` 模組進行資料庫操作
- **ecs_ws**: 使用資料庫服務進行任務和設備管理
- **rcs_ws**: 使用 AGV 和交通區域管理功能
- **tafl_wcs_ws**: 使用貨架和載具管理功能
- **外部系統**: 任何需要資料持久化的模組

### 外部依賴
- **ROS 2**: `rclpy`, `rclpy.executors`
- **Python 標準庫**: `json`, `datetime`, `threading`

## 🏗️ 專案結構

```
db_proxy_ws/
├── src/
│   ├── db_proxy/                  # 主要代理服務套件
│   │   ├── db_proxy/
│   │   │   ├── agvc_database_node.py      # AGVC 資料庫節點 (使用虛擬環境 sqlmodel)
│   │   │   ├── agvc_database_client.py    # 資料庫客戶端 (使用虛擬環境 sqlmodel)
│   │   │   ├── connection_pool_manager.py # 連線池管理器 (使用虛擬環境 sqlalchemy)
│   │   │   ├── ros_converter.py           # ROS 訊息轉換器
│   │   │   ├── agvc_logger_sub.py         # AGVC 日誌訂閱器
│   │   │   ├── models/                    # 資料模型定義 (使用虛擬環境 sqlmodel) - 24個模型檔案
│   │   │   │   ├── __init__.py           # 模型匯出
│   │   │   │   ├── agvc_task.py          # 任務模型 (Task, Work, TaskStatus)
│   │   │   │   ├── agvc_rcs.py           # RCS 模型 (AGV, AGVContext, TrafficZone)
│   │   │   │   ├── rack.py               # 貨架模型 (Rack)
│   │   │   │   ├── rack_status.py        # 貨架狀態模型 (RackStatus)
│   │   │   │   ├── carrier.py            # 載具模型 (Carrier)
│   │   │   │   ├── carrier_status.py     # 載具狀態模型 (CarrierStatus)
│   │   │   │   ├── agvc_eqp.py           # 設備模型 (Eqp, EqpPort, EqpSignal)
│   │   │   │   ├── agvc_location.py      # 位置模型 (Location, LocationStatus)
│   │   │   │   ├── agvc_product.py       # 產品模型 (Product, ProcessSettings)
│   │   │   │   ├── agvc_kuka.py          # KUKA 模型 (KukaNode, KukaEdge)
│   │   │   │   ├── client.py             # 客戶端模型 (Client)
│   │   │   │   ├── machine.py            # 機台模型 (Machine) - 含 workspace 陣列
│   │   │   │   ├── user.py               # 使用者模型 (User)
│   │   │   │   ├── room.py               # 房間模型 (Room)
│   │   │   │   ├── license.py            # 授權模型 (License)
│   │   │   │   ├── node.py               # 節點模型 (Node)
│   │   │   │   ├── node_type.py          # 節點類型模型 (NodeType)
│   │   │   │   ├── edge.py               # 邊緣模型 (Edge)
│   │   │   │   ├── agv_status.py         # AGV 狀態模型 (AGVStatus)
│   │   │   │   ├── log_level.py          # 日誌等級模型 (LogLevel)
│   │   │   │   ├── rosout_log.py         # ROS 日誌模型 (RosoutLog)
│   │   │   │   ├── runtime_log.py        # 運行時日誌模型 (RuntimeLog)
│   │   │   │   ├── modify_log.py         # 修改日誌模型 (ModifyLog)
│   │   │   │   └── audit_log.py          # 稽核日誌模型 (AuditLog)
│   │   │   ├── crud/                     # CRUD 操作模組 - 21個實作檔案
│   │   │   │   ├── __init__.py           # CRUD 模組匯出
│   │   │   │   ├── base_crud.py          # 基礎 CRUD 類別
│   │   │   │   ├── task_crud.py          # 任務 CRUD
│   │   │   │   ├── rack_crud.py          # 貨架 CRUD
│   │   │   │   ├── carrier_crud.py       # 載具 CRUD
│   │   │   │   ├── carrier_status_crud.py # 載具狀態 CRUD
│   │   │   │   ├── agv_crud.py           # AGV CRUD
│   │   │   │   ├── agv_status_crud.py    # AGV 狀態 CRUD
│   │   │   │   ├── eqp_crud.py           # 設備 CRUD
│   │   │   │   ├── location_crud.py      # 位置 CRUD
│   │   │   │   ├── product_crud.py       # 產品 CRUD
│   │   │   │   ├── process_settings_crud.py # 製程設定 CRUD
│   │   │   │   ├── machine_crud.py       # 機台 CRUD
│   │   │   │   ├── user_crud.py          # 使用者 CRUD
│   │   │   │   ├── room_crud.py          # 房間 CRUD
│   │   │   │   ├── license_crud.py       # 授權 CRUD
│   │   │   │   ├── node_crud.py          # 節點 CRUD
│   │   │   │   ├── traffic_crud.py       # 交通區域 CRUD
│   │   │   │   ├── runtime_log_crud.py   # 運行日誌 CRUD
│   │   │   │   ├── rosout_log_crud.py    # ROS 日誌 CRUD
│   │   │   │   ├── modify_log.py         # 修改日誌 CRUD
│   │   │   │   └── audit_log_crud.py     # 稽核日誌 CRUD
│   │   │   ├── sql/                      # SQL 腳本和初始化
│   │   │   │   ├── db_install.py         # 資料庫初始化 (使用虛擬環境 sqlalchemy)
│   │   │   │   ├── sql_query.py          # SQL 查詢工具
│   │   │   │   └── init_data/            # 初始化資料
│   │   │   ├── examples/                 # 使用範例 (目前為空)
│   │   │   └── test/                     # 標準測試目錄 - 3個測試檔案
│   │   │       ├── test_connection_pool_manager.py  # 連線池測試
│   │   │       ├── test_license.py                  # 授權測試
│   │   │       └── test_base_crud.py                # 基礎 CRUD 測試
│   │   ├── package.xml
│   │   └── setup.py
│   └── db_proxy_interfaces/       # 服務和訊息介面定義
│       ├── srv/                   # 服務定義
│       │   ├── SqlQuery.srv              # SQL 查詢服務
│       │   ├── GenericQuery.srv          # 通用查詢服務
│       │   ├── UpdateTask.srv            # 任務更新服務
│       │   ├── UpdateRack.srv            # 貨架更新服務
│       │   ├── UpdateCarrier.srv         # 載具更新服務
│       │   ├── CarrierQuery.srv          # 載具查詢服務
│       │   ├── RackQuery.srv             # 貨架查詢服務
│       │   ├── EqpSignalQuery.srv        # 設備信號查詢服務
│       │   ├── AcquireTrafficArea.srv    # 獲取交通區域服務
│       │   ├── ReleaseTrafficArea.srv    # 釋放交通區域服務
│       │   └── AddTrafficArea.srv        # 新增交通區域服務
│       ├── msg/                   # 訊息定義
│       │   ├── Task.msg                  # 任務訊息
│       │   ├── Tasks.msg                 # 任務列表訊息
│       │   ├── Rack.msg                  # 貨架訊息
│       │   ├── Racks.msg                 # 貨架列表訊息
│       │   ├── Carrier.msg               # 載具訊息
│       │   ├── Work.msg                  # 工作訊息
│       │   ├── Works.msg                 # 工作列表訊息
│       │   ├── Location.msg              # 位置訊息
│       │   ├── Locations.msg             # 位置列表訊息
│       │   ├── Eqp.msg                   # 設備訊息
│       │   ├── Eqps.msg                  # 設備列表訊息
│       │   ├── EqpSignal.msg             # 設備信號訊息
│       │   ├── AGV.msg                   # AGV 訊息
│       │   ├── AGVs.msg                  # AGV 列表訊息
│       │   ├── Fetch.msg                 # 抓取訊息
│       │   └── Tables.msg                # 資料表訊息
│       ├── CMakeLists.txt
│       └── package.xml
├── scripts/                       # 資料庫管理腳本
│   ├── check_db_init.sh           # 資料庫初始化檢查腳本
│   ├── init_database.sh           # 一鍵資料庫初始化腳本
│   ├── test_connection.py         # 連線測試腳本 (使用虛擬環境套件)
│   ├── check_db_status.sh         # 資料庫狀態檢查腳本
│   └── README.md                  # 腳本使用說明
├── CLAUDE.md                      # AI Agent 指導文檔
└── README.md                      # 工作空間說明
```

## ⚙️ 主要功能

### 1. 資料庫連線管理 (使用虛擬環境 SQLAlchemy)
**ConnectionPoolManager 核心功能**:
- **連線池管理**: 使用 SQLAlchemy QueuePool，支援高並發存取
- **連線配置**: 預設 5 個基本連線，最大 10 個連線，30 秒超時
- **自動重連**: 連線斷線時自動重新建立連線，180 秒連線回收
- **效能監控**: 即時監控連線池狀態和效能指標，5 秒間隔日誌
- **資料庫初始化**: 使用 SQLModel.metadata.create_all 自動建立資料表
- **時區設定**: PostgreSQL 自動設定為 Asia/Taipei 時區

### 2. 任務管理系統
- **任務 CRUD**: 完整的任務建立、讀取、更新、刪除操作
- **任務狀態追蹤**: 即時更新任務執行狀態
- **工作流程管理**: 支援複雜的工作流程定義和執行
- **優先級管理**: 任務優先級排程和管理

### 3. 載具和貨架管理
- **載具追蹤**: 即時追蹤載具位置和狀態
- **貨架管理**: 貨架狀態、位置、內容物管理
- **庫存管理**: 與貨架關聯的產品庫存管理
- **搬運記錄**: 完整的搬運歷史記錄

### 4. 設備監控系統
- **設備狀態**: 即時監控設備運行狀態
- **信號管理**: 設備信號的讀取和控制
- **設備埠管理**: 設備連接埠的配置和管理
- **警報系統**: 設備異常警報和通知

### 5. 交通區域管理
- **區域控制**: 交通區域的獲取和釋放
- **衝突避免**: 防止多個 AGV 同時進入同一區域
- **路徑規劃**: 支援路徑規劃的交通管制
- **動態調整**: 即時調整交通區域配置

### 6. 通用查詢服務
- **SQL 查詢**: 支援原生 SQL 查詢，使用 SQLAlchemy text() 函數
- **通用 CRUD**: 通用的資料庫操作介面，支援 SELECT/INSERT/UPDATE/DELETE
- **批次操作**: 支援批次資料處理和交易管理
- **JSON 回應**: 統一的 JSON 格式回應，支援 datetime 序列化

## 🔧 核心 API

### AGVCDatabaseNode 主節點
```python
from db_proxy.agvc_database_node import AGVCDatabaseNode
import rclpy
from rclpy.executors import MultiThreadedExecutor

# 初始化 ROS 2 節點
rclpy.init()
node = AGVCDatabaseNode()

# 使用多執行緒執行器
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()

# 清理
node.destroy_node()
rclpy.shutdown()
```

### AGVCDatabaseClient 客戶端
```python
import rclpy
from rclpy.node import Node
from db_proxy.agvc_database_client import AGVCDatabaseClient
from db_proxy_interfaces.srv import GenericQuery

# 建立客戶端節點
rclpy.init()
node = Node('test_client')
db_client = AGVCDatabaseClient(node)

# 同步查詢範例
result = db_client.generic_query(
    table_name="task",
    columns=["id", "name", "status_id"],
    data=[],
    condition="status_id = 1",
    mode="select"
)

if result and result.success:
    print(f"查詢成功: {result.results}")
else:
    print("查詢失敗")

# 非同步查詢範例
def callback(result):
    if result and result.success:
        print(f"非同步查詢成功: {result.results}")

db_client.async_generic_query(
    table_name="rack",
    columns=["id", "name", "location_id"],
    data=[],
    condition="location_id IS NOT NULL",
    mode="select",
    callback=callback
)

# 清理
db_client.destroy()
node.destroy_node()
rclpy.shutdown()
```

### ConnectionPoolManager 連線池管理
```python
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select, text
from db_proxy.models import Task

# 建立連線池
db_url = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
pool = ConnectionPoolManager(db_url)

# 使用 Session 進行資料庫操作
with pool.get_session() as session:
    # SQLModel 查詢
    tasks = session.exec(select(Task).where(Task.status_id == 1)).all()
    print(f"找到 {len(tasks)} 個任務")

    # 原生 SQL 查詢
    result = session.exec(text("SELECT COUNT(*) FROM task")).first()
    print(f"總任務數: {result}")

    # 新增資料
    new_task = Task(
        name="測試任務",
        description="API 測試",
        status_id=1,
        priority=1
    )
    session.add(new_task)
    session.commit()
    session.refresh(new_task)
    print(f"新增任務 ID: {new_task.id}")

# 關閉連線池
pool.shutdown()
```

### CRUD 操作範例
```python
from db_proxy.crud.task_crud import task_crud
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task

# 建立連線池
pool = ConnectionPoolManager("postgresql+psycopg2://agvc:password@192.168.100.254/agvc")

# 使用 CRUD 操作
with pool.get_session() as session:
    # 建立任務
    task_data = {
        "name": "新任務",
        "description": "CRUD 測試",
        "status_id": 1,
        "priority": 2
    }
    new_task = task_crud.create(session, obj_in=task_data)

    # 查詢任務
    task = task_crud.get(session, id=new_task.id)
    print(f"任務名稱: {task.name}")

    # 更新任務
    update_data = {"description": "已更新的描述"}
    updated_task = task_crud.update(session, db_obj=task, obj_in=update_data)

    # 刪除任務
    task_crud.remove(session, id=task.id)
```

## 📡 主要服務介面

### SqlQuery.srv
```
# Request
string query_string

---
# Response
string json_result
bool success
string message
```

### GenericQuery.srv
```
# Request
string table_name
string[] columns
string[] data
string condition
string mode   # "select", "insert", "update", "delete"

---
# Response
bool success
string[] results
string message
```

### UpdateTask.srv
```
# Request
Task task

---
# Response
Task task
bool success
string message
```

## 🗄️ 資料模型

### 核心資料表
- **Task**: 任務資料表
- **Work**: 工作資料表
- **Rack**: 貨架資料表
- **Carrier**: 載具資料表
- **AGV**: AGV 資料表
- **Location**: 位置資料表
- **Eqp**: 設備資料表
- **TrafficZone**: 交通區域資料表

### 狀態管理表
- **TaskStatus**: 任務狀態
- **RackStatus**: 貨架狀態
- **CarrierStatus**: 載具狀態
- **LocationStatus**: 位置狀態

## 🚀 使用方法

### 1. 腳本工具 (推薦使用)
本工作空間提供完整的腳本工具，簡化資料庫管理操作：

```bash
cd /app/db_proxy_ws/scripts

# 檢查資料庫初始化狀態
./check_db_init.sh

# 一鍵初始化資料庫
./init_database.sh

# 測試連線功能
python3 test_connection.py

# 檢查系統狀態
./check_db_status.sh
```

詳細說明請參考：`/app/db_proxy_ws/scripts/README.md`

### 1.1 腳本工具效能指標 (基於實際測試)

| 腳本名稱 | 執行時間 | 成功率 | 主要功能 |
|---------|---------|--------|----------|
| check_db_init.sh | ~1 秒 | 100% | 初始化狀態檢查 |
| init_database.sh | ~1 秒 | 100% | 一鍵資料庫初始化 |
| test_connection.py | ~3-5 秒 | 100% | 連線功能測試 |
| check_db_status.sh | ~2-3 秒 | 100% | 系統狀態監控 |

**環境相容性**:
- ✅ **宿主機環境**: 基本功能可用，psycopg2 連線測試通過
- ✅ **AGVC 容器環境**: 完整功能可用，包含 ROS 2 和虛擬環境套件
- ✅ **自動降級**: 自動檢測環境並選擇最佳測試方式

### 2. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/db_proxy_ws && colcon build
source install/setup.bash
```

### 2. 虛擬環境套件檢查
```bash
# 檢查虛擬環境套件安裝狀態
/opt/pyvenv_env/bin/pip3 list | grep -E "(sqlalchemy|psycopg2|sqlmodel)"

# 檢查套件版本
/opt/pyvenv_env/bin/python3 -c "
import sqlalchemy, psycopg2, sqlmodel
print(f'SQLAlchemy: {sqlalchemy.__version__}')
print(f'psycopg2: {psycopg2.__version__}')
print(f'SQLModel: {sqlmodel.__version__}')
"

# 如需重新安裝套件
/opt/pyvenv_env/bin/pip3 install sqlalchemy psycopg2 sqlmodel
```

### 3. 資料庫環境初始化 (AGVC 管理系統)

#### Docker 容器環境說明
本工作空間的 PostgreSQL 資料庫透過 Docker 容器運行，僅在 **AGVC 管理系統**環境中提供。AGV 車載系統不包含資料庫服務。

**容器服務配置** (基於 docker-compose.agvc.yml):
- **PostgreSQL**: 使用官方 postgres:latest 映像，固定 IP: 192.168.100.254:5432
- **pgAdmin4**: 使用 dpage/pgadmin4 映像，Web 管理介面: http://localhost:5050

#### 檢查資料庫初始化狀態
```bash
# 使用腳本檢查初始化狀態 (推薦)
cd /app/db_proxy_ws/scripts
./check_db_init.sh

# 或手動檢查
docker compose -f docker-compose.agvc.yml ps postgres
pg_isready -h 192.168.100.254 -p 5432
```

#### 資料庫初始化步驟
**⚠️ 重要**: 只有完成以下初始化後才能執行 db_install 腳本

```bash
# 一鍵初始化 (推薦)
cd /app/db_proxy_ws/scripts
./init_database.sh

# 驗證初始化結果
./check_db_init.sh
```

**腳本功能**:
- 自動啟動 PostgreSQL 容器 (如果未運行)
- 建立 agvc 使用者 (如果不存在)
- 建立 agvc 和 test_db 資料庫 (如果不存在)
- 授予適當權限
- 驗證初始化結果

**手動初始化** (如果腳本失敗):
```bash
# 連線到 PostgreSQL 並執行 SQL (注意：RosAGV 系統中正確的用戶是 agvc，不是 postgres)
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d postgres -c "
CREATE DATABASE test_db OWNER agvc;
GRANT ALL PRIVILEGES ON DATABASE test_db TO agvc;
"

# 如果需要重新建立 agvc 資料庫 (通常已存在)
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d postgres -c "
CREATE DATABASE agvc OWNER agvc;
GRANT ALL PRIVILEGES ON DATABASE agvc TO agvc;
"
```

### 4. 啟動資料庫服務
```bash
# 啟動 AGVC 資料庫節點 (AGVC 環境)
ros2 run db_proxy agvc_database_node

# 使用參數指定資料庫連線
ros2 run db_proxy agvc_database_node --ros-args -p db_url_agvc:="postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

# 檢查服務狀態
ros2 service list | grep agvc
ros2 topic list | grep agvc
```

### 5. 執行資料表初始化 (db_install)
```bash
# ⚠️ 確保已完成上述資料庫環境初始化後再執行

# 使用虛擬環境執行資料表初始化
cd /app/db_proxy_ws/src/db_proxy
python3 -m db_proxy.sql.db_install

# 或直接執行初始化腳本
ros2 run db_proxy db_install

# 檢查資料表是否建立成功
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "\dt"
```

### 6. 存取方式和連線資訊

#### PostgreSQL 連線資訊
- **連線位址**: 192.168.100.254:5432
- **生產資料庫**: agvc
- **測試資料庫**: test_db
- **使用者名稱**: agvc
- **密碼**: password

#### pgAdmin4 Web 管理介面
- **Web 介面**: http://localhost:5050/
- **登入帳號**: yazelin@ching-tech.com (基於 docker-compose.agvc.yml)
- **登入密碼**: password

**pgAdmin4 中新增伺服器連線設定**:
1. 開啟 http://localhost:5050/ 並登入
2. 右鍵點選 "Servers" → "Register" → "Server..."
3. 填入以下資訊：
   - **General 頁籤**:
     * Name: agvc
   - **Connection 頁籤**:
     * Host name/address: 192.168.100.254
     * Port: 5432
     * Username: agvc
     * Password: password
     * Save password: ✓

#### 連線字串格式範例
```bash
# psql 指令行連線
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc

# SQLAlchemy 連線字串
postgresql+psycopg2://agvc:password@192.168.100.254/agvc

# psycopg2 連線參數
host=192.168.100.254 port=5432 dbname=agvc user=agvc password=password

# Python 字典格式
{
    "host": "192.168.100.254",
    "port": 5432,
    "database": "agvc",
    "user": "agvc",
    "password": "password"
}
```

### 7. 連線測試
```bash
# 使用腳本進行完整連線測試 (推薦)
cd /app/db_proxy_ws/scripts
python3 test_connection.py

# 或簡單的連線測試
pg_isready -h 192.168.100.254 -p 5432
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "SELECT version();"
```

**測試腳本功能**:
- psycopg2 直接連線測試
- ConnectionPoolManager 連線池測試（支援 ROS 2 和 SQLAlchemy 模式）
- SQLModel 模型載入測試
- 資料庫資料表檢查

**測試驗證結果** (基於實際測試):
- ✅ **通過率**: 4/4 (100%)
- ✅ **執行時間**: 約 3-5 秒
- ✅ **ROS 2 支援**: 自動檢測環境並自動降級
- ✅ **錯誤處理**: 優雅處理各種環境問題

### 4. 使用客戶端服務
```python
import rclpy
from rclpy.node import Node
from db_proxy_interfaces.srv import SqlQuery, UpdateTask
from db_proxy_interfaces.msg import Task

class DatabaseClient(Node):
    def __init__(self):
        super().__init__('db_client')
        self.sql_client = self.create_client(SqlQuery, 'sql_query')
        self.task_client = self.create_client(UpdateTask, 'update_task')
    
    def query_tasks(self):
        request = SqlQuery.Request()
        request.query_string = "SELECT * FROM task WHERE status_id = 1"
        future = self.sql_client.call_async(request)
        return future
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/db_proxy_ws && colcon build

# 執行測試
source install/setup.bash && colcon test
colcon test-result --verbose
```

### 2. 虛擬環境套件測試
```bash
# 測試 SQLAlchemy 連線
/opt/pyvenv_env/bin/python3 -c "
from sqlalchemy import create_engine, text
engine = create_engine('postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
with engine.connect() as conn:
    result = conn.execute(text('SELECT 1'))
    print('✅ SQLAlchemy 連線成功')
"

# 測試 SQLModel 模型
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/db_proxy_ws && source install/setup.bash && /opt/pyvenv_env/bin/python3 -c "
from db_proxy.models import Task, Work, Rack
print('✅ SQLModel 模型載入成功')
print(f'Task 模型: {Task.__tablename__}')
print(f'Work 模型: {Work.__tablename__}')
print(f'Rack 模型: {Rack.__tablename__}')
"
```

### 3. 資料庫服務測試
```bash
# 啟動資料庫節點
ros2 run db_proxy agvc_database_node &

# 測試 SQL 查詢服務
ros2 service call /agvc/sql_query db_proxy_interfaces/srv/SqlQuery "{query_string: 'SELECT COUNT(*) FROM task'}"

# 測試通用查詢服務
ros2 service call /agvc/generic_query db_proxy_interfaces/srv/GenericQuery "{table_name: 'task', mode: 'select', columns: ['id', 'name'], condition: 'status_id = 1'}"

# 測試任務更新服務
ros2 service call /agvc/update_task db_proxy_interfaces/srv/UpdateTask "{task: {name: '測試任務', description: '服務測試', status_id: 1, priority: 1}}"
```

### 4. 連線池效能測試
```python
# 測試連線池效能和並發處理
import threading
import time
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select, text

pool = ConnectionPoolManager("postgresql+psycopg2://agvc:password@192.168.100.254/agvc")

def test_concurrent_queries(thread_id):
    """測試並發查詢"""
    for i in range(10):
        with pool.get_session() as session:
            result = session.exec(text("SELECT COUNT(*) FROM task")).first()
            print(f"執行緒 {thread_id}, 查詢 {i+1}: {result}")
            time.sleep(0.1)

# 建立多個執行緒測試並發
threads = []
for i in range(5):
    thread = threading.Thread(target=test_concurrent_queries, args=(i,))
    threads.append(thread)
    thread.start()

# 等待所有執行緒完成
for thread in threads:
    thread.join()

print("✅ 並發測試完成")
pool.shutdown()
```

### 5. CRUD 功能測試
```python
# 測試完整的 CRUD 操作
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.task_crud import task_crud
from db_proxy.models import Task

pool = ConnectionPoolManager("postgresql+psycopg2://agvc:password@192.168.100.254/agvc")

with pool.get_session() as session:
    # Create - 建立測試任務
    task_data = {
        "name": "CRUD 測試任務",
        "description": "測試 CRUD 操作",
        "status_id": 1,
        "priority": 1
    }
    new_task = task_crud.create(session, obj_in=task_data)
    print(f"✅ 建立任務: ID={new_task.id}, 名稱={new_task.name}")

    # Read - 讀取任務
    task = task_crud.get(session, id=new_task.id)
    print(f"✅ 讀取任務: {task.name}")

    # Update - 更新任務
    update_data = {"description": "已更新的描述"}
    updated_task = task_crud.update(session, db_obj=task, obj_in=update_data)
    print(f"✅ 更新任務: {updated_task.description}")

    # Delete - 刪除任務
    task_crud.remove(session, id=task.id)
    print("✅ 刪除任務完成")

pool.shutdown()
```

## 🔧 配置說明

### PostgreSQL 環境配置 (docker-compose.agvc.yml)

#### Docker 容器配置
```yaml
# PostgreSQL 容器配置
postgres:
  image: postgres:latest
  container_name: postgres_container
  restart: always
  networks:
    bridge_network:
      ipv4_address: 192.168.100.254  # 固定 IP
  environment:
    POSTGRES_USER: postgres          # 系統管理員帳號
    POSTGRES_PASSWORD: password      # 系統管理員密碼
    POSTGRES_DB: postgres           # 預設資料庫
  ports:
    - "5432:5432"
  volumes:
    - postgres_data:/var/lib/postgresql/data

# pgAdmin4 容器配置
pgadmin:
  image: dpage/pgadmin4
  container_name: pgadmin_container
  restart: always
  networks:
    bridge_network:
      ipv4_address: 192.168.100.101  # 固定 IP
  environment:
    PGADMIN_DEFAULT_EMAIL: yazelin@ching-tech.com    # 登入帳號
    PGADMIN_DEFAULT_PASSWORD: password               # 登入密碼
  ports:
    - "5050:80"
  volumes:
    - pgadmin_data:/var/lib/pgadmin
```

#### 資料庫使用者和資料庫配置
```sql
-- 系統管理員
POSTGRES_USER: postgres
POSTGRES_PASSWORD: password

-- 應用程式使用者
USER: agvc
PASSWORD: password
PRIVILEGES: OWNER (對 agvc 和 test_db 資料庫)

-- 資料庫
PRODUCTION_DB: agvc (生產環境)
TEST_DB: test_db (測試環境)
DEFAULT_DB: postgres (系統預設)
```

#### 完整設定順序
1. **Docker 容器啟動**: `docker compose -f docker-compose.agvc.yml up -d postgres pgadmin`
2. **資料庫初始化檢查**: 檢查 agvc 使用者和資料庫是否存在
3. **建立使用者和資料庫**: 執行 SQL 初始化腳本
4. **執行 db_install**: 建立資料表結構
5. **啟動 db_proxy 服務**: 啟動 ROS 2 資料庫代理節點

### 資料庫連線配置
```yaml
# AGVC 資料庫連線 (預設)
db_url_agvc: "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

# 連線池配置
pool_size: 5                    # 基本連線池大小
max_overflow: 5                 # 最大溢出連線數 (總計 10 個連線)
pool_timeout: 30                # 連線超時時間（秒）
pool_recycle: 180               # 連線回收時間（秒）
```

### ConnectionPoolManager 參數
```python
# 連線池管理器配置常數
POOL_SIZE = 5                   # 基本連線池大小
MAX_POOL_SIZE = 10              # 最大連線池大小
POOL_TIMEOUT = 30               # 連線超時時間
POOL_RECYCLE = 180              # 連線回收時間
LOG_INTERVAL = 5                # 監控日誌間隔（秒）
```

### 資料庫初始化配置
```python
# 支援的資料模型 (自動建立資料表)
SUPPORTED_MODELS = [
    "Task", "Work", "TaskStatus",           # 任務相關
    "Rack", "RackStatus",                   # 貨架相關
    "Carrier", "CarrierStatus",             # 載具相關
    "AGV", "AGVContext", "TrafficZone",     # AGV 和交通管制
    "Eqp", "EqpPort", "EqpSignal",         # 設備相關
    "Location", "LocationStatus",           # 位置相關
    "Product", "ProcessSettings",           # 產品和製程
    "Client", "Machine", "User", "Room",    # 基礎資料
    "KukaNode", "KukaEdge",                # KUKA 整合
    "License"                               # 授權管理
]
```

## � 故障排除

### 常見問題

#### 1. Docker 容器啟動問題
**症狀**: PostgreSQL 或 pgAdmin4 容器無法啟動
**解決方法**:
```bash
# 檢查容器狀態
docker compose -f docker-compose.agvc.yml ps

# 檢查容器日誌
docker compose -f docker-compose.agvc.yml logs postgres
docker compose -f docker-compose.agvc.yml logs pgadmin

# 檢查端口衝突
netstat -tulpn | grep -E "(5432|5050)"
ss -tulpn | grep -E "(5432|5050)"

# 重啟容器
docker compose -f docker-compose.agvc.yml restart postgres pgadmin

# 強制重建容器
docker compose -f docker-compose.agvc.yml down
docker compose -f docker-compose.agvc.yml up -d postgres pgadmin --force-recreate
```

#### 2. 資料庫初始化問題
**症狀**: agvc 使用者或資料庫不存在，db_install 執行失敗
**解決方法**:
```bash
# 檢查資料庫初始化狀態
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U postgres -d postgres -c "
SELECT 'Users:' as type, usename as name FROM pg_user WHERE usename = 'agvc'
UNION ALL
SELECT 'Databases:', datname FROM pg_database WHERE datname IN ('agvc', 'test_db');
"

# 如果 agvc 使用者不存在，手動建立
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U postgres -d postgres -c "
CREATE USER agvc WITH PASSWORD 'password';
"

# 如果資料庫不存在，手動建立
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U postgres -d postgres -c "
CREATE DATABASE agvc OWNER agvc;
CREATE DATABASE test_db OWNER agvc;
GRANT ALL PRIVILEGES ON DATABASE agvc TO agvc;
GRANT ALL PRIVILEGES ON DATABASE test_db TO agvc;
"
```

#### 3. 虛擬環境套件問題
**症狀**: `ModuleNotFoundError: No module named 'sqlalchemy'` 或類似錯誤
**解決方法**:
```bash
# 檢查虛擬環境套件
/opt/pyvenv_env/bin/pip3 list | grep -E "(sqlalchemy|psycopg2|sqlmodel)"

# 重新安裝套件
/opt/pyvenv_env/bin/pip3 uninstall sqlalchemy psycopg2 sqlmodel
/opt/pyvenv_env/bin/pip3 install sqlalchemy psycopg2 sqlmodel

# 檢查套件安裝位置
/opt/pyvenv_env/bin/python3 -c "
import sqlalchemy, psycopg2, sqlmodel
print(f'SQLAlchemy: {sqlalchemy.__file__}')
print(f'psycopg2: {psycopg2.__file__}')
print(f'SQLModel: {sqlmodel.__file__}')
"
```

#### 4. PostgreSQL 連線問題
**症狀**: `psycopg2.OperationalError: could not connect to server` 或連線超時
**解決方法**:
```bash
# 檢查 PostgreSQL 容器狀態
docker compose -f docker-compose.agvc.yml ps postgres

# 檢查 PostgreSQL 服務狀態
pg_isready -h 192.168.100.254 -p 5432

# 檢查網路連線
ping 192.168.100.254
telnet 192.168.100.254 5432

# 檢查容器網路配置
docker network ls
docker network inspect rosagv_bridge_network

# 檢查防火牆設定
sudo ufw status
sudo iptables -L | grep 5432

# 測試資料庫認證
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc

# 檢查 PostgreSQL 容器日誌
docker compose -f docker-compose.agvc.yml logs postgres | tail -50
```

#### 5. pgAdmin4 Web 介面存取問題
**症狀**: 無法存取 http://localhost:5050/ 或登入失敗
**解決方法**:
```bash
# 檢查 pgAdmin4 容器狀態
docker compose -f docker-compose.agvc.yml ps pgadmin

# 檢查端口映射
docker port pgadmin_container

# 檢查容器日誌
docker compose -f docker-compose.agvc.yml logs pgadmin

# 檢查端口衝突
netstat -tulpn | grep 5050
lsof -i :5050

# 重啟 pgAdmin4 容器
docker compose -f docker-compose.agvc.yml restart pgadmin

# 測試本地連線
curl -I http://localhost:5050/

# 登入資訊確認
echo "Email: yazelin@ching-tech.com"
echo "Password: password"
```

#### 6. 權限設定問題
**症狀**: `permission denied for database` 或 `must be owner of database`
**解決方法**:
```bash
# 檢查使用者權限
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U postgres -d postgres -c "
SELECT
    d.datname as database,
    r.rolname as owner,
    has_database_privilege('agvc', d.datname, 'CREATE') as can_create,
    has_database_privilege('agvc', d.datname, 'CONNECT') as can_connect
FROM pg_database d
JOIN pg_roles r ON d.datdba = r.oid
WHERE d.datname IN ('agvc', 'test_db');
"

# 重新授予權限
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U postgres -d postgres -c "
ALTER DATABASE agvc OWNER TO agvc;
ALTER DATABASE test_db OWNER TO agvc;
GRANT ALL PRIVILEGES ON DATABASE agvc TO agvc;
GRANT ALL PRIVILEGES ON DATABASE test_db TO agvc;
"

# 檢查資料表權限 (如果資料表已存在)
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "
SELECT schemaname, tablename, tableowner
FROM pg_tables
WHERE schemaname = 'public'
LIMIT 10;
"
```

#### 7. db_install 執行失敗問題
**症狀**: `db_install` 腳本執行失敗，資料表建立錯誤
**解決方法**:
```bash
# 檢查資料庫是否已初始化
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "SELECT current_user, current_database();"

# 檢查虛擬環境套件
python3 -c "
try:
    from db_proxy.models import Task, Work, Rack
    print('✅ 模型匯入成功')
except ImportError as e:
    print(f'❌ 模型匯入失敗: {e}')
"

# 手動執行 db_install
cd /app/db_proxy_ws/src/db_proxy
python3 -c "
from db_proxy.sql.db_install import main
main()
"

# 檢查資料表建立結果
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "
SELECT table_name, table_type
FROM information_schema.tables
WHERE table_schema = 'public'
ORDER BY table_name;
"

# 如果仍然失敗，檢查詳細錯誤
python3 -c "
import logging
logging.basicConfig(level=logging.DEBUG)
from db_proxy.sql.db_install import main
main()
"
```

#### 8. ROS 2 環境問題
**症狀**: `No module named 'rclpy'` 或 `No module named 'yaml'`
**解決方法**:
```bash
# 檢查 ROS 2 環境
docker exec agvc_server bash -c "source /opt/ros/jazzy/setup.bash && python3 -c 'import rclpy; print(\"✅ rclpy 可用\")'"

# 設定正確的 RMW 實作
docker exec agvc_server bash -c "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source /opt/ros/jazzy/setup.bash && python3 -c 'import rclpy; print(\"✅ ROS 2 環境正常\")'"

# 安裝缺失的 Python 套件
docker exec agvc_server bash -c "/opt/pyvenv_env/bin/pip3 install pyyaml"

# 在 AGVC 容器內執行完整測試
docker exec agvc_server bash -c "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && source /opt/ros/jazzy/setup.bash && cd /app/db_proxy_ws/scripts && /opt/pyvenv_env/bin/python3 test_connection.py"

# 如果 ROS 2 不可用，腳本會自動降級到 SQLAlchemy 模式
python3 test_connection.py  # 在宿主機執行，自動使用簡化模式
```

#### 9. 連線池耗盡問題
**症狀**: `QueuePool limit of size X overflow Y reached` 或連線超時
**解決方法**:
```bash
# 檢查連線池狀態
ros2 topic echo /agvc/db_pool_status

# 調整連線池參數
python3 -c "
from db_proxy.connection_pool_manager import ConnectionPoolManager
pool = ConnectionPoolManager(
    'postgresql+psycopg2://agvc:password@192.168.100.254/agvc',
    pool_size=10,
    max_overflow=10,
    pool_timeout=60
)
"

# 檢查長時間運行的查詢
psql -h 192.168.100.254 -U agvc -d agvc -c "
SELECT pid, now() - pg_stat_activity.query_start AS duration, query
FROM pg_stat_activity
WHERE (now() - pg_stat_activity.query_start) > interval '5 minutes';
"
```

#### 4. ROS 2 服務無回應
**症狀**: 服務調用超時或無回應
**解決方法**:
```bash
# 檢查服務狀態
ros2 service list | grep agvc
ros2 service type /agvc/sql_query
ros2 service type /agvc/generic_query

# 檢查節點狀態
ros2 node list | grep agvc
ros2 node info /agvc_database_node

# 重啟資料庫節點
ros2 lifecycle set /agvc_database_node shutdown
ros2 run db_proxy agvc_database_node
```

#### 5. 資料表建立失敗
**症狀**: `sqlalchemy.exc.ProgrammingError` 或資料表不存在
**解決方法**:
```bash
# 手動執行資料庫初始化
cd /app/db_proxy_ws/src/db_proxy
python3 -m db_proxy.sql.db_install

# 檢查資料表是否存在
psql -h 192.168.100.254 -U agvc -d agvc -c "\dt"

# 檢查 SQLModel 模型定義
python3 -c "
from db_proxy.models import Task
print(f'Task 資料表: {Task.__tablename__}')
print(f'Task 欄位: {list(Task.__table__.columns.keys())}')
"
```

### 除錯工具
```bash
# 檢查 Docker 容器狀態
docker compose -f docker-compose.agvc.yml ps
docker compose -f docker-compose.agvc.yml logs postgres
docker compose -f docker-compose.agvc.yml logs pgadmin

# 檢查所有資料庫相關節點
ros2 node list | grep -E "(db|agvc)"

# 監控 PostgreSQL 容器效能
docker stats postgres_container
docker exec postgres_container top

# 檢查容器內 PostgreSQL 程序
docker exec postgres_container ps aux | grep postgres

# 檢查 ROS 2 日誌
ros2 topic echo /rosout | grep agvc

# 檢查容器內資料庫日誌
docker exec postgres_container tail -f /var/log/postgresql/postgresql-*.log

# 檢查連線數和活動
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "
SELECT
    count(*) as total_connections,
    state,
    application_name
FROM pg_stat_activity
WHERE datname = 'agvc'
GROUP BY state, application_name
ORDER BY total_connections DESC;
"

# 檢查資料庫大小
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "
SELECT
    pg_database.datname,
    pg_size_pretty(pg_database_size(pg_database.datname)) AS size
FROM pg_database
WHERE datname IN ('agvc', 'test_db');
"

# 檢查網路連線
docker network inspect rosagv_bridge_network
netstat -tulpn | grep -E "(5432|5050)"
```

### 日誌和診斷
```bash
# 啟用詳細日誌
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 檢查 Docker 容器日誌
docker compose -f docker-compose.agvc.yml logs --tail=100 postgres
docker compose -f docker-compose.agvc.yml logs --tail=100 pgadmin

# 即時監控容器日誌
docker compose -f docker-compose.agvc.yml logs -f postgres &
docker compose -f docker-compose.agvc.yml logs -f pgadmin &

# 檢查 SQLAlchemy 詳細日誌
python3 -c "
import logging
logging.basicConfig(level=logging.DEBUG)
logging.getLogger('sqlalchemy.engine').setLevel(logging.INFO)
logging.getLogger('sqlalchemy.pool').setLevel(logging.DEBUG)
from db_proxy.connection_pool_manager import ConnectionPoolManager
pool = ConnectionPoolManager('postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
with pool.get_session() as session:
    result = session.execute('SELECT version()').fetchone()
    print(f'PostgreSQL 版本: {result[0]}')
pool.shutdown()
"

# 檢查系統資源
free -h
df -h
iostat 1 5

# 檢查容器資源使用
docker stats postgres_container pgadmin_container

# 檢查網路狀態
netstat -tulpn | grep -E "(5432|5050)"
ss -tulpn | grep -E "(5432|5050)"

# 檢查 PostgreSQL 內部狀態
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "
SELECT
    setting as max_connections,
    unit
FROM pg_settings
WHERE name = 'max_connections';

SELECT
    count(*) as current_connections,
    max_val as max_connections
FROM pg_stat_activity,
     (SELECT setting::int as max_val FROM pg_settings WHERE name = 'max_connections') as max_conn;
"

# 檢查資料庫效能統計
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "
SELECT
    schemaname,
    tablename,
    n_tup_ins as inserts,
    n_tup_upd as updates,
    n_tup_del as deletes,
    n_live_tup as live_tuples,
    n_dead_tup as dead_tuples
FROM pg_stat_user_tables
ORDER BY n_live_tup DESC
LIMIT 10;
"
```

## 🔗 相關文檔

- **web_api_ws**: Web API 工作空間，使用本工作空間的 CRUD 模組進行資料庫操作
- **ecs_ws**: ECS 工作空間，使用資料庫服務進行任務和設備管理
- **rcs_ws**: RCS 工作空間，使用 AGV 和交通區域管理功能
- **tafl_wcs_ws**: TAFL WCS 工作空間，使用貨架和載具管理功能
- **SQLModel 官方文檔**: [SQLModel Documentation](https://sqlmodel.tiangolo.com/)
- **SQLAlchemy 官方文檔**: [SQLAlchemy Documentation](https://docs.sqlalchemy.org/)
- **PostgreSQL 官方文檔**: [PostgreSQL Documentation](https://www.postgresql.org/docs/)


