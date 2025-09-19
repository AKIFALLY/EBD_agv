# db_proxy_ws - PostgreSQL 資料庫代理服務工作空間

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档

# 資料庫領域知識（工作空間層）
@docs-ai/knowledge/agv-domain/wcs-database-design.md    # 資料庫設計
@docs-ai/knowledge/agv-domain/wcs-workid-system.md      # WorkID 系統
@docs-ai/knowledge/agv-domain/license-table-design.md   # 授權表設計
@docs-ai/operations/development/database-operations.md   # 資料庫操作

# 通用協議
@docs-ai/knowledge/protocols/ros2-interfaces.md         # ROS2 介面

## 📋 工作空間概述

**資料庫代理服務工作空間** 專注於為 RosAGV AGVC 管理系統提供統一的 PostgreSQL 資料庫存取介面和資料管理服務。

### 資料庫代理服務工作空間特有功能
- **🗄️ 統一資料庫存取**: 提供標準化的資料庫操作介面
- **🔄 ROS 2 服務整合**: 將資料庫操作封裝為 ROS 2 服務
- **📊 CRUD 操作支援**: 完整的建立、讀取、更新、刪除功能
- **🛠️ 資料庫管理工具**: 初始化、狀態檢查、連接測試工具

**⚠️ 重要**: 此模組專為 AGVC 管理系統設計，必須在 AGVC 容器內執行，需要與 PostgreSQL 容器通訊。

## 📂 工作空間結構

### 目錄架構
```
db_proxy_ws/
├── src/
│   ├── db_proxy/                         # 主要 ROS 2 套件
│   │   ├── db_proxy/                     # 核心模組目錄
│   │   │   ├── agvc_database_node.py     # 主要資料庫節點
│   │   │   ├── connection_pool_manager.py # 連線池管理器
│   │   │   ├── models/                   # SQLModel 資料模型
│   │   │   ├── crud/                     # CRUD 操作層
│   │   │   ├── services/                 # 業務服務層
│   │   │   ├── sql/                      # 資料庫初始化
│   │   │   └── ros_converter.py          # ROS 訊息轉換器
│   │   └── setup.py                     # 套件安裝配置
│   └── db_proxy_interfaces/             # ROS 2 介面定義
│       ├── msg/                         # 訊息定義
│       └── srv/                         # 服務定義
├── scripts/                             # 工具腳本
├── docs/                                # 文檔目錄
├── CLAUDE.md                            # 模組文檔
└── README.md                            # 基本說明
```

## 🔧 核心特色

### 🟢 連線池管理 (ConnectionPoolManager)
- **QueuePool 機制**: 使用 SQLAlchemy QueuePool 實現高效連線管理
- **連線參數**: pool_size=5, max_overflow=5, timeout=30s, recycle=180s
- **即時監控**: 每 5 秒記錄連線池狀態統計
- **自動表格建立**: 自動執行 SQLModel.metadata.create_all()
- **Session 管理**: 提供 get_session() 方法取得資料庫 session

### 📊 SQLModel ORM 整合
- **現代化 ORM**: 基於 SQLModel (Pydantic v2 + SQLAlchemy 2.x)
- **25 個資料模型**: 完整的 AGVC 系統資料模型
- **型別安全**: 全面的 Python type hints 支援
- **FastAPI 整合**: 原生支援 FastAPI 生態系統

### 🤖 ROS 2 服務介面
- **11 個 ROS 2 服務**: 標準化的資料庫存取介面
- **17 個 ROS 2 訊息**: 結構化資料傳輸格式
- **訊息轉換**: 自動的 SQLModel ↔ ROS Message 轉換
- **多執行緒支援**: MultiThreadedExecutor 支援並發處理

### 📋 CRUD 操作層
- **BaseCRUD 抽象**: 通用的 CRUD 操作基礎類別
- **23 個 CRUD 實作**: 每個資料模型的專用 CRUD 實作
- **統一介面**: create, read, update, delete, get_by_id, get_all 等
- **關係處理**: 支援複雜的資料庫關係查詢

## 🚀 資料庫代理服務專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

### 資料庫代理服務特定啟動
```bash
# 【推薦方式】透過根目錄統一工具
# 參考: ../../CLAUDE.md 開發指導

# 【直接啟動】資料庫代理服務
cd /app/db_proxy_ws
build_ws db_proxy_ws
ros2 run db_proxy db_proxy_node

# 檢查 PostgreSQL 連接
python3 scripts/test_connection.py

# 初始化資料庫 (首次使用)
python3 src/db_proxy/db_proxy/sql/db_install.py

# 啟動主要資料庫節點
ros2 run db_proxy agvc_database_node

# 或使用直接執行方式
python3 src/db_proxy/db_proxy/agvc_database_node.py
```

### 基本功能測試
```bash
# 測試 ROS 2 服務
ros2 service list | grep db_proxy
ros2 service call /sql_query db_proxy_interfaces/srv/SqlQuery "sql: 'SELECT COUNT(*) FROM task'"

# 測試 CRUD 操作
ros2 service call /carrier_query db_proxy_interfaces/srv/CarrierQuery "query_type: 'get_all'"
ros2 service call /rack_query db_proxy_interfaces/srv/RackQuery "query_type: 'get_all'"

# 檢查連線池狀態 (透過日誌)
# 連線池狀態會自動記錄在節點日誌中，每 5 秒一次
```

## 📊 技術實作詳解

**詳細的資料庫操作技術請參考**: @docs-ai/operations/development/database-operations.md

### 模組特有實作

#### AGVCDatabaseNode 主要節點
- **節點名稱**: `agvc_database_node`
- **資料庫連線**: `postgresql+psycopg2://agvc:password@192.168.100.254/agvc`
- **連線池管理**: ConnectionPoolManager (5+5 連線配置)
- **自動初始化**: 執行預設資料初始化

#### ROS 2 服務介面 (11個服務)
```python
# 實際服務列表 (基於 srv/ 目錄)
/sql_query                # 通用 SQL 查詢
/carrier_query            # Carrier 查詢
/rack_query              # Rack 查詢  
/eqp_signal_query        # 設備信號查詢
/update_task             # 任務更新
/update_rack             # Rack 更新
/update_carrier          # Carrier 更新
/acquire_traffic_area    # 交通區域取得
/add_traffic_area        # 交通區域新增
/release_traffic_area    # 交通區域釋放
/generic_query           # 通用查詢
```

#### SQLModel 資料模型 (25個模型)
基於實際 models/ 目錄檔案：
- **日誌模型**: log_level, rosout_log, runtime_log, modify_log, audit_log
- **拓撲模型**: node, edge, node_type
- **Kuka模型**: agvc_kuka (KukaNode, KukaEdge)
- **系統模型**: client, machine, user, license
- **設備模型**: agvc_eqp (Eqp, EqpPort, EqpSignal)
- **位置模型**: agvc_location (Location, LocationStatus)
- **產品模型**: agvc_product (ProcessSettings, Product)
- **RCS模型**: agvc_rcs (AGV, AGVContext, TrafficZone), agv_status
- **WCS模型**: room, rack_status, rack, carrier, carrier_status
- **任務模型**: agvc_task (Task, TaskStatus, Work)
- **條件模型**: task_condition_history

#### CRUD 操作層 (23個CRUD實作)
基於 BaseCRUD 抽象類別，包含 __init__.py 在內的 23 個檔案，每個主要資料模型都有對應的 CRUD 操作類別

## 🛠️ 實際使用範例

### ROS 2 服務呼叫範例
```bash
# SQL 查詢服務
ros2 service call /sql_query db_proxy_interfaces/srv/SqlQuery \
  "sql: 'SELECT * FROM task WHERE status_id = 1 LIMIT 10'"

# Carrier 查詢服務
ros2 service call /carrier_query db_proxy_interfaces/srv/CarrierQuery \
  "query_type: 'get_by_id', carrier_id: 'CARR001'"

# Rack 查詢服務
ros2 service call /rack_query db_proxy_interfaces/srv/RackQuery \
  "query_type: 'get_all'"

# 任務更新服務
ros2 service call /update_task db_proxy_interfaces/srv/UpdateTask \
  "task_id: 'TASK001', status: 'completed'"
```

### Python CRUD 操作範例
詳細的 CRUD 操作模式請參考: @docs-ai/operations/development/database-operations.md

```python
# 基本使用模式
with pool_manager.get_session() as session:
    # 查詢任務
    tasks = task_crud.get_all(session)
    
    # 查詢特定任務
    task = task_crud.get_by_id(session, "TASK001")
    
    # 更新任務狀態
    if task:
        task.status = "completed"
        updated_task = task_crud.update(session, task)
```

### 資料庫初始化
```python
# 執行完整初始化 (實際的方式)
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.sql.db_install import initialize_default_data

# 建立連線池
pool_manager = ConnectionPoolManager("postgresql+psycopg2://agvc:password@192.168.100.254/agvc")
# 初始化預設資料
initialize_default_data(pool_manager)
```

## 🚨 資料庫代理服務專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### 資料庫代理服務特有問題
```bash
# 使用統一工具進行 AGVC 健康檢查
r agvc-check

# 檢查 db_proxy 節點狀態
ros2 node list | grep database
ros2 node info /agvc_database_node
```

#### 服務測試
```bash
# 檢查所有 db_proxy 服務
ros2 service list | grep -E "(sql_query|carrier_query|rack_query|update_)"

# 測試基本 SQL 查詢
ros2 service call /sql_query db_proxy_interfaces/srv/SqlQuery \
  "sql: 'SELECT 1 as test'"
```

#### 連線池監控
```bash
# 檢查連線池狀態 (透過節點日誌)
ros2 node info /agvc_database_node
# 連線池狀態會每 5 秒記錄一次在節點日誌中

# 檢查資料庫容器狀態
docker compose -f docker-compose.agvc.yml ps postgres
```

## 📋 技術限制和注意事項

### 環境依賴
- **AGVC 容器專用**: 必須在 AGVC 容器內執行，不能在 AGV 容器中使用
- **PostgreSQL 依賴**: 需要 PostgreSQL 容器正常運行 (192.168.100.254:5432)
- **網路設定**: 在 docker-compose.agvc.yml 的 bridge 網路 (192.168.100.0/24) 中運行
- **資料庫設定**: 使用 agvc 使用者連接 agvc 資料庫

### 效能和安全考量
詳細的效能最佳化和安全性指導請參考: @docs-ai/operations/development/database-operations.md

- **連線池配置**: 預設 5+5 連線，可根據負載調整
- **ROS 2 整合**: 11個標準化服務，支援多執行緒並發處理  
- **資料一致性**: 統一的資料庫存取入口，使用 SQLAlchemy session 管理事務
- **安全防護**: 參數化查詢防止 SQL 注入，環境變數管理敏感資訊

## 🔗 交叉引用

### 相關模組
- **Web API 服務**: `../web_api_ws/CLAUDE.md` - 資料庫整合使用者
- **TAFL WCS 系統**: `../tafl_wcs_ws/CLAUDE.md` - TAFL 決策引擎資料存取

### 專業指導
- **資料庫操作**: @docs-ai/operations/development/database-operations.md

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節