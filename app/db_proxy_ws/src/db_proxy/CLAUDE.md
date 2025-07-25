# db_proxy - 資料庫代理服務

## 專案概述
DB Proxy 是 RosAGV 系統的核心資料庫代理服務，提供統一的資料庫存取介面、連線池管理、CRUD 操作和資料模型定義。支援 PostgreSQL 資料庫，整合 SQLAlchemy ORM，為整個 AGV 系統提供可靠的資料持久化服務。

## 核心模組

### 主要類別
- **AgvcDatabaseNode** (`agvc_database_node.py`): 主要資料庫節點，提供 ROS 2 服務介面
- **ConnectionPoolManager** (`connection_pool_manager.py`): 資料庫連線池管理器
- **AgvcDatabaseClient** (`agvc_database_client.py`): 資料庫客戶端，提供高階 API
- **RosConverter** (`ros_converter.py`): ROS 訊息與資料庫模型轉換器

### 資料模型 (models/)
- **AgvcTask** - AGV 任務模型
- **AgvcAgv** - AGV 設備模型
- **AgvcRack** - 料架模型
- **AgvcLocation** - 位置模型
- **AgvcCarrier** - 載具模型
- **AgvcMachine** - 機台模型
- **TaskStatus** - 任務狀態模型

### CRUD 操作 (crud/)
- **BaseCrud** - 基礎 CRUD 操作類別
- **TaskCrud** - 任務相關 CRUD 操作
- **AgvCrud** - AGV 相關 CRUD 操作
- **RackCrud** - 料架相關 CRUD 操作
- **LocationCrud** - 位置相關 CRUD 操作

## 關鍵檔案

### 核心檔案
- `/db_proxy/agvc_database_node.py` - 主要資料庫節點
- `/db_proxy/connection_pool_manager.py` - 連線池管理器
- `/db_proxy/agvc_database_client.py` - 資料庫客戶端 API
- `/db_proxy/ros_converter.py` - ROS 訊息轉換器

### 模型定義
- `/db_proxy/models/agvc_task.py` - 任務資料模型
- `/db_proxy/models/agvc_agv.py` - AGV 資料模型
- `/db_proxy/models/agvc_rack.py` - 料架資料模型
- `/db_proxy/models/agvc_location.py` - 位置資料模型

### CRUD 操作
- `/db_proxy/crud/base_crud.py` - 基礎 CRUD 類別
- `/db_proxy/crud/task_crud.py` - 任務 CRUD 操作
- `/db_proxy/crud/agv_crud.py` - AGV CRUD 操作

### 初始化腳本
- `/db_proxy/sql/init_data/` - 資料庫初始化腳本集合
- `/db_proxy/sql/init_data/18_kuka_map.py` - KUKA 地圖匯入
- `/db_proxy/sql/init_data/19_ct_map.py` - CT 地圖匯入

## 開發指令

### 基本構建
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 構建 db_proxy_ws
build_ws db_proxy_ws

# 單獨構建 db_proxy
cd /app/db_proxy_ws
colcon build --packages-select db_proxy
```

### 服務啟動
```bash
# 啟動資料庫節點
ros2 run db_proxy agvc_database_node

# 啟動資料庫發布節點
ros2 run db_proxy agvc_database_publish_node

# 啟動日誌發布節點
ros2 run db_proxy agvc_logger_pub

# 啟動日誌訂閱節點
ros2 run db_proxy agvc_logger_sub
```

### 資料庫初始化
```bash
# 執行完整初始化
cd /app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data
python3 run_all_init.py

# 單獨執行特定初始化
python3 01_node_types.py
python3 18_kuka_map.py
python3 19_ct_map.py
```

## 配置設定

### 資料庫連線配置
```python
# connection_pool_manager.py
DATABASE_CONFIG = {
    "host": "localhost",
    "port": 5432,
    "database": "agvc_db",
    "username": "agvc_user",
    "password": "agvc_password",
    "pool_size": 10,
    "max_overflow": 20,
    "pool_timeout": 30,
    "pool_recycle": 3600
}
```

### SQLAlchemy 設定
```python
# 連線字串格式
DATABASE_URL = "postgresql://user:password@host:port/database"

# 引擎配置
engine_config = {
    "echo": False,
    "pool_pre_ping": True,
    "pool_recycle": 3600
}
```

## 整合點

### 與其他專案整合
- **web_api_ws**: 使用 connection_pool_manager 進行資料庫操作
- **rcs_ws**: 透過 AgvcDatabaseClient 進行任務和 AGV 狀態查詢
- **wcs_ws**: 使用資料庫服務進行任務分配和狀態更新
- **ecs_ws**: 透過 ROS 2 服務介面進行設備狀態同步

### ROS 2 服務
```bash
# 提供的服務
/db_proxy/get_task_by_id        # 根據 ID 獲取任務
/db_proxy/get_agv_status        # 獲取 AGV 狀態
/db_proxy/update_task_status    # 更新任務狀態
/db_proxy/get_rack_location     # 獲取料架位置
/db_proxy/test_connection       # 測試資料庫連線

# 發布話題
/db_proxy/task_updates          # 任務狀態更新
/db_proxy/agv_status_updates    # AGV 狀態更新
/db_proxy/system_logs           # 系統日誌
```

### 客戶端查詢類別
```python
# 專用查詢客戶端
from db_proxy.db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.db_proxy.rack_query_client import RackQueryClient
from db_proxy.db_proxy.eqp_signal_query_client import EqpSignalQueryClient
```

## 測試方法

### 單元測試
```bash
# 執行所有測試
cd /app/db_proxy_ws/src/db_proxy
python3 -m pytest test/

# 測試連線池管理器
python3 test/test_connection_pool_manager.py

# 測試基礎 CRUD 操作
python3 test/test_base_crud.py

# 測試任務條件
python3 test/test_new_task_conditions.py
```

### 整合測試
```bash
# 測試資料庫服務
python3 test_services_simple.py

# 測試新架構
python3 test_new_architecture.py

# 測試 AGV 旋轉條件
python3 test/test_agv_rotation_condition.py
```

### 調試工具
```bash
# 使用調試腳本
cd /app/db_proxy_ws/src/db_proxy/docs/testing
python3 debug_connection.py
python3 debug_crud_operations.py
python3 simple_query_test.py
```

## 故障排除

### 常見問題

#### 資料庫連線失敗
```bash
# 檢查 PostgreSQL 服務狀態
docker compose -f docker-compose.agvc.yml ps postgres

# 測試資料庫連線
python3 -c "from db_proxy.db_proxy.connection_pool_manager import test_connection; test_connection()"

# 檢查連線池狀態
ros2 service call /db_proxy/test_connection
```

#### 連線池耗盡
```bash
# 檢查連線池狀態
python3 -c "from db_proxy.db_proxy.connection_pool_manager import get_pool_status; print(get_pool_status())"

# 重置連線池
ros2 service call /db_proxy/reset_connection_pool

# 檢查長時間運行的查詢
# 在 PostgreSQL 中執行: SELECT * FROM pg_stat_activity WHERE state = 'active';
```

#### SQLAlchemy 模型錯誤
```bash
# 檢查模型定義
python3 -c "from db_proxy.db_proxy.models import *; print('Models loaded successfully')"

# 驗證資料庫 schema
python3 db_proxy/sql/verify_schema.py

# 重新創建表格
python3 db_proxy/sql/recreate_tables.py
```

### 除錯技巧
- 使用 `echo=True` 啟用 SQLAlchemy SQL 日誌
- 監控 `/db_proxy/system_logs` 話題掌握系統狀態
- 檢查連線池配置是否適合負載
- 使用 PostgreSQL 的 `pg_stat_activity` 監控活動連線

### 效能監控
- 連線池使用率應保持在 80% 以下
- 查詢回應時間監控
- 資料庫鎖定情況檢查
- 記憶體使用情況分析
