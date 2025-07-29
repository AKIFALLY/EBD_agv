# db_proxy - 資料庫代理核心套件

## 📚 Context Loading
../CLAUDE.md  # 引用上層 db_proxy_ws 工作空間文档

## 📋 套件概述
db_proxy 是 db_proxy_ws 工作空間中的 **核心資料庫代理套件**，提供 ConnectionPoolManager、SQLModel ORM 整合、ROS 2 服務介面等核心功能。負責所有資料庫存取的統一入口和抽象層實作。

**🎯 定位**: 資料庫代理核心實作，提供統一的資料存取介面

## 🔧 核心套件特色
- **AGVCDatabaseNode**: 主要資料庫節點實作 (`agvc_database_node.py`)
- **ConnectionPoolManager**: 自動連線池管理和監控 (`connection_pool_manager.py`)
- **SQLModel 整合**: 26 個資料模型定義 (`models/` 目錄)
- **CRUD 抽象層**: 23 個 CRUD 操作實作 (`crud/` 目錄)
- **ROS 2 服務**: 11 個標準化資料庫服務介面
- **ROS 訊息轉換**: 自動 SQLModel ↔ ROS Message 轉換 (`ros_converter.py`)

## 🚀 套件專用啟動

### 資料庫節點啟動
```bash
# 【推薦方式】透過上層工作空間工具
# 參考: ../CLAUDE.md 開發環境設定

# 【直接啟動】資料庫代理節點
python3 db_proxy/agvc_database_node.py
# 或使用 ROS 2 方式
ros2 run db_proxy agvc_database_node
```

### 套件核心功能測試
```bash
# 連線池狀態檢查
ros2 service call /sql_query db_proxy_interfaces/srv/SqlQuery "sql: 'SELECT 1 as test'"

# CRUD 服務測試
ros2 service call /carrier_query db_proxy_interfaces/srv/CarrierQuery "query_type: 'get_all'"
ros2 service call /rack_query db_proxy_interfaces/srv/RackQuery "query_type: 'get_all'"
```

## 🚨 db_proxy 套件專項故障排除

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### 套件核心功能問題

#### 連線池管理問題
```bash
# 檢查 ConnectionPoolManager 狀態
python3 -c "
from db_proxy.connection_pool_manager import ConnectionPoolManager
pool = ConnectionPoolManager('postgresql://postgres:postgres@postgres_container:5432/postgres')
print('連線池初始化成功')
"

# 檢查連線池統計 (透過節點日誌，每5秒記錄一次)
ros2 node info /agvc_database_node
```

#### SQLModel 和 CRUD 問題
```bash
# 檢查 26 個資料模型載入
python3 -c "from db_proxy.models import *; print('所有 SQLModel 模型載入成功')"

# 檢查 23 個 CRUD 操作
python3 -c "from db_proxy.crud import *; print('所有 CRUD 操作載入成功')"

# 驗證資料庫 schema
python3 db_proxy/sql/verify_schema.py
```

#### ROS 2 服務問題
```bash
# 檢查 11 個資料庫服務
ros2 service list | grep -E "(sql_query|carrier_query|rack_query|update_)"

# 測試服務可用性
ros2 service call /sql_query db_proxy_interfaces/srv/SqlQuery "sql: 'SELECT version()'"
```

### 套件效能監控要點
- **連線池**: 5+5 連線配置，使用率應 < 80%
- **ROS 2 服務**: 11 個服務的回應時間監控
- **SQLModel 查詢**: 透過 `echo=True` 啟用 SQL 日誌監控
- **記憶體使用**: 26 個模型 + 23 個 CRUD 的記憶體效能
