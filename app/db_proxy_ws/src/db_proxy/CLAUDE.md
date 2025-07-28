# db_proxy CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/operations/development/database-operations.md  
@docs-ai/operations/development/database-operations.md
@docs-ai/operations/maintenance/system-diagnostics.md

## 概述
PostgreSQL資料庫代理服務核心套件，提供連線池管理、ORM模型定義與ROS 2服務介面

## 關鍵特色
- **連線池管理**: ConnectionPoolManager 自動管理資料庫連線生命週期
- **SQLModel整合**: 現代化ORM模型，支援FastAPI生態系統
- **ROS 2服務**: 提供標準化資料庫存取ROS服務介面
- **CRUD抽象**: 統一的資料庫操作介面和模型轉換器

## 快速開始
```bash
# 進入容器並啟動服務
agvc_enter
start_db && ros2 run db_proxy agvc_database_node
```

## 詳細指導
具體操作請參考: @docs-ai/operations/development/database-operations.md

## 故障排除
基本除錯請參考: @docs-ai/operations/maintenance/system-diagnostics.md

### 常見問題
```bash
# 連線池狀態檢查
ros2 service call /db_proxy/test_connection

# 資料庫初始化
python3 sql/init_data/run_all_init.py
```

詳細除錯流程請參考相關 prompts 檔案。

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
