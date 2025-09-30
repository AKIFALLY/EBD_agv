# AI Agent 資料庫操作原則

## 資料庫配置
- **資料庫**: PostgreSQL 16
- **容器**: postgres_container (192.168.100.254:5432)
- **主資料庫**: agvc
- **測試資料庫**: test_db
- **用戶**: agvc/password
- **管理工具**: pgAdmin4 (port 5050)

## 連接字串
```python
# 標準連接字串
DB_URL = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

# 測試資料庫
TEST_DB_URL = 'postgresql+psycopg2://agvc:password@192.168.100.254/test_db'
```

## 資料庫初始化
```bash
# 1. 創建用戶和資料庫（宿主機執行）
cd /home/ct/RosAGV/app/db_proxy_ws/scripts
./init_database.sh

# 2. 創建資料表（容器內執行）
docker compose -f docker-compose.agvc.yml exec agvc_server bash
cd /app/db_proxy_ws/src/db_proxy
python3 -m db_proxy.sql.db_install
```

## 資料庫操作指令
```bash
# 連接資料庫
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc

# 常用 SQL 指令
\dt                    # 列出所有資料表
\d table_name         # 查看表結構
\l                    # 列出所有資料庫
\du                   # 列出所有用戶
\q                    # 退出

# 查詢資料
SELECT * FROM agvs;
SELECT * FROM tasks WHERE status = 'pending';
```

## SQLModel ORM 使用
```python
# 模型定義
from sqlmodel import SQLModel, Field
from typing import Optional

class AGV(SQLModel, table=True):
    __tablename__ = "agvs"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    status: str

# 建立連接
from sqlmodel import create_engine, Session

engine = create_engine(DB_URL)
```

## CRUD 操作
```python
# 創建
with Session(engine) as session:
    agv = AGV(name="AGV001", status="idle")
    session.add(agv)
    session.commit()

# 讀取
with Session(engine) as session:
    agv = session.get(AGV, agv_id)
    agvs = session.exec(select(AGV)).all()

# 更新
with Session(engine) as session:
    agv = session.get(AGV, agv_id)
    agv.status = "busy"
    session.commit()

# 刪除
with Session(engine) as session:
    agv = session.get(AGV, agv_id)
    session.delete(agv)
    session.commit()
```

## 連線池配置
```python
# 建議配置
from sqlalchemy.pool import QueuePool

engine = create_engine(
    DB_URL,
    poolclass=QueuePool,
    pool_size=5,          # 核心連線數
    max_overflow=5,       # 最大溢出連線
    pool_timeout=30,      # 等待逾時(秒)
    pool_recycle=180     # 連線回收(秒)
)
```

## 資料庫備份與恢復
```bash
# 備份
docker compose -f docker-compose.agvc.yml exec postgres pg_dump -U agvc -d agvc > backup-$(date +%Y%m%d).sql

# 恢復
docker compose -f docker-compose.agvc.yml exec -T postgres psql -U agvc -d agvc < backup.sql

# 重置資料庫
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -c "DROP DATABASE IF EXISTS agvc;"
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres -c "CREATE DATABASE agvc OWNER agvc;"
```

## 常見問題解決

### 容器未啟動
```bash
docker compose -f docker-compose.agvc.yml up -d postgres
# 等待 10 秒後重新執行
```

### 連接失敗
```bash
# 檢查網路
docker network inspect rosagv_agvc_network

# 檢查端口
docker compose -f docker-compose.agvc.yml ps postgres
ss -tulpn | grep 5432
```

### 權限問題
```bash
# 授予權限
docker compose -f docker-compose.agvc.yml exec postgres psql -U postgres
GRANT ALL PRIVILEGES ON DATABASE agvc TO agvc;
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO agvc;
```

### 資料庫效能監控
```sql
-- 連接數
SELECT count(*) FROM pg_stat_activity WHERE datname = 'agvc';

-- 緩存命中率
SELECT datname, blks_hit::float / (blks_read + blks_hit) AS cache_hit_ratio
FROM pg_stat_database WHERE datname = 'agvc';

-- 慢查詢
SELECT query, calls, mean_exec_time
FROM pg_stat_statements
ORDER BY mean_exec_time DESC LIMIT 10;
```

## 關鍵規則
1. **容器內操作**: 資料庫在 Bridge 網路，使用 192.168.100.254
2. **連線池管理**: 使用 SQLAlchemy QueuePool
3. **ORM 優先**: 使用 SQLModel 而非原生 SQL
4. **測試隔離**: 測試使用 test_db，不用 agvc
5. **定期備份**: 生產環境定期執行 pg_dump