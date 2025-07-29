# 資料庫操作指導

## 🎯 適用場景
- RosAGV 系統中的 PostgreSQL 資料庫操作
- SQLModel ORM 的使用和最佳實踐
- 資料庫連線池管理和效能最佳化
- ROS 2 資料庫服務整合

## 📋 RosAGV 資料庫架構

### 資料庫環境
RosAGV 使用 PostgreSQL 16 作為主要資料庫，透過 db_proxy_ws 提供統一的資料存取介面：

```
資料庫架構
├── PostgreSQL 16 (postgres_container)
│   ├── 連接埠: 5432
│   ├── 管理員: postgres/password
│   ├── 應用用戶: agvc/password  
│   └── 資料庫實例:
│       ├── postgres (PostgreSQL 預設資料庫)
│       ├── agvc (應用程式主要資料庫)
│       └── test_db (測試資料庫)
├── db_proxy_ws (資料庫代理服務)
│   ├── ConnectionPoolManager (連線池管理)
│   ├── SQLModel ORM (28個資料模型)
│   ├── CRUD 操作層 (20個專用CRUD)
│   └── ROS 2 服務介面 (12個服務)
└── pgAdmin4 (管理工具)
    ├── 連接埠: 5050
    └── Web 管理介面
```

### 部署特性
- **容器化**: 在 AGVC 管理系統的 Bridge 網路中運行
- **網路地址**: postgres_container (192.168.100.254:5432)
- **持久化**: 使用 Docker Volume 進行資料持久化
- **備份**: 透過 pg_dump 進行資料備份

## 🚀 資料庫初始化

**⚠️ 重要**: 系統首次部署時必須先執行完整的資料庫初始化流程

### 初始化方式選擇
RosAGV 提供兩種資料庫初始化方式：
- **方式1: 腳本初始化** (目前推薦) - 使用 shell 腳本 + ROS 2 指令
- **方式2: 套件初始化** (備選方案) - 程式化初始化邏輯

### 標準初始化流程 (方式1)

#### 步驟1: 創建用戶和資料庫
```bash
# 在宿主機執行
cd /home/ct/RosAGV/app/db_proxy_ws/scripts
./init_database.sh
```

**此腳本會執行**:
- 檢查並啟動 PostgreSQL 容器
- 創建 `agvc` 用戶 (密碼: password)
- 創建 `agvc` 資料庫 (應用程式主要資料庫)
- 創建 `test_db` 資料庫 (測試用)
- 授予 agvc 用戶完整權限
- 驗證連接和權限

#### 步驟2: 創建資料表和初始數據
```bash
# 在 AGVC 容器內執行
docker compose -f docker-compose.agvc.yml exec agvc_server bash
cd /app/db_proxy_ws/src/db_proxy
python3 -m db_proxy.sql.db_install
```

**此步驟會執行**:
- 創建所有 SQLModel 資料表
- 插入預設的系統數據
- 重置資料庫序列
- 驗證初始化結果

### 初始化驗證
```bash
# 檢查資料庫連接
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "SELECT current_user, current_database();"

# 檢查資料表
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "\dt"
```

### 故障排除

#### 常見問題1: PostgreSQL 容器未啟動
```bash
# 解決方案
docker compose -f docker-compose.agvc.yml up -d postgres
# 等待 10 秒後重新執行 init_database.sh
```

#### 常見問題2: 宿主機無法連接容器
```bash
# 檢查容器網路
docker network inspect rosagv_agvc_network

# 檢查端口映射
docker compose -f docker-compose.agvc.yml ps postgres
```

#### 常見問題3: 用戶已存在錯誤
```bash
# init_database.sh 腳本有檢查機制，會跳過已存在用戶
# 如需重置，可手動刪除用戶後重新執行
```

## 📋 多資料庫共存

### 資料庫實例說明
RosAGV 系統在單一 PostgreSQL 實例中維護三個資料庫：

```
PostgreSQL 實例 (postgres_container)
├── postgres 資料庫
│   ├── 用途: PostgreSQL 系統預設資料庫
│   ├── 擁有者: postgres 用戶
│   └── 用於: 系統管理和維護
│
├── agvc 資料庫  
│   ├── 用途: RosAGV 應用程式主要資料庫
│   ├── 擁有者: agvc 用戶
│   ├── 連接字串: postgresql+psycopg2://agvc:password@192.168.100.254/agvc
│   └── 包含: 所有業務資料表和應用數據
│
└── test_db 資料庫
    ├── 用途: 單元測試和整合測試
    ├── 擁有者: agvc 用戶
    ├── 連接字串: postgresql+psycopg2://agvc:password@192.168.100.254/test_db
    └── 用於: 測試環境隔離
```

### 應用程式連接配置
```python
# 所有 RosAGV 模組使用統一連接字串
DB_URL = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

# 模組連接範例
# - web_api_ws
# - opui  
# - agvcui
# - db_proxy_ws
# - ecs_ws
# - rcs_ws
# - ai_wcs_ws
```

## 🔧 連線池管理

### ConnectionPoolManager 設計
RosAGV 使用高效的連線池管理機制，基於 SQLAlchemy QueuePool：

```python
# 連線池配置參數
class ConnectionPoolManager:
    def __init__(self, db_url):
        self.engine = create_engine(
            db_url,
            poolclass=QueuePool,
            pool_size=5,          # 核心連線數
            max_overflow=5,       # 最大溢出連線
            pool_timeout=30,      # 連線等待逾時 (秒)
            pool_recycle=180      # 連線回收時間 (秒)
        )
```

## 🔧 SQLModel ORM 操作

### 模型定義
```python
from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime

class AGVBase(SQLModel):
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(max_length=50)
    status: str = Field(max_length=20)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None

class AGV(AGVBase, table=True):
    __tablename__ = "agvs"
    
    # 關聯關係
    tasks: List["Task"] = Relationship(back_populates="agv")
```

### CRUD 操作
```python
# 創建
async def create_agv(session: AsyncSession, agv_data: AGVBase) -> AGV:
    agv = AGV.from_orm(agv_data)
    session.add(agv)
    await session.commit()
    await session.refresh(agv)
    return agv

# 讀取
async def get_agv(session: AsyncSession, agv_id: int) -> Optional[AGV]:
    statement = select(AGV).where(AGV.id == agv_id)
    result = await session.exec(statement)
    return result.first()

# 更新
async def update_agv(session: AsyncSession, agv_id: int, agv_data: dict) -> Optional[AGV]:
    agv = await get_agv(session, agv_id)
    if agv:
        for key, value in agv_data.items():
            setattr(agv, key, value)
        agv.updated_at = datetime.utcnow()
        await session.commit()
        await session.refresh(agv)
    return agv

# 刪除
async def delete_agv(session: AsyncSession, agv_id: int) -> bool:
    agv = await get_agv(session, agv_id)
    if agv:
        await session.delete(agv)
        await session.commit()
        return True
    return False
```

## 📊 資料庫管理

### 連接管理
```python
# 資料庫會話管理
async def get_session() -> AsyncSession:
    async with AsyncSession(engine) as session:
        try:
            yield session
        finally:
            await session.close()

# 使用依賴注入
@app.get("/agvs/{agv_id}")
async def get_agv_endpoint(
    agv_id: int,
    session: AsyncSession = Depends(get_session)
):
    return await get_agv(session, agv_id)
```

### 遷移管理
```bash
# 使用 Alembic 進行資料庫遷移
alembic init alembic
alembic revision --autogenerate -m "Initial migration"
alembic upgrade head

# 在容器中執行遷移
docker compose -f docker-compose.agvc.yml exec agvc_server alembic upgrade head
```

## 🔍 查詢最佳化

### 複雜查詢
```python
# 聯表查詢
async def get_agvs_with_tasks(session: AsyncSession):
    statement = (
        select(AGV, Task)
        .join(Task, AGV.id == Task.agv_id)
        .where(AGV.status == "active")
        .order_by(AGV.created_at.desc())
    )
    result = await session.exec(statement)
    return result.all()

# 分頁查詢
async def get_agvs_paginated(
    session: AsyncSession,
    skip: int = 0,
    limit: int = 100
):
    statement = select(AGV).offset(skip).limit(limit)
    result = await session.exec(statement)
    return result.all()
```

### 效能優化
```python
# 使用索引
class AGV(AGVBase, table=True):
    __tablename__ = "agvs"
    
    name: str = Field(max_length=50, index=True)
    status: str = Field(max_length=20, index=True)

# 批量操作
async def bulk_update_agvs(session: AsyncSession, updates: List[dict]):
    await session.execute(
        update(AGV),
        updates
    )
    await session.commit()
```

## 🛠️ 開發工具

### pgAdmin4 使用
```bash
# 存取 pgAdmin4
http://localhost:5050

# 登入資訊
Email: admin@admin.com
Password: admin

# 連接資料庫
Host: postgres_container
Port: 5432
Database: postgres
Username: postgres
Password: postgres
```

### 資料庫診斷
```sql
-- 檢查連接數
SELECT count(*) FROM pg_stat_activity;

-- 檢查資料庫大小
SELECT pg_size_pretty(pg_database_size('postgres'));

-- 檢查表大小
SELECT 
    schemaname,
    tablename,
    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as size
FROM pg_tables 
WHERE schemaname = 'public'
ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC;

-- 檢查慢查詢
SELECT query, mean_time, calls 
FROM pg_stat_statements 
ORDER BY mean_time DESC 
LIMIT 10;
```

## 🔒 安全性考量

### 連接安全
```python
# 使用環境變數
import os
DATABASE_URL = os.getenv("DATABASE_URL")

# 連接池安全配置
engine = create_async_engine(
    DATABASE_URL,
    pool_pre_ping=True,  # 檢查連接有效性
    pool_recycle=3600,   # 1小時回收連接
    connect_args={
        "server_settings": {
            "application_name": "rosagv_app",
        }
    }
)
```

### SQL 注入防護
```python
# 使用參數化查詢
statement = select(AGV).where(AGV.name == name)  # 安全
# 避免字串拼接
# statement = f"SELECT * FROM agvs WHERE name = '{name}'"  # 危險
```

## 📋 最佳實踐

### 事務管理
```python
# 明確的事務控制
async def transfer_task(session: AsyncSession, from_agv_id: int, to_agv_id: int, task_id: int):
    async with session.begin():
        # 更新任務分配
        await session.execute(
            update(Task).where(Task.id == task_id).values(agv_id=to_agv_id)
        )
        
        # 更新 AGV 狀態
        await session.execute(
            update(AGV).where(AGV.id == from_agv_id).values(status="idle")
        )
        await session.execute(
            update(AGV).where(AGV.id == to_agv_id).values(status="busy")
        )
        
        # 如果出現異常，自動回滾
```

### 錯誤處理
```python
from sqlalchemy.exc import IntegrityError, SQLAlchemyError

async def safe_create_agv(session: AsyncSession, agv_data: AGVBase):
    try:
        agv = AGV.from_orm(agv_data)
        session.add(agv)
        await session.commit()
        await session.refresh(agv)
        return agv
    except IntegrityError as e:
        await session.rollback()
        raise ValueError(f"AGV name already exists: {e}")
    except SQLAlchemyError as e:
        await session.rollback()
        raise RuntimeError(f"Database error: {e}")
```

### 連接池監控
```python
# 監控連接池狀態
def get_pool_status():
    pool = engine.pool
    return {
        "size": pool.size(),
        "checked_in": pool.checkedin(),
        "checked_out": pool.checkedout(),
        "overflow": pool.overflow(),
        "invalid": pool.invalid()
    }
```

## 🔧 故障排除

### 常見問題
```python
# 連接超時
# 解決：調整連接池配置
engine = create_async_engine(
    DATABASE_URL,
    pool_timeout=30,
    pool_recycle=3600
)

# 記憶體洩漏
# 解決：確保會話正確關閉
async with AsyncSession(engine) as session:
    # 操作
    pass  # 自動關閉

# 死鎖問題
# 解決：統一事務順序，減少事務時間
```

### 效能調優
```sql
-- 分析查詢計劃
EXPLAIN ANALYZE SELECT * FROM agvs WHERE status = 'active';

-- 建立索引
CREATE INDEX idx_agv_status ON agvs(status);
CREATE INDEX idx_task_agv_id ON tasks(agv_id);

-- 更新統計資訊
ANALYZE agvs;
```

## 🔗 交叉引用
- 技術棧詳細: @docs-ai/context/system/technology-stack.md
- Web 開發: @docs-ai/operations/development/web-development.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- AGVC 工作空間: @docs-ai/context/workspaces/agvc-workspaces.md
