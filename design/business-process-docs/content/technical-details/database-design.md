# 資料庫設計

## 概述

RosAGV 系統採用 PostgreSQL 16 作為主要資料庫，透過 SQLModel ORM 提供完整的資料存取層。系統設計支援多資料庫共存、連線池管理和完整的 CRUD 操作。

## 系統架構

### 資料庫部署架構

```
PostgreSQL 資料庫系統
├── PostgreSQL 16 容器 (postgres_container)
│   ├── 網路地址: 192.168.100.254:5432
│   ├── 管理員帳戶: postgres/password
│   ├── 應用程式帳戶: agvc/password
│   └── Docker Volume 持久化
├── db_proxy_ws (資料庫代理服務)
│   ├── SQLModel ORM (28個資料模型)
│   ├── ConnectionPoolManager (連線池管理)
│   ├── CRUD 操作層 (20個專用CRUD)
│   └── ROS 2 服務介面 (12個服務)
└── pgAdmin4 管理工具
    ├── Web 介面: http://localhost:5050
    ├── 登入: admin@admin.com / admin
    └── 資料庫視覺化管理
```

### 多資料庫架構

#### 資料庫實例配置
```sql
-- 系統維護三個獨立資料庫
PostgreSQL 實例架構
├── postgres 資料庫
│   ├── 用途: PostgreSQL 系統預設資料庫
│   ├── 擁有者: postgres 用戶
│   └── 功能: 系統管理和維護
│
├── agvc 資料庫 (主要應用資料庫)
│   ├── 用途: RosAGV 應用程式主要資料庫
│   ├── 擁有者: agvc 用戶
│   ├── 連接字串: postgresql+psycopg2://agvc:password@192.168.100.254/agvc
│   └── 內容: 所有業務資料表和應用數據
│
└── test_db 資料庫
    ├── 用途: 單元測試和整合測試
    ├── 擁有者: agvc 用戶
    ├── 連接字串: postgresql+psycopg2://agvc:password@192.168.100.254/test_db
    └── 功能: 測試環境隔離
```

## 核心資料模型

### 業務實體設計

#### AGV 車輛管理
```python
# AGV 主實體
class AGV(SQLModel, table=True):
    __tablename__ = "agvs"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(max_length=50, index=True)
    agv_type: str = Field(max_length=20)  # Cargo/Loader/Unloader
    status: str = Field(max_length=20, index=True)  # idle/busy/charging
    current_location_id: Optional[int] = Field(foreign_key="location.id")
    battery_level: Optional[float] = Field(default=100.0)
    last_heartbeat: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    tasks: List["Task"] = Relationship(back_populates="agv")
    current_location: Optional["Location"] = Relationship(back_populates="agvs")
```

#### 任務管理系統
```python
# 任務實體
class Task(SQLModel, table=True):
    __tablename__ = "tasks"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    work_id: str = Field(max_length=50, index=True)
    task_type: str = Field(max_length=30)
    priority: int = Field(default=50, index=True)
    status: str = Field(max_length=20, index=True)  # pending/assigned/executing/completed
    
    # 任務位置
    source_location_id: Optional[int] = Field(foreign_key="location.id")
    target_location_id: Optional[int] = Field(foreign_key="location.id")
    
    # 關聯 AGV
    agv_id: Optional[int] = Field(foreign_key="agvs.id")
    rack_id: Optional[int] = Field(foreign_key="rack.id")
    
    # 時間管理
    created_at: datetime = Field(default_factory=datetime.utcnow)
    assigned_at: Optional[datetime] = None
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    
    # 關聯關係
    agv: Optional["AGV"] = Relationship(back_populates="tasks")
    source_location: Optional["Location"] = Relationship(
        sa_relationship_kwargs={"foreign_keys": "[Task.source_location_id]"}
    )
    target_location: Optional["Location"] = Relationship(
        sa_relationship_kwargs={"foreign_keys": "[Task.target_location_id]"}
    )
```

#### 地圖和路徑系統
```python
# 路徑節點 (CT AGV 系統)
class Node(SQLModel, table=True):
    __tablename__ = "node"
    
    id: int = Field(primary_key=True)  # TagNo
    x: float = Field()  # 座標 X (像素)
    y: float = Field()  # 座標 Y (像素)
    node_type_id: Optional[int] = Field(foreign_key="node_type.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    edges_from: List["Edge"] = Relationship(
        sa_relationship_kwargs={"foreign_keys": "[Edge.from_id]"}
    )
    edges_to: List["Edge"] = Relationship(
        sa_relationship_kwargs={"foreign_keys": "[Edge.to_id]"}
    )

# KUKA 節點系統
class KukaNode(SQLModel, table=True):
    __tablename__ = "kuka_node"
    
    id: int = Field(primary_key=True)  # nodeNumber
    uuid: str = Field(max_length=100)  # nodeUuid
    x: float = Field()  # 座標 X (像素，從公尺轉換)
    y: float = Field()  # 座標 Y (像素，從公尺轉換)
    node_type_id: Optional[int] = Field(foreign_key="node_type.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
```

#### 設備和端口管理
```python
# 設備實體
class Equipment(SQLModel, table=True):
    __tablename__ = "eqp"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    location_id: Optional[int] = Field(foreign_key="location.id")
    name: str = Field(max_length=100)
    description: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    ports: List["EquipmentPort"] = Relationship(back_populates="equipment")
    signals: List["EquipmentSignal"] = Relationship(back_populates="equipment")

# 設備端口
class EquipmentPort(SQLModel, table=True):
    __tablename__ = "eqp_port"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    eqp_id: Optional[int] = Field(foreign_key="eqp.id")
    name: str = Field(max_length=50)
    description: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    equipment: Optional["Equipment"] = Relationship(back_populates="ports")
    signals: List["EquipmentSignal"] = Relationship(back_populates="port")
```

### 生產管理實體

#### 射出機和作業員管理
```python
# 射出機實體
class Machine(SQLModel, table=True):
    __tablename__ = "machine"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(max_length=50)
    location_1: Optional[int] = None  # 停車格1的 node_id
    location_2: Optional[int] = None  # 停車格2的 node_id
    status: str = Field(max_length=20, default="active")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None

# 操作平板設備
class Client(SQLModel, table=True):
    __tablename__ = "client"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    device_id: str = Field(max_length=100, unique=True)  # android_id
    machine_id: Optional[int] = Field(foreign_key="machine.id")
    op: Optional[str] = None  # JSON 格式作業員狀態
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
```

#### 產品和製程管理
```python
# 產品實體
class Product(SQLModel, table=True):
    __tablename__ = "product"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(max_length=100)
    size_type: str = Field(max_length=10)  # S/L
    process_settings_id: Optional[int] = Field(foreign_key="process_settings.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    process_settings: Optional["ProcessSettings"] = Relationship(back_populates="products")

# 製程設定
class ProcessSettings(SQLModel, table=True):
    __tablename__ = "process_settings"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    settings: str = Field()  # JSON 格式製程參數
    description: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    products: List["Product"] = Relationship(back_populates="process_settings")
    rooms: List["Room"] = Relationship(back_populates="process_settings")
```

#### 承載系統管理
```python
# Rack 承載架台
class Rack(SQLModel, table=True):
    __tablename__ = "rack"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(max_length=50)
    current_location_id: Optional[int] = Field(foreign_key="location.id")
    status: str = Field(max_length=20, index=True)  # empty/loading/full/ng
    size_type: str = Field(max_length=10)  # S/L 產品適配
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    carriers: List["Carrier"] = Relationship(back_populates="rack")
    current_location: Optional["Location"] = Relationship(back_populates="racks")

# Carrier 承載框架
class Carrier(SQLModel, table=True):
    __tablename__ = "carrier"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    rack_id: Optional[int] = Field(foreign_key="rack.id")
    position: str = Field(max_length=10)  # A面/B面位置
    product_id: Optional[int] = Field(foreign_key="product.id")
    status: str = Field(max_length=20, index=True)  # normal/ng/processing
    quantity: int = Field(default=0)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None
    
    # 關聯關係
    rack: Optional["Rack"] = Relationship(back_populates="carriers")
    product: Optional["Product"] = Relationship()
```

## 連線池管理

### ConnectionPoolManager 設計

```python
class ConnectionPoolManager:
    def __init__(self, db_url: str):
        self.engine = create_async_engine(
            db_url,
            # 連線池配置
            poolclass=QueuePool,
            pool_size=5,              # 核心連線數
            max_overflow=5,           # 最大溢出連線
            pool_timeout=30,          # 連線等待逾時 (秒)
            pool_recycle=180,         # 連線回收時間 (秒)
            pool_pre_ping=True,       # 連線健康檢查
            
            # 連線參數
            connect_args={
                "server_settings": {
                    "application_name": "rosagv_app",
                }
            }
        )
        
    async def get_session(self) -> AsyncSession:
        """取得異步資料庫會話"""
        async with AsyncSession(self.engine, expire_on_commit=False) as session:
            try:
                yield session
            finally:
                await session.close()
                
    def get_pool_status(self) -> dict:
        """監控連線池狀態"""
        pool = self.engine.pool
        return {
            "size": pool.size(),
            "checked_in": pool.checkedin(),
            "checked_out": pool.checkedout(),
            "overflow": pool.overflow(),
            "invalid": pool.invalid()
        }
```

### 連線池最佳化

#### 效能調整參數
```python
# 高負載環境配置
HIGH_LOAD_CONFIG = {
    "pool_size": 10,           # 增加核心連線數
    "max_overflow": 10,        # 增加溢出連線
    "pool_timeout": 60,        # 延長等待時間
    "pool_recycle": 3600,      # 1小時回收連線
}

# 開發環境配置
DEV_CONFIG = {
    "pool_size": 2,            # 減少資源佔用
    "max_overflow": 3,         # 限制最大連線
    "pool_timeout": 10,        # 快速失敗
    "pool_recycle": 300,       # 5分鐘回收
}
```

## CRUD 操作設計

### 基礎 CRUD 模式

```python
from typing import Generic, TypeVar, Type, Optional, List
from sqlmodel import SQLModel, select, update, delete

ModelType = TypeVar("ModelType", bound=SQLModel)

class CRUDBase(Generic[ModelType]):
    def __init__(self, model: Type[ModelType]):
        self.model = model
    
    async def create(self, session: AsyncSession, obj_in: ModelType) -> ModelType:
        """創建新記錄"""
        session.add(obj_in)
        await session.commit()
        await session.refresh(obj_in)
        return obj_in
    
    async def get(self, session: AsyncSession, id: int) -> Optional[ModelType]:
        """根據 ID 取得記錄"""
        statement = select(self.model).where(self.model.id == id)
        result = await session.exec(statement)
        return result.first()
    
    async def get_multi(
        self, 
        session: AsyncSession, 
        skip: int = 0, 
        limit: int = 100
    ) -> List[ModelType]:
        """分頁查詢"""
        statement = select(self.model).offset(skip).limit(limit)
        result = await session.exec(statement)
        return result.all()
    
    async def update(
        self, 
        session: AsyncSession, 
        db_obj: ModelType, 
        obj_in: dict
    ) -> ModelType:
        """更新記錄"""
        for field, value in obj_in.items():
            if hasattr(db_obj, field):
                setattr(db_obj, field, value)
        
        db_obj.updated_at = datetime.utcnow()
        session.add(db_obj)
        await session.commit()
        await session.refresh(db_obj)
        return db_obj
    
    async def delete(self, session: AsyncSession, id: int) -> bool:
        """刪除記錄"""
        obj = await self.get(session, id)
        if obj:
            await session.delete(obj)
            await session.commit()
            return True
        return False
```

### 特化 CRUD 操作

#### AGV 專用 CRUD
```python
class CRUDAGVs(CRUDBase[AGV]):
    async def get_by_name(self, session: AsyncSession, name: str) -> Optional[AGV]:
        """根據名稱查找 AGV"""
        statement = select(AGV).where(AGV.name == name)
        result = await session.exec(statement)
        return result.first()
    
    async def get_available_agvs(
        self, 
        session: AsyncSession, 
        agv_type: str = None
    ) -> List[AGV]:
        """查找可用的 AGV"""
        statement = select(AGV).where(AGV.status == "idle")
        if agv_type:
            statement = statement.where(AGV.agv_type == agv_type)
        result = await session.exec(statement)
        return result.all()
    
    async def update_status(
        self, 
        session: AsyncSession, 
        agv_id: int, 
        status: str
    ) -> Optional[AGV]:
        """更新 AGV 狀態"""
        agv = await self.get(session, agv_id)
        if agv:
            agv.status = status
            agv.last_heartbeat = datetime.utcnow()
            agv.updated_at = datetime.utcnow()
            session.add(agv)
            await session.commit()
            await session.refresh(agv)
        return agv

# 實例化 AGV CRUD
agv_crud = CRUDAGVs(AGV)
```

#### 任務管理 CRUD
```python
class CRUDTask(CRUDBase[Task]):
    async def get_pending_tasks(
        self, 
        session: AsyncSession, 
        priority_min: int = 0
    ) -> List[Task]:
        """取得待處理任務"""
        statement = (
            select(Task)
            .where(Task.status == "pending")
            .where(Task.priority >= priority_min)
            .order_by(Task.priority.desc(), Task.created_at.asc())
        )
        result = await session.exec(statement)
        return result.all()
    
    async def assign_task(
        self, 
        session: AsyncSession, 
        task_id: int, 
        agv_id: int
    ) -> Optional[Task]:
        """分配任務給 AGV"""
        task = await self.get(session, task_id)
        if task and task.status == "pending":
            task.agv_id = agv_id
            task.status = "assigned"
            task.assigned_at = datetime.utcnow()
            task.updated_at = datetime.utcnow()
            session.add(task)
            await session.commit()
            await session.refresh(task)
        return task
    
    async def complete_task(
        self, 
        session: AsyncSession, 
        task_id: int
    ) -> Optional[Task]:
        """完成任務"""
        task = await self.get(session, task_id)
        if task:
            task.status = "completed"
            task.completed_at = datetime.utcnow()
            task.updated_at = datetime.utcnow()
            session.add(task)
            await session.commit()
            await session.refresh(task)
        return task

# 實例化任務 CRUD
task_crud = CRUDTask(Task)
```

## 資料庫初始化

### 自動化初始化流程

#### Shell 腳本初始化
```bash
#!/bin/bash
# init_database.sh - 資料庫初始化腳本

set -e  # 遇到錯誤立即退出

echo "🚀 開始 RosAGV 資料庫初始化..."

# 1. 檢查 PostgreSQL 容器狀態
echo "📋 檢查 PostgreSQL 容器狀態..."
if ! docker compose -f docker-compose.agvc.yml ps postgres | grep -q "Up"; then
    echo "❌ PostgreSQL 容器未運行，正在啟動..."
    docker compose -f docker-compose.agvc.yml up -d postgres
    echo "⏳ 等待 PostgreSQL 啟動完成..."
    sleep 10
fi

# 2. 創建應用用戶
echo "👤 創建 agvc 用戶..."
docker compose -f docker-compose.agvc.yml exec -T postgres psql -U postgres -d postgres <<EOF
DO \$\$
BEGIN
    IF NOT EXISTS (SELECT FROM pg_catalog.pg_roles WHERE rolname = 'agvc') THEN
        CREATE ROLE agvc WITH LOGIN PASSWORD 'password';
        ALTER ROLE agvc CREATEDB;
        GRANT ALL PRIVILEGES ON DATABASE postgres TO agvc;
        RAISE NOTICE 'User agvc created successfully';
    ELSE
        RAISE NOTICE 'User agvc already exists';
    END IF;
END
\$\$;
EOF

# 3. 創建應用資料庫
echo "🗄️ 創建 agvc 資料庫..."
docker compose -f docker-compose.agvc.yml exec -T postgres psql -U postgres -d postgres <<EOF
SELECT 'CREATE DATABASE agvc OWNER agvc' 
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'agvc')\gexec
EOF

# 4. 創建測試資料庫
echo "🧪 創建 test_db 資料庫..."
docker compose -f docker-compose.agvc.yml exec -T postgres psql -U postgres -d postgres <<EOF
SELECT 'CREATE DATABASE test_db OWNER agvc' 
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'test_db')\gexec
EOF

# 5. 驗證初始化結果
echo "✅ 驗證資料庫初始化..."
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "SELECT current_user, current_database();" || {
    echo "❌ 資料庫連接驗證失敗"
    exit 1
}

echo "🎉 資料庫初始化完成！"
```

#### Python 資料表初始化
```python
# db_install.py - 資料表和初始數據創建
import asyncio
from sqlmodel import SQLModel
from db_proxy.database import engine
from db_proxy.models import *  # 導入所有模型

async def create_tables():
    """創建所有資料表"""
    async with engine.begin() as conn:
        # 創建所有表結構
        await conn.run_sync(SQLModel.metadata.create_all)
        print("✅ 所有資料表創建完成")

async def insert_initial_data():
    """插入初始化數據"""
    from db_proxy.sql.initial_data import insert_all_initial_data
    await insert_all_initial_data()
    print("✅ 初始數據插入完成")

async def main():
    """主初始化流程"""
    print("🚀 開始創建資料表和初始數據...")
    
    # 創建表結構
    await create_tables()
    
    # 插入初始數據
    await insert_initial_data()
    
    print("🎉 資料庫初始化完成！")

if __name__ == "__main__":
    asyncio.run(main())
```

## 效能最佳化

### 索引策略

#### 核心索引設計
```sql
-- AGV 效能索引
CREATE INDEX idx_agv_status ON agvs(status);
CREATE INDEX idx_agv_type_status ON agvs(agv_type, status);
CREATE INDEX idx_agv_location ON agvs(current_location_id);

-- 任務效能索引
CREATE INDEX idx_task_status ON tasks(status);
CREATE INDEX idx_task_priority ON tasks(priority DESC);
CREATE INDEX idx_task_status_priority ON tasks(status, priority DESC);
CREATE INDEX idx_task_agv_id ON tasks(agv_id);
CREATE INDEX idx_task_created_at ON tasks(created_at);

-- 位置和地圖索引
CREATE INDEX idx_location_status ON location(location_status_id);
CREATE INDEX idx_location_room ON location(room_id);
CREATE INDEX idx_node_coordinates ON node(x, y);
CREATE INDEX idx_kuka_node_coordinates ON kuka_node(x, y);

-- 承載系統索引
CREATE INDEX idx_rack_status ON rack(status);
CREATE INDEX idx_rack_location ON rack(current_location_id);
CREATE INDEX idx_carrier_rack ON carrier(rack_id);
CREATE INDEX idx_carrier_status ON carrier(status);
```

### 查詢最佳化

#### 複雜查詢範例
```python
# 高效能任務查詢
async def get_optimal_task_assignment(
    session: AsyncSession,
    agv_type: str,
    current_location_id: int
) -> Optional[Task]:
    """最佳化任務分配查詢"""
    
    # 使用 CTE 和子查詢最佳化
    statement = text("""
    WITH available_tasks AS (
        SELECT t.*, 
               ABS(sl.x - :current_x) + ABS(sl.y - :current_y) as distance
        FROM tasks t
        JOIN location sl ON t.source_location_id = sl.id
        JOIN node sn ON sl.node_id = sn.id
        WHERE t.status = 'pending'
          AND t.priority >= 50
    ),
    current_pos AS (
        SELECT n.x, n.y 
        FROM location l 
        JOIN node n ON l.node_id = n.id 
        WHERE l.id = :location_id
    )
    SELECT at.* 
    FROM available_tasks at
    CROSS JOIN current_pos cp
    ORDER BY at.priority DESC, at.distance ASC
    LIMIT 1;
    """)
    
    # 取得當前位置座標
    current_pos = await get_location_coordinates(session, current_location_id)
    
    result = await session.exec(
        statement, 
        {
            "current_x": current_pos.x,
            "current_y": current_pos.y,
            "location_id": current_location_id
        }
    )
    return result.first()
```

### 批量操作最佳化

```python
# 批量狀態更新
async def bulk_update_agv_status(
    session: AsyncSession, 
    status_updates: List[dict]
):
    """批量更新 AGV 狀態"""
    if not status_updates:
        return
    
    # 使用 bulk_update_mappings 提高效能
    await session.execute(
        update(AGV),
        status_updates
    )
    await session.commit()

# 批量任務創建
async def bulk_create_tasks(
    session: AsyncSession, 
    tasks_data: List[dict]
):
    """批量創建任務"""
    tasks = [Task(**task_data) for task_data in tasks_data]
    session.add_all(tasks)
    await session.commit()
    
    # 批量刷新以取得 ID
    for task in tasks:
        await session.refresh(task)
    
    return tasks
```

## 監控和維護

### 資料庫監控

#### 連線池監控
```python
class DatabaseMonitor:
    def __init__(self, pool_manager: ConnectionPoolManager):
        self.pool_manager = pool_manager
    
    async def get_connection_stats(self) -> dict:
        """取得連線統計"""
        pool_status = self.pool_manager.get_pool_status()
        
        # 查詢活動連線
        async with self.pool_manager.get_session() as session:
            active_connections = await session.exec(
                text("SELECT count(*) FROM pg_stat_activity WHERE state = 'active'")
            )
            
        return {
            "pool_status": pool_status,
            "active_connections": active_connections.scalar(),
            "timestamp": datetime.utcnow()
        }
    
    async def get_table_sizes(self) -> List[dict]:
        """取得資料表大小統計"""
        async with self.pool_manager.get_session() as session:
            result = await session.exec(text("""
                SELECT 
                    schemaname,
                    tablename,
                    pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) as size,
                    pg_total_relation_size(schemaname||'.'||tablename) as size_bytes
                FROM pg_tables 
                WHERE schemaname = 'public'
                ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC;
            """))
            
            return [dict(row) for row in result.all()]
```

### 備份和恢復

#### 自動備份腳本
```bash
#!/bin/bash
# backup_database.sh - 資料庫備份腳本

BACKUP_DIR="/app/backups"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BACKUP_FILE="agvc_backup_${TIMESTAMP}.sql"

# 創建備份目錄
mkdir -p ${BACKUP_DIR}

# 執行備份
docker compose -f docker-compose.agvc.yml exec -T postgres pg_dump \
    -U agvc \
    -d agvc \
    --no-password \
    --verbose \
    --clean \
    --if-exists > "${BACKUP_DIR}/${BACKUP_FILE}"

# 壓縮備份文件
gzip "${BACKUP_DIR}/${BACKUP_FILE}"

# 清理舊備份 (保留7天)
find ${BACKUP_DIR} -name "agvc_backup_*.sql.gz" -mtime +7 -delete

echo "✅ 備份完成: ${BACKUP_FILE}.gz"
```

## 相關文檔

- [快速開始指南](../getting-started/quick-start-guide.md) - 系統快速部署
- [系統架構](../system-architecture/dual-environment.md) - 雙環境架構設計
- [技術棧詳細](../system-architecture/technology-stack.md) - 技術組件說明
- [效能調優](../operations/performance-tuning.md) - 系統效能最佳化
- [監控設定](monitoring-setup.md) - 系統監控配置