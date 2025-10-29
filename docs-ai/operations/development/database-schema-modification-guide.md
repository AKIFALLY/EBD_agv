# 資料庫 Schema 修改操作指南

## 🎯 適用場景
- 修改現有資料表結構時的完整操作流程
- 避免常見的資料庫修改錯誤和問題
- 確保資料表重建和初始化的正確性

## 📋 修改流程總覽

### 問題背景
在修改 db_proxy_ws 中的 node 資料表時，經常遇到以下問題：
1. 修改模型後資料表結構沒有更新
2. 初始化資料與新結構不相符
3. 資料庫連接地址錯誤
4. SQLModel metadata 沒有正確重載

## 🔧 完整修改流程

### 第一步：修改資料模型
修改 `/app/db_proxy_ws/src/db_proxy/db_proxy/models/` 目錄下的模型檔案

**範例：修改 node.py**
```python
from typing import Optional, Dict, Any
from sqlmodel import SQLModel, Field
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime, JSON
from pydantic import ConfigDict
from enum import Enum

class NodeTypeEnum(str, Enum):
    NONE = "NONE"
    REST_AREA = "休息區"
    CHARGING_AREA = "充電區"
    TRANSPORT_POINT = "搬運點"

class Node(SQLModel, table=True):
    __tablename__ = "node"

    id: Optional[int] = Field(default=None, primary_key=True)
    # 新增欄位
    x: float = Field(description="X座標")
    y: float = Field(description="Y座標")
    theta: float = Field(description="角度(Θ)")
    type: NodeTypeEnum = Field(default=NodeTypeEnum.NONE, description="節點類型")
    # JSON 欄位
    group_1: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="群組1設定"
    )
    # ... 其他欄位
```

### 第二步：更新初始化資料
修改 `/app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/` 目錄下的初始化檔案

**範例：修改 05_nodes.py**
```python
from db_proxy.models.node import Node, NodeTypeEnum, PGVEnum

def initialize_nodes(session):
    # 建立預設群組配置
    def create_default_group_config(movable_point=0.0, action_mode="向量",
                                   speed_setting=1.0, vector_angle=0.0, area_protection=0.5):
        return {
            "可移動點": movable_point,
            "動作模式": action_mode,
            "速度設定": speed_setting,
            "向量角度": vector_angle,
            "區域防護": area_protection
        }

    default_nodes = [
        {"id": 1, "x": 0.0, "y": 0.0, "theta": 0.0,
         "type": NodeTypeEnum.TRANSPORT_POINT, "pgv": PGVEnum.FRONT,
         "group_1": create_default_group_config()},
        # ... 更多節點
    ]
```

### 第三步：重新建置工作空間
**⚠️ 關鍵步驟：必須重新建置以更新安裝的模型**

```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
cd /app/db_proxy_ws &&
colcon build --packages-select db_proxy &&
source install/setup.bash
"
```

### 第四步：確認正確的資料庫連接地址
**⚠️ 重要：確認資料庫實際位置**

常見連接地址：
- 本地容器：`postgresql+psycopg2://agvc:password@192.168.100.254/agvc`
- 遠端主機：`postgresql+psycopg2://agvc:password@192.168.1.215/agvc`

### 第五步：刪除並重建資料表
**⚠️ 危險操作：會刪除所有現有資料**

```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
cd /app/db_proxy_ws &&
python3 -c \"
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlalchemy import text

# 使用正確的資料庫地址
db_url = 'postgresql+psycopg2://agvc:password@192.168.1.215/agvc'
pool_manager = ConnectionPoolManager(db_url, 1)

with pool_manager.get_session() as session:
    # 刪除舊表
    session.exec(text('DROP TABLE IF EXISTS node CASCADE'))
    session.commit()

    # 清除 metadata 並重新建立
    from sqlmodel import SQLModel
    SQLModel.metadata.clear()
    from db_proxy.models.node import Node
    SQLModel.metadata.create_all(pool_manager.engine)
    session.commit()

pool_manager.shutdown()
\"
"
```

### 第六步：執行初始化
```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
cd /app/db_proxy_ws &&
python3 -c \"
import sys
sys.path.insert(0, '/app/db_proxy_ws/src/db_proxy')
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.sql.init_data.init_manager import initialize_all_data
from db_proxy.sql.db_install import reset_all_sequences

db_url = 'postgresql+psycopg2://agvc:password@192.168.1.215/agvc'
pool_agvc = ConnectionPoolManager(db_url, 1)

with pool_agvc.get_session() as session:
    initialize_all_data(session)
    reset_all_sequences(session)

pool_agvc.shutdown()
\"
"
```

### 第七步：驗證表結構和資料
```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
cd /app/db_proxy_ws &&
python3 -c \"
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlalchemy import text

db_url = 'postgresql+psycopg2://agvc:password@192.168.1.215/agvc'
pool_agvc = ConnectionPoolManager(db_url, 1)

with pool_agvc.get_session() as session:
    # 檢查表結構
    columns_sql = text('''
        SELECT column_name, data_type
        FROM information_schema.columns
        WHERE table_name = 'node'
        ORDER BY ordinal_position
    ''')
    columns = session.exec(columns_sql).fetchall()
    for col_name, data_type in columns:
        print(f'   ✓ {col_name}: {data_type}')

    # 測試資料查詢
    test_sql = text('SELECT id, x, y, theta, type, pgv FROM node LIMIT 3')
    result = session.exec(test_sql).fetchall()
    for row in result:
        print(f'Node {row[0]}: x={row[1]}, y={row[2]}, θ={row[3]}, type={row[4]}, pgv={row[5]}')

pool_agvc.shutdown()
\"
"
```

## 🚨 常見問題和解決方案

### 問題1: "cannot import name 'NodeTypeEnum'"
**原因**: 修改模型後沒有重新建置工作空間
**解決**: 執行 `colcon build --packages-select db_proxy`

### 問題2: "column node.theta does not exist"
**原因**: SQLModel metadata 沒有正確更新
**解決**: 執行 `SQLModel.metadata.clear()` 後重新建立表

### 問題3: 連接資料庫失敗
**原因**: 資料庫地址錯誤
**解決**: 確認資料庫實際位置 (192.168.1.215 vs 192.168.100.254)

### 問題4: 初始化資料格式錯誤
**原因**: 初始化資料沒有更新以符合新 schema
**解決**: 更新 init_data 檔案中的資料格式

### 問題5: 相對導入錯誤 "attempted relative import"
**原因**: 直接執行 db_install.py 而不是作為模組
**解決**: 使用 `sys.path.insert(0, '/app/db_proxy_ws/src/db_proxy')` 或模組方式執行

## ✅ 檢查清單

修改資料表 schema 時的完整檢查清單：

- [ ] 1. 修改資料模型 (.py 檔案)
- [ ] 2. 更新初始化資料
- [ ] 3. 重新建置工作空間 (`colcon build`)
- [ ] 4. 確認正確的資料庫連接地址
- [ ] 5. 刪除舊表 (`DROP TABLE`)
- [ ] 6. 清除 SQLModel metadata
- [ ] 7. 重新建立表 (`create_all`)
- [ ] 8. 執行資料初始化
- [ ] 9. 重置序列 (`reset_all_sequences`)
- [ ] 10. 驗證表結構和資料

## 📝 最佳實踐

1. **備份資料**: 修改前先備份重要資料
2. **測試環境**: 先在測試環境驗證修改
3. **段落式修改**: 複雜修改分成多個步驟進行
4. **版本控制**: 記錄每次 schema 變更
5. **文檔更新**: 更新相關的 API 文檔

## 🔗 相關文件
- 資料庫操作: docs-ai/operations/development/database-operations.md
- SQLModel 使用: db_proxy_ws/CLAUDE.md
- 初始化系統: db_proxy_ws/src/db_proxy/CLAUDE.md

---
**記錄時間**: 2025-09-24
**適用版本**: RosAGV AGVC 系統，PostgreSQL 16，SQLModel 0.0.14+