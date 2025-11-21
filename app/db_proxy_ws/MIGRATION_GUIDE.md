# Task.type 字段移除迁移指南

## 📋 概述

**变更日期**: 2025-11-18
**影响范围**: `task` 表结构，Task 模型定义
**风险等级**: 低（已验证无业务逻辑依赖）

本次迁移移除了 `task` 表中的 `type` 字段，该字段仅在已弃用的 TAFL WCS 系统中使用，当前活跃的 KUKA WCS 系统完全不依赖此字段。

## 🎯 迁移原因

### 字段冗余分析
经过代码库深度分析发现：

1. **KUKA WCS（活跃系统）**: 完全不使用 `type` 字段
   - 使用 `work_id` 区分任务类型（如 KUKA_MOVE=210001, KUKA_RACK_MOVE=220001）
   - 所有任务路由和调度逻辑基于 `work_id`

2. **TAFL WCS（已弃用系统）**: 仅在测试文件中使用
   - 使用 `Task.type == "loader_take"` 等字符串值
   - 该系统已于 2025-11-18 正式弃用

3. **数据库现状**:
   - 当前所有任务记录的 `type` 字段均为 NULL
   - 无历史数据依赖

### 字段使用统计
| 系统 | 使用情况 | 引用位置 |
|------|---------|---------|
| KUKA WCS | ❌ 不使用 | - |
| TAFL WCS | ⚠️ 已弃用 | `tafl_wcs_ws/test/` (3个测试文件) |
| 数据库数据 | 📊 全为 NULL | 0 条非空记录 |

## 📦 迁移内容

### 1. 数据库 Schema 变更
```sql
-- 执行的 SQL
ALTER TABLE task DROP COLUMN type;
```

### 2. 模型定义更新
```python
# 移除前
class Task(SQLModel, table=True):
    __tablename__ = "task"
    id: Optional[int] = Field(default=None, primary_key=True)
    type: Optional[str] = None  # ← 已删除
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")
    ...

# 移除后
class Task(SQLModel, table=True):
    __tablename__ = "task"
    id: Optional[int] = Field(default=None, primary_key=True)
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")
    ...
```

### 3. 测试文件标记
为已弃用测试文件添加清晰的警告信息：
- `tafl_wcs_ws/test/test_loader_flows.py`
- `tafl_wcs_ws/test/test_unloader_flows.py`
- `tafl_wcs_ws/test/test_loader_port_rules.py`

## 🚀 执行步骤

### 自动执行（推荐）
```bash
# 在 AGVC 容器内执行
cd /app/db_proxy_ws/src/db_proxy
python3 db_proxy/sql/migrations/001_drop_task_type_column.py
```

### 手动执行
```bash
# 1. 备份表结构
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "\d task" > task_backup_$(date +%Y%m%d).txt

# 2. 验证数据
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "SELECT COUNT(*) FROM task WHERE type IS NOT NULL;"

# 3. 执行删除
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "ALTER TABLE task DROP COLUMN type;"

# 4. 验证结果
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "\d task"
```

## 🧪 验证测试

### 自动化测试
```bash
# 执行完整功能测试
python3 /app/agents/test_task_type_removal.py
```

测试覆盖：
- ✅ Task 模型定义验证（确认 type 字段已移除）
- ✅ 数据库连接测试
- ✅ 任务 CRUD 操作（创建、查询、更新、删除）

### 手动验证
```python
# 验证 Task 模型
from db_proxy.models import Task
assert not hasattr(Task, 'type'), "type 字段应该已被移除"

# 验证数据库连接
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select

db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
pool = ConnectionPoolManager(db_url, pool_size=1)

with pool.get_session() as session:
    tasks = session.exec(select(Task)).all()
    print(f"成功查询 {len(tasks)} 个任务")

pool.shutdown()
```

## 🔄 回滚方案

如需回滚，执行以下命令：

```bash
# 自动回滚
cd /app/db_proxy_ws/src/db_proxy
python3 db_proxy/sql/migrations/001_drop_task_type_column.py downgrade

# 手动回滚
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "ALTER TABLE task ADD COLUMN type VARCHAR;"
```

**注意**: 回滚后需要重新编辑 `agvc_task.py` 恢复 `type` 字段定义。

## 📊 影响评估

### 不受影响的系统
- ✅ **KUKA WCS**: 完全不依赖 type 字段
- ✅ **RCS 调度系统**: 使用 work_id 和 status_id
- ✅ **Web API**: 前端不显示 type 字段
- ✅ **数据库代理**: ConnectionPoolManager 和 CRUD 操作

### 受影响的系统
- ⚠️ **TAFL WCS 测试**: 已标记为弃用，不再维护
- ⚠️ **历史文档**: ai-agents/archived/tafl-language-rules.md（已归档）

### 数据影响
- 📊 **现有数据**: 无影响（type 字段全为 NULL）
- 📊 **新建任务**: 无影响（不使用 type 字段）

## ✅ 验证检查清单

迁移完成后，请确认：

- [ ] 数据库表结构已更新（`\d task` 无 type 列）
- [ ] Task 模型编译成功（`colcon build --packages-select db_proxy`）
- [ ] 自动化测试全部通过（`test_task_type_removal.py`）
- [ ] CRUD 操作正常（创建、查询、更新、删除）
- [ ] KUKA WCS 任务创建正常
- [ ] RCS 任务调度正常

## 📚 相关文档

- [CHANGELOG.md](./CHANGELOG.md) - 详细变更记录
- [Task 字段使用分析报告](../../agents/) - 完整的字段使用情况分析

## 🤝 支持

如遇问题，请联系：
- **开发团队**: RosAGV Team
- **问题追踪**: GitHub Issues

---

**文档版本**: 1.0
**创建日期**: 2025-11-18
**最后更新**: 2025-11-18
