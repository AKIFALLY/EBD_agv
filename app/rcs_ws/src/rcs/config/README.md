# CT 任務分配系統使用文檔

## 📋 概述

基於 YAML 配置的 CT AGV 任務分配系統，實現 work_id → AGV 的智能映射和分配。

**核心功能**:
- ✅ work_id 直接映射到指定 AGV
- ✅ AGV 能力定義（房間限制、並發任務數）
- ✅ 優先級覆蓋
- ✅ 熱重載（10秒檢測一次配置變更）
- ✅ 完整的測試覆蓋（14個測試全部通過）

---

## 📂 文件結構

```
rcs_ws/src/rcs/
├── config/
│   ├── ct_task_allocation.yaml     # 配置文件
│   └── README.md                    # 本文档
├── rcs/
│   ├── ct_task_allocator.py        # 任务分配器
│   └── simple_ct_manager.py        # CT 管理器（已集成）
└── test/
    └── test_ct_task_allocator.py   # 单元测试
```

---

## ⚙️ 配置文件说明

### 完整配置示例

```yaml
version: "1.0"
enabled: true

# AGV 能力定义
agv_capabilities:
  cargo02:
    model: "Cargo"
    rooms: [1, 2]                    # 允许的房间列表（空列表=所有房间）
    max_concurrent_tasks: 1          # 最大并发任务数
    enabled: true                    # 是否启用
    description: "Cargo AGV 负责入口/出口传送箱搬运"

# work_id → AGV 映射
work_id_allocations:
  2000102:                           # work_id
    agv_name: "cargo02"              # 分配给哪台 AGV
    priority_override: null          # 优先级覆盖（null=不覆盖）
    room_override: null              # 房间覆盖（null=不覆盖）
    description: "Cargo AGV 放料到入口传送箱"

  2051101:                           # 带优先级覆盖的示例
    agv_name: "unloader02"
    priority_override: 3             # 覆盖任务优先级为 3
    room_override: null

# 默认分配策略
default_allocation:
  enabled: false                     # 是否启用默认分配
  fallback_agv: null                 # 默认 AGV
  log_unmapped: true                 # 记录未映射的 work_id

# 热重载配置
reload_config:
  enabled: true                      # 启用热重载
  check_interval: 10.0               # 检查间隔（秒）
  log_on_reload: true                # 重载时记录日志

# 调试选项
debug:
  verbose_logging: false
  log_allocation_decisions: true     # 记录分配决策
  log_skipped_tasks: true            # 记录跳过的任务
```

---

## 🚀 使用方法

### 1. 修改配置文件

编辑 `ct_task_allocation.yaml`:

```bash
# 在宿主机编辑
nano /home/ct/RosAGV/app/config/rcs/ct_task_allocation.yaml

# 或在容器内编辑
cd /app/config/rcs
nano ct_task_allocation.yaml
```

### 2. 添加新的 work_id 映射

```yaml
work_id_allocations:
  2040601:                           # 新的 work_id
    agv_name: "loader02"
    priority_override: 4
    room_override: null
    description: "LoaderAGV 拿泡药机F"
```

### 3. 配置生效

**自动生效（热重载）**:
- 保存配置文件后，系统会在 10 秒内自动检测变更
- 无需重启 RCS 服务
- 日志会显示：`檢測到配置文件變更，正在重新加載...`

**手动重启（如需要）**:
```bash
# 在容器内
cd /app
manage_rcs stop
manage_rcs start

# 或直接重启 RCS 核心节点
ros2 run rcs rcs_core
```

---

## 📊 工作流程

```
1. TAFL WCS 创建 Task
   ↓
   status_id = 1 (PENDING)
   work_id = 2051101
   parameters.model = "Unloader"

2. CT Manager (每秒执行 dispatch)
   ↓
   检查配置文件变更
   获取可用 AGV 列表

3. CtTaskAllocator.allocate_task()
   ↓
   查找 work_id=2051101 → agv="unloader02"
   验证 unloader02 是否可用
   检查房间限制
   返回 (agv_name, priority_override)

4. CT Manager.assign_task_to_agv()
   ↓
   更新 task.agv_id = unloader02.id
   应用优先级覆盖（如果有）
   更新 task.status_id = 2 (READY_TO_EXECUTE)

5. 任务分配完成
```

---

## 🧪 测试

### 运行测试

```bash
# 在容器内
cd /app/rcs_ws
python3 -m pytest src/rcs/test/test_ct_task_allocator.py -v
```

### 测试覆盖

- ✅ 配置加载和验证
- ✅ work_id 正确映射
- ✅ AGV 不可用时返回 None
- ✅ AGV 被禁用时跳过
- ✅ 房间限制验证
- ✅ 优先级覆盖应用
- ✅ 未映射 work_id 处理
- ✅ 热重载机制
- ✅ 配置统计信息

**结果**: 14/14 测试全部通过 ✅

---

## 🔍 故障排除

### 问题1: 任务没有被分配

**检查清单**:
1. 配置文件中是否定义了该 work_id?
   ```bash
   grep "2051101" ct_task_allocation.yaml
   ```

2. AGV 是否在配置中启用?
   ```yaml
   agv_capabilities:
     unloader02:
       enabled: true  # 确保为 true
   ```

3. AGV 是否在数据库中启用?
   ```sql
   SELECT name, enable FROM agv WHERE name = 'unloader02';
   ```

4. 房间限制是否满足?
   ```yaml
   rooms: [1, 2]  # 任务的 room_id 必须在列表中
   ```

5. 查看 RCS 日志
   ```bash
   ros2 run rcs rcs_core
   # 观察日志输出
   ```

### 问题2: 配置修改没有生效

**解决方案**:
1. 确认热重载已启用
   ```yaml
   reload_config:
     enabled: true
   ```

2. 等待 10 秒（check_interval）

3. 查看日志确认重载
   ```
   [INFO] 检测到配置文件变更，正在重新加载...
   [INFO] 配置文件重载成功
   ```

4. 如果仍未生效，手动重启 RCS

### 问题3: AGV 能力配置错误

**常见错误**:
```yaml
# ❌ 错误：YAML 格式错误
agv_capabilities:
  cargo02:
  model: "Cargo"  # 缩进错误

# ✅ 正确
agv_capabilities:
  cargo02:
    model: "Cargo"
```

---

## 📝 最佳实践

### 1. 配置文件管理

- ✅ 修改前先备份配置
- ✅ 使用版本控制追踪变更
- ✅ 添加清晰的 description 注释
- ✅ 定期检查配置一致性

### 2. 任务优先级

```yaml
# 紧急任务
priority_override: 1  # 最高优先级

# 普通任务
priority_override: null  # 使用任务默认优先级

# 低优先级任务
priority_override: 10  # 最低优先级
```

### 3. 房间限制

```yaml
# 所有房间都允许
rooms: []

# 只允许特定房间
rooms: [1, 2]

# 单一房间
rooms: [1]
```

### 4. AGV 能力规划

```yaml
agv_capabilities:
  # 专用 AGV
  cargo02:
    max_concurrent_tasks: 1  # 串行执行

  # 高性能 AGV
  loader03:
    max_concurrent_tasks: 2  # 可并行任务
```

---

## 📊 监控和诊断

### 查看配置统计

```python
# 在 Python 中
allocator = CtTaskAllocator(config_path, logger)
stats = allocator.get_config_stats()
print(stats)
```

输出示例:
```python
{
    'version': '1.0',
    'enabled': True,
    'total_work_id_mappings': 25,
    'total_agvs_configured': 3,
    'hot_reload_enabled': True,
    'config_path': '/app/config/rcs/ct_task_allocation.yaml',
    'last_modified': 1729508742.0
}
```

### 日志级别

配置 debug 选项：
```yaml
debug:
  verbose_logging: true              # 详细日志（生产环境建议 false）
  log_allocation_decisions: true     # 记录每次分配决策
  log_skipped_tasks: true            # 记录跳过的任务
```

---

## 🔗 相关文档

- **RCS 工作空间**: `/app/rcs_ws/CLAUDE.md`
- **TAFL WCS**: `/app/tafl_wcs_ws/CLAUDE.md`
- **数据库模型**: `/app/db_proxy_ws/CLAUDE.md`
- **Work ID 系统**: `@docs-ai/knowledge/agv-domain/wcs-workid-system.md`

---

## 📞 技术支持

如遇到问题，请检查：
1. RCS 日志输出
2. 配置文件格式（YAML 语法）
3. 数据库中的 AGV 状态
4. 测试是否全部通过

**测试命令**:
```bash
cd /app/rcs_ws
python3 -m pytest src/rcs/test/test_ct_task_allocator.py -v
```

---

**版本**: 1.0
**最後更新**: 2025-10-21
**狀態**: ✅ 生產就緒
