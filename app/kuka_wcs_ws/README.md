# KUKA WCS - KUKA Warehouse Control System

KUKA 仓库控制系统，专注于 KUKA rack 搬移和旋转的业务逻辑处理。

## 快速开始

### 构建工作空间

```bash
# [容器内] 进入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# [容器内] 构建工作空间
cd /app
build_ws kuka_wcs_ws
agvc_source
```

### 启动节点

```bash
# [容器内] 启动 KUKA WCS 节点
manage_kuka_wcs start

# [容器内] 查看状态
manage_kuka_wcs status

# [容器内] 查看日志
manage_kuka_wcs logs

# [容器内] 停止节点
manage_kuka_wcs stop
```

## 核心功能

### 1. Rack 满载搬运（已实现）
- 自动检测房间出口的满载 rack
- 判断尾批（房间内无可放载具）
- 自动创建搬运任务到人工收料区

### 2. Rack 旋转（框架已完成）
- 根据 A/B 面状态判断是否需要旋转
- 旋转 180 度调整 rack 方向
- TODO: 实现具体旋转逻辑

### 3. 系统区域管理（框架已完成）
- 射出机到系统准备区
- 系统准备区到房间入口
- 房间入口空车到房间出口
- TODO: 实现具体流程逻辑

## 架构设计

### 处理器模式
```
KukaWcsNode (主节点，1秒定时扫描)
    ├── RackFullHandler (满载搬运)
    ├── RackRotationHandler (旋转)
    └── SystemAreaHandler (系统区域)
```

### 协作模式
```
kuka_wcs → 创建任务 → rcs_ws → 调度执行 → kuka_fleet → KUKA Fleet Manager
```

## 开发指南

### 添加新处理器

1. 创建处理器文件：
```python
# kuka_wcs/task_handlers/my_handler.py
from .base_handler import BaseHandler

class MyHandler(BaseHandler):
    def check_and_create_tasks(self, session):
        # 实现业务逻辑
        return []
```

2. 注册到主节点：
```python
# kuka_wcs/kuka_wcs_node.py
def _init_handlers(self):
    from kuka_wcs.task_handlers.my_handler import MyHandler
    self.handlers = [
        RackFullHandler(self),
        MyHandler(self),
    ]
```

3. 重建并重启：
```bash
ba && sa && manage_kuka_wcs restart
```

### 数据库操作
```python
# 使用 KukaWcsDbBridge 进行数据库操作
db = KukaWcsDbBridge(logger)

# 查询 locations
locations = db.query_locations(session, type="room_outlet")

# 查询 rack
rack = db.get_rack_at_location(session, location_id)

# 创建任务
task = db.create_kuka_task(
    session=session,
    work_id=220001,  # KUKA_RACK_MOVE
    nodes=[from_node, to_node],
    rack_id=rack.id
)
```

## 故障排除

### 节点未启动
```bash
# 查看状态
manage_kuka_wcs status

# 查看日誌
tail -f /tmp/kuka_wcs.log
```

### 数据库连接失败
```bash
# 检查 PostgreSQL
docker compose -f docker-compose.agvc.yml ps postgres

# 测试连接
python3 -c "from db_proxy.connection_pool_manager import ConnectionPoolManager; pool = ConnectionPoolManager('postgresql+psycopg2://agvc:password@192.168.100.254/agvc'); print('OK')"
```

### 任务未创建
```bash
# 检查处理器日志
tail -f /tmp/kuka_wcs.log | grep RackFullHandler

# 检查数据库数据
ros2 service call /sql_query db_proxy_interfaces/srv/SqlQuery "sql: 'SELECT * FROM locations WHERE type = \"room_outlet\"'"
```

## 技术栈

- **ROS 2 Jazzy**: 机器人操作系统
- **SQLModel**: Python ORM (基于 Pydantic v2 + SQLAlchemy)
- **PostgreSQL**: 数据库
- **Python 3.12**: 编程语言

## 依赖

- `rclpy`: ROS 2 Python 客户端库
- `db_proxy`: 数据库代理服务（ConnectionPoolManager, SQLModel 模型）
- `shared_constants`: 共享常数（work_id 等）

## 许可证

Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.

## 更多信息

详细文档请查看 [CLAUDE.md](./CLAUDE.md)
