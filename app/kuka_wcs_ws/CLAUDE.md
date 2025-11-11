# kuka_wcs_ws - KUKA 仓库控制系统工作空间

## 📚 Context Loading
../CLAUDE.md  # 引用根目录系统文档

## 📋 工作空间概述

**KUKA 仓库控制系统工作空间** 专注于 KUKA rack 搬移和旋转的业务逻辑处理，采用纯 Python 实现，不依赖 TAFL。

### KUKA WCS 工作空间特有功能
- **📦 Rack 满载搬运**: 自动检测房间出口满载 rack 并搬运到人工收料区
- **🔄 Rack 旋转逻辑**: 根据 A/B 面状态自动旋转 rack（待实现）
- **🏭 系统区域管理**: 射出机、系统准备区、空车停放区相关流程（待实现）
- **🤝 协作模式**: 与 rcs_ws 协作（kuka_wcs 创建任务，rcs 调度）

## 🏗️ 工作空间结构

### 目录架构
```
kuka_wcs_ws/
├── src/
│   └── kuka_wcs/                        # 主要 ROS 2 套件
│       ├── kuka_wcs/                    # 核心模块目录
│       │   ├── kuka_wcs_node.py         # ROS2 主节点（1秒定时扫描）
│       │   ├── db_bridge.py             # 数据库操作封装
│       │   └── task_handlers/           # 任务处理器目录
│       │       ├── base_handler.py      # 基础处理器类
│       │       ├── rack_full_handler.py # 满载搬运处理器
│       │       ├── rack_rotation_handler.py # 旋转处理器（待实现）
│       │       └── system_area_handler.py   # 系统区域处理器（待实现）
│       ├── launch/
│       │   └── kuka_wcs.launch.py
│       ├── config/
│       │   └── kuka_wcs_config.yaml
│       ├── test/                        # 测试目录
│       ├── resource/                    # ROS2 资源
│       ├── package.xml                  # ROS2 包配置
│       ├── setup.py                     # Python 安装配置
│       └── setup.cfg                    # 构建配置
├── CLAUDE.md                            # 工作空间文档
└── README.md                            # 说明文档
```

## 🔧 核心特色

### 🟢 处理器模式架构
- **BaseHandler**: 所有处理器的抽象基类，定义统一接口
- **RackFullHandler**: 满载搬运处理器（已实现）
- **RackRotationHandler**: 旋转处理器（框架已完成）
- **SystemAreaHandler**: 系统区域处理器（框架已完成）

### 📊 纯 Python 实现
- **无 TAFL 依赖**: 所有逻辑用 Python 代码实现
- **类型安全**: 使用 SQLModel 进行数据库操作
- **可维护性**: 清晰的代码结构，易于调试和扩展

### 🤖 ROS 2 节点集成
- **定时扫描**: 1秒定时器检查所有处理器
- **数据库驱动**: 基于 PostgreSQL 的任务创建
- **协作调度**: 与 rcs_ws 协作完成任务执行

## 🚀 KUKA WCS 专用开发

**⚠️ 通用开发环境请参考**: ../CLAUDE.md 开发指导章节

### KUKA WCS 特定启动
```bash
# 【容器内】启动 KUKA WCS 节点
manage_kuka_wcs start

# 【容器内】查看状态
manage_kuka_wcs status

# 【容器内】查看日志
manage_kuka_wcs logs

# 【容器内】停止节点
manage_kuka_wcs stop

# 【宿主机】一键重启（开发流程）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
manage_kuka_wcs stop && ba && sa && manage_kuka_wcs start"
```

### 基本功能测试
```bash
# 【容器内】检查 ROS2 节点
ros2 node list | grep kuka_wcs_node
ros2 node info /agvc/kuka_wcs_node

# 【容器内】检查数据库连接
python3 -c "
from db_proxy.connection_pool_manager import ConnectionPoolManager
pool = ConnectionPoolManager('postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
print('✅ 数据库连接成功')
"
```

## 📊 技术实作详解

### 模块特有实作

#### KukaWcsNode 主要节点
- **节点名称**: `kuka_wcs_node`
- **命名空间**: `/agvc`
- **数据库连线**: ConnectionPoolManager (5+5 连线配置)
- **定时扫描**: 1秒定时器
- **处理器管理**: 动态加载任务处理器

#### 任务处理器架构
```python
# BaseHandler 抽象类
class BaseHandler(ABC):
    @abstractmethod
    def check_and_create_tasks(self, session: Session) -> List[Task]:
        """检查条件并创建任务"""
        pass

# RackFullHandler 实现
class RackFullHandler(BaseHandler):
    def check_and_create_tasks(self, session: Session) -> List[Task]:
        # 1. 查询房间出口位置
        # 2. 检查每个出口的 rack
        # 3. 判断是否满载或尾批
        # 4. 创建搬运任务
        pass
```

#### 数据库桥接层
```python
# KukaWcsDbBridge 提供简化的数据库操作
db = KukaWcsDbBridge(logger)

# 查询 locations
locations = db.query_locations(session, type="room_outlet")

# 查询 racks
rack = db.get_rack_at_location(session, location_id)

# 创建 KUKA 任务
task = db.create_kuka_task(
    session=session,
    work_id=220001,
    nodes=[from_node, to_node],
    rack_id=rack.id,
    priority=8
)
```

## 🔗 与其他系统的整合

### 与 RCS 的协作关系
```
kuka_wcs (本工作空间)
    ↓ 创建任务
Task 表 (status_id=1 PENDING, work_id=220001, model="kuka400i")
    ↓ 扫描并调度
rcs_ws (简化的 KUKA 管理器)
    ↓ 调用
kuka_fleet_adapter
    ↓ API 调用
KUKA Fleet Manager (外部系统)
```

### 依赖的工作空间
- **db_proxy_ws**: 数据库连线池和 SQLModel 模型
- **rcs_ws**: 任务调度和执行
- **kuka_fleet_ws**: KUKA Fleet Manager API 客户端
- **shared_constants_ws**: 共享常数定义（work_id 等）

## 🛠️ 实际使用范例

### 开发流程
```bash
# 1. [容器内] 修改代码后重建
cd /app/kuka_wcs_ws
colcon build --packages-select kuka_wcs
source install/setup.bash

# 2. [容器内] 重启服务
manage_kuka_wcs restart

# 3. [容器内] 观察日志
manage_kuka_wcs logs
```

### 添加新的任务处理器
```python
# 1. 创建新处理器文件
# kuka_wcs/task_handlers/my_handler.py

from .base_handler import BaseHandler
from sqlmodel import Session
from typing import List
from db_proxy.models import Task

class MyHandler(BaseHandler):
    def check_and_create_tasks(self, session: Session) -> List[Task]:
        # 实现业务逻辑
        created_tasks = []
        # ... 你的逻辑
        return created_tasks

# 2. 在 kuka_wcs_node.py 中注册
def _init_handlers(self):
    from kuka_wcs.task_handlers.my_handler import MyHandler
    self.handlers = [
        RackFullHandler(self),
        MyHandler(self),  # 添加新处理器
    ]
```

## 🚨 KUKA WCS 专项故障排除

**⚠️ 通用故障排除请参考**: ../CLAUDE.md 故障排除章节

### KUKA WCS 特有问题
```bash
# 节点未启动
manage_kuka_wcs status  # 查看详细状态
manage_kuka_wcs logs    # 查看错误日志

# 数据库连接失败
docker compose -f docker-compose.agvc.yml ps postgres  # 检查数据库
python3 -c "from db_proxy.connection_pool_manager import ConnectionPoolManager; ..."

# 任务未创建
# 查看处理器日志，确认业务逻辑是否触发
tail -f /tmp/kuka_wcs.log | grep -E "(RackFullHandler|check_and_create)"
```

### 常见错误
1. **ImportError: No module named 'kuka_wcs'**
   - 原因：工作空间未建置或未载入
   - 解决：`ba && sa` 或 `build_ws kuka_wcs_ws && agvc_source`

2. **Database connection failed**
   - 原因：PostgreSQL 未运行
   - 解决：`docker compose -f docker-compose.agvc.yml up -d postgres`

3. **No handlers initialized**
   - 原因：处理器导入失败
   - 解决：检查处理器文件语法错误

## 📋 技术限制和注意事项

### 环境依赖
- **AGVC 容器专用**: 必须在 AGVC 容器内执行
- **PostgreSQL 依赖**: 需要 PostgreSQL 容器正常运行
- **RCS 协作**: 需要 rcs_ws 运行以执行创建的任务

### 扩展性设计
- **处理器模式**: 易于添加新的业务逻辑处理器
- **纯 Python**: 无需学习 TAFL 语法
- **数据库驱动**: 所有状态存储在数据库，支持集群部署

## 🔗 交叉引用

### 相关模块
- **rcs_ws**: `../rcs_ws/CLAUDE.md` - 任务调度系统
- **kuka_fleet_ws**: `../kuka_fleet_ws/CLAUDE.md` - KUKA Fleet 适配器
- **db_proxy_ws**: `../db_proxy_ws/CLAUDE.md` - 数据库代理服务
- **tafl_wcs_ws**: `../tafl_wcs_ws/CLAUDE.md` - 原 TAFL 流程系统（参考）

### 通用支援
详细指导请参考: ../CLAUDE.md 交叉引用章节
