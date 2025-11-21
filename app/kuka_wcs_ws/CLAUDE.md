# kuka_wcs_ws - KUKA 仓库控制系统工作空间

## 📚 Context Loading
../CLAUDE.md  # 引用根目录系统文档

## 📋 工作空间概述

**KUKA 仓库控制系统工作空间** 专注于 KUKA rack 搬移和旋转的业务逻辑处理，采用纯 Python 实现，不依赖 TAFL。

### 核心功能
- **🔄 Rack 旋转**: 根据 carrier_bitmap 和 direction 自动判断并创建旋转任务
- **🏭 系统区域监控**: 监控 6 个区域（系统准备区、房间入口/出口、空架回收区、满料回收区、射出机作业区）
- **📦 业务流程**: 5 个自动化流程判断（空料补给、空架移出、满料补给、射出机出料、满料回收）
- **🤝 协作模式**: 与 rcs_ws 协作（kuka_wcs 创建任务，rcs 调度执行）

## 🏗️ 架构设计

### 处理器模式
```
kuka_wcs_node (1秒定时扫描)
    ├── RackRotationHandler    # Rack 旋转判断
    └── SystemAreaHandler       # 系统区域监控 + 5流程判断
```

### 关键组件
- **kuka_wcs_node.py**: ROS2 主节点，管理所有处理器
- **db_bridge.py**: 数据库操作封装（locations, racks, tasks 查询与创建）
- **base_handler.py**: 抽象基类，定义统一接口
- **rack_rotation_handler.py**: 旋转逻辑处理器
- **system_area_handler.py**: 区域监控与业务流程判断
- **config/kuka_wcs_config.yaml**: 配置文件

## 🔄 Rack 旋转条件

### 入口(27)旋转条件
```yaml
条件1: B面有货(>0000) 且 A面空(=0000) 且 direction=90°
条件2: B面空(=0000) 且 A面有货(>0000) 且 direction=-90°
```
**说明**: 只要有货（非 0000）即可旋转，不需要满载

### 出口(26)旋转条件
```yaml
条件1: FFFF0000 且 direction=90°
条件2: 0000FFFF 且 direction=-90°
```
**说明**: 必须满载（FFFF）才旋转

### 旋转路径
- **入口旋转**: `[27, 87, 27]`
- **出口旋转**: `[26, 86, 26]`

### 完成检查
- **入口(27)**: rack.carrier_bitmap = `00000000` (空载) → 任务完成
- **出口(26)**: rack.carrier_bitmap = `FFFFFFFF` (满载) → 任务完成

## 🏭 系统区域监控

### 6 个监控区域
| 区域名称 | location_ids | 用途 |
|---------|-------------|------|
| system_prepare | 2-9 | 系统准备区（8个站点）|
| room_entrance | 27 | 房间入口（1个站点）|
| room_exit | 26 | 房间出口（1个站点）|
| empty_rack_recycle | 11-13 | 空架回收区（3个站点）|
| full_rack_recycle | 21-22 | 满料回收区（2个站点）|
| injection_work | 15,14,25,23,46,44,47,45 | 射出机作业区（8个站点）|

### 站点状态编码
- `0`: 无架（empty）
- `2`: 有任务占用（task_occupied）
- `5`: 空架（carrier_bitmap = 00000000）
- `6`: 满架（carrier_bitmap = FFFFFFFF）
- `7`: 部分载货（非 00000000 也非 FFFFFFFF）

### 5 个业务流程

#### 流程1: 房间出口需要空料架
```
要料: 房间出口(status=0)
出料: a.房间入口(status=5) 若無 → b.空架回收区(status=5)
```

#### 流程2: 房间入口需要移出空料架
```
额外需求: 系统准备区(status=6 or 7)
出料: 房间入口(status=5)
要料: 空架回收区(status=0)
```

#### 流程3: 房间入口需要满料架
```
要料: 房间入口(status=0)
出料: 系统准备区(status=6 or 7)
```

#### 流程4: 射出机作业区需要出料
```
要料: 系统准备区(status=0)
出料: 射出机作业区(status=6 or 7)
```

#### 流程5: 房间出口满料移到回收区
```
出料: 房间出口(status=6) -- 只能是满架
要料: 满料回收区(status=0)
```

## 🔧 服务管理

```bash
# [容器内] 管理 KUKA WCS 节点
manage_kuka_wcs start      # 启动节点
manage_kuka_wcs stop       # 停止节点
manage_kuka_wcs restart    # 重启节点
manage_kuka_wcs status     # 查看状态
manage_kuka_wcs logs       # 查看日志

# [宿主机] 一键重启（开发流程）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
manage_kuka_wcs stop && ba && sa && manage_kuka_wcs start"
```

## 📊 数据库操作

### KukaWcsDbBridge 核心方法
```python
# 查询 locations
db.query_locations(session, node_id=27)

# 查询 rack
db.get_rack_at_location(session, location_id)

# 获取路径节点（含 waypoint）
db.get_waypoint_nodes(session, source_location_id, target_location_id)

# 创建 KUKA 任务
db.create_kuka_task(
    session=session,
    work_id=220001,
    nodes=[27, 87, 27],
    rack_id=1,
    room_id=2,
    location_id=27,
    rotation_angle=180,
    priority=50
)
```

## 🔗 系统协作

### 任务流程
```
kuka_wcs (创建任务)
    ↓
Task 表 (status_id=1 PENDING, work_id=220001)
    ↓
rcs_ws (扫描并调度)
    ↓
kuka_fleet_adapter (API 调用)
    ↓
KUKA Fleet Manager (外部系统)
```

### 依赖工作空间
- **db_proxy_ws**: 数据库连线池和 SQLModel 模型
- **rcs_ws**: 任务调度和执行
- **kuka_fleet_ws**: KUKA Fleet Manager API 客户端

## 🚨 故障排除

### 常见问题
```bash
# 节点未启动
manage_kuka_wcs status
manage_kuka_wcs logs

# 数据库连接失败
docker compose -f docker-compose.agvc.yml ps postgres

# 任务未创建
tail -f /tmp/kuka_wcs.log | grep -E "(旋转|满足条件|创建任务)"
```

### 配置文件位置
- 开发目录: `/app/kuka_wcs_ws/src/kuka_wcs/config/kuka_wcs_config.yaml`
- 安装目录: `/app/kuka_wcs_ws/install/kuka_wcs/share/kuka_wcs/config/kuka_wcs_config.yaml`

## 🔗 交叉引用
- **rcs_ws**: `../rcs_ws/CLAUDE.md` - 任务调度系统
- **kuka_fleet_ws**: `../kuka_fleet_ws/CLAUDE.md` - KUKA Fleet 适配器
- **db_proxy_ws**: `../db_proxy_ws/CLAUDE.md` - 数据库代理服务
