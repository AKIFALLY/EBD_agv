# wcs_ws - WCS 系统工作空间

## 📚 Context Loading
../CLAUDE.md  # 引用根目录系统文档

## 📋 工作空间概述

**WCS 系统工作空间** 提供仓储控制系统（Warehouse Control System）的核心功能，包括 PLC 双向通信、自动任务建立等。

## 📦 套件说明

### 1. alan_room_task_build
**功能**: 监控 PLC DM 区域，根据 work_id 自动建立 Task

**核心特性**:
- 每 1 秒监控 PLC DM2500-2509（10 words）
- Loader AGV: DM2500-2501（32-bit work_id）
- Unloader AGV: DM2502-2503（32-bit work_id）
- 自动创建任务并清除对应 DM
- location_id 防重机制

**服务管理**:
```bash
manage_room_task_build start      # 启动节点
manage_room_task_build stop       # 停止节点
manage_room_task_build restart    # 重启节点
manage_room_task_build status     # 查看状态
manage_room_task_build logs       # 查看日志
```

### 2. transfer_box_task_build
**功能**: 通用传送箱 PLC 双向通信与自动任务建立

**核心特性**:
- **读取**: 每 2 秒从 PLC 读取 carrier_bitmap，更新 Rack 表
- **写入**: 每 3 秒检查传送箱上的 Rack，写入 carrier_bitmap 到 PLC
- **任务**: 边缘触发检测 work_id，自动创建任务
- **清理**: 每 10 秒检查入口/出口，无 Rack 时清空 DM

**PLC 数据映射**:
| 位置 | 说明 | PLC DM 范围 |
|------|------|-------------|
| 入口传送箱 (27) | carrier_bitmap | DM4000-4007 (8 words) |
| 出口传送箱 (26) | carrier_bitmap | DM4100-4107 (8 words) |

**5 个定时器**:
1. **Timer 1** (3秒): 写入 PLC - 检查 Rack → 写入 carrier_bitmap
2. **Timer 2** (1秒): 边缘触发 - 检测 work_id 变化 → 创建任务
3. **Timer 3** (2秒): 读取 PLC - 读取 carrier_bitmap → 更新 Rack
4. **Timer 4** (10秒): 更新回馈 - 写入任务状态到 PLC
5. **Timer 5** (10秒): 清空 DM - 入口/出口都无 Rack 时清空

**边缘触发逻辑**:
- `is_work_id_initialized`: 防止重启时误触发
- `last_location_ids`: 记录 Rack 位置变化
- `last_write_conditions`: 记录写入状态
- `last_plc_bitmaps`: 记录 PLC 边缘触发

**服务管理**:
```bash
manage_transfer_box_task_build start      # 启动节点
manage_transfer_box_task_build stop       # 停止节点
manage_transfer_box_task_build restart    # 重启节点
manage_transfer_box_task_build status     # 查看状态
manage_transfer_box_task_build logs       # 查看日志
```

## 🔧 通用特性

### 数据库操作
- 使用 SQLModel + ConnectionPoolManager
- 不依赖 TAFL 系统
- 直接操作 PostgreSQL

### 防重机制
- 检查未完成任务 (status_id IN [1,2,3])
- location_id 唯一性约束
- 创建成功后清除 PLC DM

## 🚀 开发流程

### 构建与重启
```bash
# [宿主机] 一键重启（alan_room）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
manage_room_task_build stop &&
cd /app/wcs_ws && colcon build --packages-select alan_room_task_build &&
source install/setup.bash && manage_room_task_build start"

# [宿主机] 一键重启（transfer_box）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source &&
manage_transfer_box_task_build stop &&
cd /app/wcs_ws && colcon build --packages-select transfer_box_task_build &&
source install/setup.bash && manage_transfer_box_task_build start"
```

## 🚨 故障排除

### 常见问题
```bash
# 节点未启动
manage_room_task_build status
manage_transfer_box_task_build status

# PLC 连接失败
docker compose -f docker-compose.agvc.yml ps keyence_plc

# 任务未创建
tail -f /tmp/room_task_build.log
tail -f /tmp/transfer_box_task_build.log

# 查看 PLC 数据
# 在 keyence_plc 容器内或通过 PLC 监控工具
```

### 配置文件位置
- **alan_room_task_build**: 参数在代码中（DM2500-2509）
- **transfer_box_task_build**: `transfer_box_task_build/config.py`
  - `RACK_CHECK_INTERVAL = 3.0` (写入)
  - `PLC_READ_INTERVAL = 2.0` (读取)
  - `CLEAR_DM_INTERVAL = 10.0` (清空)

## 🔗 交叉引用
- **plc_proxy_ws**: `../plc_proxy_ws/CLAUDE.md` - PLC 通信
- **keyence_plc_ws**: `../keyence_plc_ws/CLAUDE.md` - Keyence PLC 客户端
- **db_proxy_ws**: `../db_proxy_ws/CLAUDE.md` - 数据库代理
- **rcs_ws**: `../rcs_ws/CLAUDE.md` - 任务调度系统
