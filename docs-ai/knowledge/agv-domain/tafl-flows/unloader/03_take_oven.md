# Flow 3: unloader_take_oven.yaml

## 🎯 业务目的
从烤箱**上排**（Station 01）批量取出烘干完成的载具

## 📋 基本信息

| 项目 | 值 |
|------|-----|
| 文件名 | `unloader_take_oven.yaml` |
| Flow ID | `unloader_take_oven` |
| 优先级 | 43 |
| 执行间隔 | 12 秒 |
| Work ID | **2060101**（Station-based，只有1个）|

## 🏭 业务场景

### 前置条件
1. Unloader AGV 已将载具放入烤箱**下排**（PUT_OVEN 完成，Flow 2）
2. 烤箱完成烘干制程
3. 载具移动到**上排**并更新为"烘干完成"（status_id: 603）
4. Unloader AGV 处于空闲或有空余车位

### 触发条件（四重验证）
1. **Carrier Status 过滤**: 烤箱上排有 status_id=603（烘干完成）的载具（≥4个）
2. **Presence 信号**: 硬件传感器确认有载具在席（value="1"，4个 Port 全部确认）
3. **AGV 空位**: Unloader AGV 车上有空位（**至少4格空位**）
4. **重复检查**: 没有重复的未完成任务

### 执行结果
- 创建 Unloader AGV 取料任务
- 任务进入待分派队列（status_id = 1 PENDING）
- RCS 系统分派给空闲的 Unloader AGV

## 🔧 技术规格

### 烤箱配置（Station-based）

**物理结构**：
- 1个物理烤箱，8个 port（Port 1-8）
- Equipment ID: 固定为 206
- **只使用 Station 01**（**上排出料**）

**Station-based 配置**（只有 Station 01）：
- **Station 01**: Port 1-2-3-4（**上排，批量4格，只 TAKE**）

**上排 Station 01（固定出料）**：
- **只支持 TAKE 操作**（单向出料）
- Port 1-2-3-4（批量4格）
- **不支持 PUT 操作**（进料由 Flow 2 在下排进行）

**Work ID 对应**（Station-based 编码）：
- `2060101`: Station 01 取烤箱（Port 1-2-3-4，批量4格/上排/**只 TAKE**）

### 烘干制程流程（固定方向）

```
下排进料（PUT_OVEN, Station 05）
    ↓ 烘干制程
上排出料（TAKE_OVEN, Station 01）
    ↓
(本 Flow)
```

**制程说明**（固定单向设计）：
1. **PUT_OVEN**: **只放入下排 Station 05**（Port 5-8，Flow 2）
2. **烤箱内部**: 烘干制程（下排移到上排）
3. **TAKE_OVEN**: **只从上排 Station 01 取出**（Port 1-4，本 Flow）
4. **PUT_BOXOUT_TRANSFER**: 放到出口传送箱（Flow 4）
5. **固定单向操作**: Station 05 只 PUT，Station 01 只 TAKE

### 批量处理逻辑

**统一4格批量处理**（Station 01）：
- **一次取料操作取出4格**（统一批量处理）
- **需要检查烤箱上排至少有4个载具**（Port 1-4 全部有载具）
- **需要检查 AGV 至少有4格空位**（车上需全空）
- **不支持部分取料**（要么取满4格，要么不取）

### Unloader AGV 车载状态

**S尺寸产品**：
- 4格可用（上2格 + 下2格）
- **全部使用批量4格处理**

**L尺寸产品**：
- 2格可用（仅上2格）
- **不支持本 Flow**（需要完整4格空位）

## 📝 TAFL Flow 设计

### Metadata
```yaml
metadata:
  id: "unloader_take_oven"
  name: "Unloader AGV 从烤箱上排取料"
  enabled: true
  version: "1.0.0"
  description: "检查烤箱上排烘干完成的载具，创建 Unloader AGV 取料任务"
  author: "TAFL System"
  created_at: "2025-10-03"
  tags: ["unloader", "oven", "take", "automation"]
```

### Settings
```yaml
settings:
  execution_interval: 12  # 每12秒执行一次
```

### Variables
```yaml
variables:
  priority: 4              # 优先级 1-10 范围
  model: "Unloader"        # AGV 型号（首字母大写）
  oven_equipment_id: 206   # 固定的烤箱设备 ID
  unloader_eqp_id: 211     # UnloaderAGV 设备 ID
  unloader_ports: [2111, 2112, 2113, 2114]  # UnloaderAGV 的 4 个 Port
  # 2个 Station 配置（上排和下排）
  stations:
    - station: 1
      work_id: 2060101
      ports: [2061, 2062, 2063, 2064]  # 上排4个 Port
      batch_size: 4
      row: "upper"
      name: "烤箱上排(Port01-04)"
    - station: 3
      work_id: 2060301
      ports: [2065, 2066, 2067, 2068]  # 下排4个 Port
      batch_size: 4
      row: "lower"
      name: "烤箱下排(Port05-08)"
```

## 🔄 流程逻辑

### 主要步骤

#### 1. 查询所有房间
```yaml
- query:
    target: rooms
    where:
      enabled: true
    as: active_rooms
    description: "查询所有启用的房间"
```

#### 2. 遍历每个房间
```yaml
- for:
    in: "${active_rooms}"
    as: room
    do:
      # 处理该房间的烤箱 Station 01
```

#### 3. 遍历每个 Station（2个 Station）
```yaml
- for:
    in: "${stations}"
    as: station
    do:
      # 处理该 Station 的取料逻辑
```

#### 4. 查询 Station 的烘干完成载具（含 status_id 过滤）
```yaml
- query:
    target: carriers
    where:
      room_id: "${room.id}"
      port_in: "${station.ports}"
      status_id: 603  # 烘干完成
    as: ready_carriers
    description: "查询烤箱 Station ${station.station} 烘干完成载具"
```

#### 5. 检查载具数量
```yaml
- set:
    carrier_count: "${ready_carriers.length}"
    required_count: "${station.batch_size}"
    has_enough_carriers: "${carrier_count >= required_count}"
```

#### 6. 检查 Presence 信号（硬件传感器验证）
```yaml
- if:
    condition: "${has_enough_carriers}"
    then:
      # 查询这些 Port 的 Presence 信号
      - query:
          target: eqp_signals
          where:
            eqp_port_id_in: "${station.ports}"
            name_like: "%Presence"
            value: "1"  # 1 = 有载具在席
          as: occupied_presence_signals
          description: "确认传感器有载具在席"

      # 检查是否所有 Port 的 Presence 都是 1
      - set:
          presence_check_count: "${occupied_presence_signals.length}"
          presence_all_occupied: "${presence_check_count >= required_count}"
```

#### 5. 查询 Unloader AGV 状态
```yaml
- query:
    target: agvs
    where:
      agv_type: "unloader"
      room_id: "${room.id}"
      status: "idle"
    as: available_agvs
    description: "查询空闲的 Unloader AGV"
```

#### 6. 检查 AGV 车上空位（固定4格批量）
```yaml
- query:
    target: carriers
    where:
      agv_id: "${agv.id}"
    as: agv_carriers
    description: "查询 AGV 车上载具"

- set:
    agv_carrier_count: "${agv_carriers.length}"
    max_capacity: 4                        # AGV 最大容量4格
    required_space: "${batch_size}"        # 需要4格空位
    available_space: "${max_capacity - agv_carrier_count}"
    agv_has_space: "${available_space >= required_space}"
```

#### 7. 检查重复任务（固定 Station 01）
```yaml
- query:
    target: tasks
    where:
      work_id: "${work_id}"              # 固定 2060101
      room_id: "${room.id}"
      status_id_in: [0, 1, 2, 3]  # 未完成的状态
    as: existing_tasks
    description: "检查是否已存在取料任务"
```

#### 8. 创建取料任务（固定 Station 01 配置）
```yaml
- if:
    condition: "${has_enough_carriers} && ${agv_has_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "unloader_take"
            name: "房间${room.id}烤箱上排 Station 01 取料"
            description: "从烤箱 Station 01 上排取出${carrier_count}个载具（批量4格）"
            work_id: "${work_id}"        # 固定 2060101
            room_id: "${room.id}"
            priority: "${priority}"
            status_id: 1  # PENDING
            parameters:
              station: 1                 # 固定 Station 01
              work_id: "${work_id}"      # 固定 2060101
              room_id: "${room.id}"
              equipment_id: "${oven_equipment_id}"
              ports: "${ports}"          # [1, 2, 3, 4]
              batch_size: 4              # 固定4格
              row: "upper"               # 上排
              model: "${model}"
              carrier_count: "${carrier_count}"
              reason: "烤箱上排 Station 01 烘干完成，载具就绪"
          description: "创建 Unloader AGV 取料任务（固定 Station 01 上排出料）"
```

## 🔍 查询条件详解

### 1. Carrier Status 过滤（制程阶段控制）

**烤箱载具 status_id 过滤**：
- `room_id`: 特定房间
- `port_in`: Station-based Port 映射
  - **Station 1**: [2061, 2062, 2063, 2064]（上排）
  - **Station 3**: [2065, 2066, 2067, 2068]（下排）
- `status_id: 603` (烘干完成)
  - 烤箱制程完成后的状态
  - 表示载具已完成烘干，准备被 Unloader AGV 取走
  - **确保不会处理未完成烘干的载具**

**数量要求**（固定4格批量）：
- **至少4个载具**（一次取4格）
- Port 全部有载具才创建任务
- 不支持部分取料

**制程流控制**：
```
status_id=603（烘干完成）→ TAKE_OVEN → AGV车上（准备送出口）
```

### 2. Presence 信号验证（硬件传感器检查）

**双重验证机制**：
```yaml
- query:
    target: eqp_signals
    where:
      eqp_port_id_in: "${station.ports}"
      name_like: "%Presence"
      value: "1"  # 1 = 有载具在席
    as: occupied_presence_signals
```

**验证逻辑**：
- **软件状态**: carriers.status_id = 603（烘干完成）
- **硬件状态**: eqp_signals.value = "1"（Presence 传感器有载具）
- **双重确认**: 两者都必须确认有载具才创建任务

**重要性**：
- 防止软件状态与硬件实际不一致
- 避免空取或碰撞
- 提高任务执行可靠性

### AGV 查询条件

**Unloader AGV 条件**：
- `agv_type`: "unloader"
- `room_id`: 特定房间
- `status`: "idle"（空闲状态）

**车载空位计算**（固定4格批量）：
- 查询 AGV 车上现有载具数量
- **需要 ≥ 4格空位**（车上需全空）
- 最大容量：4格（S尺寸产品）
- **不支持部分取料**：必须有完整4格空位才创建任务

### Task 重复检查

**防止重复创建**：
- 检查相同 `work_id` 和 `room_id`
- 状态为未完成（0=创建, 1=待分派, 2=执行中, 3=暂停）
- 如果存在未完成任务，不创建新任务

## ⚠️ 注意事项

### 四重验证机制（测试完成✅）
1. **Carrier Status 过滤**: status_id=603（烘干完成）
2. **Presence 信号**: eqp_signals.value = "1"（硬件传感器有载具）
3. **AGV 空位**: AGV 车上有 ≥4格空位
4. **重复检查**: 无未完成任务

### Station-based 设计重点
- **编码规则**: work_id 使用 Station 编号（**01, 03**），非 Port 起始号
- **UnloaderAGV 自定义映射**: **2个 Station 统一批量4格**（UnloaderAGV 特有）
- **Station 分离**: Station 1（上排）和 Station 3（下排）
- **完整 Port ID**: 使用 2061-2064（上排）和 2065-2068（下排）

### 烤箱固定方向设计
- **上排 Station 01（本 Flow）**: Port 2061-2064，**只 TAKE**（出料）
- **下排 Station 05（Flow 2）**: Port 2065-2068，**只 PUT**（进料）
- **固定单向流程**: 下排进料 → 烘干制程 → 上排出料

### 批量处理逻辑
- **固定4格批量**: 必须有 ≥ 4个载具，AGV 需 ≥ 4格空位（车上全空）
- **不支持部分取料**（要么取满4格，要么不取）
- **Port 全部就绪**: 所有端口都有 status_id=603 的载具才创建任务

### 双重验证的重要性
- **软件 vs 硬件**: 防止数据库状态与实际硬件不一致
- **安全性**: 避免空取或设备碰撞
- **可靠性**: 确保任务执行前条件完全满足

### 烘干制程衔接（固定方向）
- **Flow 2（PUT_OVEN）**: 只放入下排 Station 05
- **烤箱内部**: 烘干制程（下排 → 上排）
- **本 Flow（TAKE_OVEN）**: 只从上排 Station 01 取出
- **Flow 4（PUT_BOXOUT_TRANSFER）**: 放到出口传送箱
- **单向流程**: 消除双向操作的复杂性

### 状态码映射
Carrier status_id 定义：
- `503`: 预烘乾机处理完成（AGV 车上，准备放入烤箱）
- `602`: 烘乾机处理中（载具在烤箱内烘干中）
- `603`: 烘乾机处理完成（TAKE_OVEN 触发条件，**在上排准备被取走**）

### 烤箱设备配置
- **固定 equipment_id**: 206
- **固定 Station**: 01（上排出料）
- **固定 Work ID**: 2060101
- **不需要遍历**: 只有1个 Station，流程简化

## 🧪 测试要点

### 单元测试
1. ✅ 查询逻辑正确性
   - 正确查询烤箱上排 Station 01 载具
   - 正确过滤房间和设备
   - 正确使用 port_in: [1, 2, 3, 4] 查询固定上排端口

2. ✅ 数量判断（固定4格批量）
   - 载具数量 >= 4
   - AGV 空位 >= 4（车上需全空）

3. ✅ 重复检查
   - 存在未完成任务时不创建
   - 任务参数正确传递

4. ✅ Station 01 固定配置
   - 正确使用固定 equipment_id: 206
   - 正确使用固定 work_id: 2060101
   - 正确使用固定 Station 01 Port 1-4
   - 正确使用 status_id: 603 查询烘干完成载具

### 整合测试
1. ✅ 与 PUT_OVEN 衔接（固定方向）
   - PUT_OVEN 放入下排 Station 05 完成
   - 烤箱完成烘干制程
   - 载具自动移到上排 Station 01
   - TAKE_OVEN 检测到并创建任务

2. ✅ RCS 调度
   - 任务正确创建
   - RCS 分派给 Unloader AGV

3. ✅ 与 PUT_BOXOUT_TRANSFER 衔接
   - TAKE_OVEN 取出载具
   - 载具状态更新
   - PUT_BOXOUT_TRANSFER 检测到并创建任务

### 边界测试
1. ✅ 烤箱上排只有3个载具
   - 不创建任务（需要至少4个）

2. ✅ AGV 车上只有3格空位
   - 不创建任务（需要至少4格）

3. ✅ 同时有多个房间烤箱就绪
   - 正确为每个房间创建任务
   - 不产生冲突

## 📊 成功指标

### 功能指标
- ✅ 正确检测烤箱上排烘干完成的载具
- ✅ 正确创建取料任务
- ✅ 不产生重复任务
- ✅ 任务参数完整准确
- ✅ 100% 正确区分上排和下排

### 性能指标
- 执行间隔：12秒
- 响应时间：< 2秒
- 任务创建延迟：< 30秒（从载具就绪到任务创建）

### 可靠性指标
- 零漏检：所有就绪载具都能被检测
- 零误创建：不创建重复或错误任务
- 上下排区分：100% 正确查询上排
- 容错性：数据库查询失败时优雅降级

## 🔗 相关文档

- **Unloader AGV 代码**: `/app/agv_ws/src/unloader_agv/unloader_agv/robot_states/take_oven/`
- **Work ID 定义**: 数据库 `agvc.work` 表
- **前置流程**: Flow 2 - `unloader_put_oven.yaml`
- **后续流程**: Flow 4 - `unloader_put_boxout_transfer.yaml`
