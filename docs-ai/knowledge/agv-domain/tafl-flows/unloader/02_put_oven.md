# Flow 2: unloader_put_oven.yaml

## 🎯 业务目的
将 Unloader AGV 车上的载具放入烤箱**下排**（Station 05），进入烘干制程

## 📋 基本信息

| 项目 | 值 |
|------|-----|
| 文件名 | `unloader_put_oven.yaml` |
| Flow ID | `unloader_put_oven` |
| 优先级 | 44 |
| 执行间隔 | 12 秒 |
| Work ID | **2060502**（Station-based，只有1个）|

## 🏭 业务场景

### 前置条件
1. Unloader AGV 已从烤箱上排取出载具（TAKE_OVEN 完成）
2. AGV 车上有载具需要放入烤箱下排（status_id: 601 烤箱上排完成）
3. **烤箱下排（Station 05）有空位**
4. Unloader AGV 处于空闲或已完成取料
5. **目标房间为 Room 2**（烤箱 206 位于房间2）

### 触发条件（五重验证）
1. **Carrier Status 过滤**: AGV 车上有 status_id=601（烤箱上排完成）的载具（≥4个）
2. **EqP Port 状态**: 烤箱下排 Station 05 有4格空位（Port 5-8 全部空闲）
3. **Presence 信号**: 硬件传感器确认无载具在席（value="0"，4个 Port 全部确认）
4. **房间过滤**: 只在 Room 2 创建任务（烤箱 206 专属房间）
5. **重复检查**: 没有重复的未完成任务

### 执行结果
- 创建 Unloader AGV 放料任务
- 任务进入待分派队列（status_id = 1 PENDING）
- RCS 系统分派给对应的 Unloader AGV

## 🔧 技术规格

### 烤箱配置（Station-based）

**物理结构**：
- Equipment 206（烤箱）
- 8个 Port（Port 1-8）
- **只使用 Station 05**（**下排进料**）
- Equipment ID: 固定为 206

**Station-Port 映射**（UnloaderAGV 自定义）：
- **Station 05**: Port 5-6-7-8（**批量4格**/下排/**只 PUT**）

**下排配置**（Station 05，固定进料）：
- **只支持 PUT 操作**（单向进料）
- Station 05: Port 5-6-7-8（批量4格）
- **不支持 TAKE 操作**（出料由 Flow 3 在上排进行）

**Work ID 对应**（Station-based 编码）：
- `2060502`: Station 05 放烤箱（Port 5-6-7-8，批量4格/下排/**只 PUT**）

### 烘干制程流程（Station-based，固定方向）

```
下排进料（PUT_OVEN, Station 05）
    ↓ 烘干制程
上排出料（TAKE_OVEN, Station 01）
    ↓
(Flow 3)
```

**制程说明**（固定单向设计）：
1. **Unloader PUT_OVEN**: **只放入下排 Station 05**（Port 5-8）
2. **烤箱内部**: 烘干制程
3. **Unloader TAKE_OVEN**: **只从上排 Station 01 取出**（Port 1-4，Flow 3）
4. **固定单向操作**: Station 05 只 PUT，Station 01 只 TAKE

### 批量处理逻辑

**统一4格批量处理**（Station 05）：
- **Station 05 包含4格**（Port 5-6-7-8）
- **一次放料放入4格**（统一批量处理）
- **需要检查 AGV 至少有4个载具**（车上需全满）
- **需要检查烤箱至少有4格空位**（Port 5-8 全部空闲）
- **不支持部分放料**（要么放满4格，要么不放）

### Unloader AGV 车载状态

**S尺寸产品**：
- 4格可用（上2格 + 下2格）
- **全部使用批量4格处理**

**L尺寸产品**：
- 2格可用（仅上2格）
- **不支持本 Flow**（需要完整4格）

## 📝 TAFL Flow 设计

### Metadata
```yaml
metadata:
  id: "unloader_put_oven"
  name: "Unloader AGV 放料到烤箱（Station-based）"
  enabled: true
  version: "2.0.0"
  description: "检查 Unloader AGV 车上载具和烤箱空位，创建放料任务（Station-based 设计）"
  author: "TAFL System"
  created_at: "2025-10-03"
  updated_at: "2025-10-16"
  tags: ["unloader", "oven", "put", "automation", "station-based"]
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
  target_room_id: 2        # 目标房间（烤箱 206 所属房间）
  oven_equipment_id: 206   # 固定的烤箱设备 ID
  unloader_eqp_id: 211     # UnloaderAGV 设备 ID
  unloader_ports: [2111, 2112, 2113, 2114]  # UnloaderAGV 的 4 个 Port
  # 固定 Station 配置（只有 Station 05，无需遍歷）
  station: 5               # Station 05（下排进料）
  work_id: 2060502         # 唯一的 Work ID
  ports: [2065, 2066, 2067, 2068]  # Port 5-8（完整 Port ID）
  batch_size: 4            # 批量4格
  row: "lower"             # 下排
```

## 🔄 流程逻辑

### 主要步骤

#### 1. 查询目标房间（Room 2）
```yaml
- query:
    target: rooms
    where:
      id: "${target_room_id}"  # 固定 Room 2
      enable: 1
    as: target_rooms
    description: "查询烤箱所在房间"
```

#### 2. 遍历目标房间（实际只有一个）
```yaml
- for:
    in: "${target_rooms}"
    as: room
    do:
      # 处理该房间的烤箱
```

#### 3. 查询 Unloader AGV（使用 model 过滤）
```yaml
- query:
    target: agvs
    where:
      model: "Unloader"  # AGV 表无 room_id 栏位
      enable: 1
    as: unloader_agvs
    description: "查询所有 Unloader AGV"
```

#### 4. 遍历每个 Unloader AGV
```yaml
- for:
    in: "${unloader_agvs}"
    as: agv
    do:
      # 检查该 AGV 车上载具
```

#### 5. 查询 AGV 车上载具（含 status_id 过滤）
```yaml
- query:
    target: carriers
    where:
      port_in: "${unloader_ports}"  # UnloaderAGV Port 上的载具
      status_id: 601  # 烤箱上排完成
    as: agv_carriers
    description: "查询 UnloaderAGV Port 上烤箱上排完成的载具"
```

#### 6. 检查载具数量
```yaml
- set:
    agv_carrier_count: "${agv_carriers.length}"
    required_count: "${batch_size}"
    has_enough_carriers: "${agv_carrier_count >= required_count}"
```

#### 7. 查询烤箱下排空位（固定配置）
```yaml
- if:
    condition: "${has_enough_carriers}"
    then:
      - query:
          target: eqp_ports
          where:
            equipment_id: "${oven_equipment_id}"  # 固定 206
            port_in: "${ports}"                   # [2065, 2066, 2067, 2068]
            status: "empty"
          as: empty_ports
          description: "查询烤箱下排 Station 05 空位"
```

#### 8. 检查空位数量
```yaml
- set:
    empty_count: "${empty_ports.length}"
    required_space: "${batch_size}"  # 4格
    has_enough_space: "${empty_count >= required_space}"
```

#### 9. 检查 Presence 信号（硬件传感器验证）
```yaml
- if:
    condition: "${has_enough_space}"
    then:
      # 查询这些 Port 的 Presence 信号
      - query:
          target: eqp_signals
          where:
            eqp_port_id_in: "${ports}"
            name_like: "%Presence"
            value: "0"  # 0 = 无载具在席
          as: empty_presence_signals
          description: "确认传感器无载具在席"

      # 检查是否所有 Port 的 Presence 都是 0
      - set:
          presence_check_count: "${empty_presence_signals.length}"
          presence_all_clear: "${presence_check_count >= required_space}"
```

#### 10. 双重验证 + 重复检查
```yaml
- if:
    condition: "${has_enough_space && presence_all_clear}"
    then:
      # 检查是否已存在未完成的放料任务
      - query:
          target: tasks
          where:
            work_id: "${work_id}"
            room_id: "${room.id}"
            status_id_in: [0, 1, 2, 3]  # 未完成的状态
          as: existing_tasks
          description: "检查是否已存在放料任务"
```

#### 11. 创建放料任务（含双重验证）
```yaml
- if:
    condition: "${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "unloader_put"
            name: "房间${room.id}烤箱下排 Station${station} 放料"
            description: "AGV 有${agv_carrier_count}个载具，烤箱下排有${empty_count}格空位，传感器确认无载具（批量4格）"
            work_id: "${work_id}"
            room_id: "${room.id}"
            priority: "${priority}"
            status_id: 1  # PENDING
            parameters:
              station: "${station}"
              work_id: "${work_id}"
              room_id: "${room.id}"
              equipment_id: "${oven_equipment_id}"
              ports: "${ports}"
              batch_size: "${batch_size}"
              row: "${row}"
              model: "${model}"
              carrier_count: "${agv_carrier_count}"
              presence_verified: true  # 已验证 Presence 信号
              reason: "AGV 有载具，烤箱下排有空位，传感器确认无载具（双重验证）"
          description: "创建 Unloader AGV 放料任务（批量4格，含传感器验证）"
```

## 🔍 查询条件详解

### 1. Carrier Status 过滤（制程阶段控制）

**AGV 车上载具 status_id 过滤**：
- `port_in: [2111, 2112, 2113, 2114]` (UnloaderAGV Port)
- `status_id: 601` (烤箱上排完成)
  - TAKE_OVEN 完成后，载具状态更新为 601
  - 表示载具已完成烤箱上排制程，准备放入烤箱下排
  - **确保不会处理错误阶段的载具**

**数量要求**：
- **至少4个载具**（批量4格）
- **不支持部分放料**（必须满4格）

**制程流控制**：
```
status_id=601（烤箱上排完成）→ PUT_OVEN → status_id=602（烤箱处理中）
```

### 2. Equipment Port 查询（软件状态检查）

**烤箱下排 Station 05 空位**（固定配置）：
- `equipment_id: 206` (固定的烤箱设备ID)
- `port_in: [2065, 2066, 2067, 2068]` (完整 Port ID)
- `status: "empty"` (空位)

**空位要求**（固定4格批量）：
- **至少4格空位**（Port 全部空闲）
- **不支持部分放料**（必须4格全空）

**Port 映射**（固定 Station 05）：
- **Station 05**: Port 2065-2066-2067-2068（下排进料位置，批量4格）
- **PUT_OVEN 只使用下排 Station 05**

### 3. Presence 信号验证（硬件传感器检查）

**双重验证机制**：
```yaml
- query:
    target: eqp_signals
    where:
      eqp_port_id_in: [2065, 2066, 2067, 2068]
      name_like: "%Presence"
      value: "0"  # 0 = 无载具在席
    as: empty_presence_signals
```

**验证逻辑**：
- **软件状态**: eqp_ports.status = "empty"
- **硬件状态**: eqp_signals.value = "0"（Presence 传感器）
- **双重确认**: 两者都必须确认空位才创建任务

**重要性**：
- 防止软件状态与硬件实际不一致
- 避免碰撞和设备损坏
- 提高任务执行可靠性

### 4. Room 过滤（设备定位控制）

**目标房间过滤**：
- `target_room_id: 2` (烤箱 206 位于房间2)
- 只查询 Room 2，不查询其他房间
- 防止为错误房间创建任务

### 5. Task 重复检查

**防止重复创建**：
- 检查相同 `work_id` 和 `room_id`
- 状态为未完成（0=创建, 1=待分派, 2=执行中, 3=暂停）
- 如果存在未完成任务，不创建新任务

## ⚠️ 注意事项

### 五重验证机制（测试完成✅）
1. **Carrier Status 过滤**: status_id=601（烤箱上排完成）
2. **EqP Port 状态**: eqp_ports.status = "empty"
3. **Presence 信号**: eqp_signals.value = "0"（硬件传感器）
4. **Room 过滤**: target_room_id = 2（烤箱专属房间）
5. **重复检查**: 无未完成任务

### Station-based 设计重点
- **编码规则**: work_id 使用 Station 编号（**只有 05**），非 Port 起始号
- **UnloaderAGV 自定义映射**: **Station 05 固定批量4格**（UnloaderAGV 特有）
- **单向操作**: **Station 05 只支持 PUT 操作**（固定下排进料）
- **完整 Port ID**: 使用 2065-2068 而非 5-8

### 烤箱固定方向设计
- **下排 Station 05（本 Flow）**: Port 2065-2068，**只 PUT**（进料）
- **上排 Station 01（Flow 3）**: Port 2061-2064，**只 TAKE**（出料）
- **固定单向流程**: 下排进料 → 烘干制程 → 上排出料

### 批量处理逻辑
- **固定4格批量**: 必须有 ≥ 4个载具，烤箱需 ≥ 4格空位（Port 全部空闲）
- **不支持部分放料**（要么放满4格，要么不放）
- **车上需全满**: AGV 车上必须有4个 status_id=601 的载具才创建任务

### 双重验证的重要性
- **软件 vs 硬件**: 防止数据库状态与实际硬件不一致
- **安全性**: 避免碰撞和设备损坏
- **可靠性**: 确保任务执行前条件完全满足

### 烘干制程衔接（固定方向）
- **本 Flow（PUT_OVEN）**: 只放入下排 Station 05
- **烤箱内部**: 烘干制程
- **Flow 3（TAKE_OVEN）**: 只从上排 Station 01 取出
- **单向流程**: 消除双向操作的复杂性

### AGV 状态判断
- 检查车上载具数量（**≥ 4个**）
- 建议检查最近一次任务是否为 TAKE_OVEN
- 确保载具 status_id = 601（烤箱上排完成）

### 烤箱设备配置
- **固定 equipment_id**: 206
- **固定 Station**: 05（下排进料）
- **固定 Work ID**: 2060502
- **不需要遍历**: 只有1个 Station，流程简化

## 🧪 测试要点

### 单元测试
1. ✅ 查询逻辑正确性
   - 正确查询 AGV 车上载具
   - 正确查询烤箱下排空位

2. ✅ 数量判断
   - 载具数量 >= 2
   - 空位数量 >= 2

3. ✅ 重复检查
   - 存在未完成任务时不创建
   - 任务参数正确传递

4. ✅ Port 组端口映射
   - 正确使用固定 equipment_id: 206
   - 正确区分 Port 5-6 和 Port 7-8 的下排端口
   - 正确区分上排端口和下排端口

### 整合测试
1. ✅ 与 TAKE_PRE_DRYER 衔接
   - TAKE_PRE_DRYER 完成
   - 载具状态更新为在车上
   - PUT_OVEN 检测到并创建任务

2. ✅ RCS 调度
   - 任务正确创建
   - RCS 分派给对应的 Unloader AGV
   - AGV 执行放料动作

3. ✅ 与烤箱制程衔接
   - 放料到下排完成
   - 烤箱开始烘干
   - 载具最终移到上排

### 边界测试
1. ✅ AGV 只有1个载具
   - 不创建任务（需要至少2个）

2. ✅ 烤箱下排只有1格空位
   - 不创建任务（需要至少2格）

3. ✅ 多个 Unloader AGV
   - 正确分配任务
   - 不产生冲突

## 📊 成功指标

### 功能指标
- ✅ 正确检测 AGV 车上载具
- ✅ 正确检测烤箱下排空位
- ✅ 正确创建放料任务
- ✅ 不产生重复任务
- ✅ 任务参数完整准确

### 性能指标
- 执行间隔：12秒
- 响应时间：< 2秒
- 任务创建延迟：< 30秒（从载具就绪到任务创建）

### 可靠性指标
- 零漏检：所有就绪载具都能被检测
- 零误创建：不创建重复或错误任务
- 上下排区分：100% 正确区分上排和下排

## 🔗 相关文档

- **Unloader AGV 代码**: `/app/agv_ws/src/unloader_agv/unloader_agv/robot_states/put_oven/`
- **Work ID 定义**: 数据库 `agvc.work` 表
- **前置流程**: Flow 1 - `unloader_take_pre_dryer.yaml`
- **后续流程**: Flow 3 - `unloader_take_oven.yaml`
