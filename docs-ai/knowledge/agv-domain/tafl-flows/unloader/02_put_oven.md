# Flow 2: unloader_put_oven.yaml

## 🎯 業務目的
將 Unloader AGV 車上的載具放入烤箱**下排**（Station 05），進入烘幹制程

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `unloader_put_oven.yaml` |
| Flow ID | `unloader_put_oven` |
| 優先級 | 44 |
| 執行間隔 | 12 秒 |
| Work ID | **2060502**（Station-based，只有1個）|

## 🏭 業務場景

### 前置條件
1. Unloader AGV 已從預烘機取出載具（TAKE_PRE_DRYER 完成）
2. AGV 車上有載具需要放入烤箱（status_id: 503）
3. **烤箱下排（Station 05）有空位**
4. Unloader AGV 處於空閒或已完成取料

### 觸發條件
- Unloader AGV 車上有載具（**至少4個載具，批量4格**）
- **烤箱下排 Station 05 有4格空位**（Port 5-6-7-8 全部空閒）
- 沒有重復的未完成任務

### 執行結果
- 創建 Unloader AGV 放料任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- RCS 系統分派給對應的 Unloader AGV

## 🔧 技術規格

### 烤箱配置（Station-based）

**物理結構**：
- Equipment 206（烤箱）
- 8個 Port（Port 1-8）
- **只使用 Station 05**（**下排進料**）
- Equipment ID: 固定為 206

**Station-Port 映射**（UnloaderAGV 自定義）：
- **Station 05**: Port 5-6-7-8（**批量4格**/下排/**只 PUT**）

**下排配置**（Station 05，固定進料）：
- **只支持 PUT 操作**（單向進料）
- Station 05: Port 5-6-7-8（批量4格）
- **不支持 TAKE 操作**（出料由 Flow 3 在上排進行）

**Work ID 對應**（Station-based 編碼）：
- `2060502`: Station 05 放烤箱（Port 5-6-7-8，批量4格/下排/**只 PUT**）

### 烘幹制程流程（Station-based，固定方向）

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
  priority: 44             # 高优先级（进入烘干制程）
  model: "UNLOADER"        # AGV 型号
  oven_equipment_id: 206   # 固定的烤箱设备 ID
  # 固定 Station 配置（只有 Station 05，无需遍歷）
  station: 5               # Station 05（下排进料）
  work_id: 2060502         # 唯一的 Work ID
  ports: [5, 6, 7, 8]      # Port 5-8（下排）
  batch_size: 4            # 批量4格
  row: "lower"             # 下排
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
      # 处理该房间的烤箱
```

#### 3. 查询 Unloader AGV
```yaml
- query:
    target: agvs
    where:
      agv_type: "unloader"
      room_id: "${room.id}"
    as: unloader_agvs
    description: "查询房间${room.id}的 Unloader AGV"
```

#### 4. 遍历每个 Unloader AGV
```yaml
- for:
    in: "${unloader_agvs}"
    as: agv
    do:
      # 检查该 AGV 车上载具
```

#### 5. 查询 AGV 车上载具
```yaml
- query:
    target: carriers
    where:
      agv_id: "${agv.id}"
      status_id: 503  # 预烘乾机处理完成
    as: agv_carriers
    description: "查询 AGV ${agv.name} 车上载具"
```

#### 6. 检查载具数量
```yaml
- set:
    carrier_count: "${agv_carriers.length}"
    has_carriers: "${carrier_count >= 2}"  # 至少2个载具
```

#### 7. 查询烤箱 Station 05 空位（固定配置，无需遍历）
```yaml
- query:
    target: equipment_ports
    where:
      equipment_id: "${oven_equipment_id}"  # 固定 206
      port_in: "${ports}"                   # [5, 6, 7, 8]
      status: "empty"
    as: empty_ports
    description: "查询 Station 05 下排空位（Port 5-8）"
```

#### 8. 检查空位数量（固定4格批量）
```yaml
- set:
    empty_count: "${empty_ports.length}"
    required_space: 4                     # 固定4格批量
    has_space: "${empty_count >= required_space}"
```

#### 9. 检查重复任务
```yaml
- query:
    target: tasks
    where:
      work_id: "${work_id}"               # 固定 2060502
      room_id: "${room.id}"
      agv_id: "${agv.id}"
      status_id_in: [0, 1, 2, 3]  # 未完成的状态
    as: existing_tasks
    description: "检查是否已存在放料任务"
```

#### 10. 创建放料任务（固定 Station 05）
```yaml
- if:
    condition: "${has_carriers} && ${has_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "unloader_put"
            name: "房间${room.id}烤箱下排 Station 05 放料"
            description: "将 AGV ${agv.name} 车上载具放入烤箱 Station 05（批量4格/下排）"
            work_id: "${work_id}"         # 固定 2060502
            room_id: "${room.id}"
            agv_id: "${agv.id}"
            priority: "${priority}"
            status_id: 1  # PENDING
            parameters:
              station: 5                  # 固定 Station 05
              work_id: "${work_id}"       # 固定 2060502
              room_id: "${room.id}"
              agv_id: "${agv.id}"
              agv_name: "${agv.name}"
              model: "${model}"
              carrier_count: "${carrier_count}"
              empty_ports: "${empty_count}"
              ports: "${ports}"           # [5, 6, 7, 8]
              batch_size: 4               # 固定4格
              row: "lower"                # 下排
              equipment_id: "${oven_equipment_id}"
              reason: "AGV 车上有载具，烤箱下排 Station 05 有空位"
          description: "创建 Unloader AGV 放料任务（固定 Station 05 下排进料）"
```

## 🔍 查询条件详解

### Carrier 查询条件

**AGV 车上载具**：
- `agv_id`: 特定 Unloader AGV
- `status_id: 503` (预烘乾机处理完成)
  - TAKE_PRE_DRYER 完成后，载具状态更新为 503
  - 表示载具已完成预烘干制程，在 AGV 车上准备放入烤箱

**数量要求**：
- 至少2个载具（一次放2格）
- S尺寸最多4个，L尺寸最多2个

### Equipment Port 查询条件

**烤箱下排 Station 05 空位**（固定配置）：
- `equipment_id: 206` (固定的烤箱设备ID)
- `port_in: [5, 6, 7, 8]` (固定 Station 05 Port 映射)
- `status: "empty"` (空位)

**空位要求**（固定4格批量）：
- **至少4格空位**（一次放4格，Port 5-8 全部空闲）

**Port 映射**（固定 Station 05）：
- **Station 05**: Port 5-6-7-8（下排进料位置，批量4格）
- **PUT_OVEN 只使用下排 Station 05**（Port 5-8）

### Task 重复检查

**防止重复创建**：
- 检查相同 `work_id`、`room_id`、`agv_id`
- 状态为未完成（0=创建, 1=待分派, 2=执行中, 3=暂停）
- 如果存在未完成任务，不创建新任务

## ⚠️ 注意事项

### Station-based 设计重点
- **编码规则**: work_id 使用 Station 编号（**只有 05**），非 Port 起始号
- **UnloaderAGV 自定义映射**: **Station 05 固定批量4格**（UnloaderAGV 特有）
- **单向操作**: **Station 05 只支持 PUT 操作**（固定下排进料）

### 烤箱固定方向设计
- **下排 Station 05（本 Flow）**: Port 5-8，**只 PUT**（进料）
- **上排 Station 01（Flow 3）**: Port 1-4，**只 TAKE**（出料）
- **固定单向流程**: 下排进料 → 烘干制程 → 上排出料

### 批量处理逻辑
- **固定4格批量**: 必须有 ≥ 4个载具，烤箱需 ≥ 4格空位（Port 5-8 全部空闲）
- **不支持部分放料**（要么放满4格，要么不放）
- **车上需全满**: AGV 车上必须有4个载具才创建任务

### 烘干制程衔接（固定方向）
- **本 Flow（PUT_OVEN）**: 只放入下排 Station 05
- **烤箱内部**: 烘干制程
- **Flow 3（TAKE_OVEN）**: 只从上排 Station 01 取出
- **单向流程**: 消除双向操作的复杂性

### AGV 状态判断
- 检查车上载具数量（**≥ 4个**）
- 建议检查最近一次任务是否为 TAKE_PRE_DRYER
- 确保载具 status_id = 503（预烘干完成）

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
