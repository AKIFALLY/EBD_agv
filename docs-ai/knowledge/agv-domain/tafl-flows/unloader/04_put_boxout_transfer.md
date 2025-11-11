# Flow 4: unloader_put_boxout_transfer.yaml

## 🎯 業務目的
將 Unloader AGV 車上的載具放入出口傳送箱，等待 Cargo AGV 收集

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `unloader_put_boxout_transfer.yaml` |
| Flow ID | `unloader_put_boxout_transfer` |
| 優先級 | 42 |
| 執行間隔 | 12 秒 |
| Work ID | **2020102**（Station-based，只有1個）|

## 🏭 業務場景

### 前置條件
1. Unloader AGV 已從烤箱上排取出載具（TAKE_OVEN 完成）
2. AGV 車上有烘幹完成的載具
3. 出口傳送箱有空位
4. Unloader AGV 處於空閒或已完成取料

### 觸發條件
- Unloader AGV 車上有載具（**至少4個，批量4格**）
- 出口傳送箱 Station 01 有空位（**至少4格空位**）
- 沒有重復的未完成任務

### 執行結果
- 創建 Unloader AGV 放料任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- RCS 系統分派給對應的 Unloader AGV
- 載具最終等待 Cargo AGV 收集並裝載回 Rack

## 🔧 技術規格

### 出口傳送箱配置（Station-based）

**物理結構**：
- Equipment 202（出口傳送箱）
- 4個 Port（Port 1-4）
- **只使用 Station 01**（**批量4格**）
- Equipment ID: `room_id * 100 + 2`

**Station-Port 映射**（UnloaderAGV 自定義）：
- **Station 01**: Port 1-2-3-4（**批量4格**）

**Work ID 對應**（Station-based 編碼）：
- `2020102`: Station 01 放出口傳送箱（Port 1-2-3-4，批量4格）

**關鍵特點**：
- ✅ **只有1個 Work ID**（統一4格批量處理）
- ✅ **每個房間1個出口傳送箱**（Equipment ID = `room_id * 100 + 2`）
- ✅ **UnloaderAGV 特定映射**（Station 01 批量4格）

### 完整制程流程

```
入口传送箱（Cargo AGV 卸载）
    ↓
Loader AGV（前段制程）
    ↓
Unloader AGV（后段制程）
    ↓
出口传送箱（Unloader 放料）← 本 Flow
    ↓
Cargo AGV（出口装载回 Rack）
```

**本 Flow 在制程中的位置**：
- 最后一个 Unloader AGV 操作
- 连接 Unloader 后段制程和 Cargo 出口装载
- 完成后等待 Cargo AGV 收集

### 批量处理逻辑

**统一4格批量处理**（Station 01）：
- **一次放料操作放入4格**（统一批量处理）
- **需要检查 AGV 至少有4个载具**（车上需全满）
- **需要检查出口传送箱至少有4格空位**（Port 1-4 全部空闲）
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
  id: "unloader_put_boxout_transfer"
  name: "Unloader AGV 放料到出口传送箱"
  enabled: true
  version: "1.0.0"
  description: "检查 Unloader AGV 车上载具和出口传送箱空位，创建放料任务"
  author: "TAFL System"
  created_at: "2025-10-03"
  tags: ["unloader", "boxout", "transfer", "put", "automation"]
```

### Settings
```yaml
settings:
  execution_interval: 12  # 每12秒执行一次
```

### Variables
```yaml
variables:
  priority: 42             # 高优先级（送至出口等待收集）
  model: "UNLOADER"        # AGV 型号
  boxout_equipment_type: 202  # 出口传送箱设备类型
  # 固定 Station 配置（只有 Station 01，无需遍历）
  priority: 42             # 高优先级（送至出口等待收集）
  model: "UNLOADER"        # AGV 型号
  boxout_equipment_type: 202  # 出口传送箱设备类型
  # 固定 Station 配置（只有 Station 01，无需遍历）
  station: 1               # Station 01（批量4格）
  work_id: 2020102         # 唯一的 Work ID
  ports: [1, 2, 3, 4]      # Port 1-4
  ports: [1, 2, 3, 4]      # Port 1-4
  batch_size: 4            # 批量4格
```

## 🔄 流程逻辑

### 主要步骤

#### 1. 查询所有房间
#### 1. 查询所有房间
```yaml
- query:
    target: rooms
    where:
      enabled: true
    as: active_rooms
    description: "查询所有启用的房间"
      enabled: true
    as: active_rooms
    description: "查询所有启用的房间"
```

#### 2. 遍历每个房间
#### 2. 遍历每个房间
```yaml
- for:
    in: "${active_rooms}"
    in: "${active_rooms}"
    as: room
    do:
      # 处理该房间的出口传送箱
```

#### 3. 查询 Unloader AGV
#### 3. 查询 Unloader AGV
```yaml
- query:
    target: agvs
    where:
      agv_type: "unloader"
      room_id: "${room.id}"
      agv_type: "unloader"
      room_id: "${room.id}"
    as: unloader_agvs
    description: "查询房间${room.id}的 Unloader AGV"
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
#### 5. 查询 AGV 车上载具
```yaml
- query:
    target: carriers
    where:
      agv_id: "${agv.id}"
      status_id: 603  # 烘乾机处理完成
      agv_id: "${agv.id}"
      status_id: 603  # 烘乾机处理完成
    as: agv_carriers
    description: "查询 AGV ${agv.name} 车上载具"
    description: "查询 AGV ${agv.name} 车上载具"
```

#### 6. 检查载具数量（固定4格批量）
#### 6. 检查载具数量（固定4格批量）
```yaml
- set:
    carrier_count: "${agv_carriers.length}"
    required_count: "${batch_size}"        # 固定4格
    has_enough_carriers: "${carrier_count >= required_count}"
    carrier_count: "${agv_carriers.length}"
    required_count: "${batch_size}"        # 固定4格
    has_enough_carriers: "${carrier_count >= required_count}"
```

#### 7. 查询出口传送箱 Station 01 空位（固定配置）
#### 7. 查询出口传送箱 Station 01 空位（固定配置）
```yaml
- set:
    boxout_equipment_id: "${room.id * 100 + 2}"  # 动态计算

- query:
    target: equipment_ports
    where:
      equipment_id: "${boxout_equipment_id}"
      port_in: "${ports}"                   # [1, 2, 3, 4] 固定 Station 01
      status: "empty"
    as: empty_ports
    description: "查询房间${room.id}出口传送箱 Station 01 空位（Port 1-4）"
- set:
    boxout_equipment_id: "${room.id * 100 + 2}"  # 动态计算

- query:
    target: equipment_ports
    where:
      equipment_id: "${boxout_equipment_id}"
      port_in: "${ports}"                   # [1, 2, 3, 4] 固定 Station 01
      status: "empty"
    as: empty_ports
    description: "查询房间${room.id}出口传送箱 Station 01 空位（Port 1-4）"
```

#### 8. 检查空位数量（固定4格批量）
#### 8. 检查空位数量（固定4格批量）
```yaml
- set:
    empty_count: "${empty_ports.length}"
    required_space: 4                      # 固定4格批量
    has_space: "${empty_count >= required_space}"
    required_space: 4                      # 固定4格批量
    has_space: "${empty_count >= required_space}"
```

#### 9. 检查重复任务
#### 9. 检查重复任务
```yaml
- query:
    target: tasks
    where:
      work_id: "${work_id}"
      room_id: "${room.id}"
      agv_id: "${agv.id}"
      status_id_in: [0, 1, 2, 3]  # 未完成的状态
    as: existing_tasks
    description: "检查是否已存在放料任务"
- query:
    target: tasks
    where:
      work_id: "${work_id}"
      room_id: "${room.id}"
      agv_id: "${agv.id}"
      status_id_in: [0, 1, 2, 3]  # 未完成的状态
    as: existing_tasks
    description: "检查是否已存在放料任务"
```

#### 10. 创建放料任务（固定 Station 01 配置）
#### 10. 创建放料任务（固定 Station 01 配置）
```yaml
- if:
    condition: "${has_enough_carriers} && ${has_space} && ${existing_tasks.length == 0}"
    condition: "${has_enough_carriers} && ${has_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "unloader_put"
            name: "房间${room.id}出口传送箱 Station 01 放料"
            description: "将 AGV ${agv.name} 车上载具放入出口传送箱 Station 01（批量4格）"
            work_id: "${work_id}"        # 固定 2020102
            name: "房间${room.id}出口传送箱 Station 01 放料"
            description: "将 AGV ${agv.name} 车上载具放入出口传送箱 Station 01（批量4格）"
            work_id: "${work_id}"        # 固定 2020102
            room_id: "${room.id}"
            agv_id: "${agv.id}"
            agv_id: "${agv.id}"
            priority: "${priority}"
            status_id: 1  # PENDING
            parameters:
              station: 1                 # 固定 Station 01
              work_id: "${work_id}"      # 固定 2020102
              station: 1                 # 固定 Station 01
              work_id: "${work_id}"      # 固定 2020102
              room_id: "${room.id}"
              agv_id: "${agv.id}"
              agv_name: "${agv.name}"
              agv_id: "${agv.id}"
              agv_name: "${agv.name}"
              model: "${model}"
              carrier_count: "${carrier_count}"
              empty_ports: "${empty_count}"
              ports: "${ports}"          # [1, 2, 3, 4]
              batch_size: 4              # 固定4格
              boxout_equipment_id: "${boxout_equipment_id}"
              reason: "AGV 车上有载具，出口传送箱 Station 01 有空位"
          description: "创建 Unloader AGV 放料任务（固定 Station 01 批量4格）"
              carrier_count: "${carrier_count}"
              empty_ports: "${empty_count}"
              ports: "${ports}"          # [1, 2, 3, 4]
              batch_size: 4              # 固定4格
              boxout_equipment_id: "${boxout_equipment_id}"
              reason: "AGV 车上有载具，出口传送箱 Station 01 有空位"
          description: "创建 Unloader AGV 放料任务（固定 Station 01 批量4格）"
```

## 🔍 查询条件详解

### Carrier 查询条件
### Carrier 查询条件

**AGV 车上载具**：
- `agv_id`: 特定 Unloader AGV
- `status_id: 603` (烘乾机处理完成)
  - TAKE_OVEN 完成后，载具状态保持为 603
  - 表示载具已完成烘干制程，在 AGV 车上准备放入出口传送箱
**AGV 车上载具**：
- `agv_id`: 特定 Unloader AGV
- `status_id: 603` (烘乾机处理完成)
  - TAKE_OVEN 完成后，载具状态保持为 603
  - 表示载具已完成烘干制程，在 AGV 车上准备放入出口传送箱

**数量要求**（固定4格批量）：
- **至少4个载具**（一次放4格）
- S尺寸：4格可用（需全满）
- L尺寸：不支持（需要完整4格）
- S尺寸：4格可用（需全满）
- L尺寸：不支持（需要完整4格）

### Equipment Port 查询条件（固定 Station 01）
### Equipment Port 查询条件（固定 Station 01）

**出口传送箱 Station 01 空位**：
- `equipment_id`: 出口传送箱设备ID
  - 计算公式：`room_id * 100 + 2`
  - Equipment Type: 202
- `port_in: [1, 2, 3, 4]` (固定 Station 01)
- `status`: "empty"（空位）
- `equipment_id`: 出口传送箱设备ID
  - 计算公式：`room_id * 100 + 2`
  - Equipment Type: 202
- `port_in: [1, 2, 3, 4]` (固定 Station 01)
- `status`: "empty"（空位）

**空位要求**（固定4格批量）：
- **至少4格空位**（一次放4格）
- Port 1-4 全部空闲才创建任务
- 不支持部分放料
- **至少4格空位**（一次放4格）
- Port 1-4 全部空闲才创建任务
- 不支持部分放料

**Station 01 Port 映射**：
- **Station 01**: Port 1-2-3-4（**批量4格**）
- PUT_BOXOUT_TRANSFER 只使用 Station 01（Port 1-4）

### Task 重复检查
- **Station 01**: Port 1-2-3-4（**批量4格**）
- PUT_BOXOUT_TRANSFER 只使用 Station 01（Port 1-4）

### Task 重复检查

**防止重复创建**：
- 检查相同 `work_id`、`room_id`、`agv_id`
- 检查相同 `work_id`、`room_id`、`agv_id`
- 状态为未完成（0=创建, 1=待分派, 2=执行中, 3=暂停）
- 如果存在未完成任务，不创建新任务

## ⚠️ 注意事项

### 入口/出口传送箱区分
- **入口传送箱（BOX_IN_TRANSFER）**:
  - Equipment ID: `room_id * 100 + 1`
  - Equipment ID: `room_id * 100 + 1`
  - Cargo AGV 卸载（从 Rack 到入口箱）
  - Loader AGV 取料（从入口箱到车上）

- **出口传送箱（BOX_OUT_TRANSFER）**:
  - Equipment ID: `room_id * 100 + 2`
  - Equipment ID: `room_id * 100 + 2`
  - Unloader AGV 放料（从车上到出口箱）← 本 Flow
  - Cargo AGV 装载（从出口箱到 Rack）

### Station-based 设计重点
- **编码规则**: work_id 使用 Station 编号（**只有 01**），非 Port 起始号
- **UnloaderAGV 自定义映射**: **Station 01 固定批量4格**（UnloaderAGV 特有）
- **单一Station**: 只有 Station 01，流程简化

### 批量处理逻辑
- **固定4格批量**: 必须有 ≥ 4个载具，出口传送箱需 ≥ 4格空位（Port 1-4 全空）
- **固定4格批量**: 必须有 ≥ 4个载具，出口传送箱需 ≥ 4格空位（Port 1-4 全空）
- **不支持部分放料**（要么放满4格，要么不放）
- **车上需全满**: AGV 车上必须有4个载具才创建任务
- **车上需全满**: AGV 车上必须有4个载具才创建任务

### 与 Cargo AGV 的协作
- Unloader PUT_BOXOUT_TRANSFER 完成
- 载具状态更新为"在出口传送箱"
- Cargo AGV Flow（`cargo_exit_load.yaml`）检测到
- Cargo AGV 从出口箱取出装载回 Rack

### 完整制程闭环
```
Cargo 入口卸载 → Loader 前段制程 → Unloader 后段制程 →
Unloader 出口放料 → Cargo 出口装载 → 完成
```

### 出口传送箱设备配置
- **固定 equipment_type**: 202
- **固定 Station**: 01（批量4格）
- **固定 Work ID**: 2020102
- **不需要遍历**: 只有1个 Station，流程简化
- **Equipment ID 计算**: `room_id * 100 + 2`
- **不要与入口传送箱混淆**: 入口 = `room_id * 100 + 1`

### AGV 状态判断
- 检查车上载具数量（**≥ 4个**）
- 建议检查最近一次任务是否为 TAKE_OVEN
- 确保载具 status_id = 603（烘干完成）
- 建议检查最近一次任务是否为 TAKE_OVEN
- 确保载具 status_id = 603（烘干完成）

## 🧪 测试要点

### 单元测试
1. ✅ 查询逻辑正确性
   - 正确查询 AGV 车上载具
   - 正确查询出口传送箱 Station 01 空位
   - 正确使用 port_in: [1, 2, 3, 4] 查询固定端口

2. ✅ 数量判断（固定4格批量）
   - 载具数量 >= 4
   - 空位数量 >= 4（Port 1-4 全空）

3. ✅ 重复检查
   - 存在未完成任务时不创建
   - 任务参数正确传递

4. ✅ Station 01 固定配置
   - 正确使用固定 equipment_type: 202
   - 正确使用固定 work_id: 2020102
   - 正确使用固定 Station 01 Port 1-4
   - 正确计算 equipment_id: `room_id * 100 + 2`
   - 不与入口传送箱混淆（+1）

### 整合测试
1. ✅ 与 TAKE_OVEN 衔接
   - TAKE_OVEN 完成
   - 载具状态更新为在车上
   - PUT_BOXOUT_TRANSFER 检测到并创建任务

2. ✅ RCS 调度
   - 任务正确创建
   - RCS 分派给对应的 Unloader AGV
   - AGV 执行放料动作

3. ✅ 与 Cargo AGV 衔接
   - PUT_BOXOUT_TRANSFER 完成
   - 载具状态更新为在出口传送箱
   - Cargo AGV (`cargo_exit_load.yaml`) 检测到
   - Cargo AGV 创建装载任务

### 完整流程测试
1. ✅ Unloader 四个 Flow 串联
   - TAKE_PRE_DRYER → PUT_OVEN → TAKE_OVEN → PUT_BOXOUT_TRANSFER
   - 每个流程正确衔接
   - 状态转换正确

2. ✅ 与 Cargo 协作
   - Cargo 入口卸载 → Unloader 后段制程 → Cargo 出口装载
   - 完整制程闭环

### 边界测试
1. ✅ AGV 只有3个载具
   - 不创建任务（需要至少4个）

2. ✅ 出口传送箱只有3格空位
   - 不创建任务（需要至少4格）

3. ✅ 出口传送箱已满（0格空位）
   - 不创建任务
   - 等待 Cargo AGV 清空

4. ✅ 多个 Unloader AGV
   - 正确为每个 AGV 分配任务
   - 不产生冲突

## 📊 成功指标

### 功能指标
- ✅ 正确检测 AGV 车上载具
- ✅ 正确检测出口传送箱空位
- ✅ 正确创建放料任务
- ✅ 不产生重复任务
- ✅ 任务参数完整准确
- ✅ 正确区分入口和出口传送箱

### 性能指标
- 执行间隔：12秒
- 响应时间：< 2秒
- 任务创建延迟：< 30秒（从载具就绪到任务创建）

### 可靠性指标
- 零漏检：所有就绪载具都能被检测
- 零误创建：不创建重复或错误任务
- 入出口区分：100% 正确使用出口传送箱
- 与 Cargo 协作：完整的制程闭环

## 🔗 相关文档

- **Unloader AGV 代码**: `/app/agv_ws/src/unloader_agv/unloader_agv/robot_states/put_boxout_transfer/`
- **Work ID 定义**: 数据库 `agvc.work` 表
- **前置流程**: Flow 3 - `unloader_take_oven.yaml`
- **后续流程**: Cargo AGV - `cargo_exit_load.yaml`
