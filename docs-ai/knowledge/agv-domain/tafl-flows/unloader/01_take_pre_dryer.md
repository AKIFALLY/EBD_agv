# Flow 1: unloader_take_pre_dryer.yaml

## 🎯 業務目的
從預烘機批量取料，啟動 Unloader AGV 的後段制程流程

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `unloader_take_pre_dryer.yaml` |
| Flow ID | `unloader_take_pre_dryer` |
| 優先級 | 45 |
| 執行間隔 | 12 秒 |
| Work ID | **2050101, 2050301**（Station-based，只有2個）|

## 🏭 業務場景

### 前置條件
1. Loader AGV 已經將載具放入預烘機（PUT_PRE_DRYER）
2. 預烘機完成預烘幹制程
3. 載具狀態更新為"預烘幹完成"（status_id: 503）
4. Unloader AGV 處於空閒或有空餘車位

### 觸發條件
- 預烘機有完成預烘幹的載具（status_id: 503）
- Unloader AGV 車上有空位（最多4格）
- 沒有重復的未完成任務

### 執行結果
- 創建 Unloader AGV 取料任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- RCS 系統分派給空閒的 Unloader AGV

## 🔧 技術規格

### 預烘機配置（Station-based）

**物理結構**：
- Equipment 205（預烘機）
- 8個 Port（Port 1-8）
- **2個 Station**（**Station 01, 03**）

**Station-Port 映射**（UnloaderAGV 自定義）：
- **Station 01**: Port 1-2-5-6（**批量4格**）
- **Station 03**: Port 3-4-7-8（**批量4格**）

**Work ID 對應**（Station-based 編碼）：
- `2050101`: Station 01 取預烘（Port 1-2-5-6，**批量4格**）
- `2050301`: Station 03 取預烘（Port 3-4-7-8，**批量4格**）

**關鍵特點**：
- ✅ **只有2個 Work ID**（全部4格批量處理）
- ✅ **統一批量處理**（不再有2格標準處理）
- ✅ **UnloaderAGV 特定映射**（Station 跨上下排，提升效率）

### Station-based 設計說明

**編碼規則**：`room_id + equipment_type + station + action_type`
- work_id 中的 "01/03" 代表 Station 編號（非 Port 起始號）
- 示例：`2050101` = room2 + pre_dryer(05) + station01(01) + take(01)

**UnloaderAGV 自定義映射**：
- 在 `equipment_stations.py` 中實作
- **所有 Station 統一批量4格處理**（UnloaderAGV 特有）
- Station 01/03 都跨上下排（Port 1-2-5-6 和 Port 3-4-7-8）

### Unloader AGV 車載配置

**S尺寸產品**：
- 4格可用（上2格 + 下2格）
- 最大容量：4個 Carrier

**L尺寸產品**：
- 2格可用（僅上2格）
- 最大容量：2個 Carrier

### 批量處理邏輯

**統一4格批量處理**（**所有 Station 01/03**）：
- **每個 Station 包含4格**（跨上下排）
- **一次任務取出4格**（統一批量處理）
- **需要檢查 AGV 至少有4格空位**（車上需全空）
- **不支持部分取料**（要麼取滿4格，要麼不取）

**效率提升**：
- 取消2格標準處理，統一為4格批量
- 減少任務次數，提升整體產能
- 簡化邏輯，降低復雜度

## 📝 TAFL Flow 設計

### Metadata
```yaml
metadata:
  id: "unloader_take_pre_dryer"
  name: "Unloader AGV 从预烘机取料"
  enabled: true
  version: "1.0.0"
  description: "检查预烘机完成预烘干的载具，创建 Unloader AGV 取料任务"
  author: "TAFL System"
  created_at: "2025-10-03"
  tags: ["unloader", "pre_dryer", "take", "automation"]
```

### Settings
```yaml
settings:
  execution_interval: 12  # 每12秒执行一次
```

### Variables
```yaml
variables:
  priority: 45        # 高优先级（启动后段制程）
  model: "UNLOADER"   # AGV 型号
  stations:           # 2个 Station 配置（Station-based，全部4格批量）
    - station: 1      # Station 01
      work_id: 2050101
      ports: [1, 2, 5, 6]
      batch_size: 4   # 批量4格
    - station: 3      # Station 03
      work_id: 2050301
      ports: [3, 4, 7, 8]
      batch_size: 4   # 批量4格
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
      # 处理该房间的预烘机
```

#### 3. 遍历每个 Station
```yaml
- for:
    in: "${stations}"
    as: station
    do:
      # 检查该 Station 的载具状态
```

#### 4. 查询 Station 的载具
```yaml
- query:
    target: carriers
    where:
      room_id: "${room.id}"
      equipment_type: "PRE_DRYER"
      port_in: "${station.ports}"    # Station-based Port 映射
      status_id: 503                 # 预烘乾机处理完成
    as: ready_carriers
    description: "查询 Station ${station.station} 预烘干完成的载具"
```

#### 5. 检查载具数量（批量处理）
```yaml
- set:
    carrier_count: "${ready_carriers.length}"
    required_count: "${station.batch_size}"   # 2格或4格
    has_enough_carriers: "${carrier_count >= required_count}"
```

#### 6. 查询 Unloader AGV 状态
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

#### 7. 检查 AGV 车上空位（批量处理）
```yaml
- query:
    target: carriers
    where:
      agv_id: "${agv.id}"
    as: agv_carriers
    description: "查询 AGV 车上载具"

- set:
    agv_carrier_count: "${agv_carriers.length}"
    max_capacity: 4                            # AGV 最大容量4格
    required_space: "${station.batch_size}"    # 需要的空位（2格或4格）
    available_space: "${max_capacity - agv_carrier_count}"
    agv_has_space: "${available_space >= required_space}"
```

#### 8. 检查重复任务
```yaml
- query:
    target: tasks
    where:
      work_id: "${station.work_id}"
      room_id: "${room.id}"
      status_id_in: [0, 1, 2, 3]  # 未完成的状态
    as: existing_tasks
    description: "检查是否已存在取料任务"
```

#### 9. 创建取料任务
```yaml
- if:
    condition: "${has_enough_carriers} && ${agv_has_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "unloader_take"
            name: "房间${room.id}预烘机 Station${station.station} 取料"
            description: "从预烘机 Station ${station.station} 取出${carrier_count}个载具（${station.batch_size}格批量）"
            work_id: "${station.work_id}"
            room_id: "${room.id}"
            priority: "${priority}"
            status_id: 1  # PENDING
            parameters:
              station: "${station.station}"
              work_id: "${station.work_id}"
              room_id: "${room.id}"
              ports: "${station.ports}"
              batch_size: "${station.batch_size}"
              model: "${model}"
              carrier_count: "${carrier_count}"
              reason: "预烘机完成预烘干，载具就绪"
          description: "创建 Unloader AGV 取料任务（Station-based）"
```

## 🔍 查询条件详解

### Carrier 查询条件（Station-based）

**必要条件**：
- `room_id`: 特定房间
- `equipment_type`: "PRE_DRYER"（预烘机）
- `port_in`: Station-based Port 映射（**全部4格批量**）
  - **Station 01**: [1, 2, 5, 6]（**批量4格**）
  - **Station 03**: [3, 4, 7, 8]（**批量4格**）

**状态条件**：
- `status_id: 503`（预烘乾机处理完成）
  - Loader AGV PUT_PRE_DRYER 完成后的状态
  - 表示载具已完成预烘干制程，可以被 Unloader AGV 取走

### AGV 查询条件

**Unloader AGV 条件**：
- `agv_type`: "unloader"
- `room_id`: 特定房间
- `status`: "idle"（空闲状态）

**车载空位计算**（**统一4格批量处理**）：
- 查询 AGV 车上现有载具数量
- **所有 Station 01/03**: 需要 ≥ 4格空位（**车上需全空**）
- 最大容量：4格（S尺寸产品）
- **不支持部分取料**：必须有完整4格空位才创建任务

### Task 重复检查

**防止重复创建**：
- 检查相同 `work_id` 和 `room_id`
- 状态为未完成（0=创建, 1=待分派, 2=执行中, 3=暂停）
- 如果存在未完成任务，不创建新任务

## ⚠️ 注意事项

### Station-based 设计重点
- **编码规则**: work_id 使用 Station 编号（**01/03**），非 Port 起始号
- **UnloaderAGV 自定义映射**: **所有 Station 统一批量4格**（UnloaderAGV 特有）
- **设计简化**: 从4个 Station 简化为2个，从混合批量简化为统一4格

### 批量处理逻辑
- **所有 Station 01/03**: 必须有 ≥ 4个载具，AGV 需 ≥ 4格空位（**车上全空**）
- **不支持部分取料**（要么取满4格，要么不取）
- **效率提升**: 统一批量处理，减少任务次数

### 状态码映射
Carrier status_id 定义：
- `503`: 预烘乾机处理完成（TAKE_PRE_DRYER 触发条件）
- Loader PUT_PRE_DRYER 完成后更新为此状态
- 表示载具已完成预烘干制程，可以被 Unloader AGV 取走

### Station-Port 映射（UnloaderAGV）
- **Station 01**: Port 1-2-5-6（**批量4格**，跨上下排）
- **Station 03**: Port 3-4-7-8（**批量4格**，跨上下排）

### 产品尺寸考量
- **S尺寸**: 最多4格容量，**全部使用批量4格处理**
- **L尺寸**: 最多2格容量，**不支持本 Flow**（需要完整4格空位）
- 本 Flow 专为 S尺寸产品设计

## 🧪 测试要点

### 单元测试
1. ✅ 查询逻辑正确性
   - 正确查询预烘干完成的载具
   - 正确过滤房间和端口

2. ✅ 数量判断
   - 载具数量 >= 2
   - AGV 空位 >= 2

3. ✅ 重复检查
   - 存在未完成任务时不创建
   - 任务参数正确传递

### 整合测试
1. ✅ 与 Loader AGV 衔接
   - Loader PUT_PRE_DRYER 完成
   - 状态更新为可取走

2. ✅ RCS 调度
   - 任务正确创建
   - RCS 分派给 Unloader AGV

3. ✅ 完整流程
   - Unloader 执行取料
   - 载具状态更新
   - 任务状态更新为完成

## 📊 成功指标

### 功能指标
- ✅ 正确检测预烘干完成的载具
- ✅ 正确创建取料任务
- ✅ 不产生重复任务
- ✅ 任务参数完整准确

### 性能指标
- 执行间隔：12秒
- 响应时间：< 2秒
- 任务创建延迟：< 30秒（从载具就绪到任务创建）

### 可靠性指标
- 零漏检：所有就绪载具都能被检测
- 零误创建：不创建重复或错误任务
- 容错性：数据库查询失败时优雅降级

## 🔗 相关文档

- **Unloader AGV 代码**: `/app/agv_ws/src/unloader_agv/unloader_agv/robot_states/take_pre_dryer/`
- **Work ID 定义**: 数据库 `agvc.work` 表
- **Cargo Flow 参考**: `cargo_entrance_unload.yaml`
