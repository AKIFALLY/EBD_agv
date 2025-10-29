# Loader AGV - 裝載車

## 🏗️ Loader AGV 概述

Loader AGV 是 RosAGV 系統中負責多工位自動載料操作的裝載車，具備完整的機械臂系統和精密的端口管理能力，專門處理傳送箱、清潔機、浸潤機、預乾燥機之間的載料作業。

## ⚙️ 核心特性

### 基本規格
- **移動系統**: 麥克納姆輪全向移動
- **機械臂**: Robot 控制系統 (共用 agv_base.robot.Robot)
- **控制架構**: 3層狀態機 (Base-AGV-Robot)
- **光通訊**: Hokuyo 8bit 光通訊模組 (前方配置)
- **開發狀態**: 基礎架構完成，業務邏輯持續優化中

### 機械結構
```
Loader AGV 結構組成
├── 移動底盤
│   ├── 麥克納姆輪系統 (全向移動)
│   ├── 精密定位系統
│   └── 3層狀態機控制
├── 機械臂系統
│   ├── Robot 控制器 (agv_base.robot.Robot)
│   ├── LoaderRobotParameter 參數管理
│   ├── PLC 通訊介面
│   └── PGNO 指令系統
├── 端口管理系統
│   ├── AGV 端口 (port1-4)
│   ├── 工位端口 (Transfer, Cleaner, Soaker, Pre-dryer)
│   ├── 端口狀態檢測
│   └── 動態端口分配
└── 光通訊系統
    ├── Hokuyo 8bit 光通訊模組 (前方)
    └── PLC 資料通訊
```

## 🏭 工作流程

### 主要載料操作

#### 1. Take Transfer - 傳送箱取料
```python
# 從傳送箱取料到 AGV 端口
操作流程:
1. AGV 移動到傳送箱位置
2. 檢測 AGV 可用端口
3. 機械臂執行取料操作
4. 將物料放置到 AGV 端口
5. 更新端口狀態
```

#### 2. Put/Take Cleaner - 清潔機載料
```python
# AGV 與清潔機間的雙向載料
Put Cleaner:
1. 從 AGV 端口取出待清潔物料
2. 放入清潔機指定端口
3. 啟動清潔程序

Take Cleaner:
1. 檢查清潔機完成狀態
2. 從清潔機取出清潔完成物料
3. 放入 AGV 端口準備後續處理
```

#### 3. Put/Take Soaker - 浸潤機載料
```python
# AGV 與浸潤機間的雙向載料
工位支援: Soaker port1-6 (6個端口)
操作特點:
- 浸潤時間較長，需要狀態追蹤
- 支援批量處理
- 動態端口分配
```

#### 4. Put Pre-dryer - 預乾燥機載料
```python
# AGV 到預乾燥機的單向載料
工位支援: Pre-dryer port1-8 (8個端口)
操作流程:
1. 從 AGV 端口取出浸潤完成物料
2. 放入預乾燥機端口
3. 啟動預乾燥程序
```

## 🎯 端口使用規則（Port Usage Rules for S-Size Carriers）

### 端口分工策略
Loader AGV 的 4 個端口依據 carrier 來源和目的地進行功能分區，確保物料流向清晰且不會混淆：

#### Port 1、3（前段流程端口）
- **用途**: 存放「準備進入泡藥機」的 carrier
- **來源 Carrier 類型**:
  - 從入口傳送箱取出（status_id: 101）
  - 從清洗機取出（status_id: 303 清洗完成）
- **目的地**: 送往泡藥機或清洗機
- **優先順序**: Port 1 → Port 3
- **容量**: 2 格

#### Port 2、4（後段流程端口）
- **用途**: 存放「泡藥完成」的 carrier
- **來源 Carrier 類型**:
  - 從泡藥機取出（status_id: 403 泡藥完成/強化機處理完成）
- **目的地**: 送往預烘機
- **優先順序**: Port 2 → Port 4
- **容量**: 2 格

### 完整流程鏈
```
入口傳送箱 → 清洗機 → 泡藥機 → 預烘機
     ↓         ↓        ↓       ↓
  Port 1/3  Port 1/3  Port 2/4  完成
```

### 端口配置詳細信息

#### Port ID 對應關係
| 物理端口 | eqp_port.id | 用途分類 | Carrier Status |
|---------|------------|---------|----------------|
| Port 1  | 2101       | 前段流程 | 101, 303       |
| Port 2  | 2102       | 後段流程 | 403            |
| Port 3  | 2103       | 前段流程 | 101, 303       |
| Port 4  | 2104       | 後段流程 | 403            |

#### Carrier Status ID 說明
| Status ID | 名稱 | 說明 | 可放入端口 |
|-----------|------|------|-----------|
| 101 | 進入入口傳送箱 | 從入口傳送箱取出的 carrier | Port 1, 3 |
| 303 | 清洗機處理完成 | 從清洗機取出，準備進入泡藥機 | Port 1, 3 |
| 403 | 強化機處理完成 | 從泡藥機取出，準備進入預烘機 | Port 2, 4 |

### 業務規則

#### 關鍵規則
1. **只有清洗完成的 carrier 才能放入泡藥機**
   - Put Soaker 必須從 Port 1 或 3 取出
   - 且 carrier 的 status_id 必須為 303

2. **泡藥完成的 carrier 只能放到 Port 2、4**
   - Take Soaker 必須檢查 Port 2 或 4 有空位
   - 取出的 carrier status_id 為 403

3. **前段流程 carrier 不能占用後段端口**
   - status_id: 101, 303 → 只能放 Port 1, 3
   - status_id: 403 → 只能放 Port 2, 4

#### 優先順序邏輯
- **取料優先級**:
  - Port 1 優先於 Port 3（前段流程）
  - Port 2 優先於 Port 4（後段流程）
- **放料優先級**:
  - Port 1 優先於 Port 3（前段流程）
  - Port 2 優先於 Port 4（後段流程）

### 執行層級

#### TAFL 流程層（已實作）
- 任務生成時檢查端口使用規則
- 確保 carrier 分配符合端口分工
- 實現優先順序邏輯

#### AGV 狀態機層（待實作）
- 執行任務時二次驗證端口規則
- 防止錯誤的端口使用
- 提供異常處理機制

### 未來規劃

#### L 尺寸 Carrier 支援
目前端口使用規則僅針對 S 尺寸 carrier 設計。L 尺寸 carrier 的端口規劃待後續補充，可能包括：
- 不同的端口分配策略
- 容量調整（L 尺寸可能占用更多空間）
- 流程適配調整

## 🔧 技術實作

### 狀態機架構
```python
# LoaderContext (AGV 層狀態)
class LoaderContext:
    # AGV 層業務邏輯
    # 工位協調和路徑規劃
    
# RobotContext (Robot 層狀態)  
class RobotContext(BaseContext):
    IDLE = 0
    CHECK_IDLE = 1
    WRITE_CHG_PARA = 2
    WRITE_PGNO = 4
    CHECK_PGNO = 5
    ACTING = 6
    FINISH = 99
```

### 端口管理系統
```python
# 端口狀態管理
class LoaderPortManager:
    # AGV 端口狀態
    agv_port1: bool             # AGV Port 1 狀態
    agv_port2: bool             # AGV Port 2 狀態
    agv_port3: bool             # AGV Port 3 狀態
    agv_port4: bool             # AGV Port 4 狀態
    
    # 工位端口狀態
    boxin_port1_4: bool         # Transfer 端口狀態
    cleaner_port1_4: bool       # Cleaner 端口狀態
    soaker_port1_6: bool        # Soaker 端口狀態
    pre_dryer_port1_8: bool     # Pre-dryer 端口狀態
```

### 機械臂控制
```python
# 實際機械臂控制實作
class RobotContext(BaseContext):
    def __init__(self, initial_state):
        # Robot 初始化
        self.robot_parameter = LoaderRobotParameter()
        self.robot = Robot(self.node, self.robot_parameter)
    
    def update_port_parameters(self):
        # 動態端口參數更新
        self.robot_parameter.loader_agv_port_front = self.get_loader_agv_port_front
        self.robot_parameter.boxin_port = self.get_boxin_port
        self.robot_parameter.cleaner_port = self.get_cleaner_port
        # 同步更新參數到 PLC
        self.robot_parameter.calculate_parameter()
        self.robot.update_parameter()
```

## 📊 開發狀態

### 已完成功能 (70%)
- ✅ **基礎架構**: 3層狀態機框架完整實作
- ✅ **移動系統**: 麥克納姆輪控制系統
- ✅ **機械臂整合**: Robot 控制系統整合完成
- ✅ **端口管理**: 4個AGV端口的動態管理
- ✅ **測試框架**: 完整的測試套件架構
- ✅ **PLC 通訊**: PGNO 指令系統實作

### 開發中功能 (30%)
- 🔄 **工位邏輯優化**: 持續優化各工位操作效率
- 🔄 **視覺定位精度**: 提升視覺引導精確度
- 🔄 **錯誤恢復機制**: 完善異常處理流程
- 🔄 **效能調優**: 操作速度和精度的最佳化

### 技術優勢
- **LoaderRobotParameter**: 靈活的參數配置系統
- **動態調整**: 根據實際情況調整操作參數
- **多工位支援**: Transfer、Cleaner、Soaker、Pre-dryer

## 🔗 系統整合

### ROS 2 架構
- **Node 管理**: 標準化的 ROS 2 節點架構
- **Service 介面**: 統一的服務呼叫介面
- **Topic 通訊**: 狀態和資料的即時同步

### PLC 整合
- **PGNO 指令**: 標準化的機械臂控制指令
- **狀態回報**: 即時的操作狀態回報
- **錯誤處理**: 完善的異常處理機制

## 💡 部署指導

### 配置要求
```yaml
# Loader AGV 配置範例
loader_config:
  agv_ports: 4              # AGV 端口數量
  work_stations:
    transfer: 4             # Transfer 端口數
    cleaner: 4              # Cleaner 端口數  
    soaker: 6               # Soaker 端口數
    pre_dryer: 8            # Pre-dryer 端口數
```

### 維護重點
- **端口校正**: 定期校正所有端口位置
- **參數調優**: 根據實際負載調整參數
- **狀態監控**: 實時監控各工位狀態
- **測試驗證**: 定期執行完整測試套件

## 🎯 應用優勢

### 生產效率
- **任務調度**: 最佳化工位利用率
- **並行作業**: 減少等待時間
- **狀態追蹤**: 完整的作業狀態可視化

### 系統可靠性
- **穩定架構**: 基礎架構完整且穩定
- **測試保障**: 完整的測試套件支援
- **持續改進**: 業務邏輯持續優化中

---

**相關文檔：**
- [AGV 車型介紹](vehicle-types.md) - 三種車型對比
- [Unloader AGV](unloader.md) - 後段卸載作業
- [系統架構](../system-architecture/dual-environment.md) - 雙環境設計