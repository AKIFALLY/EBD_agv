# Loader AGV - 裝載車

## 🏗️ Loader AGV 概述

Loader AGV 是 RosAGV 系統中負責多工位自動載料操作的智能裝載車，具備完整的機械臂系統和精密的端口管理能力，專門處理傳送箱、清潔機、浸潤機、預乾燥機之間的載料作業。

## ⚙️ 核心特性

### 基本規格
- **移動系統**: 麥克納姆輪全向移動
- **機械臂**: Robot 控制系統 (共用 agv_base.robot.Robot)
- **端口管理**: 動態端口分配 (AGV port1-4)
- **控制架構**: 3層狀態機 (Base-AGV-Robot)
- **開發狀態**: 100% 實作完成，具備完整測試套件

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
- 智能端口分配
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
# 智能端口狀態管理
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

## 📊 效能優勢

### 多工位支援
- **4種工位**: Transfer、Cleaner、Soaker、Pre-dryer
- **智能調度**: 根據工位狀態動態分配任務
- **並行處理**: 多個端口同時作業

### 參數化控制
- **LoaderRobotParameter**: 靈活的參數配置系統
- **動態調整**: 根據實際情況調整操作參數
- **視覺引導**: 精確的定位和載料操作

### 完整測試覆蓋
- **單元測試**: 完整的測試套件
- **整合測試**: 跨模組功能測試
- **生產驗證**: 實際生產環境驗證

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
- **智能調度**: 最佳化工位利用率
- **並行作業**: 減少等待時間
- **狀態追蹤**: 完整的作業狀態可視化

### 系統可靠性
- **完整實作**: 100% 開發完成度
- **測試保障**: 完整的測試覆蓋
- **錯誤恢復**: 智能的錯誤處理和恢復

---

**相關文檔：**
- [AGV 車型介紹](vehicle-types.md) - 三種車型對比
- [Unloader AGV](unloader.md) - 後段卸載作業
- [系統架構](../system-architecture/dual-environment.md) - 雙環境設計