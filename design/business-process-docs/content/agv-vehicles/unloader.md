# Unloader AGV - 卸載車

## 🏭 Unloader AGV 概述

Unloader AGV 是 RosAGV 系統中專門負責後段卸料操作的自動導引車，主要處理預、烤箱、出口箱之間的卸料作業。採用與其他 AGV 一致的麥克納姆輪技術和機械臂系統，具備自動分揀和處理邏輯。

## ⚙️ 核心特性

### 基本規格
- **移動系統**: 麥克納姆輪全向移動
- **機械臂**: Robot 控制系統 (共用 agv_base.robot.Robot)
- **控制架構**: 3層狀態機 (Base-AGV-Robot)
- **光通訊**: Hokuyo 8bit 光通訊模組 (前方配置)
- **開發狀態**: 40% 實作完成，基礎架構完整

### 機械結構
```
Unloader AGV 結構組成
├── 移動底盤
│   ├── 麥克納姆輪系統 (全向移動)
│   ├── 3層狀態機控制
│   └── 精密定位系統
├── 機械臂系統
│   ├── Robot 控制器 (agv_base.robot.Robot)
│   ├── UnloaderRobotParameter 參數管理
│   ├── PLC 通訊介面
│   └── PGNO 指令系統
├── 工位管理系統
│   ├── Pre-dryer 端口管理 (8個端口)
│   ├── Oven 端口管理 (上下排各4格)
│   ├── Boxout Transfer 端口管理 (4格)
│   └── 分揀邏輯控制
└── 光通訊系統
    ├── Hokuyo 8bit 光通訊模組 (前方)
    └── PLC 資料通訊
```

## 🏭 工作流程

### 主要卸料操作

#### 1. Take/Put Pre Dryer - 預烘機操作
```python
# 預烘機的雙向卸料操作
Take Pre Dryer:
1. 檢查預烘機完成狀態
2. 從預烘機取出乾燥物料
3. 運送到下一個處理站

Put Pre Dryer:
1. 將待處理物料放入預烘機
2. 啟動預烘乾程序
3. 記錄處理開始時間
```

#### 2. Take/Put Oven - 烤箱操作
```python
# 烤箱的雙向卸料操作
Take Oven:
1. 等待烤箱完成信號
2. 從烤箱上排 (port 1-4) 取出烘乾完成的物料

Put Oven:
1. 將預烘乾完成的物料放入烤箱下排 (port 5-8)
2. 自動進行烘乾制程 (下排 → 上排)
3. 烘乾完成後載具自動移到上排

**烘乾制程流程**:
下排進料 (PUT_OVEN, port 5-8) → 烘乾制程 → 上排出料 (TAKE_OVEN, port 1-4)
```

#### 3. Take/Put Boxout Transfer - 出口傳送箱
```python
# 出口傳送箱的雙向操作
Take Boxout Transfer:
1. 從出口傳送箱取出待處理物料
2. 運送到指定工位

Put Boxout Transfer:
1. 將完成品放入出口傳送箱
2. 更新產品狀態
3. 通知下游系統
```

#### 4. 自動分揀功能
```python
# 自動分揀和處理邏輯
分揀標準:
- 產品類型識別
- 品質等級分類
- 目標工位分配
- 異常品處理
```

## 📋 Work ID 清單

### Unloader AGV 工作定義

#### 預烘機取料（批量 4 格操作）
- **2050101**: UnloaderAGV取預烘Station01 - 從預烘機 Station 01（Port 1-2-5-6）批量取料
- **2050301**: UnloaderAGV取預烘Station03 - 從預烘機 Station 03（Port 3-4-7-8）批量取料

#### 烤箱操作（批量 4 格操作）
**取料 (TAKE_OVEN - 上排出料位置)**:
- **2060101**: UnloaderAGV取烤箱Station01 - 從烤箱上排 Station 01（Port 1-2-3-4）批量取料

**放料 (PUT_OVEN - 下排進料位置)**:
- **2060502**: UnloaderAGV放烤箱Station05 - 放入烤箱下排 Station 05（Port 5-6-7-8）批量放料

#### 出口傳送箱（批量 4 格操作）
- **2020102**: UnloaderAGV放出口傳送箱Station01 - 放入出口箱 Station 01（Port 1-2-3-4）批量放料

### Work ID 編碼規則
格式：`room_id + equipment_type + port_start + action_type`

**設備類型編碼**:
- `02` = BOX_OUT_TRANSFER（出口傳送箱）
- `05` = PRE_DRYER（預烘機）
- `06` = OVEN（烤箱）

**Station 編碼**（UnloaderAGV 批量操作）:
- 預烘機:
  - `01` = Station 01（Port 1-2-5-6，批量 4 格）
  - `03` = Station 03（Port 3-4-7-8，批量 4 格）
- 烤箱:
  - `01` = Station 01（Port 1-2-3-4，上排，批量 4 格）
  - `05` = Station 05（Port 5-6-7-8，下排，批量 4 格）

**動作類型編碼**:
- `01` = TAKE（取料）
- `02` = PUT（放料）

**重要**: UnloaderAGV 所有操作都是 **4 個 port 為單位** 的批量處理模式

## 🔧 技術實作 (開發中)

### 狀態機架構
```python
# UnloaderContext (AGV 層狀態)
class UnloaderContext:
    # 繼承完整的 3層狀態機架構
    # 專門針對後段工位的卸料操作
    
# RobotContext (Robot 層狀態)
class UnloaderRobotContext:
    # 繼承通用 Robot 狀態機步驟
    IDLE = 0
    CHECK_IDLE = 1
    WRITE_CHG_PARA = 2
    WRITE_PGNO = 4
    CHECK_PGNO = 5
    ACTING = 6
    FINISH = 99
```

### 工位管理系統
```python
# Unloader 專用工位管理
class UnloaderWorkstationManager:
    pre_dryer_ports: dict       # 預烘機端口狀態
    oven_ports: dict           # 烤箱端口狀態
    boxout_ports: dict         # 出口箱端口狀態
    sorting_logic: object      # 分揀邏輯控制
```

### 分揀邏輯 (規劃中)
```python
# 自動分揀系統
class SortingSystem:
    def classify_product(self, product_data):
        # 產品分類邏輯
        pass
        
    def determine_destination(self, product_class):
        # 目標工位決策
        pass
        
    def handle_exception(self, exception_type):
        # 異常品處理
        pass
```

## 📊 開發狀態

### 已完成功能 (40%)
- ✅ **基礎架構**: 3層狀態機框架完整
- ✅ **移動系統**: 麥克納姆輪控制
- ✅ **機械臂整合**: Robot 控制系統整合
- ✅ **PLC 通訊**: 基礎通訊協議實作
- ✅ **測試框架**: 基礎測試環境

### 開發中功能 (60%)
- 🔄 **工位特化邏輯**: 預烘機、烤箱、出口箱專用邏輯
- 🔄 **分揀系統**: 自動分揀和品質分類
- 🔄 **狀態追蹤**: 完整的作業狀態管理
- 🔄 **錯誤處理**: 異常情況處理機制
- 🔄 **效能最佳化**: 操作速度和精度調優

## 🎯 設計目標

### 功能目標
- **高效卸料**: 快速且準確的卸料操作
- **自動分揀**: 基於品質和類型的自動分揀
- **狀態追蹤**: 完整的產品狀態追蹤
- **異常處理**: 自動異常品處理

### 技術目標
- **架構一致性**: 與其他 AGV 保持技術架構一致
- **模組化設計**: 易於擴展和維護
- **高可靠性**: 穩定的長時間運行
- **整合性**: 與整個 RosAGV 系統無縫整合

## 🔗 系統整合

### 與其他 AGV 的協作
```
生產流程協作
Loader AGV → 前段載料作業
     ↓
Cargo Mover → 中段搬運作業  
     ↓
Unloader AGV → 後段卸料作業
```

### ROS 2 整合 (規劃)
- **標準介面**: 統一的 ROS 2 服務介面
- **狀態同步**: 與 AGVC 管理系統的狀態同步
- **事件通知**: 重要事件的即時通知

### PLC 整合 (開發中)
- **PGNO 指令**: 標準化的機械臂控制
- **設備通訊**: 與預烘機、烤箱的通訊
- **狀態回報**: 即時的操作狀態回報

## 💡 未來發展

### 近期目標 (完成度 60% → 80%)
1. **完善工位邏輯**: 實作所有工位的專用邏輯
2. **分揀系統**: 實作基礎的分揀功能
3. **測試驗證**: 完整的功能測試

### 中期目標 (完成度 80% → 100%)
1. **分揀功能**: 進階的品質分類和分揀
2. **效能最佳化**: 操作速度和精度優化
3. **生產部署**: 實際生產環境部署

### 長期目標
1. **視覺整合**: 視覺系統輔助的品質檢測
2. **設備監控**: 設備狀態監控和維護排程
3. **柔性製造**: 支援多種產品的柔性生產

## 🛠️ 開發指導

### 當前開發重點
- **工位邏輯實作**: 專注於預烘機和烤箱的邏輯
- **狀態管理**: 完善的狀態追蹤和管理
- **測試覆蓋**: 擴展測試覆蓋範圍

### 技術債務
- **分揀邏輯**: 需要完整的分揀系統實作
- **異常處理**: 各種異常情況的處理機制
- **效能調優**: 操作速度和精度的最佳化

---

**相關文檔：**
- [AGV 車型介紹](vehicle-types.md) - 三種車型對比
- [Loader AGV](loader.md) - 前段載料作業
- [Cargo Mover AGV](cargo-mover.md) - 中段搬運作業