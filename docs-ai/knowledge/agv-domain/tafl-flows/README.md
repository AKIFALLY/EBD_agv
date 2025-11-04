# TAFL Flows - AGV 自動化流程設計

## 📋 概述

TAFL (Task Automation Flow Language) Flows 是 RosAGV 系統中使用 TAFL 語言定義的 AGV 自動化任務流程。本目錄包含所有 AGV 類型的 TAFL Flow 設計文檔。

## 🚗 AGV 類型與職責

RosAGV 系統包含 3 種 AGV，各自負責不同的制程階段：

### 1. [Cargo AGV](./cargo/)
**職責**: 物料搬運（Rack ↔ 傳送箱）
- 從 Rack 取出載具，卸載到入口傳送箱
- 從出口傳送箱取出載具，裝載回 Rack
- **狀態**: ✅ 設計完成（2個 Work ID，2個 TAFL Flows）

### 2. [Loader AGV](./loader/)
**職責**: 前段制程（入口 → 預烘機）
- 從入口傳送箱取出載具
- 經過清洗機、泡藥機（可多次浸泡）制程
- 放入預烘機進行預烘幹
- **狀態**: ✅ 設計完成（22個 Work ID，6個 TAFL Flows）

### 3. [Unloader AGV](./unloader/)
**職責**: 後段制程（預烘機 → 出口）
- 從預烘機取出預烘幹完成的載具
- 經過烤箱烘幹制程
- 放入出口傳送箱等待 Cargo 收集
- **狀態**: ✅ 設計完成（5個 Work ID，4個 TAFL Flows）

## 🔄 完整制程流程

```
┌──────────────────────────────────────────────────────────┐
│                    RosAGV 完整制程                         │
├──────────────────────────────────────────────────────────┤
│                                                            │
│  Rack（貨架儲存）                                          │
│    ↓                                                       │
│  [Cargo AGV] 卸載                                         │
│    ↓                                                       │
│  入口傳送箱（BOX_IN_TRANSFER）                            │
│    ↓                                                       │
│  [Loader AGV] 前段制程 ✅                                 │
│    ├─ TAKE_BOXIN_TRANSFER: 取入口箱                       │
│    ├─ PUT_CLEANER: 放清洗機                               │
│    ├─ TAKE_CLEANER: 取清洗機                              │
│    ├─ PUT_SOAKER: 放泡藥機（可多次浸泡）                  │
│    ├─ TAKE_SOAKER: 取泡藥機                               │
│    └─ PUT_PRE_DRYER: 放預烘機                             │
│    ↓                                                       │
│  預烘機（PRE_DRYER）- 預烘干                              │
│    ↓                                                       │
│  [Unloader AGV] 後段制程 ✅                               │
│    ├─ TAKE_PRE_DRYER: 取預烘機                           │
│    ├─ PUT_OVEN: 放烤箱下排                               │
│    │   (烤箱烘干制程)                                     │
│    ├─ TAKE_OVEN: 取烤箱上排                              │
│    └─ PUT_BOXOUT_TRANSFER: 放出口箱                      │
│    ↓                                                       │
│  出口傳送箱（BOX_OUT_TRANSFER）                          │
│    ↓                                                       │
│  [Cargo AGV] 裝載                                         │
│    ↓                                                       │
│  Rack（貨架儲存）                                          │
│                                                            │
└──────────────────────────────────────────────────────────┘
```

## 📂 目錄結構

```
tafl-flows/
├── README.md                      # 本文件 - TAFL Flows 總覽
├── cargo/                         # ✅ Cargo AGV Flows
│   ├── README.md                  # Cargo 總覽
│   ├── 01_entrance_unload.md      # Flow 1: 入口卸載（Rack → 入口箱）
│   └── 02_exit_load.md            # Flow 2: 出口裝載（出口箱 → Rack）
├── loader/                        # ✅ Loader AGV Flows
│   ├── README.md                  # Loader 總覽
│   ├── 01_take_boxin_transfer.md  # Flow 1: 取入口傳送箱
│   ├── 02_put_cleaner.md          # Flow 2: 放清洗機
│   ├── 03_take_cleaner.md         # Flow 3: 取清洗機
│   ├── 04_put_soaker.md           # Flow 4: 放泡藥機（特殊設備）
│   ├── 05_take_soaker.md          # Flow 5: 取泡藥機（特殊設備）
│   └── 06_put_pre_dryer.md        # Flow 6: 放預烘機（銜接 Unloader）
└── unloader/                      # ✅ Unloader AGV Flows
    ├── README.md                  # Unloader 總覽
    ├── 01_take_pre_dryer.md       # Flow 1: 取預烘機
    ├── 02_put_oven.md             # Flow 2: 放烤箱
    ├── 03_take_oven.md            # Flow 3: 取烤箱
    └── 04_put_boxout_transfer.md  # Flow 4: 放出口傳送箱
```

## 🎯 Work ID 系統總覽

### Station-based 編碼規則

Work ID 格式：`room_id + equipment_type + station + action_type`

**範例**: `2050101`
- `2`: room_id (房間2)
- `05`: equipment_type (預烘機)
- `01`: station (Station 01)
- `01`: action_type (TAKE)

### 已實現的 Work IDs

#### Cargo AGV（2個）✅

| Work ID | 起點 | 終點 | KUKA API | 批量 | 操作 | 說明 |
|---------|------|------|----------|------|------|------|
| 2000102 | Rack | 入口箱(201) | rack_move | 4格 | 卸載 | Cargo入口卸載（啟動制程）|
| 2000201 | 出口箱(202) | Rack | rack_move | 4格 | 裝載 | Cargo出口裝載（完成循環）|

#### Unloader AGV（5個）✅

| Work ID | 設備 | Station | Ports | 批量 | 操作 | 說明 |
|---------|------|---------|-------|------|------|------|
| 2050101 | 預烘機(205) | 01 | 1-2-5-6 | 4格 | TAKE | 取預烘機 Station 01 |
| 2050301 | 預烘機(205) | 03 | 3-4-7-8 | 4格 | TAKE | 取預烘機 Station 03 |
| 2060101 | 烤箱(206) | 01 | 1-2-3-4 | 4格 | TAKE | 取烤箱上排 Station 01 |
| 2060502 | 烤箱(206) | 05 | 5-6-7-8 | 4格 | PUT | 放烤箱下排 Station 05 |
| 2020102 | 出口箱(202) | 01 | 1-2-3-4 | 4格 | PUT | 放出口傳送箱 Station 01 |

#### Loader AGV（22個）✅

| Work ID | 設備 | Station | Ports | 批量 | 操作 | 說明 |
|---------|------|---------|-------|------|------|------|
| **入口傳送箱（2個）** |
| 2010101 | 入口箱(201) | 01 | 1-2 | 標準 | TAKE | 取入口箱 Station 01 |
| 2010301 | 入口箱(201) | 03 | 3-4 | 標準 | TAKE | 取入口箱 Station 03 |
| **清洗機（2個）** |
| 2030101 | 清洗機(203) | 01 | 1-2 | 標準 | TAKE | 取清洗機上層 Station 01 |
| 2030302 | 清洗機(203) | 03 | 3-4 | 標準 | PUT | 放清洗機下層 Station 03 |
| **泡藥機（12個）- 特殊設備** |
| 2040101 | 泡藥機(204) | 01 | 1 | 1格 | TAKE | 取泡藥機A |
| 2040102 | 泡藥機(204) | 01 | 1 | 1格 | PUT | 放泡藥機A |
| 2040201 | 泡藥機(204) | 02 | 2 | 1格 | TAKE | 取泡藥機B |
| 2040202 | 泡藥機(204) | 02 | 2 | 1格 | PUT | 放泡藥機B |
| 2040301 | 泡藥機(204) | 03 | 3 | 1格 | TAKE | 取泡藥機C |
| 2040302 | 泡藥機(204) | 03 | 3 | 1格 | PUT | 放泡藥機C |
| 2040401 | 泡藥機(204) | 04 | 4 | 1格 | TAKE | 取泡藥機D |
| 2040402 | 泡藥機(204) | 04 | 4 | 1格 | PUT | 放泡藥機D |
| 2040501 | 泡藥機(204) | 05 | 5 | 1格 | TAKE | 取泡藥機E |
| 2040502 | 泡藥機(204) | 05 | 5 | 1格 | PUT | 放泡藥機E |
| 2040601 | 泡藥機(204) | 06 | 6 | 1格 | TAKE | 取泡藥機F |
| 2040602 | 泡藥機(204) | 06 | 6 | 1格 | PUT | 放泡藥機F |
| **預烘機（4個）** |
| 2050102 | 預烘機(205) | 01 | 1-2 | 標準 | PUT | 放預烘機 Station 01 |
| 2050302 | 預烘機(205) | 03 | 3-4 | 標準 | PUT | 放預烘機 Station 03 |
| 2050502 | 預烘機(205) | 05 | 5-6 | 標準 | PUT | 放預烘機 Station 05 |
| 2050702 | 預烘機(205) | 07 | 7-8 | 標準 | PUT | 放預烘機 Station 07 |

## 🔧 核心設計原則

### 1. 批量處理多樣性
- ✅ **UnloaderAGV**: 統一4格批量（高效處理）
- ✅ **LoaderAGV**: 混合批量處理
  - 標準設備：1 station = 2 ports（入口箱、清洗機、預烘機）
  - 特殊設備：1 station = 1 port（泡藥機，6個獨立 Station）
- ✅ **CargoAGV**: 統一4格批量（KUKA Rack 搬運）
- 🎯 **目標**：根據設備特性和製程需求優化批量大小

### 2. Station-based 架構
- ✅ **每個 Work ID = 1個 Station**：一對一映射
- ✅ **每個 Work ID = 1個任務產生條件**：TAFL Flow 簡化
- ✅ **標準設備 vs 特殊設備**：靈活支援不同設備類型
- 🎯 **目標**：清晰的任務定義，易於追蹤和除錯

### 3. 固定方向流程
- ✅ **烤箱（UnloaderAGV）**：
  - Station 05（下排）：只支持 PUT 操作（進料）
  - Station 01（上排）：只支持 TAKE 操作（出料）
- ✅ **清洗機（LoaderAGV）**：
  - Station 03（下層）：只支持 PUT 操作（進料）
  - Station 01（上層）：只支持 TAKE 操作（出料）
- 🎯 **目標**：消除雙向操作複雜性，固定單向流程

### 4. AGV 自定義映射
- ✅ **UnloaderAGV**: 自定義 Station-Port 映射（跨上下排批量4格）
- ✅ **LoaderAGV**: 標準設備 + 特殊設備混合映射（1格精密操作）
- ✅ **CargoAGV**: KUKA Rack-based 操作（統一4格批量）
- 🎯 **目標**：每種 AGV 根據作業特性優化 Station 配置

### 5. 特殊設備支援
- ✅ **泡藥機（Equipment 204）**: 6個獨立 Station，1 station = 1 port
- ✅ **支援多次浸泡**: 製程1（泡1次）vs 製程2（泡2次）
- ✅ **EquipmentStations 模組**: 統一管理標準與特殊設備配置
- 🎯 **目標**：靈活支援不同設備類型和製程需求

## 📊 TAFL Flow 開發流程

### 1. 設計階段
- [ ] 分析業務需求和制程流程
- [ ] 定義 Work ID 和 Station 配置
- [ ] 編寫 Flow 設計文檔（本目錄）
- [ ] 技術評審和驗證

### 2. 實作階段
- [ ] 編寫 TAFL YAML 配置文件
- [ ] 實作 AGV 狀態機邏輯
- [ ] 更新資料庫初始化腳本

### 3. 測試階段
- [ ] 單元測試：查詢邏輯、數量判斷、重複檢查
- [ ] 整合測試：Flow 間衔接、RCS 調度
- [ ] 邊界測試：異常情況處理

### 4. 部署階段
- [ ] 載入到 TAFL WCS 系統
- [ ] 生產環境驗證
- [ ] 監控和優化

## 🔗 相關文檔

### AGV 領域知識
- [WCS Work ID 系統](../wcs-workid-system.md) - Work ID 編碼規則和系統設計
- [WCS 資料庫設計](../wcs-database-design.md) - 資料表結構和關係
- [WCS 系統設計](../wcs-system-design.md) - WCS 整體架構

### TAFL 系統
- [TAFL 語言規範](../../system/tafl/tafl-language-specification.md) - TAFL 語法和語義
- [TAFL 使用指南](../../system/tafl/tafl-user-guide.md) - TAFL 開發指南
- [TAFL Editor 規範](../../system/tafl/tafl-editor-specification.md) - TAFL 編輯器功能

### 實作代碼
- **AGV 狀態機**: `/app/agv_ws/src/*/robot_states/`
- **Equipment Stations**: `/app/shared_constants_ws/src/shared_constants/shared_constants/equipment_stations.py`
- **Work ID 初始化**: `/app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/13_works_tasks.py`
- **TAFL WCS**: `/app/tafl_wcs_ws/`

## 📈 實施進度

| AGV 類型 | Work IDs | Flows | 設計完成 | YAML 實作 | 測試完成 | 生產部署 |
|---------|----------|-------|---------|----------|---------|---------|
| Cargo    | 2個 | 2個 | ✅ | ⏳ | ⏳ | ⏳ |
| Loader   | 22個 | 6個 | ✅ | ⏳ | ⏳ | ⏳ |
| Unloader | 5個 | 4個 | ✅ | ⏳ | ⏳ | ⏳ |
| **總計** | **29個** | **12個** | **✅** | **⏳** | **⏳** | **⏳** |

## 📅 版本歷史

- **2025-10-16**:
  - 建立 TAFL Flows 目錄結構
  - 完成 Unloader AGV 設計文檔（5個 Work ID，4個 Flow）
  - 完成 Cargo AGV 設計文檔（2個 Work ID，2個 Flow）
  - 完成 Loader AGV 設計文檔（22個 Work ID，6個 Flow）
  - **設計階段完成**：3種 AGV，共 29個 Work ID，12個 TAFL Flow
- **未來計劃**:
  - YAML 實作（/app/config/tafl/flows/）
  - 整合測試和驗證
  - 生產環境部署

## 💡 貢獻指南

### 新增 AGV TAFL Flow
1. 在對應的 AGV 目錄創建 Flow 設計文檔
2. 更新該 AGV 的 README.md
3. 更新本總覽文檔的 Work ID 表格
4. 提交 Pull Request 並進行技術評審

### 修改現有 Flow
1. 更新對應的 Flow 設計文檔
2. 記錄版本歷史和變更原因
3. 驗證與其他 Flow 的衔接
4. 更新測試案例

### 文檔規範
- 使用清晰的標題結構（H1-H6）
- 提供完整的技術規格和範例
- 包含測試要點和成功指標
- 交叉引用相關文檔
