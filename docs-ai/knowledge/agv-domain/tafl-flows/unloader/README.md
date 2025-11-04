# Unloader AGV TAFL Flows

## 📋 概述

Unloader AGV 負責 RosAGV 系統的**後段制程**，從預烘機取出載具，經過烤箱烘幹，最終放入出口傳送箱等待 Cargo AGV 收集。

## 🔄 完整制程流程

```
入口傳送箱（Cargo 卸載）
    ↓
Loader AGV（前段制程）
    ↓ 預烘干
預烘機（status: 503 預烘干完成）
    ↓
┌─────────────────────────────────┐
│   Unloader AGV 後段制程（4個 Flow） │
├─────────────────────────────────┤
│ Flow 1: TAKE_PRE_DRYER          │ ← 從預烘機取料
│ Flow 2: PUT_OVEN                │ ← 放入烤箱下排
│         (烤箱烘干制程)           │
│ Flow 3: TAKE_OVEN               │ ← 從烤箱上排取出
│ Flow 4: PUT_BOXOUT_TRANSFER     │ ← 放入出口傳送箱
└─────────────────────────────────┘
    ↓
Cargo AGV（出口裝載回 Rack）
```

## 🎯 4 個 TAFL Flows

### Flow 1: [取預烘機](./01_take_pre_dryer.md)
- **Work ID**: 2050101, 2050301
- **Station 配置**: 2個 Station（01, 03），全部批量4格
- **觸發條件**: 預烘機有完成預烘干的載具（status_id: 503）
- **執行結果**: 從預烘機 Station 01/03 批量取出4格載具

### Flow 2: [放入烤箱](./02_put_oven.md)
- **Work ID**: 2060502
- **Station 配置**: 1個 Station（05 下排進料），批量4格
- **觸發條件**: AGV 車上有載具，烤箱下排有空位
- **執行結果**: 放入烤箱 Station 05（下排進料，Port 5-8）

### Flow 3: [取烤箱](./03_take_oven.md)
- **Work ID**: 2060101
- **Station 配置**: 1個 Station（01 上排出料），批量4格
- **觸發條件**: 烤箱上排有烘干完成的載具（status_id: 603）
- **執行結果**: 從烤箱 Station 01（上排出料，Port 1-4）批量取出4格

### Flow 4: [放出口傳送箱](./04_put_boxout_transfer.md)
- **Work ID**: 2020102
- **Station 配置**: 1個 Station（01），批量4格
- **觸發條件**: AGV 車上有烘干完成的載具
- **執行結果**: 放入出口傳送箱 Station 01（Port 1-4）

## 🔧 技術規格總覽

### Work ID 系統（Station-based）

| Equipment | 設備類型 | Work ID | Station | Ports | 批量 | 操作 |
|-----------|---------|---------|---------|-------|------|------|
| 205 | 預烘機 | 2050101 | 01 | 1-2-5-6 | 4格 | TAKE |
| 205 | 預烘機 | 2050301 | 03 | 3-4-7-8 | 4格 | TAKE |
| 206 | 烤箱 | 2060101 | 01 | 1-2-3-4 | 4格 | TAKE（上排出料）|
| 206 | 烤箱 | 2060502 | 05 | 5-6-7-8 | 4格 | PUT（下排進料）|
| 202 | 出口傳送箱 | 2020102 | 01 | 1-2-3-4 | 4格 | PUT |

**關鍵設計特點**：
- ✅ **共5個 Work ID**（2個預烘 + 2個烤箱 + 1個出口箱）
- ✅ **全部4格批量處理**（統一批量，無混合2/4格邏輯）
- ✅ **每個 Work ID = 1個任務產生條件**
- ✅ **烤箱固定方向**（Station 05 下排進料 → Station 01 上排出料）

### UnloaderAGV 自定義 Station-Port 映射

定義在 `equipment_stations.py` 的 `UNLOADER_CUSTOM_MAPPING`：

```python
UNLOADER_CUSTOM_MAPPING = {
    202: {  # 出口傳送箱
        "mappings": {
            1: [1, 2, 3, 4],  # Station 01 → 4 Ports（批量4格）
        },
    },
    205: {  # 預烘機
        "mappings": {
            1: [1, 2, 5, 6],  # Station 01 → 4 Ports（批量4格，跨上下排）
            3: [3, 4, 7, 8],  # Station 03 → 4 Ports（批量4格，跨上下排）
        },
    },
    206: {  # 烤箱
        "mappings": {
            1: [1, 2, 3, 4],  # Station 01 → 4 Ports（上排/只拿/批量4格）
            5: [5, 6, 7, 8],  # Station 05 → 4 Ports（下排/只放/批量4格）
        },
    },
}
```

### 烤箱固定方向設計

**固定單向流程**（消除雙向操作複雜性）：
- **Station 05（下排）**: 只支持 PUT 操作（進料）
- **烤箱內部**: 烘干制程（下排 → 上排）
- **Station 01（上排）**: 只支持 TAKE 操作（出料）

## 📊 Carrier 狀態轉換

```
503（預烘干完成）← Flow 1 TAKE_PRE_DRYER
    ↓
602（烘干中）    ← Flow 2 PUT_OVEN
    ↓
603（烘干完成）  ← Flow 3 TAKE_OVEN
    ↓
202（在出口箱）  ← Flow 4 PUT_BOXOUT_TRANSFER
```

## 🎯 設計簡化成果

### 從複雜到簡單

**修正前**：
- 16 個 Work ID（4個預烘 + 8個烤箱 + 2個出口箱 + 2個入口箱）
- 混合批量處理（2格標準 + 4格批量）
- 烤箱雙向操作（上下排都可 PUT/TAKE）

**修正後**：
- ✅ **5 個 Work ID**（68.75% 減少）
- ✅ **統一4格批量**（100% 統一）
- ✅ **烤箱固定方向**（單向流程）

### 核心改進
1. **複雜度降低**: Work ID 從 16 個簡化為 5 個
2. **邏輯統一**: 全部使用4格批量處理
3. **流程清晰**: 烤箱固定單向流程（下進上出）
4. **維護簡化**: TAFL Flow 設計簡化，減少迭代邏輯

## 🔗 相關文檔

### AGV 領域知識
- [WCS Work ID 系統](../wcs-workid-system.md) - Work ID 編碼規則和系統設計
- [WCS 資料庫設計](../wcs-database-design.md) - 資料表結構和關係
- [WCS 系統設計](../wcs-system-design.md) - WCS 整體架構

### TAFL 系統
- [TAFL 語言規範](../../system/tafl/tafl-language-specification.md)
- [TAFL 使用指南](../../system/tafl/tafl-user-guide.md)
- [TAFL Editor 規範](../../system/tafl/tafl-editor-specification.md)

### 實作代碼
- **AGV 狀態機**: `/app/agv_ws/src/unloader_agv/unloader_agv/robot_states/`
- **Equipment Stations**: `/app/shared_constants_ws/src/shared_constants/shared_constants/equipment_stations.py`
- **Work ID 初始化**: `/app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/13_works_tasks.py`

## 📅 版本歷史

- **2025-10-16**: 第四次重大修正 - 統一4格批量處理，簡化為5個 Work ID
- **2025-10-03**: 第三次修正 - Station-based 設計完善
- **2025-09-19**: 初始設計 - 建立4個 TAFL Flows

## 💡 實施指南

### TAFL Flow 開發順序
1. ✅ 設計階段：完成所有 Flow 設計文檔（本目錄）
2. ⏳ 實作階段：編寫 YAML 配置文件
3. ⏳ 測試階段：單元測試、整合測試、邊界測試
4. ⏳ 部署階段：載入到 TAFL WCS 系統

### 關鍵注意事項
- **每個 Work ID = 1個任務產生條件**：確保 TAFL Flow 與 Work ID 一對一對應
- **統一批量處理**：所有操作使用4格批量，不支持部分操作
- **固定方向流程**：烤箱 Station 05 只 PUT，Station 01 只 TAKE
- **狀態碼驗證**：確保 Carrier status_id 轉換正確
