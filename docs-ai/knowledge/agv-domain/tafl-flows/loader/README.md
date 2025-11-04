# Loader AGV TAFL Flows

## 📋 概述

Loader AGV 負責 RosAGV 系統的**前段制程**，從入口傳送箱取出載具，經過清洗機、泡藥機、預烘機等多道工序，完成預處理後交給 Unloader AGV 進行後段烘幹制程。

## 🔄 完整制程流程

```
入口傳送箱（Cargo 卸載）
    ↓
┌─────────────────────────────────────┐
│   Loader AGV 前段制程（6個 Flow）    │
├─────────────────────────────────────┤
│ Flow 1: TAKE_BOXIN_TRANSFER        │ ← 取入口傳送箱
│ Flow 2: PUT_CLEANER                │ ← 放清洗機
│ Flow 3: TAKE_CLEANER               │ ← 取清洗機
│ Flow 4: PUT_SOAKER                 │ ← 放泡藥機（6台）
│ Flow 5: TAKE_SOAKER                │ ← 取泡藥機（6台）
│ Flow 6: PUT_PRE_DRYER              │ ← 放預烘機
└─────────────────────────────────────┘
    ↓
預烘機（預烘干完成）
    ↓
Unloader AGV（後段制程：預烘機 → 烤箱 → 出口）
```

## 🎯 6 個 TAFL Flows

### Flow 1: [取入口傳送箱](./01_take_boxin_transfer.md)
- **Work IDs**: 2010101, 2010301
- **Station 配置**: 2個 Station（01, 03），每個2格
- **觸發條件**: 入口傳送箱有載具，Loader AGV 有空位
- **執行結果**: 從入口傳送箱取出載具，啟動前段制程

### Flow 2: [放清洗機](./02_put_cleaner.md)
- **Work ID**: 2030302
- **Station 配置**: 1個 Station（03 下層），2格
- **觸發條件**: AGV 車上有載具，清洗機下層有空位
- **執行結果**: 放入清洗機下層進行清洗

### Flow 3: [取清洗機](./03_take_cleaner.md)
- **Work ID**: 2030101
- **Station 配置**: 1個 Station（01 上層），2格
- **觸發條件**: 清洗機上層有清洗完成載具
- **執行結果**: 從清洗機上層取出載具

### Flow 4: [放泡藥機](./04_put_soaker.md)
- **Work IDs**: 2040102, 2040202, 2040302, 2040402, 2040502, 2040602
- **Station 配置**: 6個 Station（01-06，泡藥機 A-F），每個1格
- **觸發條件**: AGV 車上有載具，泡藥機有空位
- **執行結果**: 放入泡藥機進行藥水浸泡處理（特殊設備）

### Flow 5: [取泡藥機](./05_take_soaker.md)
- **Work IDs**: 2040101, 2040201, 2040301, 2040401, 2040501, 2040601
- **Station 配置**: 6個 Station（01-06，泡藥機 A-F），每個1格
- **觸發條件**: 泡藥機有泡藥完成載具
- **執行結果**: 從泡藥機取出載具（特殊設備）

### Flow 6: [放預烘機](./06_put_pre_dryer.md)
- **Work IDs**: 2050102, 2050302, 2050502, 2050702
- **Station 配置**: 4個 Station（01, 03, 05, 07），每個2格
- **觸發條件**: AGV 車上有載具，預烘機有空位
- **執行結果**: 放入預烘機進行預烘干，銜接 Unloader AGV

## 🔧 技術規格總覽

### Work ID 系統（22個）

#### 入口傳送箱（2個）

| Work ID | 設備 | Station | Ports | 批量 | 操作 | 說明 |
|---------|------|---------|-------|------|------|------|
| 2010101 | 入口箱(201) | 01 | 1-2 | 2格 | TAKE | 取入口傳送箱 Station 01 |
| 2010301 | 入口箱(201) | 03 | 3-4 | 2格 | TAKE | 取入口傳送箱 Station 03 |

#### 清洗機（2個）

| Work ID | 設備 | Station | Ports | 批量 | 操作 | 說明 |
|---------|------|---------|-------|------|------|------|
| 2030101 | 清洗機(203) | 01 | 1-2 | 2格 | TAKE | 取清洗機上層 Station 01 |
| 2030302 | 清洗機(203) | 03 | 3-4 | 2格 | PUT | 放清洗機下層 Station 03 |

#### 泡藥機（12個 - 特殊設備）

**PUT 操作（6個）**：

| Work ID | 設備 | Station | Port | 批量 | 操作 | 說明 |
|---------|------|---------|------|------|------|------|
| 2040102 | 泡藥機(204) | 01 | 1 | 1格 | PUT | 放泡藥機 A (Station 01) |
| 2040202 | 泡藥機(204) | 02 | 2 | 1格 | PUT | 放泡藥機 B (Station 02) |
| 2040302 | 泡藥機(204) | 03 | 3 | 1格 | PUT | 放泡藥機 C (Station 03) |
| 2040402 | 泡藥機(204) | 04 | 4 | 1格 | PUT | 放泡藥機 D (Station 04) |
| 2040502 | 泡藥機(204) | 05 | 5 | 1格 | PUT | 放泡藥機 E (Station 05) |
| 2040602 | 泡藥機(204) | 06 | 6 | 1格 | PUT | 放泡藥機 F (Station 06) |

**TAKE 操作（6個）**：

| Work ID | 設備 | Station | Port | 批量 | 操作 | 說明 |
|---------|------|---------|------|------|------|------|
| 2040101 | 泡藥機(204) | 01 | 1 | 1格 | TAKE | 取泡藥機 A (Station 01) |
| 2040201 | 泡藥機(204) | 02 | 2 | 1格 | TAKE | 取泡藥機 B (Station 02) |
| 2040301 | 泡藥機(204) | 03 | 3 | 1格 | TAKE | 取泡藥機 C (Station 03) |
| 2040401 | 泡藥機(204) | 04 | 4 | 1格 | TAKE | 取泡藥機 D (Station 04) |
| 2040501 | 泡藥機(204) | 05 | 5 | 1格 | TAKE | 取泡藥機 E (Station 05) |
| 2040601 | 泡藥機(204) | 06 | 6 | 1格 | TAKE | 取泡藥機 F (Station 06) |

#### 預烘機（4個 - LoaderAGV 部分）

| Work ID | 設備 | Station | Ports | 批量 | 操作 | 說明 |
|---------|------|---------|-------|------|------|------|
| 2050102 | 預烘機(205) | 01 | 1-2 | 2格 | PUT | 放預烘機 Station 01 |
| 2050302 | 預烘機(205) | 03 | 3-4 | 2格 | PUT | 放預烘機 Station 03 |
| 2050502 | 預烘機(205) | 05 | 5-6 | 2格 | PUT | 放預烘機 Station 05 |
| 2050702 | 預烘機(205) | 07 | 7-8 | 2格 | PUT | 放預烘機 Station 07 |

**關鍵設計特點**：
- ✅ **共22個 Work ID**（2個入口箱 + 2個清洗機 + 12個泡藥機 + 4個預烘機 + 2個預烘機由UnloaderAGV使用）
- ✅ **標準設備2格批量**（入口箱、清洗機、預烘機）
- ✅ **特殊設備1格處理**（泡藥機：1 station = 1 port）
- ✅ **清洗機固定方向**（Station 03 下層進料 → Station 01 上排出料）

### Equipment Stations 配置

#### 標準設備（1 station = 2 ports）

定義在 `equipment_stations.py` 的 `STANDARD_EQUIPMENT`：

```python
STANDARD_EQUIPMENT = {
    201: {  # 入口傳送箱
        "name": "入口傳送箱",
        "stations": [1, 3],  # Station 01, 03
        "ports_per_station": 2,
    },
    203: {  # 清洗機
        "name": "清洗機",
        "stations": [1, 3],  # Station 01, 03
        "ports_per_station": 2,
    },
    205: {  # 預烘機
        "name": "預烘機",
        "stations": [1, 3, 5, 7],  # Station 01, 03, 05, 07
        "ports_per_station": 2,
    },
}
```

#### 特殊設備（1 station = 1 port）

定義在 `equipment_stations.py` 的 `SPECIAL_EQUIPMENT`：

```python
SPECIAL_EQUIPMENT = {
    204: {  # 泡藥機
        "name": "泡藥機",
        "stations": [1, 2, 3, 4, 5, 6],  # Station 01-06 (泡藥機 A-F)
        "ports_per_station": 1,
    },
}
```

### 清洗機固定方向設計

**固定單向流程**（消除雙向操作複雜性）：
- **Station 03（下層）**: 只支持 PUT 操作（進料）
- **清洗機內部**: 清洗制程（下層 → 上層）
- **Station 01（上層）**: 只支持 TAKE 操作（出料）

## 📊 Carrier 狀態轉換

```
201（入口箱）      ← Flow 1 TAKE_BOXIN_TRANSFER
    ↓
302（清洗中）       ← Flow 2 PUT_CLEANER
    ↓
303（清洗完成）     ← Flow 3 TAKE_CLEANER
    ↓
402（泡藥中）       ← Flow 4 PUT_SOAKER
    ↓
403（泡藥完成）     ← Flow 5 TAKE_SOAKER
    ↓
502（預烘中）       ← Flow 6 PUT_PRE_DRYER
    ↓
503（預烘完成）     ← Unloader TAKE_PRE_DRYER
```

## 🎯 設計特點

### 批量處理多樣性
- **標準設備 2格批量**: 入口傳送箱、清洗機、預烘機（1 station = 2 ports）
- **特殊設備 1格處理**: 泡藥機（1 station = 1 port）
- **靈活批量**: 根據設備特性優化處理效率

### 清洗機固定方向
- **Station 03 下層進料**: 只支持 PUT 操作
- **Station 01 上層出料**: 只支持 TAKE 操作
- **單向流程**: 消除雙向操作的複雜性

### 泡藥機特殊處理
- **6個獨立 Station**: 泡藥機 A-F（Station 01-06）
- **1 station = 1 port**: 每個泡藥機獨立操作
- **靈活分配**: 根據泡藥機狀態動態分配任務

### 系統銜接
- **前置銜接**: Cargo ENTRANCE_UNLOAD → 入口傳送箱
- **後續銜接**: Loader PUT_PRE_DRYER → Unloader TAKE_PRE_DRYER
- **完整循環**: Cargo → Loader → Unloader → Cargo

## 🔗 相關文檔

### AGV 領域知識
- [WCS Work ID 系統](../../wcs-workid-system.md) - Work ID 編碼規則和系統設計
- [WCS 資料庫設計](../../wcs-database-design.md) - 資料表結構和關係
- [WCS 系統設計](../../wcs-system-design.md) - WCS 整體架構

### TAFL 系統
- [TAFL 語言規範](../../../system/tafl/tafl-language-specification.md) - TAFL 語法和語義
- [TAFL 使用指南](../../../system/tafl/tafl-user-guide.md) - TAFL 開發指南
- [TAFL Editor 規範](../../../system/tafl/tafl-editor-specification.md) - TAFL 編輯器功能

### 實作代碼
- **Loader AGV 狀態機**: `/app/agv_ws/src/loader_agv/loader_agv/robot_states/`
  - `take_transfer/` - 取入口傳送箱流程
  - `put_cleaner/`, `take_cleaner/` - 清洗機流程
  - `put_soaker/`, `take_soaker/` - 泡藥機流程
  - `put_pre_dryer/` - 放預烘機流程
- **Equipment Stations**: `/app/shared_constants_ws/src/shared_constants/shared_constants/equipment_stations.py`
- **Work ID 初始化**: `/app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/13_works_tasks.py`

## 📅 版本歷史

- **2025-10-16**: 建立 Loader AGV TAFL Flows 設計文檔
- **未來計劃**:
  - YAML 實作和測試
  - 與 Cargo/Unloader 整合測試
  - 泡藥機多次浸泡邏輯優化

## 💡 實施指南

### TAFL Flow 開發順序
1. ✅ 設計階段：完成所有 Flow 設計文檔（本目錄）
2. ⏳ 實作階段：編寫 YAML 配置文件
3. ⏳ 測試階段：單元測試、整合測試、邊界測試
4. ⏳ 部署階段：載入到 TAFL WCS 系統

### 關鍵注意事項
- **批量處理**: 標準設備2格，泡藥機1格
- **清洗機方向**: Station 03 只 PUT，Station 01 只 TAKE
- **泡藥機遍歷**: 需遍歷6個 Station，獨立判斷空位
- **狀態碼驗證**: 確保 Carrier status_id 轉換正確
