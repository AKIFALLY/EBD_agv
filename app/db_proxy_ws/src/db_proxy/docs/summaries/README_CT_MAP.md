# CT 地圖自動匯入功能

## 概述

本功能將 CT 地圖匯入邏輯整合到 db_install 模組中，實現自動化地圖資料匯入。

## 功能特點

- ✅ 完全相容原始 `map_importer.py` 的 CT 地圖功能
- ✅ 自動從 `20250509_pathtest.json` 檔案讀取地圖資料
- ✅ 無縫整合到 db_install 初始化流程
- ✅ 強健的錯誤處理，不影響其他初始化功能
- ✅ 詳細的日誌記錄和進度顯示
- ✅ 支援資料更新和重複執行

## 檔案結構

```
db_proxy_ws/src/db_proxy/db_proxy/sql/init_data/
├── 19_ct_map.py              # CT 地圖匯入模組
├── 20250509_pathtest.json    # CT 地圖資料檔案
├── init_manager.py           # 初始化管理器（已修改）
├── test_ct_map_import.py     # 測試腳本
└── README_CT_MAP.md          # 本說明文件
```

## 使用方法

### 1. 正常執行 db_install

```bash
cd db_proxy_ws/src/db_proxy
python -m db_proxy.sql.db_install
```

CT 地圖匯入功能會自動在節點資料初始化階段執行。

### 2. 單獨測試地圖匯入功能

```bash
cd db_proxy_ws/src/db_proxy
python db_proxy/sql/test_ct_map_import.py
```

## 技術實作細節

### 核心邏輯

1. **檔案讀取**：使用相對路徑從 `init_data/20250509_pathtest.json` 讀取資料
2. **兩階段處理**：
   - 第一階段：處理所有節點，使用 TagNo 作為節點 ID
   - 第二階段：處理所有邊，從 CanToMoveSet 建立連接關係
3. **座標轉換**：使用 `ct_unit_2_px` 函數轉換 CT 單位到像素
4. **資料更新**：支援節點更新，邊則跳過已存在的項目

### 資料格式

#### 輸入格式 (20250509_pathtest.json)
```json
[
  {
    "TagNo": 2,
    "Tag_X": 17510,
    "Tag_Y": 11200,
    "Station": 5,
    "CanToMoveSet": [
      {
        "CanToMoveTag": 21,
        "PGV": 0,
        "加權": 0,
        ...
      }
    ]
  }
]
```

#### 座標轉換
- **CT 單位到像素**：`mm to px 12.5 mm => 1 px`
- **轉換函數**：`ct_unit_2_px(y, x) = y / 12.5, x / 12.5`

### 錯誤處理策略

- **檔案不存在**：記錄警告，跳過匯入，不中斷初始化流程
- **JSON 格式錯誤**：記錄錯誤，回滾變更，繼續其他初始化
- **資料庫錯誤**：回滾變更，記錄詳細錯誤資訊
- **節點處理錯誤**：跳過問題節點，繼續處理其他節點
- **邊已存在**：跳過已存在的邊，只處理新邊（適合重複執行）

### 日誌記錄

- 📖 檔案讀取狀態
- 📊 處理進度統計
- ✅ 成功匯入統計
- ⚠️  警告和跳過的項目
- ❌ 錯誤詳細資訊

## 資料庫表結構

### Node 表
- `id`: 節點 ID（使用 TagNo）
- `node_type_id`: 節點類型 ID（可選）
- `name`: 節點名稱（可選）
- `description`: 節點描述（可選）
- `x`, `y`: 轉換後的像素座標
- `created_at`, `updated_at`: 時間戳記

### Edge 表
- `id`: 邊 ID（自動生成）
- `from_id`, `to_id`: 起始和結束節點 ID
- `weight`: 邊權重（預設 1.0）
- `name`: 邊名稱（格式：起始節點-結束節點）
- `description`: 邊描述（可選）
- `created_at`, `updated_at`: 時間戳記

## 驗證和測試

### 自動驗證項目

1. **資料完整性**：檢查是否有孤立的邊
2. **節點 ID 範圍**：統計節點 ID 的最小值、最大值和數量
3. **邊連接統計**：統計邊的連接情況
4. **匯入統計**：顯示匯入的節點和邊數量

### 手動驗證步驟

1. **執行 db_install**：
   ```bash
   python -m db_proxy.sql.db_install
   ```

2. **檢查日誌輸出**：
   - 確認看到 "🗺️ 初始化 CT 地圖資料..." 訊息
   - 確認匯入統計資訊正確

3. **資料庫查詢驗證**：
   ```sql
   -- 檢查節點數量
   SELECT COUNT(*) FROM node;
   
   -- 檢查邊數量
   SELECT COUNT(*) FROM edge;
   
   -- 檢查節點 ID 範圍
   SELECT MIN(id), MAX(id), COUNT(*) FROM node;
   
   -- 檢查邊的連接情況
   SELECT COUNT(DISTINCT from_id), COUNT(DISTINCT to_id), COUNT(*) FROM edge;
   ```

## 故障排除

### 常見問題

1. **檔案找不到**
   - 確認 `20250509_pathtest.json` 存在於 `init_data` 目錄中
   - 檢查檔案權限

2. **JSON 格式錯誤**
   - 使用 JSON 驗證工具檢查檔案格式
   - 確認檔案編碼為 UTF-8

3. **資料庫連接問題**
   - 檢查資料庫連接設定
   - 確認 Node 和 Edge 表已建立

4. **匯入資料不完整**
   - 檢查日誌中的警告訊息
   - 使用測試腳本進行詳細檢查

### 重新匯入資料

如果需要重新匯入地圖資料：

1. 使用測試腳本的清除功能
2. 或手動清除資料：
   ```sql
   DELETE FROM edge;
   DELETE FROM node;
   ```
3. 重新執行 db_install

## 維護和更新

### 更新地圖資料

1. 替換 `20250509_pathtest.json` 檔案
2. 重新執行 db_install（會自動更新現有資料）

### 擴展功能

- 可以在 `19_ct_map.py` 中加入更多地圖處理邏輯
- 支援多種地圖格式
- 加入地圖驗證功能

## 相關檔案

- `web_api_ws/src/web_api/web_api/routers/map_importer.py` - 原始參考實作
- `db_proxy_ws/src/db_proxy/db_proxy/models/node.py` - Node 模型定義
- `db_proxy_ws/src/db_proxy/db_proxy/models/edge.py` - Edge 模型定義
- `db_proxy_ws/src/db_proxy/db_proxy/sql/db_install.py` - 主要安裝腳本
