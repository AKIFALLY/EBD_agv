# Path JSON 檔案目錄

📍 位置：`/home/ct/RosAGV/app/config/path/`

此目錄用於存放所有 LabVIEW 路徑相關的 JSON 檔案。

## 📁 檔案說明

### 測試檔案（用於開發測試）
- `test_add_nodes.json` - 新增節點測試檔案（節點 200-204）
- `test_modify_nodes.json` - 修改節點測試檔案（節點 1-6）
- `20250509_pathtest.json` - 刪除節點測試檔案

### 歷史路徑檔案
- `20250604_path.json` - 2025/06/04 路徑配置
- `20250616_path.json` - 2025/06/16 路徑配置
- `20250731Path.json` - 2025/07/31 路徑配置

### 地圖測試檔案 (KUKA測試地圖檔)
- `map_test_test1_ep4.json` - EP4 地圖測試配置
- `map_test_test1_jgc.json` - JGC 地圖測試配置

## 🚀 使用方式

### 在 AGVCUI 地圖頁面匯入

1. 訪問 http://localhost:8001/map
2. 點擊左側工具列的「路徑節點」按鈕
3. 點擊「匯入路徑檔」按鈕
4. 瀏覽至 `/home/ct/RosAGV/app/config/path/` 選擇 JSON 檔案
5. 查看差異預覽，選擇要執行的操作
6. 點擊「確認執行」

### LabVIEW JSON 格式

```json
[
  {
    "TagNo": 1,              // 節點 ID
    "Tag_X": 21674,         // X 座標 (mm)
    "Tag_Y": 11970,         // Y 座標 (mm)
    "Station": 2,           // 節點類型
    "CanToMoveSet": [       // 可移動連接列表
      {
        "CanToMoveTag": 11,
        "PGV": 0,
        "加權": 0,
        "Act": [12, 12, 12],
        "Speed": [],
        "SHIFT": [0, 0, 3],
        "Inposition": [],
        "SafeSensorSetting": []
      }
    ]
  }
]
```

## 📊 測試驗證

詳細測試指南請參考：
- `/home/ct/RosAGV/agents/README_test_files.md` - 測試檔案使用說明
- `/home/ct/RosAGV/agents/EDGE_FIX_VERIFICATION.md` - Edge 修復驗證指南

## ⚠️ 注意事項

1. **備份資料庫**：匯入前建議備份資料庫
   ```bash
   docker compose -f docker-compose.agvc.yml exec postgres pg_dump -U agvc -d agvc > backup-$(date +%Y%m%d-%H%M%S).sql
   ```

2. **Edge 處理邏輯**：
   - 只刪除和更新 LabVIEW 檔案中涉及節點的 Edge
   - 自動保留其他節點的 Edge
   - 不會無條件刪除所有 Edge（已修復）

3. **座標系統**：
   - Tag_X/Tag_Y：原始座標 (mm)
   - 系統會自動轉換為像素座標 (px)
   - 轉換比例：1 mm = 0.08 px
