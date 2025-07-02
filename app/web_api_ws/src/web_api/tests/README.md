# Web API 測試工具

本目錄包含 Web API 專案的測試工具和腳本。

## 測試檔案

### Kuka API 測試

- `test_kuka_api.py` - Kuka API 功能測試腳本
- `create_test_task.py` - 創建測試任務的工具腳本
- `quick_test.py` - 快速測試腳本（用於驗證修正）

## 使用方法

### 1. 創建測試任務

在測試 Kuka API 之前，需要先創建測試任務：

```bash
cd /app/web_api_ws/src/web_api/tests
python create_test_task.py
```

這個腳本會：
- 創建主要測試任務（mission_code: "mission202309250005"）
- 可選擇創建額外的測試任務
- 列出所有現有的測試任務

### 2. 測試 Kuka API

確保 web_api 伺服器正在運行，然後執行：

```bash
python test_kuka_api.py
```

這個腳本會：
- 測試基本的 missionStateCallback API
- 可選擇測試所有任務狀態
- 顯示 API 回應結果

## 測試前準備

1. **啟動資料庫服務**
   ```bash
   # 確保 PostgreSQL 資料庫正在運行
   ```

2. **啟動 web_api 伺服器**
   ```bash
   cd /app/web_api_ws
   source install/setup.bash
   ros2 run web_api api_server
   ```

3. **創建測試任務**
   ```bash
   python tests/create_test_task.py
   ```

4. **執行測試**
   ```bash
   # 快速測試（推薦）
   python tests/quick_test.py

   # 完整測試
   python tests/test_kuka_api.py
   ```

## 測試資料

測試腳本會創建以下測試任務：

| Mission Code | 任務名稱 | 說明 |
|--------------|----------|------|
| mission202309250005 | Kuka API 測試任務 - mission202309250005 | 主要測試任務 |
| 1357 | Kuka API 測試任務 - 1357 | 快速測試任務 |
| test_mission_001 | Kuka API 測試任務 - test_mission_001 | 額外測試任務 1 |
| test_mission_002 | Kuka API 測試任務 - test_mission_002 | 額外測試任務 2 |
| test_mission_003 | Kuka API 測試任務 - test_mission_003 | 額外測試任務 3 |

## 故障排除

### 常見問題

1. **連接錯誤**: 確保 web_api 伺服器正在運行在 localhost:8000
2. **任務不存在**: 先執行 `create_test_task.py` 創建測試任務
3. **資料庫錯誤**: 確保資料庫連接正常且包含必要的基礎資料

### 檢查清單

- [ ] PostgreSQL 資料庫正在運行
- [ ] web_api 伺服器已啟動
- [ ] 測試任務已創建
- [ ] 網路連接正常
