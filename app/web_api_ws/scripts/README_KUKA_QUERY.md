# KUKA Container 查詢工具

## 功能說明

`query_kuka_containers.py` 是用於查詢 KUKA Fleet Manager 中容器（Container）資訊的命令列工具。

## 使用方式

### 基本用法

```bash
# [容器內] 必須在 AGVC 容器內執行
cd /app/web_api_ws/scripts
python3 query_kuka_containers.py
```

### 完整使用範例

```bash
# 列出所有容器（表格格式）
python3 query_kuka_containers.py

# 只顯示在地圖中的容器
python3 query_kuka_containers.py --status in

# 只顯示不在地圖中的容器
python3 query_kuka_containers.py --status out

# JSON 格式輸出
python3 query_kuka_containers.py --json

# 簡單列表（僅容器編號）
python3 query_kuka_containers.py --simple

# 查詢特定容器
python3 query_kuka_containers.py --code RACK001

# 查詢特定位置的容器
python3 query_kuka_containers.py --position node-uuid-123

# 組合過濾條件
python3 query_kuka_containers.py --status in --json
```

## 輸出格式

### 表格格式（預設）
```
Container Code  Status    Position
-----------------------------------------
RACK001         🟢 In Map  node-uuid-123
RACK002         🟢 In Map  node-uuid-456
RACK003         ⚪ Out     N/A
-----------------------------------------
總計: 3 個容器
```

### JSON 格式
```json
{
  "timestamp": "2025-01-10T10:30:00",
  "total": 3,
  "containers": [
    {
      "containerCode": "RACK001",
      "position": "node-uuid-123",
      "status": "in"
    }
  ]
}
```

### 簡單列表
```
RACK001
RACK002
RACK003
```

## 參數說明

| 參數 | 說明 |
|------|------|
| `--status in` | 過濾在地圖中的容器 |
| `--status out` | 過濾不在地圖中的容器 |
| `--position UUID` | 過濾特定位置的容器 |
| `--code CODE` | 查詢特定容器編號 |
| `--json` | JSON 格式輸出 |
| `--simple` | 簡單列表輸出（僅容器編號） |
| `--no-login` | 不自動登入 KUKA Fleet |

## 前置條件

1. **環境要求**：必須在 AGVC 容器內執行
2. **工作空間建置**：kuka_fleet_ws 已建置
3. **網路連接**：能夠連接到 KUKA Fleet Manager

## 執行流程

### 宿主機執行
```bash
# 從宿主機進入 AGVC 容器
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 在容器內執行工具
cd /app/web_api_ws/scripts
python3 query_kuka_containers.py
```

### 一行命令執行
```bash
# 從宿主機直接執行
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "cd /app/web_api_ws/scripts && python3 query_kuka_containers.py"
```

## 使用場景

### 1. 檢查容器同步狀態
測試 Rack 更新後，確認 KUKA Fleet 中的容器狀態是否正確：
```bash
# 查詢特定容器
python3 query_kuka_containers.py --code RACK001

# 查看所有在地圖中的容器
python3 query_kuka_containers.py --status in
```

### 2. 調試同步問題
檢查特定位置的容器分佈：
```bash
# 查詢特定位置
python3 query_kuka_containers.py --position node-uuid-123
```

### 3. 資料匯出
匯出容器資料用於分析：
```bash
# JSON 格式匯出
python3 query_kuka_containers.py --json > containers.json

# 簡單列表匯出
python3 query_kuka_containers.py --simple > container_codes.txt
```

### 4. 測試整合
在測試 KUKA Container 同步功能時使用：
```bash
# 測試前：記錄初始狀態
python3 query_kuka_containers.py --code RACK001 --json > before.json

# 執行測試...

# 測試後：確認變更
python3 query_kuka_containers.py --code RACK001 --json > after.json
diff before.json after.json
```

## 故障排除

### 連接失敗
```
❌ 錯誤：無法連接 KUKA Fleet Manager
```
**解決方式**：
1. 檢查 KUKA Fleet Manager 服務是否運行
2. 檢查網路連接：`ping <KUKA_HOST>`
3. 確認 KUKA API 配置正確

### 認證失敗
```
❌ 錯誤：KUKA Fleet 認證失敗
```
**解決方式**：
1. 檢查 KUKA API 憑證配置
2. 使用 `--no-login` 參數跳過自動登入
3. 手動測試 KUKA API 連接

### 匯入錯誤
```
❌ 錯誤：無法匯入 KukaApiClient
```
**解決方式**：
1. 確認在容器內執行：`pwd` 應顯示 `/app/...`
2. 確認 kuka_fleet_ws 已建置：`cd /app/kuka_fleet_ws && ls install/`
3. 載入環境：`source /app/setup.bash && all_source`

## 相關文檔

- KUKA Fleet API: `docs-ai/knowledge/protocols/kuka-fleet-api.md`
- KUKA Container 同步: `agvcui/services/kuka_sync_service.py`
- 測試文檔: `agvcui/test/test_kuka_sync_service.py`
