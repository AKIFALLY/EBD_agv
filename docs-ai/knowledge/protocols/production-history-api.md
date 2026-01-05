# Production History API - 生產記錄 API

## 概述

Production History API 用於記錄 AGV 的生產歷史資料，包含 AGV 名稱、設備端口、物料資訊等。

## API 端點

### 建立生產記錄

**端點**: `POST /api/v1/production_history/`

**描述**: 建立新的生產記錄

**請求格式**:
```json
{
  "agv_name": "AGV03",
  "description": "生產記錄範例",
  "eqp_port": "CV1",
  "material": "na"
}
```

### 請求參數說明

| 欄位 | 類型 | 必填 | 說明 |
|------|------|------|------|
| `agv_name` | string | 是 | AGV 名稱（例如: AGV01, AGV02, AGV03） |
| `description` | string | 否 | 生產記錄描述 |
| `eqp_port` | string | 是 | 設備端口名稱（例如: CV1, CV2） |
| `material` | string | 否 | 物料資訊，無物料時填 "na" |

### 回應格式

**成功回應** (HTTP 200/201):
```json
{
  "id": 1,
  "agv_name": "AGV03",
  "description": "生產記錄範例",
  "eqp_port": "CV1",
  "material": "na",
  "created_at": "2025-12-22T16:00:00Z"
}
```

**錯誤回應** (HTTP 4xx/5xx):
```json
{
  "detail": "錯誤訊息"
}
```

## 使用範例

### cURL 範例
```bash
curl -X POST "http://localhost:8001/api/v1/production_history/" \
  -H "Content-Type: application/json" \
  -d '{
    "agv_name": "AGV03",
    "description": "生產記錄範例",
    "eqp_port": "CV1",
    "material": "na"
  }'
```

### Python 範例
```python
import requests

url = "http://localhost:8001/api/v1/production_history/"
payload = {
    "agv_name": "AGV03",
    "description": "生產記錄範例",
    "eqp_port": "CV1",
    "material": "na"
}

response = requests.post(url, json=payload)
if response.status_code in (200, 201):
    print("生產記錄建立成功:", response.json())
else:
    print("建立失敗:", response.text)
```

## 相關資源

- **API 服務端口**: 8001 (AGVCUI)
- **Swagger 文檔**: `http://localhost:8001/docs`



## 觸發場景

agv_ws/agv_state/

## AGV查詢到適合任務(Status非5,15,25)跳轉到write_path_state
  description = "{agv_name}獲取到任務 {from_pot}--->{to_port} status={status}"  agv_name="task.agv_name"  meterial="task.meterial_code" eqp_port="na"


## AGV更新status
  description = "{agv_name}更新status {status_old}-->{status_new}"  agv_name="task.agv_name"  meterial="task.meterial_code" eqp_port="na"


## AGV取料完成
  description

