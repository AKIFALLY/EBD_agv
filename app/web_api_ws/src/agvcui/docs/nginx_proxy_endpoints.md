# Nginx Proxy Endpoints Configuration

## Overview
RosAGV 系統使用 nginx 作為反向代理，提供統一的存取端點，簡化服務間通訊和外部存取。

## Nginx Proxy Mapping

### 主要代理映射
| Nginx Domain | Port | Proxies to | Service | Description |
|-------------|------|------------|---------|-------------|
| `agvc.webapi` | 80 | `localhost:8000` | Web API | API Gateway 服務 |
| `agvc.ui` | 80 | `localhost:8001` | AGVCUI | 車隊管理界面 |
| `op.ui` | 80 | `localhost:8002` | OPUI | 操作員界面 |

## How It Works

### 服務間通訊
當一個服務需要呼叫另一個服務的 API 時：

1. **AGVCUI 呼叫 Web API**:
   - AGVCUI (Port 8001) 需要從 flow_wcs 獲取函數庫
   - 使用 URL: `http://agvc.webapi/api/flow/functions`
   - nginx 將請求轉發到 `localhost:8000/api/flow/functions`

2. **前端 JavaScript 呼叫 API**:
   - 當用戶透過 `http://agvc.ui/` 訪問時
   - JavaScript 中的相對 URL `/linear-flow/api/functions` 
   - 會自動解析為 `http://agvc.ui/linear-flow/api/functions`
   - nginx 將其轉發到 `localhost:8001/linear-flow/api/functions`

### 外部存取
用戶可以透過 nginx proxy 域名存取所有服務：

```bash
# 透過 nginx proxy 存取
http://agvc.webapi/docs      # API 文檔
http://agvc.ui/              # 車隊管理界面
http://op.ui/                # 操作員界面

# 直接存取 (開發環境)
http://localhost:8000/docs   # API 文檔
http://localhost:8001/       # 車隊管理界面
http://localhost:8002/       # 操作員界面
```

## Configuration in Code

### Backend (Python)
當後端服務需要呼叫其他服務時，使用 nginx proxy 域名：

```python
# linear_flow_designer.py
import httpx

async def get_flow_functions():
    async with httpx.AsyncClient() as client:
        # 使用 nginx proxy 域名
        response = await client.get("http://agvc.webapi/api/flow/functions")
        return response.json()
```

### Frontend (JavaScript)
前端代碼使用相對 URL，自動適應當前域名：

```javascript
// linearFlowDesigner.js
async function loadAvailableFunctions() {
    // 使用相對 URL - 自動適應當前域名
    const response = await fetch('/linear-flow/api/functions');
    // 如果透過 agvc.ui 訪問，這會是 http://agvc.ui/linear-flow/api/functions
    // 如果透過 localhost:8001 訪問，這會是 http://localhost:8001/linear-flow/api/functions
}
```

## Function Library Integration

### Two Approaches for Different Modes

1. **API Test Mode** (使用 flow_wcs 動態函數庫):
   - Source: `flow_wcs`
   - Flow: AGVCUI → nginx (agvc.webapi) → Web API → flow_wcs
   - 用途: 測試真實 API 功能

2. **Validation Mode** (使用配置檔案):
   - Source: `config`
   - Flow: AGVCUI → 讀取本地 YAML 檔案
   - 用途: 驗證流程結構

### API Endpoints

#### Web API Service (Port 8000)
```bash
# flow_wcs 函數庫 API
GET http://agvc.webapi/api/flow/functions           # 完整函數庫
GET http://agvc.webapi/api/flow/functions/query     # Query 類函數
GET http://agvc.webapi/api/flow/functions/task      # Task 類函數
```

#### AGVCUI Service (Port 8001)
```bash
# Linear Flow Designer API
GET http://agvc.ui/linear-flow/api/functions?source=flow_wcs  # 從 flow_wcs 獲取
GET http://agvc.ui/linear-flow/api/functions?source=config    # 從配置檔案獲取
GET http://agvc.ui/linear-flow/api/functions?source=local     # 使用本地定義
```

## Testing

### 測試 nginx proxy 連接
```bash
# 測試 Web API 透過 nginx
curl -I http://agvc.webapi/health

# 測試 AGVCUI 透過 nginx
curl -I http://agvc.ui/

# 測試函數庫 API
curl http://agvc.webapi/api/flow/functions | jq
```

### 測試服務間通訊
```bash
# 在 AGVCUI 容器內測試呼叫 Web API
docker compose -f docker-compose.agvc.yml exec agvc_server bash
curl http://agvc.webapi/api/flow/functions
```

## Troubleshooting

### 常見問題

1. **無法解析 agvc.webapi**:
   - 檢查 /etc/hosts 或 DNS 設定
   - 確認 nginx 容器正在運行
   - 檢查 docker-compose 網路配置

2. **連接被拒絕**:
   - 確認服務正在運行 (Port 8000/8001/8002)
   - 檢查防火牆規則
   - 驗證 nginx 配置正確

3. **CORS 錯誤**:
   - 檢查服務的 CORS 設定
   - 確認使用正確的域名

## Benefits

1. **統一入口**: 所有服務透過 nginx 統一管理
2. **簡化配置**: 服務間使用固定域名，不需要管理 IP 和端口
3. **負載均衡**: nginx 可以配置負載均衡（未來擴展）
4. **SSL 終止**: 可以在 nginx 層面處理 SSL（未來擴展）
5. **存取控制**: 可以在 nginx 層面實施存取控制

## Summary

- **nginx proxy** 提供統一的服務存取端點
- **agvc.webapi:80** → localhost:8000 (Web API)
- **agvc.ui:80** → localhost:8001 (AGVCUI)
- **op.ui:80** → localhost:8002 (OPUI)
- 後端服務間通訊使用 nginx proxy 域名
- 前端使用相對 URL，自動適應當前域名
- 支援開發環境 (localhost) 和生產環境 (nginx proxy) 無縫切換