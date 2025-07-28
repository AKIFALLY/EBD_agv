# KUKA Fleet Manager API 整合協定

## 🎯 適用場景
- 理解 KUKA Fleet Manager API 的完整規格和使用方式
- 實作 KUKA Fleet 系統整合和 API 串接
- 解決 KUKA Fleet API 相關的連接和呼叫問題
- 為 KUKA Fleet 適配器開發提供標準參考

## 📋 API 系統概覽

### 基本資訊
- **版本**: 2.13.0
- **主機**: 192.168.10.3:10870
- **基礎 URL**: http://192.168.10.3:10870
- **內容類型**: application/json
- **認證方式**: Token-based (無 Bearer 前綴)

### API 架構分類
```
KUKA Fleet API 架構
├── 🔐 認證系統
│   └── Token 管理和驗證
├── 🚀 任務管理 APIs
│   ├── 任務提交、查詢、控制
│   └── 工作流程管理
├── 🗺️ 地圖點位和區域 APIs
│   ├── 節點查詢、區域管理
│   └── 禁止區域控制
├── 📦 容器管理 APIs
│   ├── 容器入場、出場
│   └── 容器狀態查詢
├── 🤖 機器人管理 APIs
│   ├── 機器人狀態查詢
│   └── 機器人控制操作
└── 🔧 插件操作 APIs
    └── 插件管理和記錄查詢
```

## 🔐 認證系統

### Token 認證機制
```json
// 登入請求
POST /api/login
{
    "username": "admin",
    "password": "Admin"
}

// 成功回應
{
    "code": "SUCCESS",
    "data": {
        "token": "your_access_token",
        "expiresIn": 7200,
        "tokenType": "Bearer"
    },
    "success": true
}
```

### 重要認證特性
- **Token 有效期**: 2 小時 (7200 秒)
- **Header 格式**: `Authorization: <token>` (⚠️ 不需要 Bearer 前綴)
- **重新認證**: Token 過期時需重新登入
- **安全考量**: Token 應安全存儲，避免洩漏

## 🚀 任務管理 APIs

### 核心任務操作

#### 提交任務 (Submit Mission)
```json
POST /api/amr/submitMission
{
    "orgId": "Ching-Tech",
    "requestId": "REQ001",
    "missionCode": "MISSION001",
    "missionType": "TRANSPORT",
    "robotModels": ["KMP 400i diffDrive"],
    "robotIds": [101],
    "robotType": "LIFT",
    "priority": 50,
    "missionData": [
        {
            "sequence": 1,
            "position": "test-test1-1",
            "type": "NODE_POINT",
            "passStrategy": "AUTO"
        }
    ]
}
```

#### 任務類型說明
- **TRANSPORT**: 運輸任務
- **MOVE**: 移動任務 (RosAGV 使用)
- **RACK_MOVE**: 搬運任務 (RosAGV 使用)
- **PICK**: 取料任務
- **DROP**: 卸料任務
- **CHARGE**: 充電任務
- **MAINTENANCE**: 維護任務

#### 任務查詢 (Query Jobs)
```json
POST /api/amr/jobQuery
{
    "robotId": "",
    "status": 0,
    "limit": 0
}
```

**狀態碼對照**:
- `0`: ALL (所有狀態)
- `1`: PENDING (待執行)
- `2`: RUNNING (執行中)
- `3`: COMPLETED (已完成)
- `4`: FAILED (失敗)
- `5`: CANCELLED (已取消)

#### 任務控制操作
```json
// 取消任務
POST /api/amr/missionCancel
{
    "missionCode": "MISSION001",
    "cancelMode": "FORCE"  // NORMAL | FORCE
}

// 暫停任務
POST /api/amr/pauseMission?missionCode=MISSION001

// 恢復任務
POST /api/amr/recoverMission?missionCode=MISSION001

// 重定向任務
POST /api/amr/redirectMission
{
    "missionCode": "MISSION001",
    "targetPosition": "new-position"
}

// 操作反饋 (節點操作完成後釋放任務)
POST /api/amr/operationFeedback
{
    "containerCode": "",
    "missionCode": "MISSION001",
    "position": "A001",
    "requestId": "REQ001"
}
```

## 🗺️ 地圖點位和區域 APIs

### 區域管理
```json
// 查詢所有區域
GET /api/amr/areaQuery

// 查詢區域內節點
POST /api/amr/areaNodesQuery
{
    "areaCodes": ["area1", "area2"]
}

// 根據節點查詢區域
GET /api/amr/queryWCSAreaByMapNode?nodeUuid=node_uuid
```

### 功能節點查詢
```json
POST /api/amr/queryFunctionNode
{
    "functionType": 1,  // 0=All, 1=Pick, 2=Drop, 3=Charge, 4=Maintenance, 5=Waiting
    "mapCode": "WAREHOUSE_L1",
    "robotTypeCode": "CARGO_AGV"
}
```

### 禁止區域管理
```json
// 查詢所有禁止區域
GET /api/amr/queryAllForbiddenAreas

// 查詢特定禁止區域
GET /api/amr/queryOneForbiddenArea?forbiddenAreaCode=area_code

// 更新禁止區域狀態
POST /api/amr/updateForbiddenAreaStatus
{
    "forbiddenAreaCode": "area_code",
    "status": "0"  // 0=禁用, 1=啟用
}
```

## 📦 容器管理 APIs

### 容器生命週期管理
```json
// 容器入場
POST /api/amr/containerIn
{
    "containerCode": "CONT001",
    "containerModelCode": "PALLET_1200",
    "position": "A001",
    "isNew": true,
    "requestId": "REQ001"
}

// 容器出場
POST /api/amr/containerOut
{
    "containerCode": "CONT001",
    "position": "A001",
    "isDelete": true,
    "requestId": "REQ001"
}
```

### 容器查詢
```json
// 查詢在場容器
POST /api/amr/containerQuery
{
    "containerCode": "",
    "emptyFullStatus": 0  // 0=All, 1=Empty, 2=Full, 3=Unknown
}

// 查詢所有容器 (含離場)
POST /api/amr/containerQueryAll
{
    "containerCode": "",
    "inMapStatus": 0  // 0=All, 1=In Map, 2=Out of Map
}
```

### 容器模型管理
```json
// 查詢所有容器模型
GET /api/amr/queryAllContainerModelCode

// 查詢容器模型對應存放區域
GET /api/amr/queryAreaCodeForContainerModel?containerModelCode=PALLET_1200

// 更新容器資訊 (空/滿狀態和位置)
POST /api/amr/updateContainer
{
    "containerCode": "CONT001",
    "containerType": "PALLET",
    "emptyStatus": "FULL",
    "originPosition": "A001",
    "targetPosition": "B001",
    "reason": "Transport completed",
    "requestId": "REQ001"
}
```

## 🤖 機器人管理 APIs

### 機器人狀態查詢
```json
POST /api/amr/robotQuery
{
    "robotId": "",
    "robotType": "",
    "mapCode": "",
    "floorNumber": ""
}
```

**機器人狀態碼**:
- `0`: OFFLINE (離線)
- `1`: IDLE (空閒)
- `2`: WORKING (任務中)
- `3`: CHARGING (充電中)
- `4`: ERROR (錯誤)
- `5`: MAINTENANCE (維護中)

### 機器人操作
```json
// 機器人入場
POST /api/amr/insertRobot
{
    "robotId": "AGV001",
    "cellCode": "A001"
}

// 機器人離場
POST /api/amr/removeRobot
{
    "robotId": "AGV001",
    "withContainer": 0
}

// 充電指令
POST /api/amr/chargeRobot
{
    "robotId": "AGV001",
    "targetLevel": 80,
    "missionCode": "CHARGE001"
}

// 根據節點查詢機器人
POST /api/amr/queryRobByNodeUuidOrForeignCode?nodeCode=A001,A002

// 機器人移動搬運
POST /api/amr/robotMoveCarry
{
    "containerCode": "CONT001",
    "missionCode": "MOVE001",
    "robotId": "AGV001",
    "targetNodeCode": "B001"
}
```

## 🔧 插件操作 APIs

### 插件生命週期管理
```json
// 上傳插件
POST /api/plugin/upload
Content-Type: multipart/form-data
Form Data: jarFile (JAR file)

// 啟用插件
POST /api/plugin/start/{pluginId}?version=1.0

// 停用插件
POST /api/plugin/stop/{pluginId}

// 重載插件
POST /api/plugin/reload/{pluginId}?version=1.0

// 卸載插件
POST /api/plugin/unload/{pluginId}

// 刪除插件
POST /api/plugin/delete/{pluginId}?version=1.0
```

### 插件標籤和版本管理
```json
// 添加標籤
POST /api/plugin/addTag/{pluginId}?tag=production&version=1.0

// 列出所有插件
GET /api/plugin/listPlugins?pluginId={pluginId}
```

### 插件記錄查詢
```json
// 查詢操作記錄
GET /api/plugin/getRecord?beginTime=2024-01-01&endTime=2024-01-31&page=1&pageCount=20&status=1&taskCode=TASK001&url=/api/test

// 獲取記錄詳細內容
GET /api/plugin/getRecordHttpMessage?id=12345

// 重新發送記錄
POST /api/plugin/resend/{recordId}
{
    "parameter1": "value1",
    "parameter2": "value2"
}
```

### 插件管理最佳實踐
- **版本控制**: 為插件設置明確的版本號
- **漸進式部署**: 使用標籤管理不同環境的插件版本
- **監控記錄**: 定期檢查插件操作記錄
- **安全上傳**: 驗證 JAR 檔案的完整性和安全性

## 🔧 回應格式和錯誤處理

### 標準成功回應
```json
{
    "code": "SUCCESS",
    "data": "response_data",
    "msg": "response_message",
    "success": true
}
```

### 標準錯誤回應
```json
{
    "code": "ERROR_CODE",
    "data": null,
    "msg": "Error description",
    "success": false,
    "errorDetails": {
        "errorCode": "VALIDATION_ERROR",
        "errorMessage": "Invalid parameter: robotId cannot be empty",
        "timestamp": "2024-01-15T10:30:00Z",
        "requestId": "REQ_12345"
    }
}
```

### 常見錯誤碼
| 錯誤碼 | 說明 | 處理建議 |
|--------|------|----------|
| VALIDATION_ERROR | 參數驗證失敗 | 檢查請求參數格式和必填項目 |
| AUTHENTICATION_ERROR | 認證失敗或 Token 過期 | 重新登入獲取新 Token |
| AUTHORIZATION_ERROR | 權限不足 | 檢查用戶權限設定 |
| RESOURCE_NOT_FOUND | 資源不存在 | 確認請求的資源 ID 正確 |
| RESOURCE_CONFLICT | 資源衝突 | 檢查資源狀態，避免重複操作 |
| SYSTEM_ERROR | 系統內部錯誤 | 聯繫系統管理員 |
| TIMEOUT_ERROR | 請求超時 | 檢查網路連接，重試請求 |
| NETWORK_ERROR | 網路連接問題 | 檢查網路設定和防火牆 |

## 📊 狀態碼對照表

### HTTP 狀態碼
| 狀態碼 | 說明 | 常見場景 |
|--------|------|----------|
| 200 | OK | 成功的 API 呼叫 |
| 201 | Created | 資源創建成功 |
| 400 | Bad Request | 無效的請求參數 |
| 401 | Unauthorized | 缺少或無效的 Token |
| 403 | Forbidden | 權限不足 |
| 404 | Not Found | 資源不存在 |
| 409 | Conflict | 資源衝突 |
| 429 | Too Many Requests | 請求頻率限制 |
| 500 | Internal Server Error | 系統錯誤 |
| 503 | Service Unavailable | 系統維護中 |

### 作業狀態碼
| 狀態 | 名稱 | 說明 |
|------|------|------|
| 0 | ALL | 所有狀態 (查詢過濾器) |
| 1 | PENDING | 作業已創建，等待執行 |
| 2 | RUNNING | 作業執行中 |
| 3 | COMPLETED | 作業成功完成 |
| 4 | FAILED | 作業執行失敗 |
| 5 | CANCELLED | 作業被用戶取消 |

### 機器人狀態碼
| 狀態 | 名稱 | 說明 |
|------|------|------|
| 0 | OFFLINE | 機器人未連接 |
| 1 | IDLE | 機器人可用於任務 |
| 2 | WORKING | 機器人執行任務中 |
| 3 | CHARGING | 機器人充電中 |
| 4 | ERROR | 機器人錯誤狀態 |
| 5 | MAINTENANCE | 機器人維護模式 |

### 容器狀態碼
| 狀態 | 名稱 | 說明 |
|------|------|------|
| 0 | ALL | 所有狀態 (查詢過濾器) |
| 1 | EMPTY | 容器為空 |
| 2 | FULL | 容器有貨物 |
| 3 | UNKNOWN | 容器狀態未知 |

## 💡 最佳實踐和使用指導

### 連接配置
```python
# Python 客戶端配置範例
from kuka_fleet_adapter.kuka_api_client import KukaApiClient

client = KukaApiClient(
    base_url="http://192.168.10.3:10870",
    username="admin",
    password="Admin"
)

# 檢查連接狀態
if client.token:
    print("連接成功")
else:
    print("連接失敗")
```

### Token 管理策略
```python
# Token 有效性檢查
def ensure_valid_token(client):
    if not client.is_token_valid():
        client.force_relogin("admin", "Admin")
    return client.token is not None

# 自動重試機制
def api_call_with_retry(client, api_func, *args, **kwargs):
    try:
        return api_func(*args, **kwargs)
    except AuthenticationError:
        if ensure_valid_token(client):
            return api_func(*args, **kwargs)
        raise
```

### 任務管理最佳實踐
1. **任務代碼唯一性**: 確保每個任務代碼唯一，避免衝突
2. **任務狀態監控**: 定期查詢任務狀態，及時處理異常
3. **資源狀態確認**: 提交任務前確認機器人和容器狀態
4. **錯誤處理**: 實作完整的錯誤處理和重試機制

### 網路診斷
```bash
# 基本連接測試
ping 192.168.10.3

# API 端點測試
curl -X POST "http://192.168.10.3:10870/api/login" \
     -H "Content-Type: application/json" \
     -d '{"username":"admin","password":"Admin"}'

# 端口可達性測試
telnet 192.168.10.3 10870
```

## 🔗 系統整合考量

### 與 RosAGV 系統整合
- **適配器模式**: 透過 KukaFleetAdapter 類別進行整合
- **狀態同步**: 定期同步機器人和容器狀態
- **任務轉換**: 將 RosAGV 任務格式轉換為 KUKA Fleet API 格式
- **回調處理**: 處理 KUKA Fleet 的任務狀態回調

### 效能考量
- **連接池**: 使用連接池管理 HTTP 連接
- **並發控制**: 控制同時進行的 API 呼叫數量
- **快取策略**: 對不常變更的資料進行快取
- **重試策略**: 實作指數退避的重試機制

### 安全考量
- **Token 安全**: 安全存儲和傳輸 API Token
- **網路隔離**: 確保 API 端點的網路安全
- **存取控制**: 實作適當的存取控制和權限管理
- **日誌記錄**: 記錄 API 呼叫和錯誤資訊用於除錯

## 🔗 交叉引用
- KUKA Fleet 回調規格: @docs-ai/knowledge/protocols/kuka-fleet-callback.md
- KUKA Fleet 適配器實作: @app/kuka_fleet_ws/CLAUDE.md
- 外部系統整合: @docs-ai/knowledge/integration/external-systems.md
- API 客戶端實作: @app/kuka_fleet_ws/src/kuka_fleet_adapter/kuka_fleet_adapter/kuka_api_client.py