# TAFL API Reference

## 🎯 概述
TAFL (Task Automation Flow Language) API 提供完整的流程編輯、執行和管理功能。支援 Real Mode（真實資料庫執行）和 Simulation Mode（模擬執行）兩種執行模式。

**基礎 URL**: `http://localhost:8001/tafl`  
**認證**: 無需認證（內部系統）  
**格式**: JSON

## 📊 API 端點列表

### 1. 系統狀態檢查
```http
GET /tafl/status
```

**功能**: 檢查 TAFL 系統狀態和模組可用性

**回應範例**:
```json
{
  "status": "running",
  "enhanced_modules": true,
  "database_connected": true,
  "version": "1.0",
  "execution_modes": ["real", "simulation"]
}
```

### 2. 執行 TAFL 流程
```http
POST /tafl/execute
```

**功能**: 執行 TAFL 流程（支援 Real/Simulation 模式）

**請求體**:
```json
{
  "metadata": {
    "id": "flow_001",
    "name": "Test Flow"
  },
  "flow": [
    {
      "query": {
        "target": "locations",
        "limit": 5,
        "filter": {"status": "available"}
      }
    }
  ],
  "mode": "real"  // "real" 或 "simulation"
}
```

**回應範例 (Real Mode)**:
```json
{
  "success": true,
  "flow_id": "flow_001",
  "message": "Flow executed successfully",
  "execution_time": 0.024608,
  "mode": "real",
  "status": "completed",
  "result": {
    "locations": [
      {"id": "L001", "name": "Location 1", "status": "available"},
      {"id": "L002", "name": "Location 2", "status": "available"}
    ]
  },
  "execution_log": [
    {
      "timestamp": "2025-09-03T10:30:15.123",
      "action": "query",
      "status": "success",
      "details": "Queried 2 locations from database"
    }
  ]
}
```

**回應範例 (Simulation Mode)**:
```json
{
  "success": true,
  "flow_id": "flow_001",
  "message": "Flow simulated successfully",
  "mode": "simulation",
  "status": "completed",
  "simulated_results": {
    "locations": ["simulated_location_1", "simulated_location_2"]
  }
}
```

### 3. 取得動詞列表
```http
GET /tafl/verbs
```

**功能**: 取得所有支援的 TAFL 動詞

**回應範例**:
```json
{
  "verbs": {
    "query": "Query data from database",
    "check": "Check conditions or status",
    "create": "Create new resources",
    "update": "Update existing data",
    "if": "Conditional execution",
    "for": "Loop through collection",
    "switch": "Multi-branch conditional",
    "set": "Set variable values",
    "stop": "Stop flow execution",
    "notify": "Send notifications"
  },
  "total": 10
}
```

### 4. 取得範例流程
```http
GET /tafl/examples
```

**功能**: 取得 TAFL 流程範例列表

**回應範例**:
```json
{
  "examples": [
    {
      "id": "query_locations",
      "name": "查詢位置",
      "description": "Simple location query example",
      "flow": {...}
    },
    {
      "id": "rack_rotation",
      "name": "料架輪轉",
      "description": "Rack rotation workflow",
      "flow": {...}
    }
  ]
}
```

### 5. 儲存流程
```http
POST /tafl/save
```

**功能**: 儲存 TAFL 流程到檔案系統

**請求體**:
```json
{
  "id": "custom_flow_001",
  "name": "My Custom Flow",
  "content": {
    "metadata": {...},
    "flow": [...]
  }
}
```

**回應範例**:
```json
{
  "success": true,
  "message": "Flow saved successfully",
  "path": "/app/config/tafl/flows/custom_flow_001.yaml"
}
```

### 6. 載入流程
```http
GET /tafl/load/{flow_id}
```

**功能**: 載入已儲存的 TAFL 流程

**參數**:
- `flow_id`: 流程 ID

**回應範例**:
```json
{
  "success": true,
  "flow": {
    "metadata": {
      "id": "custom_flow_001",
      "name": "My Custom Flow"
    },
    "flow": [...]
  }
}
```

### 7. 列出所有流程
```http
GET /tafl/flows
```

**功能**: 列出所有已儲存的流程

**回應範例**:
```json
{
  "flows": [
    {
      "id": "test_comprehensive_demo",
      "name": "TAFL 完整測試展示",
      "created": "2025-09-03",
      "path": "/app/config/tafl/flows/test_comprehensive_demo.yaml"
    },
    {
      "id": "test_simple_query",
      "name": "簡單查詢測試",
      "created": "2025-09-03",
      "path": "/app/config/tafl/flows/test_simple_query.yaml"
    }
  ],
  "total": 2
}
```

### 8. 測試資料庫連接
```http
GET /tafl/test-db
```

**功能**: 測試資料庫連接（除錯用）

**回應範例**:
```json
{
  "status": "connected",
  "database": "PostgreSQL",
  "data": [
    {"id": "L001", "name": "Location 1"},
    {"id": "L002", "name": "Location 2"}
  ],
  "query_time": 0.024
}
```

## 🔧 錯誤處理

### 錯誤回應格式
```json
{
  "success": false,
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid TAFL syntax",
    "details": "Missing required field 'target' in query verb",
    "line": 15
  }
}
```

### 常見錯誤碼
| 錯誤碼 | 描述 | HTTP 狀態碼 |
|--------|------|-------------|
| `VALIDATION_ERROR` | TAFL 語法錯誤 | 400 |
| `MODULE_NOT_FOUND` | 模組載入失敗 | 500 |
| `DATABASE_ERROR` | 資料庫連接錯誤 | 503 |
| `EXECUTION_ERROR` | 執行時錯誤 | 500 |
| `NOT_FOUND` | 流程不存在 | 404 |

## 💡 使用範例

### Python 範例
```python
import requests
import json

# 基礎 URL
BASE_URL = "http://localhost:8001/tafl"

# 檢查狀態
response = requests.get(f"{BASE_URL}/status")
print(f"Status: {response.json()}")

# 執行簡單查詢（Real Mode）
flow_data = {
    "metadata": {"id": "test_001", "name": "Test Query"},
    "flow": [
        {
            "query": {
                "target": "locations",
                "limit": 3
            }
        }
    ],
    "mode": "real"
}

response = requests.post(
    f"{BASE_URL}/execute",
    json=flow_data,
    headers={"Content-Type": "application/json"}
)

result = response.json()
print(f"Execution Mode: {result['mode']}")
print(f"Execution Time: {result['execution_time']}s")
print(f"Results: {json.dumps(result['result'], indent=2)}")
```

### cURL 範例
```bash
# 檢查狀態
curl http://localhost:8001/tafl/status

# 執行查詢（Real Mode）
curl -X POST http://localhost:8001/tafl/execute \
  -H "Content-Type: application/json" \
  -d '{
    "flow": [{"query": {"target": "locations", "limit": 2}}],
    "mode": "real"
  }'

# 取得動詞列表
curl http://localhost:8001/tafl/verbs

# 列出所有流程
curl http://localhost:8001/tafl/flows
```

### JavaScript 範例
```javascript
// 使用 Fetch API
async function executeTAFLFlow() {
    const flowData = {
        metadata: {id: "js_test", name: "JavaScript Test"},
        flow: [
            {query: {target: "tasks", limit: 5}}
        ],
        mode: "real"
    };
    
    const response = await fetch('http://localhost:8001/tafl/execute', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(flowData)
    });
    
    const result = await response.json();
    console.log('Execution result:', result);
}
```

## 🔑 重要提示

### 執行模式選擇
- **Real Mode**: 執行真實的資料庫查詢，適合生產環境
- **Simulation Mode**: 模擬執行，適合測試和開發

### 性能考量
- Real Mode 查詢時間約 0.024 秒
- 建議限制查詢結果數量（使用 limit 參數）
- 大量資料操作建議使用批次處理

### 安全性
- API 僅供內部系統使用
- 所有查詢都經過參數驗證
- 支援 SQL 注入防護

## 🔗 相關文檔
- TAFL 語言規格: docs-ai/knowledge/system/tafl/tafl-language-specification.md
- TAFL 編輯器規格: docs-ai/knowledge/system/tafl/tafl-editor-specification.md
- 故障排除指南: docs-ai/knowledge/system/tafl/tafl-troubleshooting-guide.md

---

**最後更新**: 2025-09-03  
**API 版本**: 1.0  
**狀態**: ✅ **生產就緒**