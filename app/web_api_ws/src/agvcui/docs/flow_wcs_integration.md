# Flow WCS Integration with Linear Flow Designer

## Overview
Linear Flow Designer v2 現在支援與 flow_wcs 的函數庫動態整合，實現即時函數同步。

## 架構設計

### 整合架構
```
flow_wcs (Python)
    ↓
FlowExecutor.get_function_library()
    ↓
web_api (Port 8000)
    ↓
/api/flow/functions endpoint
    ↓
linear_flow_designer (Port 8001)
    ↓
JavaScript fetch API
    ↓
UI 函數庫顯示
```

## 實現細節

### 1. Flow WCS 端 (提供函數定義)

**檔案**: `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py`

```python
@classmethod
def get_function_library(cls) -> Dict:
    """Get function library metadata for external tools"""
    return {
        "query": [...],
        "check": [...],
        "task": [...],
        "action": [...],
        "control": [...],
        "special": [...]
    }
```

### 2. Web API 端 (提供 API 端點)

**檔案**: `/app/web_api_ws/src/web_api/web_api/routers/flow_functions.py`

提供以下 API 端點：
- `GET /api/flow/functions` - 獲取完整函數庫
- `GET /api/flow/functions/categories` - 獲取函數分類
- `GET /api/flow/functions/{category}` - 獲取特定分類函數
- `GET /api/flow/functions/search/{keyword}` - 搜尋函數

### 3. Linear Flow Designer 端 (消費函數定義)

**檔案**: `/app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py`

```python
@router.get("/api/functions")
async def get_available_functions(source: str = "local"):
    # source 參數控制函數來源
    # "flow_wcs" - 從 flow_wcs 動態獲取
    # "local" - 使用本地靜態定義
```

## 使用方式

### 1. 啟動服務

```bash
# 1. 啟動 Web API 服務 (包含 flow_functions 路由)
cd /app/web_api_ws
python3 src/web_api/web_api/api_server.py

# 2. 啟動 AGVCUI 服務 (包含 Linear Flow Designer)
python3 src/agvcui/agvcui/agvc_ui_server.py
```

### 2. 訪問 Linear Flow Designer

打開瀏覽器訪問: 
- 生產環境 (透過 nginx): http://agvc.ui/linear-flow/designer
- 開發環境 (直接訪問): http://localhost:8001/linear-flow/designer

**Note**: 系統會自動使用正確的 API 端點：
- 當透過 agvc.ui 訪問時，API 請求會透過 nginx 自動路由
- 當透過 localhost:8001 訪問時，API 請求會直接到本地服務

### 3. 函數庫來源指示

UI 會自動顯示函數庫來源：
- 🔄 **Flow WCS (即時)** - 從 flow_wcs 動態載入
- 📦 **本地函數庫** - 使用靜態定義 (fallback)

## 配置選項

**檔案**: `/app/config/flow_designer_config.yaml`

```yaml
function_library:
  primary_source: 'flow_wcs'  # 優先使用 flow_wcs
  fallback_source: 'local'     # 失敗時使用本地
  flow_wcs_endpoint: 'http://agvc.webapi/api/flow/functions'  # 透過 nginx proxy
```

## 優點

1. **即時同步**: 函數定義與 flow_wcs 實際實現保持同步
2. **單一真相來源**: flow_wcs 是函數定義的唯一來源
3. **向後相容**: 支援 fallback 到本地函數定義
4. **易於擴展**: 新增函數只需在 flow_wcs 中定義

## 故障排除

### 問題: 函數庫載入失敗

**症狀**: UI 顯示 "本地函數庫" 而非 "Flow WCS (即時)"

**檢查步驟**:
1. 確認 Web API 服務運行中 (Port 8000)
2. 測試 API 端點: 
   - 生產環境: `curl http://agvc.webapi/api/flow/functions`
   - 開發環境: `curl http://localhost:8000/api/flow/functions`
3. 檢查瀏覽器控制台錯誤
4. 確認 flow_wcs 模組可以被 import

### 問題: 函數執行失敗

**症狀**: 測試執行時顯示函數不存在

**解決方案**:
1. 確認函數在 FlowExecutor.register_functions() 中註冊
2. 確認函數在 get_function_library() 中有對應的 metadata
3. 檢查函數名稱拼寫是否一致

## 開發建議

### 新增函數的標準流程

1. **在 flow_wcs 實現函數**:
   ```python
   def my_new_function(self, params: Dict) -> Any:
       """實現函數邏輯"""
       pass
   ```

2. **註冊函數**:
   ```python
   def register_functions(self):
       return {
           'category.my_new_function': self.my_new_function,
           ...
       }
   ```

3. **添加 metadata**:
   ```python
   @classmethod
   def get_function_library(cls):
       return {
           "category": [
               {
                   "name": "category.my_new_function",
                   "description": "函數描述",
                   "params": ["param1", "param2"],
                   "returns": "return_type"
               }
           ]
       }
   ```

4. **測試整合**:
   - 重啟 Web API 服務
   - 刷新 Linear Flow Designer
   - 確認新函數出現在函數庫中

## API 測試範例

```bash
# 透過 nginx proxy 訪問 (生產環境)
# 獲取完整函數庫 (從 Web API 服務)
curl http://agvc.webapi/api/flow/functions

# 獲取特定分類
curl http://agvc.webapi/api/flow/functions/query

# 搜尋函數
curl http://agvc.webapi/api/flow/functions/search/task

# Linear Flow Designer API (從 AGVCUI 服務，with source parameter)
curl "http://agvc.ui/linear-flow/api/functions?source=flow_wcs"
curl "http://agvc.ui/linear-flow/api/functions?source=config"
curl "http://agvc.ui/linear-flow/api/functions?source=local"

# 直接訪問 (開發環境)
curl http://localhost:8000/api/flow/functions        # Web API 服務
curl http://localhost:8001/linear-flow/api/functions # AGVCUI 服務
```

## 未來改進

1. **快取機制**: 實現函數定義快取以減少 API 調用
2. **版本控制**: 添加函數庫版本管理
3. **熱更新**: 支援不重啟服務的函數更新
4. **函數文檔**: 自動生成函數使用文檔
5. **類型檢查**: 加強參數類型驗證