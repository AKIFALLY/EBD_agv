# TAFL 使用者指南

## 🎯 概述
本指南整合了 TAFL (Task Automation Flow Language) 系統的快速入門與故障排除，提供完整的使用者操作指導。

---

# 第一部分：快速入門

## 🚀 快速開始（5分鐘）

### Step 1: 訪問 TAFL Editor
```
http://localhost:8001/tafl/editor
```

### Step 2: 創建第一個流程
在編輯器中貼上以下內容：
```yaml
metadata:
  id: "my_first_flow"
  name: "我的第一個流程"

flow:
  - query:
      target: "locations"
      limit: 3
  - notify:
      message: "找到 $locations.count 個位置"
```

### Step 3: 執行流程
1. 選擇 **"Real Execution"** 模式
2. 點擊 **"Execute"** 按鈕
3. 查看執行結果

## 📋 測試檔案位置

所有測試檔案位於：
```
/home/ct/RosAGV/app/config/tafl/flows/
```

### 推薦測試順序
1. `test_simple_query.yaml` - 簡單查詢測試
2. `test_real_execution.yaml` - Real Mode 驗證
3. `test_comprehensive_demo.yaml` - 完整功能展示

## 🧪 測試方法對照表

| 測試方法 | 適用場景 | 難度 | 速度 |
|---------|---------|------|------|
| **UI 測試** | 互動式開發 | 簡單 | 中等 |
| **cURL 測試** | 快速驗證 | 簡單 | 快速 |
| **Python 腳本** | 自動化測試 | 中等 | 快速 |
| **內建測試檔** | 功能驗證 | 簡單 | 快速 |

## 💻 命令行快速測試

### 檢查系統狀態
```bash
curl http://localhost:8001/tafl/status | jq .
```
預期結果：
```json
{
  "enhanced_modules": true,
  "database_connected": true
}
```

### 執行簡單查詢
```bash
curl -X POST http://localhost:8001/tafl/execute \
  -H "Content-Type: application/json" \
  -d '{"flow":[{"query":{"target":"locations","limit":2}}],"mode":"real"}' \
  | jq .
```

### 查看可用動詞
```bash
curl http://localhost:8001/tafl/verbs | jq .
```

## 🐍 Python 測試腳本

創建檔案 `test_tafl.py`:
```python
#!/usr/bin/env python3
import requests
import json

BASE_URL = "http://localhost:8001/tafl"

def test_status():
    """測試系統狀態"""
    resp = requests.get(f"{BASE_URL}/status")
    data = resp.json()
    print(f"✅ Enhanced Modules: {data['enhanced_modules']}")
    print(f"✅ Database Connected: {data['database_connected']}")
    return data['enhanced_modules']

def test_execution():
    """測試執行功能"""
    flow = {
        "flow": [{"query": {"target": "locations", "limit": 2}}],
        "mode": "real"
    }
    resp = requests.post(f"{BASE_URL}/execute", json=flow)
    data = resp.json()
    print(f"✅ Mode: {data['mode']}")
    print(f"✅ Execution Time: {data['execution_time']}s")
    return data['mode'] == "real"

if __name__ == "__main__":
    print("🔍 測試 TAFL 系統...")
    if test_status() and test_execution():
        print("🎉 所有測試通過！")
    else:
        print("❌ 測試失敗")
```

執行測試：
```bash
python3 test_tafl.py
```

## 📝 TAFL 語法速查

### 10 個標準動詞
```yaml
# 資料操作
- query: {target: "table", limit: 10}
- check: {condition: "$count > 0", as: "result"}
- create: {target: "resource", data: {...}}
- update: {target: "resource", id: "123", data: {...}}

# 控制流程
- if: {condition: "...", then: [...], else: [...]}
- for: {each: "item", in: "$list", do: [...]}
- switch: {expression: "$var", cases: [...]}

# 系統操作
- set: {variable: "value"}
- stop: {reason: "完成"}
- notify: {message: "通知內容"}
```

### 變數引用
```yaml
# 引用查詢結果
$locations           # 整個結果集
$locations.count     # 結果數量
$locations[0].id     # 第一個結果的 ID

# 系統變數
$system.timestamp    # 當前時間
$system.execution_time # 執行時間
```

## ⚡ 性能驗證

### Real Mode 特徵
- ✅ 執行時間: 0.02-0.05 秒
- ✅ 返回真實資料庫記錄
- ✅ mode 欄位顯示 "real"

### Simulation Mode 特徵
- ⚡ 執行時間: < 0.001 秒
- 🔄 返回模擬資料
- 🔄 mode 欄位顯示 "simulation"

## 📚 進階開發

### 創建自定義流程
1. 複製範例檔案
```bash
cp /home/ct/RosAGV/app/config/tafl/flows/test_simple_query.yaml \
   /home/ct/RosAGV/app/config/tafl/flows/my_custom_flow.yaml
```

2. 編輯檔案
```bash
vim /home/ct/RosAGV/app/config/tafl/flows/my_custom_flow.yaml
```

3. 在 Editor 中載入測試

### 整合到應用程式
```python
# 在您的 Python 應用中
import requests

class TAFLClient:
    def __init__(self, base_url="http://localhost:8001/tafl"):
        self.base_url = base_url

    def execute_flow(self, flow, mode="real"):
        response = requests.post(
            f"{self.base_url}/execute",
            json={"flow": flow, "mode": mode}
        )
        return response.json()

# 使用範例
client = TAFLClient()
result = client.execute_flow([
    {"query": {"target": "tasks", "limit": 5}}
])
```

---

# 第二部分：故障排除

## 📋 常見問題與解決方案

### 問題 1: Real Mode 回退到 Simulation Mode

#### 症狀
- 設置 `mode="real"` 但執行結果顯示 `mode="simulation"`
- 執行時間異常快速（< 0.001s）
- 沒有真實的資料庫查詢結果

#### 診斷步驟
```bash
# 1. 檢查 TAFL 狀態
curl http://localhost:8001/tafl/status

# 應該看到:
{
  "enhanced_modules": true,  # 必須為 true
  "database_connected": true  # 必須為 true
}

# 2. 測試資料庫連接
curl http://localhost:8001/tafl/test-db

# 3. 檢查執行日誌
docker compose -f docker-compose.agvc.yml logs agvc_server | grep -i "tafl"
```

#### 可能原因與解決方案

##### 原因 1: Router 衝突
**問題**: 多個檔案定義相同的端點路徑
```python
# 檢查是否有重複的 @router.post("/execute")
grep -r "@router.post.*execute" /app/web_api_ws/src/agvcui/
```

**解決方案**: 停用衝突的端點
```python
# 在 tafl_editor.py 中註解掉舊版端點
# @router.post("/execute")  # DISABLED
```

##### 原因 2: 模組載入失敗
**問題**: Import 錯誤導致 ENHANCED_MODULES_AVAILABLE = False

**解決方案**: 實作動態載入
```python
def check_and_import_modules():
    """動態載入增強模組"""
    global ENHANCED_MODULES_AVAILABLE
    try:
        from tafl_wcs.tafl_executor_wrapper import TAFLExecutorWrapper
        from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
        ENHANCED_MODULES_AVAILABLE = True
        return True
    except ImportError as e:
        print(f"Module import failed: {e}", file=sys.stderr)
        ENHANCED_MODULES_AVAILABLE = False
        return False
```

##### 原因 3: 資料庫連接問題
**診斷**:
```bash
# 檢查 PostgreSQL 服務
docker compose -f docker-compose.agvc.yml ps postgres

# 測試連接
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
  python3 -c 'from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge;
  db = TAFLDatabaseBridge();
  print(db.query_locations(limit=1))'
"
```

### 問題 2: 模組找不到 (ModuleNotFoundError)

#### 症狀
```
ModuleNotFoundError: No module named 'tafl_wcs'
ModuleNotFoundError: No module named 'tafl'
```

#### 解決步驟
```bash
# 1. 確認在容器內
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 2. 載入環境
source /app/setup.bash
agvc_source

# 3. 檢查 PYTHONPATH
echo $PYTHONPATH

# 4. 重建相關套件
cd /app/tafl_wcs_ws
colcon build --packages-select tafl_wcs
cd /app/web_api_ws
colcon build --packages-select agvcui

# 5. 重新載入
source /app/setup.bash
agvc_source

# 6. 重啟服務
manage_web_api_launch restart
```

### 問題 3: 執行時錯誤

#### AttributeError: 'TAFLExecutorWrapper' object has no attribute 'save_execution_history'

**原因**: 呼叫不存在的方法

**解決方案**:
```python
# 註解掉或移除該行
# self.executor.save_execution_history(flow_data, result)
```

#### TypeError: 'NoneType' object is not subscriptable

**原因**: 查詢結果為 None

**解決方案**:
```python
# 加入空值檢查
if result and 'data' in result:
    locations = result['data']
else:
    locations = []
```

### 問題 4: WebSocket 連接問題

#### 症狀
- TAFL Editor 顯示 "Disconnected"
- 無法接收即時更新

#### 診斷與解決
```bash
# 檢查 Socket.IO 服務
curl http://localhost:8001/socket.io/

# 檢查 CORS 設置
grep -A 5 "cors_allowed_origins" /app/web_api_ws/src/agvcui/

# 重啟 Web 服務
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
  source /app/setup.bash &&
  agvc_source &&
  manage_web_api_launch restart
"
```

### 問題 5: 無法連接到 TAFL Editor
```bash
# 檢查服務狀態
docker compose -f docker-compose.agvc.yml ps

# 查看日誌
docker compose -f docker-compose.agvc.yml logs agvc_server | tail -50
```

## 🔧 除錯工具與技巧

### 1. 測試端點創建
```python
@router.get("/test-db")
async def test_database():
    """測試資料庫連接的專用端點"""
    try:
        from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge
        db = TAFLDatabaseBridge()
        result = db.query_locations(limit=2)
        return {"status": "connected", "data": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}
```

### 2. 詳細日誌輸出
```python
import sys

# 使用 stderr 繞過日誌系統
print(f"[DEBUG] Mode: {mode}", file=sys.stderr)
print(f"[DEBUG] Modules: {ENHANCED_MODULES_AVAILABLE}", file=sys.stderr)
```

### 3. 執行追蹤
```python
def execute_with_trace(flow_data, mode):
    trace = []
    trace.append(f"Starting execution with mode: {mode}")

    if mode == "real" and ENHANCED_MODULES_AVAILABLE:
        trace.append("Using real execution")
        result = execute_real(flow_data)
    else:
        trace.append("Falling back to simulation")
        result = execute_simulation(flow_data)

    result['trace'] = trace
    return result
```

## 📊 診斷檢查清單

### 基本檢查
- [ ] 容器是否運行中
- [ ] 服務是否正常啟動
- [ ] 資料庫是否可連接
- [ ] 模組是否正確載入

### 進階檢查
- [ ] Router 是否有衝突
- [ ] PYTHONPATH 是否正確
- [ ] 套件是否已建置
- [ ] 端口是否被佔用

### 性能檢查
- [ ] 執行時間是否正常（0.02-0.05s）
- [ ] 記憶體使用是否正常
- [ ] CPU 使用是否正常

## 🚀 快速修復指令

### 完整重啟流程
```bash
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
  source /app/setup.bash &&
  agvc_source &&
  manage_web_api_launch stop &&
  cd /app/tafl_wcs_ws && colcon build --packages-select tafl_wcs &&
  cd /app/web_api_ws && colcon build --packages-select agvcui &&
  source /app/setup.bash &&
  agvc_source &&
  manage_web_api_launch start
"
```

### 快速測試
```bash
# 測試 API
curl http://localhost:8001/tafl/status

# 測試執行
curl -X POST http://localhost:8001/tafl/execute \
  -H "Content-Type: application/json" \
  -d '{"flow":[{"query":{"target":"locations","limit":1}}],"mode":"real"}'
```

## 🎯 下一步

1. **深入學習**: 閱讀 [TAFL 語言規格](tafl-language-specification.md)
2. **API 開發**: 查看 [TAFL API Reference](tafl-api-reference.md)
3. **編輯器規格**: 了解 [TAFL Editor 規格](tafl-editor-specification.md)
4. **開發歷史**: 查看 [開發歷史記錄](tafl-development-history.md)

## 📞 支援資源

- **測試腳本**: `/home/ct/RosAGV/agents/test_api_debug.py`
- **測試流程**: `/home/ct/RosAGV/app/config/tafl/flows/`
- **API 端點**: `http://localhost:8001/tafl/`
- **Web UI**: `http://localhost:8001/tafl/editor`

## 🔗 相關文檔
- TAFL 語言規格: docs-ai/knowledge/system/tafl/tafl-language-specification.md
- TAFL 編輯器規格: docs-ai/knowledge/system/tafl/tafl-editor-specification.md
- TAFL API 參考: docs-ai/knowledge/system/tafl/tafl-api-reference.md
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md

---

**文檔版本**: 2.0 (整合版)
**最後更新**: 2025-09-26
**難度等級**: ⭐ 初級至中級
**文檔狀態**: ✅ 完整