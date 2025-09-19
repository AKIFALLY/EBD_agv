# TAFL 快速入門指南

## 🎯 概述
本指南幫助您快速開始使用 TAFL (Task Automation Flow Language) 系統，從基本測試到進階開發。

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
- check: {condition: "$count > 0"}
- create: {target: "resource", data: {...}}
- update: {target: "resource", id: "123", data: {...}}

# 控制流程
- if: {condition: "...", then: [...], else: [...]}
- for: {each: "item", in: "$list", do: [...]}
- switch: {value: "$var", cases: {...}, default: [...]}

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

## 🔧 常見問題快速解決

### 問題：執行模式總是 simulation
```bash
# 檢查模組狀態
curl http://localhost:8001/tafl/status

# 重啟服務
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
  source /app/setup.bash && 
  agvc_source && 
  manage_web_api_launch restart
"
```

### 問題：無法連接到 TAFL Editor
```bash
# 檢查服務狀態
docker compose -f docker-compose.agvc.yml ps

# 查看日誌
docker compose -f docker-compose.agvc.yml logs agvc_server | tail -50
```

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

## 🎯 下一步

1. **深入學習**: 閱讀 [TAFL 語言規格](tafl-language-specification.md)
2. **API 開發**: 查看 [TAFL API Reference](tafl-api-reference.md)
3. **故障排除**: 參考 [故障排除指南](tafl-troubleshooting-guide.md)
4. **專案細節**: 了解 [實施專案記錄](tafl-implementation-project.md)

## 📞 支援資源

- **測試腳本**: `/home/ct/RosAGV/agents/test_api_debug.py`
- **測試流程**: `/home/ct/RosAGV/app/config/tafl/flows/`
- **API 端點**: `http://localhost:8001/tafl/`
- **Web UI**: `http://localhost:8001/tafl/editor`

---

**最後更新**: 2025-09-03  
**快速入門版本**: 1.0  
**難度等級**: ⭐ 初級