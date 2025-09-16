# TAFL 故障排除指南

## 🎯 適用場景
- TAFL 執行模式問題診斷
- Router 衝突解決
- 模組載入失敗處理
- 資料庫連接問題

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

## 🔗 相關文檔
- TAFL 實施專案: @docs-ai/knowledge/system/tafl-implementation-project.md
- TAFL 編輯器規格: @docs-ai/knowledge/system/tafl-editor-specification.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md

---

**最後更新**: 2025-09-03  
**文檔狀態**: ✅ **完整**