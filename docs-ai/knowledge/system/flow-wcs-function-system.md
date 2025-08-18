# Flow WCS 函數系統架構

## 🎯 適用場景
- 理解 Linear Flow v2 函數庫的運作機制
- 開發新的 Flow 函數和擴展功能
- 解決函數註冊和驗證相關問題
- 為 Linear Flow Designer 提供函數支援

## 📋 函數系統概述

Flow WCS 使用 **裝飾器模式** 自動註冊和管理所有可用函數，透過 `@flow_function` 裝飾器實現函數的自動發現和元數據管理。

### 系統架構
```
函數註冊系統架構
├── 裝飾器層 (decorators.py)
│   ├── @flow_function 裝飾器
│   ├── function_registry 全局註冊表
│   └── get_function_library() API
├── 執行層 (flow_executor.py)
│   ├── 函數實作 (使用 @flow_function)
│   ├── FlowExecutor.register_functions()
│   └── 函數執行引擎
├── API 層 (flow_wcs_node.py)
│   ├── /api/flow/functions 端點
│   └── 動態函數列表提供
└── UI 層 (linear_flow_designer.py)
    ├── 動態獲取函數列表
    ├── 函數快取機制
    └── 函數驗證和提示
```

## 🔧 裝飾器系統 (decorators.py)

### @flow_function 裝飾器
```python
@flow_function(category: str, description: str, params: List[str], 
               returns: str, defaults: Dict[str, Any], 
               also_register_as: Optional[str])
```

**功能**：
- 自動註冊函數到全局註冊表
- 保存函數元數據（參數、返回值、預設值）
- 支援別名註冊（如 foreach）
- 運行時動態發現

### 使用範例
```python
@flow_function("query", "查詢位置資料", 
               ["type", "rooms", "has_rack"], "array",
               defaults={"type": "room_inlet", "rooms": [1,2,3,4,5], "has_rack": True})
def query_locations(self, params: Dict) -> List[Dict]:
    """Query locations from database"""
    # 實作...
```

### 特殊函數註冊
```python
@flow_function("control", "迴圈遍歷", 
               ["items", "var", "steps"], "array",
               also_register_as="foreach")  # 額外註冊為 foreach
def foreach(self, params: Dict) -> List[Any]:
    """Foreach loop implementation"""
    # 實作...
```

## 📦 函數類別和清單

### Query 函數 (查詢類)
- `query.locations` - 查詢位置資料
- `query.racks` - 查詢架台資料
- `query.tasks` - 查詢任務資料
- `query.agvs` - 查詢 AGV 資料

### Check 函數 (檢查類)
- `check.empty` - 檢查資料是否為空
- `check.rack_status` - 檢查架台狀態
- `check.task_exists` - 檢查任務是否存在
- `check.location_available` - 檢查位置是否可用
- `check.system_ready` - 檢查系統就緒狀態

### Task 函數 (任務類)
- `task.create` - 建立新任務
- `task.update` - 更新任務狀態
- `task.assign` - 分配任務給 AGV
- `task.cancel` - 取消任務

### Action 函數 (動作類)
- `action.rotate_rack` - 旋轉架台
- `action.notify` - 發送通知
- `action.log` - 記錄日誌
- `action.optimize_batch` - 最佳化任務批次
- `action.analyze_priorities` - 分析任務優先級
- `action.find_optimal_agv` - 尋找最佳 AGV
- `action.recover` - 錯誤恢復
- `action.calculate_metrics` - 計算指標
- `action.alert` - 發送警報
- `action.cleanup` - 清理資源
- `action.generate_report` - 生成報告

### Control 函數 (控制類)
- `control.wait` - 等待指定時間
- `control.stop` - 停止流程執行
- `control.count` - 計算項目數量
- `control.switch` - Switch case 控制
- `control.update_variable` - 更新變數值
- `control.foreach` - 迴圈遍歷

### Special 函數 (特殊類)
- `foreach` - 迴圈遍歷（control.foreach 的別名）
- `parallel` - 平行執行分支

## 🔄 函數快取機制

### 快取流程
```
1. Linear Flow Designer 啟動
   ↓
2. 呼叫 Flow WCS API (/api/flow/functions)
   ↓
3. Flow WCS 返回裝飾器註冊的所有函數
   ↓
4. Designer 快取到 /app/config/wcs/flow_functions_cache.yaml
   ↓
5. 後續使用快取，除非 API 更新
```

### 快取檔案位置
- **運行時快取**: `/app/config/wcs/flow_functions_cache.yaml`
- **靜態參考**: `/app/config/wcs/flow_functions.yaml` (手動維護)

### 快取更新機制
- API 優先：優先從 Flow WCS API 獲取最新函數列表
- 快取備用：API 失敗時使用快取
- 自動更新：每次 Flow WCS 重啟時重新載入所有裝飾器

## 🚀 擴展新函數

### 步驟 1：在 flow_executor.py 添加函數實作
```python
@flow_function("category", "描述", ["param1", "param2"], "return_type",
               defaults={"param1": "default_value"})
def new_function(self, params: Dict) -> Any:
    """函數實作"""
    param1 = params.get('param1', 'default_value')
    param2 = params.get('param2')
    
    # 實作邏輯
    result = do_something(param1, param2)
    
    return result
```

### 步驟 2：測試函數註冊
```python
# 執行測試腳本
python3 /app/flow_wcs_ws/src/flow_wcs/test_decorator_functions.py

# 確認函數已註冊
from flow_wcs.decorators import list_registered_functions
print(list_registered_functions())
```

### 步驟 3：重啟 Flow WCS
```bash
# 重啟服務以載入新函數
docker compose -f docker-compose.agvc.yml restart agvc_server
```

### 步驟 4：驗證 Linear Flow Designer
- 重新載入 Linear Flow Designer 頁面
- 檢查函數列表是否包含新函數
- 測試新函數的執行

## 🔍 診斷和除錯

### 檢查已註冊函數
```python
# 在容器內執行
cd /app/flow_wcs_ws/src/flow_wcs
python3 -c "
from flow_wcs.decorators import list_registered_functions, get_function_library
print('註冊的函數:')
for func in list_registered_functions():
    print(f'  - {func}')
"
```

### 檢查函數快取
```bash
# 查看快取檔案
cat /app/config/wcs/flow_functions_cache.yaml | head -50

# 檢查快取時間
yq '.meta.updated_at' /app/config/wcs/flow_functions_cache.yaml
```

### API 端點測試
```bash
# 測試函數列表 API
curl http://localhost:8000/api/flow/functions | jq '.'

# 測試函數執行
curl -X POST http://localhost:8000/api/flow/execute \
  -H "Content-Type: application/json" \
  -d '{
    "function_name": "check.empty",
    "params": {"data": []},
    "variables": {}
  }'
```

## 💡 最佳實踐

### 函數命名規範
- 使用 `category.function_name` 格式
- 類別名稱：query, check, task, action, control
- 函數名稱：使用下劃線分隔的小寫字母

### 參數設計原則
- 必要參數放前面，可選參數放後面
- 提供合理的預設值
- 參數名稱要有描述性
- 使用型別提示

### 錯誤處理
- 函數內部處理異常
- 返回有意義的錯誤訊息
- 記錄錯誤日誌
- 提供錯誤恢復機制

## 📋 重要檔案位置

### 核心實作檔案
- **裝飾器系統**: `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/decorators.py`
- **函數實作**: `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py`
- **測試工具**: `/app/flow_wcs_ws/src/flow_wcs/test_decorator_functions.py`

### 配置和快取
- **靜態參考**: `/app/config/wcs/flow_functions.yaml` (手動維護)
- **運行時快取**: `/app/config/wcs/flow_functions_cache.yaml` (自動生成)

### API 和 UI
- **Flow WCS API**: `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_wcs_node.py`
- **Linear Flow Designer**: `/app/web_api_ws/src/agvcui/agvcui/routers/linear_flow_designer.py`

## 🔗 交叉引用
- **Flow Functions 自動化工具**: @docs-ai/operations/development/flow-functions-automation.md - 自動化開發工具完整指南
- Linear Flow v2 系統: @docs-ai/knowledge/system/flow-wcs-system.md
- Flow WCS 開發指導: @docs-ai/operations/development/flow-wcs-development.md
- WCS 系統架構: @docs-ai/knowledge/agv-domain/wcs-system-design.md
- 統一工具系統: @docs-ai/operations/tools/unified-tools.md
- 模組索引: @docs-ai/context/structure/module-index.md