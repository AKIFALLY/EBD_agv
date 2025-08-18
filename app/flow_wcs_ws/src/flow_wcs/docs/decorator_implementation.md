# Flow Functions 自動註冊系統實現

## 概述
實現了基於 Python 裝飾器的自動函數註冊系統，取代原本手動維護的字串列表方式，確保函數定義和文檔始終同步。

## 實現內容

### 1. 裝飾器系統 (`decorators.py`)
- **@flow_function 裝飾器**: 自動註冊函數到全局註冊表
- **參數**:
  - `category`: 函數類別 (query, check, task, action, control)
  - `description`: 函數描述（中文）
  - `params`: 參數列表
  - `returns`: 返回值類型

### 2. 函數註冊
所有 FlowExecutor 中的函數都已加上 @flow_function 裝飾器：

#### Query 函數
- `query.locations` - 查詢位置資料
- `query.racks` - 查詢架台資料
- `query.tasks` - 查詢任務資料
- `query.agvs` - 查詢 AGV 資料

#### Check 函數
- `check.empty` - 檢查資料是否為空
- `check.rack_status` - 檢查架台狀態
- `check.task_exists` - 檢查任務是否存在
- `check.location_available` - 檢查位置是否可用
- `check.system_ready` - 檢查系統就緒狀態

#### Task 函數
- `task.create` - 建立新任務
- `task.update` - 更新任務狀態
- `task.assign` - 分配任務給 AGV
- `task.cancel` - 取消任務

#### Action 函數
- `action.rotate_rack` - 旋轉架台
- `action.notify` - 發送通知
- `action.log` - 記錄日誌
- `action.optimize_batch` - 最佳化任務批次

#### Control 函數
- `control.wait` - 等待指定時間
- `control.stop` - 停止流程執行
- `control.count` - 計算項目數量
- `control.switch` - Switch case 控制

### 3. API 端點 (`flow_functions.py`)
新增 `/api/flow/execute` 端點，支援單一函數測試執行：

```python
POST /api/flow/execute
{
    "function_name": "query.locations",
    "params": {"type": "room_inlet"},
    "variables": {}
}
```

### 4. JavaScript 整合
更新 `linearFlowDesigner.js` 使用新的執行端點：
- API 測試模式使用 `http://agvc.webapi/api/flow/execute`
- 透過 nginx proxy 呼叫 web_api 服務

## 優點

### 1. 自動同步
- 函數定義和文檔始終保持同步
- 消除手動維護錯誤

### 2. 單一真相來源
- 函數元數據直接在函數定義處聲明
- 避免重複定義

### 3. 易於擴展
- 新增函數只需加上裝飾器
- 自動出現在函數庫中

### 4. 類型安全
- 裝飾器確保必要的元數據存在
- 參數和返回值類型明確定義

## 測試

### 測試裝飾器系統
```bash
cd /app/flow_wcs_ws/src/flow_wcs
python3 test_decorator_functions.py
```

### 測試 API 端點
```bash
# 獲取函數庫
curl http://localhost:8000/api/flow/functions | jq

# 執行函數
curl -X POST http://localhost:8000/api/flow/execute \
  -H "Content-Type: application/json" \
  -d '{
    "function_name": "check.empty",
    "params": {"data": []},
    "variables": {}
  }'
```

### 測試 Linear Flow Designer
1. 開啟 http://localhost:8001/linear-flow
2. 選擇 API 測試模式
3. 創建包含函數的流程
4. 執行測試

## 維護指南

### 新增函數
1. 在函數定義上加上 @flow_function 裝飾器
2. 提供類別、描述、參數和返回值類型
3. 函數自動註冊到函數庫

### 範例
```python
@flow_function("query", "查詢設備狀態", ["device_id", "status"], "object")
def query_device(self, params: Dict) -> Dict:
    """查詢設備狀態"""
    device_id = params.get('device_id')
    status = params.get('status')
    # 實現邏輯
    return result
```

## 架構圖

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ Linear Flow     │────▶│  web_api         │────▶│  flow_wcs       │
│ Designer (JS)   │     │  /api/flow/*     │     │  FlowExecutor   │
└─────────────────┘     └──────────────────┘     └─────────────────┘
        │                       │                         │
        │                       │                         ▼
        ▼                       ▼                  ┌─────────────────┐
 ┌──────────────┐        ┌──────────────┐         │  @flow_function │
 │ API 測試模式  │        │ nginx proxy  │         │   Decorator     │
 └──────────────┘        └──────────────┘         └─────────────────┘
```

## 相關檔案
- `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/decorators.py` - 裝飾器系統
- `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py` - 函數實現
- `/app/web_api_ws/src/web_api/web_api/routers/flow_functions.py` - API 端點
- `/app/web_api_ws/src/agvcui/agvcui/static/js/linearFlowDesigner.js` - 前端整合