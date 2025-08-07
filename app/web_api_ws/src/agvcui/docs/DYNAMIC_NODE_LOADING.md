# Flow Designer V2 動態節點載入系統

## 概述
Flow Designer V2 現在支援動態節點載入功能，可以從 YAML 流程檔案中自動提取節點定義並顯示在節點選板中。這解決了原本節點選板只顯示硬編碼節點的問題。

## 問題背景
用戶提出的問題：
> "我現在的節點選板裡面的節點是怎麼來的？為什麼我的 YAML 檔裡面的節點沒有在節點選板裡？"

原因分析：
- 原本的節點選板使用硬編碼的 `nodeDefinitions`（在 JavaScript 中定義）
- 這些預設節點是通用的，如 `check_rack_a_space`、`check_rack_b_space` 等
- 實際 YAML 檔案中的節點（如 `get_room_outlet_locations`、`check_pending_tasks` 等）不在選板中

## 解決方案

### 1. 動態節點載入架構

```
YAML 檔案 → API 端點 → JavaScript 載入 → 節點選板更新
```

### 2. 實現細節

#### 後端 API 端點
新增 `/api/nodes/definitions` 端點，從所有 YAML 檔案中提取節點定義：

```python
@router.get("/api/nodes/definitions")
async def get_node_definitions():
    """獲取所有可用的節點定義（從 YAML 文件中提取）"""
    # 掃描 /app/config/wcs/flows 目錄
    # 從每個 YAML 檔案的 nodes 區段提取節點定義
    # 按類型分組返回
```

#### 前端載入機制
在 `flowDesignerV2_collapsible.js` 中新增動態載入功能：

```javascript
async loadNodeDefinitions() {
    // 1. 嘗試從 API 載入節點定義
    // 2. 如果失敗，從當前流程 YAML 提取
    // 3. 如果都失敗，使用預設定義
}
```

### 3. 載入優先順序

1. **API 載入**（最優先）
   - 從 `/api/nodes/definitions` 獲取所有 YAML 檔案中的節點
   - 提供最完整的節點庫

2. **當前流程提取**（備用）
   - 如果 API 不可用，從當前載入的流程中提取節點
   - 確保至少能使用當前流程的節點

3. **預設定義**（最後備用）
   - 使用硬編碼的預設節點定義
   - 確保系統總是有可用的節點

## 使用方式

### 開發者使用
1. 在 YAML 檔案中定義節點時，確保包含完整資訊：
```yaml
nodes:
  - id: my_custom_node
    type: condition
    name: 我的自訂節點
    description: 這是一個自訂節點
    function: my_custom_function
    inputs:
      data: { type: "object", description: "輸入資料" }
    outputs:
      result: { type: "boolean", description: "處理結果" }
```

2. 節點會自動出現在選板的對應分類中

### 用戶使用
1. 開啟 Flow Designer
2. 節點選板會自動載入所有可用節點
3. 從 YAML 檔案定義的節點會顯示在選板中
4. 可以拖拽這些節點到畫布上使用

## 優點

1. **動態性**：不需要修改 JavaScript 代碼即可新增節點
2. **一致性**：選板中的節點與實際使用的節點保持一致
3. **可擴展性**：輕鬆新增新的節點類型
4. **維護性**：節點定義集中在 YAML 檔案中管理

## 測試

### 測試頁面
開啟 `/tests/test_dynamic_nodes.html` 可以測試動態節點載入功能：
- 查看從 YAML 提取的節點
- 比較預設節點與動態節點
- 驗證節點資訊完整性

### 驗證步驟
1. 在 YAML 檔案中新增一個節點定義
2. 重新載入 Flow Designer
3. 檢查節點選板是否顯示新節點
4. 拖拽節點到畫布驗證功能

## 相關檔案

- `/app/web_api_ws/src/agvcui/agvcui/routers/flow_designer.py` - API 端點實現
- `/app/web_api_ws/src/agvcui/agvcui/static/js/flowDesignerV2_collapsible.js` - 前端載入邏輯
- `/app/config/wcs/flows/*.yaml` - 節點定義來源
- `/app/web_api_ws/src/agvcui/tests/test_dynamic_nodes.html` - 測試頁面

## 未來改進

1. **節點庫管理**：建立專門的節點庫管理界面
2. **節點模板**：提供節點模板快速創建新節點
3. **節點驗證**：自動驗證節點定義的完整性
4. **節點文檔**：自動生成節點使用文檔
5. **節點版本控制**：支援節點定義的版本管理