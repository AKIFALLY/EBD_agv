# Flow Designer YAML DSL 生成功能示範

## 🎯 功能概述
展示如何使用 Flow Designer Phase 3.2 完成的 YAML DSL 代碼生成功能，將視覺化流程圖轉換為可執行的 YAML DSL 代碼。

## 🚀 使用方法

### 1. 開啟 Flow Designer
```bash
# 啟動 AGVC 系統
agvc_start

# 開啟瀏覽器，導航至：
http://localhost:8001/flows
```

### 2. 創建視覺化流程

#### 步驟 1: 新建流程
- 點擊「新建流程」按鈕
- 輸入流程名稱，例如：「AGV 旋轉流程檢查」
- 輸入流程描述

#### 步驟 2: 拖拽節點
從左側節點選板拖拽以下節點到編輯器：

1. **條件節點**: `check_agv_rotation_flow`
   - 來源: unified_decision_engine
   - 功能: 檢查 AGV 是否需要執行旋轉流程

2. **邏輯節點**: `get_room_inlet_point` 
   - 來源: location_manager
   - 功能: 取得房間入口點

3. **動作節點**: `create_task_from_decision`
   - 來源: unified_task_manager
   - 功能: 從決策創建任務

#### 步驟 3: 連接節點
- 將條件節點的輸出連接到邏輯節點的輸入
- 將邏輯節點的輸出連接到動作節點的輸入

#### 步驟 4: 配置參數
點擊各節點設置參數：
- **check_agv_rotation_flow**: room_id = 1, agv_id = "agv01"
- **get_room_inlet_point**: room_id = 1
- **create_task_from_decision**: priority = 5

### 3. 生成 YAML DSL

#### 點擊生成按鈕
- 點擊工具欄中的紅色「生成 YAML DSL」按鈕
- 系統會自動分析流程圖並生成 YAML 代碼
- 生成的檔案會自動下載

## 📄 生成範例

### 輸入：視覺化流程圖
```
[條件節點: check_agv_rotation_flow] 
           ↓
[邏輯節點: get_room_inlet_point]
           ↓  
[動作節點: create_task_from_decision]
```

### 輸出：YAML DSL 代碼
```yaml
# AGV 旋轉流程檢查
# 描述: 由 Flow Designer 生成的 YAML DSL 流程
# 版本: 1.0
# 創建時間: 2024-01-15T10:30:00.000Z
# 生成工具: Flow Designer v3.2

# 變數定義
variables:
  room_id:
    type: integer
    value: 1
    description: "房間ID"
  agv_id:
    type: string
    value: "agv01"
    description: "AGV識別碼"
  priority:
    type: integer
    value: 5
    description: "任務優先級"

# 步驟定義
steps:
  - step: 1
    name: "AGV旋轉流程檢查"
    description: "檢查AGV是否需要執行旋轉流程"
    type: condition_nodes
    function: check_agv_rotation_flow
    parameters:
      room_id: ${room_id}
      agv_id: ${agv_id}
    outputs:
      - decisions

  - step: 2
    name: "取得房間入口點"
    description: "根據房間ID獲取入口停靠點"
    type: logic_nodes
    function: get_room_inlet_point
    parameters:
      room_id: ${room_id}
    assign_to: inlet_point

  - step: 3
    name: "從決策創建任務"
    description: "根據決策結果創建WCS任務"
    type: action_nodes
    function: create_task_from_decision
    parameters:
      decision: ${decisions}
      priority: ${priority}
    result_handler: task_creation_result
```

## 🔧 高級功能

### 1. 智能變數提取
系統會自動：
- 從節點參數中識別變數
- 推斷變數類型（integer, string, boolean, etc.）
- 生成變數定義區塊

### 2. 四種節點類型支援
- **condition_nodes**: 條件判斷節點，包含 outputs 定義
- **logic_nodes**: 邏輯處理節點，包含 assign_to 變數分配
- **action_nodes**: 動作執行節點，包含 result_handler 處理器
- **script_nodes**: 腳本控制節點，支援分支和循環結構

### 3. 複雜控制結構
對於腳本節點（if_else, for_loop），系統會生成嵌套的 YAML 結構：

```yaml
  - step: 4
    name: "條件分支"
    type: script_nodes
    control: if_else
    parameters:
      condition: ${some_condition}
      if_branch:
        - step: 1
          function: action_when_true
          parameters:
            param1: "value1"
      else_branch:
        - step: 1
          function: action_when_false
          parameters:
            param1: "value2"
```

## 🎨 生成功能特色

### 1. 完整的檔案結構
- ✅ 檔案頭部註解（流程名稱、描述、版本、時間戳）
- ✅ 變數定義區塊（類型、值、描述）
- ✅ 步驟執行區塊（順序、函數、參數）

### 2. 智能格式化
- ✅ 自動字串轉義和引號處理
- ✅ 變數引用語法（`${variable}`）
- ✅ 類型感知的值格式化
- ✅ 標準化的 YAML 縮排

### 3. 錯誤處理
- ✅ 空流程檢測和提示
- ✅ 無效節點類型警告
- ✅ 參數驗證失敗處理
- ✅ 友好的錯誤通知

### 4. 用戶體驗
- ✅ 一鍵生成和下載
- ✅ 進度指示和狀態通知
- ✅ 自動檔名生成
- ✅ 瀏覽器相容性

## 🔗 整合測試

### 生成的 YAML 可以直接用於：
1. **Simple WCS 系統**: 將檔案放入 `flows/` 目錄
2. **DSL 執行引擎**: 通過 YAMLDSLExecutor 執行
3. **OPUI 叫車**: 作為 OPUI 流程配置使用
4. **系統測試**: 用於自動化測試場景

### 驗證生成的 YAML：
```bash
# 檢查 YAML 語法
python3 -c "import yaml; yaml.safe_load(open('generated_flow.yaml'))"

# 在 Simple WCS 中測試
cd /app/simple_wcs_ws
python3 src/simple_wcs/simple_wcs/test_dsl_execution.py --file generated_flow.yaml
```

## 📈 下一步發展

### Phase 3.3 預期功能：
- **YAML → Visual**: 反向轉換，將 YAML DSL 載入為視覺化流程圖
- **即時同步**: 雙向即時同步編輯
- **預覽窗格**: 側邊欄顯示即時 YAML 預覽
- **驗證系統**: 完整的流程邏輯驗證

Flow Designer Phase 3.2 YAML DSL 代碼生成功能為可視化流程設計提供了強大的代碼生成能力，大幅提升了開發效率和流程管理的標準化程度。