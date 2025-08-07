# Flow Designer 完整使用手冊

## 🎯 學習目標

完成本手冊學習後，您將能夠：
- 熟練使用 Flow Designer 視覺化流程設計器
- 掌握 YAML DSL 語法和編寫技巧
- 運用 38個 WCS 函數進行複雜流程設計
- 實現視覺化流程圖與 YAML DSL 的雙向轉換
- 應用最佳實踐進行企業級流程設計

## 📋 先決條件

- 基本的 Web 瀏覽器操作能力
- AGV 和 WCS 系統基礎概念
- YAML 格式基本理解（推薦但非必需）

## 🚀 快速開始

### 系統存取
1. **開啟 Flow Designer**
   ```
   瀏覽器網址: http://localhost:8001/flows/create
   登入身份: 管理員或操作員
   ```

2. **界面概覽**
   ```
   ┌─────────────────────────────────────────────┐
   │ 🛠️ 工具列: 生成YAML | 載入YAML | 清空     │
   ├─────────────────────────────────────────────┤
   │ 📦 節點選板   │ 🎨 視覺化編輯器           │
   │ • Condition   │                           │
   │ • Logic       │     拖放節點到此處         │
   │ • Action      │     進行流程設計           │
   │ • Script      │                           │
   └─────────────────────────────────────────────┘
   ```

3. **第一個流程**
   - 從節點選板拖拽一個 `check_agv_rotation_flow` 節點
   - 添加一個 `get_room_inlet_point` 邏輯節點
   - 連接兩個節點
   - 點擊「生成 YAML」查看 DSL 代碼

## 📚 詳細功能指導

### 1. 視覺化流程設計器

#### 1.1 節點選板使用
**Condition Nodes (條件節點)**
```
用途: 判斷條件和決策邏輯
主要函數:
• check_agv_rotation_flow - 檢查 AGV 旋轉流程
• is_agv_at_location - 檢查 AGV 是否在指定位置
• check_rack_availability - 檢查架台可用性
• validate_task_requirements - 驗證任務需求
• is_path_clear - 檢查路徑是否暢通
• check_equipment_status - 檢查設備狀態
• verify_safety_conditions - 驗證安全條件
• is_maintenance_required - 檢查是否需要維護
• check_system_resources - 檢查系統資源
```

**Logic Nodes (邏輯節點)**
```
用途: 數據處理和邏輯運算
主要函數:
• get_room_inlet_point - 獲取房間入口點
• get_agv_current_location - 獲取 AGV 當前位置
• calculate_optimal_path - 計算最優路徑
• find_nearest_charging_station - 尋找最近充電站
• get_location_metadata - 獲取位置元數據
```

**Action Nodes (動作節點)**
```
用途: 執行具體操作和任務
主要函數:
• create_task_from_decision - 根據決策創建任務
• update_task_status - 更新任務狀態
• assign_task_to_agv - 分配任務給 AGV
• cancel_task - 取消任務
```

**Script Nodes (腳本節點)**
```
用途: 控制流程和變數操作
主要函數:
• if_else - 條件分支控制
• for_loop - 循環控制
• while_loop - 條件循環
• variable_assignment - 變數賦值
```

#### 1.2 節點操作詳解

**添加節點**
1. 從節點選板選擇所需節點類型
2. 拖拽到編輯器區域
3. 節點自動顯示預設配置

**配置節點**
1. 雙擊節點開啟配置面板
2. 設定輸入參數
3. 配置輸出變數
4. 點擊「確認」保存設定

**連接節點**
1. 點擊源節點的輸出接點
2. 拖拽到目標節點的輸入接點
3. 連接線自動建立
4. 支援多輸入多輸出連接

**刪除和編輯**
```
刪除節點: 選中節點 → 按 Delete 鍵
刪除連接: 選中連接線 → 按 Delete 鍵
移動節點: 拖拽節點到新位置
複製節點: Ctrl+C 複製，Ctrl+V 貼上
```

### 2. YAML DSL 語法指南

#### 2.1 基本結構
```yaml
# 流程標識
flow_id: "rack_rotation_inlet_flow"
description: "架台旋轉入口流程"

# 全域變數定義
variables:
  agv_id: "agv01"
  rack_id: "rack_001"
  inlet_point: ""

# 流程步驟
steps:
  - step: 1
    function: "check_agv_rotation_flow"
    type: "condition_nodes"
    source: "unified_decision_engine"
    inputs:
      agv_id: "${agv_id}"
    outputs:
      rotation_status: "rotation_result"
    
  - step: 2
    function: "get_room_inlet_point"
    type: "logic_nodes"
    source: "location_manager"
    inputs:
      room_id: "production_room_a"
    outputs:
      inlet_point: "inlet_coordinates"
```

#### 2.2 變數系統
**變數定義**
```yaml
variables:
  # 字符串變數
  agv_id: "agv01"
  
  # 數值變數
  max_speed: 1.5
  timeout: 300
  
  # 布林變數
  enable_safety: true
  
  # 列表變數
  available_racks: ["rack_001", "rack_002", "rack_003"]
  
  # 物件變數
  agv_config:
    type: "cargo_mover"
    capacity: 500
    battery_level: 85
```

**變數引用**
```yaml
# 使用 ${變數名} 引用變數
steps:
  - step: 1
    function: "is_agv_at_location"
    inputs:
      agv_id: "${agv_id}"
      target_location: "${target_position}"
      tolerance: "${position_tolerance}"
```

#### 2.3 控制結構
**條件分支**
```yaml
- step: 3
  function: "if_else"
  type: "script_nodes"
  condition: "${rotation_result} == 'success'"
  if_true:
    - function: "create_task_from_decision"
      type: "action_nodes"
      inputs:
        task_type: "move_to_inlet"
        target: "${inlet_coordinates}"
  if_false:
    - function: "update_task_status"
      type: "action_nodes"
      inputs:
        status: "failed"
        reason: "rotation_check_failed"
```

**循環控制**
```yaml
- step: 4
  function: "for_loop"
  type: "script_nodes"
  loop_variable: "rack_id"
  loop_items: "${available_racks}"
  loop_body:
    - function: "check_rack_availability"
      type: "condition_nodes"
      inputs:
        rack_id: "${rack_id}"
```

### 3. 雙向轉換功能

#### 3.1 視覺化流程圖轉 YAML DSL
**操作步驟**
1. 設計完成視覺化流程圖
2. 點擊「生成 YAML」按鈕
3. 系統自動生成對應的 YAML DSL 代碼
4. 點擊「下載」保存為 .yaml 檔案

**轉換特性**
- ✅ 完整保留節點配置和連接關係
- ✅ 自動提取和生成變數定義
- ✅ 智能推斷變數類型和預設值
- ✅ 生成符合語法規範的格式化 YAML
- ✅ 包含完整的元數據和註解

#### 3.2 YAML DSL 轉視覺化流程圖
**操作步驟**
1. 點擊「載入 YAML」按鈕
2. 選擇本地 .yaml 檔案或粘貼 YAML 內容
3. 系統自動解析並生成視覺化流程圖
4. 檢查和調整節點位置及連接

**解析特性**
- ✅ 智能識別節點類型和函數映射
- ✅ 自動重建節點連接關係
- ✅ 恢復變數定義和參數配置
- ✅ 錯誤檢測和友好提示
- ✅ 支援部分載入和增量更新

### 4. 實際業務場景範例

#### 4.1 OPUI 叫車流程
**業務場景**: 操作員透過 OPUI 界面叫車，系統自動分配和調度 AGV

**視覺化流程設計**
```
[開始] → [檢查OPUI請求] → [驗證任務需求] → [尋找可用AGV] 
    ↓
[分配任務] → [創建移動任務] → [更新任務狀態] → [結束]
```

**對應 YAML DSL**
```yaml
flow_id: "opui_call_agv_flow"
description: "OPUI 叫車流程"

variables:
  opui_request_id: ""
  agv_id: ""
  pickup_location: ""
  delivery_location: ""

steps:
  - step: 1
    function: "validate_task_requirements"
    type: "condition_nodes"
    source: "unified_decision_engine"
    inputs:
      request_id: "${opui_request_id}"
      pickup: "${pickup_location}"
      delivery: "${delivery_location}"
    outputs:
      validation_result: "task_valid"
  
  - step: 2
    function: "if_else"
    type: "script_nodes"
    condition: "${task_valid} == true"
    if_true:
      - function: "assign_task_to_agv"
        type: "action_nodes"
        source: "unified_task_manager"
        inputs:
          task_type: "transport"
          pickup: "${pickup_location}"
          delivery: "${delivery_location}"
        outputs:
          assigned_agv: "agv_id"
    
  - step: 3
    function: "create_task_from_decision"
    type: "action_nodes"
    source: "unified_task_manager"
    inputs:
      agv_id: "${agv_id}"
      task_details: "pickup_and_delivery"
    outputs:
      task_id: "created_task_id"
```

#### 4.2 NG 料架處理流程
**業務場景**: 檢測到 NG (不良品) 料架，系統自動進行隔離和處理

**流程邏輯**
1. 檢測料架狀態
2. 判斷是否為 NG 料架
3. 計算隔離位置
4. 分配 AGV 進行搬運
5. 更新料架狀態為已隔離

**YAML DSL 範例**
```yaml
flow_id: "ng_rack_handling_flow"
description: "NG 料架處理流程"

variables:
  rack_id: ""
  rack_status: ""
  isolation_area: "ng_isolation_zone"
  assigned_agv: ""

steps:
  - step: 1
    function: "check_equipment_status"
    type: "condition_nodes"
    source: "unified_decision_engine"
    inputs:
      equipment_id: "${rack_id}"
      status_type: "quality_check"
    outputs:
      quality_status: "rack_status"
  
  - step: 2
    function: "if_else"
    type: "script_nodes"
    condition: "${rack_status} == 'NG'"
    if_true:
      - function: "get_location_metadata"
        type: "logic_nodes"
        source: "location_manager"
        inputs:
          area_type: "isolation"
          criteria: "ng_storage"
        outputs:
          target_location: "isolation_point"
      
      - function: "assign_task_to_agv"
        type: "action_nodes"
        source: "unified_task_manager"
        inputs:
          task_type: "isolation_transport"
          source_rack: "${rack_id}"
          target_location: "${isolation_point}"
        outputs:
          assigned_agv: "agv_id"
```

#### 4.3 批量料架輪換流程
**業務場景**: 生產線需要大量料架輪換，系統智能調度多台 AGV 協同作業

**設計要點**
- 批量處理邏輯
- 多 AGV 協調
- 優先級管理
- 路徑優化

**YAML DSL 範例**
```yaml
flow_id: "batch_rack_rotation_flow"
description: "批量料架輪換流程"

variables:
  rack_batch: []
  available_agvs: []
  rotation_schedule: []

steps:
  - step: 1
    function: "for_loop"
    type: "script_nodes"
    loop_variable: "current_rack"
    loop_items: "${rack_batch}"
    loop_body:
      - function: "check_rack_availability"
        type: "condition_nodes"
        source: "unified_decision_engine"
        inputs:
          rack_id: "${current_rack}"
        outputs:
          availability: "rack_available"
      
      - function: "if_else"
        type: "script_nodes"
        condition: "${rack_available} == true"
        if_true:
          - function: "calculate_optimal_path"
            type: "logic_nodes"
            source: "location_manager"
            inputs:
              source: "${current_rack}"
              destination: "production_line_a"
            outputs:
              optimal_route: "calculated_path"
          
          - function: "create_task_from_decision"
            type: "action_nodes"
            source: "unified_task_manager"
            inputs:
              task_type: "batch_rotation"
              rack_id: "${current_rack}"
              path: "${calculated_path}"
```

## 💡 實用技巧

### 設計技巧
1. **模組化設計**: 將複雜流程分解為可重用的子流程
2. **變數命名**: 使用描述性的變數名，如 `agv_battery_level` 而非 `level`
3. **錯誤處理**: 在關鍵步驟後添加錯誤檢查和處理邏輯
4. **文檔註解**: 在 YAML 中添加註解說明複雜邏輯

### 效能優化
1. **批量操作**: 盡可能使用批量處理減少系統呼叫
2. **條件優化**: 將最可能的條件放在前面
3. **變數作用域**: 適當使用局部變數減少記憶體使用
4. **並行處理**: 利用系統的並行處理能力

### 除錯技巧
1. **分步測試**: 先測試單個節點，再測試完整流程
2. **日誌追蹤**: 在關鍵步驟添加日誌輸出
3. **視覺化驗證**: 用視覺化流程圖驗證邏輯正確性
4. **增量開發**: 逐步添加功能，每次添加都進行測試

## 🧪 實作練習

### 練習 1: 基礎流程設計
**目標**: 創建一個簡單的 AGV 移動流程
**步驟**:
1. 添加 `is_agv_at_location` 條件節點
2. 添加 `get_room_inlet_point` 邏輯節點
3. 添加 `create_task_from_decision` 動作節點
4. 連接節點並配置參數
5. 生成 YAML DSL 並檢查結果

### 練習 2: 條件分支流程
**目標**: 實作帶有條件判斷的複雜流程
**要求**:
- 使用 `if_else` 腳本節點
- 包含至少兩個分支
- 每個分支包含不同的處理邊輯

### 練習 3: 循環處理流程
**目標**: 處理多個料架的批量操作
**要求**:
- 使用 `for_loop` 腳本節點
- 對每個料架執行相同操作
- 包含循環內的條件判斷

### 練習 4: 完整業務流程
**目標**: 實作一個完整的生產業務流程
**要求**:
- 結合多種節點類型
- 包含錯誤處理機制
- 使用變數和控制結構
- 生成完整的 YAML DSL

## ❓ 常見問題

### Q1: 為什麼我的節點無法連接？
**A**: 檢查以下幾點：
- 輸出和輸入的資料類型是否匹配
- 節點是否已正確配置
- 是否存在循環依賴

### Q2: 生成的 YAML DSL 無法載入？
**A**: 常見原因：
- YAML 語法錯誤（縮排、引號等）
- 缺少必要的欄位
- 變數引用錯誤

### Q3: 如何優化大型流程的效能？
**A**: 建議方法：
- 使用批量處理減少節點數量
- 合理使用變數快取中間結果
- 分解大流程為多個子流程

### Q4: 如何實作複雜的條件判斷？
**A**: 可以採用：
- 組合多個條件節點
- 使用巢狀的 `if_else` 結構
- 利用邏輯節點進行條件預處理

## 🔗 相關資源

- **系統架構文檔**: 了解 Flow Designer 技術架構
- **WCS 函數參考**: 38個 WCS 函數詳細說明
- **YAML DSL 語法規範**: 完整的 DSL 語法參考
- **最佳實踐指南**: 企業級流程設計模式
- **故障排除手冊**: 常見問題和解決方案
- **開發者文檔**: 系統擴展和定制指導

---

📝 **文檔版本**: v1.0  
📅 **更新日期**: 2024-01-15  
👥 **目標用戶**: 系統管理員、流程設計師、操作員