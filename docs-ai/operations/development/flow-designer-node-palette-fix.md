# Flow Designer 節點選板顯示修復指南

## 🎯 問題描述
Flow Designer 節點選板中的節點未正確顯示，儘管 `node-types.js` 已正確載入且 `window.FlowDesigner` 變數可用。

## 🔍 根本原因分析

### 問題定位
1. **載入機制正常**: `node-types.js` 正確載入，`window.FlowDesigner` 物件可用
2. **節點定义完整**: node-types.js 包含完整的 38 個節點定義
3. **選板容器存在**: HTML 模板中的選板容器 (#condition-nodes, #action-nodes, #logic-nodes, #script-nodes) 正常
4. **問題出現點**: `loadNodeConfigurations()` 方法中的配置轉換邏輯

### 技術原因
原始實現使用了複雜的配置轉換流程：
```javascript
// ❌ 原始實現 - 複雜的配置轉換
this.nodeConfigs = {
    condition_nodes: { condition_nodes: window.FlowDesigner.CONDITION_NODES },
    logic_nodes: { logic_nodes: window.FlowDesigner.LOGIC_NODES },
    action_nodes: { action_nodes: window.FlowDesigner.ACTION_NODES },
    script_nodes: { script_nodes: window.FlowDesigner.SCRIPT_NODES }
};
// 然後調用 populateNodeTypes() 進行複雜轉換
```

這種方式導致：
1. 不必要的數據結構嵌套
2. `populateNodeTypes()` 方法中的配置解析錯誤
3. 節點屬性在轉換過程中丟失

## ✅ 解決方案

### 核心修復
採用直接賦值策略，跳過複雜的配置轉換：

```javascript
// ✅ 修復後的實現 - 直接賦值策略
if (typeof window.FlowDesigner !== 'undefined') {
    console.log('✅ 使用 node-types.js 中的節點定義');
    
    // 🔧 修復: 直接將節點類型設置到 nodeTypes，避免複雜的配置轉換
    this.nodeTypes = {
        ...window.FlowDesigner.CONDITION_NODES,
        ...window.FlowDesigner.LOGIC_NODES,
        ...window.FlowDesigner.ACTION_NODES,
        ...window.FlowDesigner.SCRIPT_NODES
    };
    
    console.log('  - 總計:', Object.keys(this.nodeTypes).length, '個節點類型');
    
    // 直接返回，跳過 populateNodeTypes 調用
    console.log('✅ 節點配置載入完成 (直接模式)');
    return;
}
```

### 選板分類修復
更新節點分類邏輯以正確處理 `node-types.js` 中的 `category` 屬性：

```javascript
// 🔧 修復: 根據節點類別決定容器 - 使用 node-types.js 中的 category 屬性
const nodeCategory = nodeType.category || nodeType.type;

switch (nodeCategory) {
    case 'condition':
    case 'input':
        containerId = 'condition-nodes'; // 條件/輸入節點 → 條件節點區域
        break;
    case 'action':
    case 'output':
    case 'process':
        containerId = 'action-nodes';     // 動作/輸出/處理節點 → 動作節點區域
        break;
    case 'logic':
    case 'control':
        containerId = 'logic-nodes';      // 邏輯/控制節點 → 邏輯節點區域
        break;
    case 'script':
    case 'storage':
        containerId = 'script-nodes';     // 腳本/存儲節點 → 腳本節點區域
        break;
    default:
        console.warn(`⚠️ 未知節點類型: ${nodeCategory} (nodeId: ${nodeId})，放入條件節點區域`);
        containerId = 'condition-nodes';  // 預設放在條件節點區域
}
```

### 節點項目修復
修復節點選板項目的屬性引用：

```javascript
// 修復節點類型和類別的正確引用
item.dataset.nodeType = nodeType.category || nodeType.type;
item.dataset.nodeCategory = nodeType.category;

// 修復節點標籤顯示
${nodeType.dslType || nodeType.category || 'node'}
```

## 📊 修復效果

### 預期結果
- **條件節點區域**: 顯示 8 個條件判斷節點 (check_agv_rotation_flow, check_rack_rotation_flow 等)
- **邏輯節點區域**: 顯示 5 個邏輯處理節點 (get_room_inlet_point, get_inlet_rotation_point 等)
- **動作節點區域**: 顯示 4 個動作執行節點 (create_task_from_decision, update_machine_parking_status 等)
- **腳本節點區域**: 顯示 4 個腳本控制節點 (if_else, for_loop, set_variable, get_variable)

### 日誌輸出
```
🚀 開始 Flow Designer 初始化...
📋 載入節點配置...
✅ 使用 node-types.js 中的節點定義
📊 載入統計:
  - 條件節點: 8
  - 邏輯節點: 5
  - 動作節點: 4
  - 腳本節點: 4
  - 總計: 21 個節點類型
✅ 節點配置載入完成 (直接模式)
🎨 創建節點選板，節點總數: 21
✅ 節點選板創建完成
📊 節點分佈統計:
  - condition-nodes: 8 個節點
  - logic-nodes: 5 個節點
  - action-nodes: 4 個節點
  - script-nodes: 4 個節點
```

## 🔧 相關檔案

### 主要修改檔案
- **flowDesignerPage.js**: 
  - `loadNodeConfigurations()` 方法: 直接賦值策略
  - `createNodePalette()` 方法: 修復節點分類邏輯
  - `createNodePaletteItem()` 方法: 修復屬性引用

### 依賴檔案 (無需修改)
- **node-types.js**: 完整的節點定義，按 DSL 系統分類
- **flow_designer.html**: HTML 模板，包含選板容器
- **flowDesignerPage.css**: 樣式定義

## 🧪 測試驗證

### 瀏覽器測試
1. 開啟 Flow Designer: `http://localhost:8001/flows/create`
2. 檢查瀏覽器控制台日誌
3. 驗證四個節點選板區域是否正確顯示節點
4. 測試節點拖拽和添加功能

### 功能測試
```javascript
// 在瀏覽器控制台執行
console.log('節點類型數量:', Object.keys(window.flowDesigner.nodeTypes).length);
console.log('條件節點:', Object.keys(window.FlowDesigner.CONDITION_NODES));
console.log('邏輯節點:', Object.keys(window.FlowDesigner.LOGIC_NODES));
console.log('動作節點:', Object.keys(window.FlowDesigner.ACTION_NODES));
console.log('腳本節點:', Object.keys(window.FlowDesigner.SCRIPT_NODES));
```

## 💡 最佳實踐

### 載入策略優化
1. **直接使用已載入資源**: 避免不必要的配置轉換
2. **保持數據結構一致性**: 直接使用 `node-types.js` 的原始結構
3. **簡化錯誤處理**: 減少轉換環節，降低出錯機率

### 開發指導
1. **節點定義**: 在 `node-types.js` 中添加新節點
2. **自動分類**: 根據 `category` 屬性自動分類到對應選板區域
3. **視覺一致性**: 使用 `NODE_STYLES` 配置確保視覺一致

## 🔗 相關文檔
- Phase 4.2 Performance Optimization: @docs-ai/operations/development/flow-designer-phase4-2-performance-optimization.md
- Flow Designer 系統架構: @docs-ai/knowledge/system/flow-designer-architecture.md
- Node Types 定義指南: `app/web_api_ws/src/agvcui/agvcui/static/js/flow-designer/node-types.js`