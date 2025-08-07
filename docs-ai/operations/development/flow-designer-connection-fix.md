# Flow Designer 節點連接修復指南

## 🎯 問題描述
Flow Designer 中節點的 socket 無法連接到另一個節點的 socket，出現錯誤：
```
Uncaught Error: 找不到節點元素: [object HTMLDivElement] 或 [object HTMLDivElement]
    at WcsFlowDesigner.createConnection (flowDesignerPage.js?v=3.2.0:2944:19)
```

## 🔍 根本原因分析

### 問題定位
程式碼中存在兩個同名的 `createConnection` 方法，但期望不同的參數類型：

1. **第1188行的方法** (正確的 socket 連接方法):
   ```javascript
   createConnection(startSocket, endSocket) {
       const sourceNodeId = startSocket.dataset.nodeId;
       const targetNodeId = endSocket.dataset.nodeId;
       const sourceKey = startSocket.dataset.socketKey;
       const targetKey = endSocket.dataset.socketKey;
       // ... 處理 socket 連接邏輯
   }
   ```

2. **第2964行的方法** (視覺連接方法):
   ```javascript
   createConnection(sourceNodeId, targetNodeId) {  // ❌ 同名方法衝突
       const sourceElement = document.getElementById(sourceNodeId);
       const targetElement = document.getElementById(targetNodeId);
       // ... 處理視覺連接線
   }
   ```

### 技術原因
- **方法覆蓋**: 第2964行的方法覆蓋了第1188行的方法
- **參數類型錯誤**: 當 socket 事件調用時，傳入 HTML 元素到期望字串ID的方法
- **錯誤消息**: `[object HTMLDivElement]` 表明傳入了DOM元素而非字串

## ✅ 解決方案

### 核心修復
重命名第2964行的方法以避免衝突：

```javascript
// ✅ 修復後: 重命名視覺連接方法
createVisualConnection(sourceNodeId, targetNodeId) {
    const sourceElement = document.getElementById(sourceNodeId);
    const targetElement = document.getElementById(targetNodeId);
    
    if (!sourceElement || !targetElement) {
        throw new Error(`找不到節點元素: ${sourceNodeId} 或 ${targetNodeId}`);
    }
    
    // 創建 SVG 連接線
    const svg = this.getOrCreateConnectionSvg();
    const connection = this.createConnectionLine(sourceElement, targetElement);
    // ... 其餘邏輯保持不變
}
```

### 相關調用更新
同時更新所有調用重命名方法的地方：

1. **rebuildConnections 方法**:
   ```javascript
   // 更新前: this.createConnection(sourceNodeId, targetNodeId);
   // 更新後:
   this.createVisualConnection(sourceNodeId, targetNodeId);
   ```

2. **createConnectionsProgressively 方法**:
   ```javascript
   // 更新前: this.flowDesigner.createConnection(conn.source, conn.target);
   // 更新後:
   this.flowDesigner.createVisualConnection(conn.source, conn.target);
   ```

## 📊 修復後的方法結構

### Socket 連接方法 (保持不變)
```javascript
createConnection(startSocket, endSocket) {
    // 處理實際的 ReteJS 節點連接
    // 參數: DOM socket 元素
    // 用途: 用戶拖拽連接 socket 時調用
}
```

### 視覺連接方法 (已重命名)
```javascript
createVisualConnection(sourceNodeId, targetNodeId) {
    // 處理 SVG 視覺連接線繪製
    // 參數: 節點ID字串
    // 用途: YAML 載入或程序化創建連接時調用
}
```

## 🧪 測試驗證

### 手動測試步驟
1. **開啟 Flow Designer**: `http://localhost:8001/flows/create`
2. **添加節點**: 從節點選板拖拽兩個節點到編輯器
3. **測試連接**: 
   - 點擊第一個節點的輸出 socket (圓點)
   - 拖拽到第二個節點的輸入 socket
   - 應該能成功創建連接線

### 瀏覽器控制台驗證
預期看到的正確日誌：
```
🖱️ 點擊節點: check_agv_rotation_flow 檢查 AGV 是否需要執行旋轉流程
開始從輸出建立連接: check_agv_rotation_flow_1
創建連接: check_agv_rotation_flow_1.output -> find_available_manual_location_2.input
✅ 連接創建成功
```

### 功能測試檢查清單
- [ ] 節點選板正確顯示所有節點類型
- [ ] 節點可以成功添加到編輯器
- [ ] 輸出 socket 可以拖拽開始連接
- [ ] 輸入 socket 可以接收連接
- [ ] 連接線正確顯示
- [ ] 重複連接會顯示警告
- [ ] YAML 載入時連接正確重建

## 🔧 相關檔案

### 主要修改檔案
- **flowDesignerPage.js**: 
  - 重命名 `createConnection` → `createVisualConnection` (line 2964)
  - 更新 `rebuildConnections` 調用 (line 2954)
  - 更新 `createConnectionsProgressively` 調用 (line 3827)

### Socket 連接相關方法
- `renderNodeSockets()`: 創建 socket DOM 元素並設置 dataset 屬性
- `setupSocketEvents()`: 設置 socket 的滑鼠事件處理
- `createConnection()`: 處理實際的 ReteJS 連接邏輯
- `renderConnection()`: 渲染連接的視覺效果

## 🚨 注意事項

### 開發注意
1. **方法命名**: 避免使用相同的方法名處理不同類型的參數
2. **參數驗證**: 檢查參數類型以避免類型錯誤
3. **錯誤處理**: 提供清晰的錯誤消息來幫助調試

### 測試重點
1. **拖拽連接**: 確保滑鼠拖拽連接功能正常
2. **程序化連接**: 確保 YAML 載入時連接重建正常
3. **錯誤處理**: 測試各種異常情況的處理

## 🔗 相關文檔
- 節點選板修復: @docs-ai/operations/development/flow-designer-node-palette-fix.md
- Performance Optimization: @docs-ai/operations/development/flow-designer-phase4-2-performance-optimization.md
- Flow Designer 架構: @docs-ai/knowledge/system/flow-designer-architecture.md