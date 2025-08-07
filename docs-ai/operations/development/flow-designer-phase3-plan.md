# Flow Designer Phase 3: 視覺化編輯器開發計劃

## 🎯 Phase 3 總體目標
建立完整的 Flow Designer 視覺化編輯器，實現 YAML DSL 與視覺化編程界面的雙向同步，為業務流程設計提供直觀的可視化工具。

## 📋 Phase 3 背景和依賴

### 已完成的基礎設施
- ✅ **Phase 1**: YAML DSL 核心語法和解析器 (100% 完成)
- ✅ **Phase 2.1**: Simple WCS FlowParser 深度整合 (100% 完成)  
- ✅ **Phase 2.2**: WCS 函數註冊器完善 (91.7% 完成，38個函數)
- ✅ **Phase 2.3**: Enhanced Simple WCS Engine 執行環境整合 (100% 完成)

### 技術基礎
- **DSL 系統**: YAMLDSLParser, VariableResolver, ExpressionEvaluator, YAMLDSLExecutor
- **函數註冊**: 38個 WCS 函數完整映射，支持 4 種節點類型 (condition, logic, action, script)
- **執行引擎**: Enhanced Simple WCS Engine 支援 DSL 與傳統流程混合執行
- **範例檔案**: 3個 DSL 範例檔案涵蓋主要業務場景

## 🏗️ Phase 3 里程碑分解

### Phase 3.1: Flow Designer 核心架構 (Week 1-2)
**目標**: 建立 Flow Designer 的基礎架構和節點系統

#### 3.1.1 節點類型系統設計
- **Condition Nodes**: 條件判斷節點 (19個函數)
  - 視覺樣式: 菱形，橙色邊框
  - 輸入: 條件參數
  - 輸出: Boolean 結果
  - 範例: `rack_at_location_exists`, `rack_side_completed`

- **Logic Nodes**: 邏輯處理節點 (9個函數)
  - 視覺樣式: 矩形，藍色邊框
  - 輸入: 處理參數
  - 輸出: 計算結果
  - 範例: `get_room_inlet_point`, `find_available_manual_location`

- **Action Nodes**: 動作執行節點 (10個函數)
  - 視覺樣式: 圓角矩形，綠色邊框
  - 輸入: 執行參數
  - 輸出: 執行狀態
  - 範例: `create_task_from_decision`, `update_machine_parking_status`

- **Script Nodes**: 腳本控制節點
  - 視覺樣式: 八角形，紫色邊框
  - 功能: 控制流程邏輯 (if/else, loops, variables)

#### 3.1.2 Flow Designer Web 界面基礎
```
Flow Designer 技術棧
├── 前端: React + TypeScript + Tailwind CSS
├── 畫布: React Flow (節點拖拽和連線)
├── 表單: React Hook Form (節點屬性編輯)
├── 語法高亮: Monaco Editor (YAML 編輯器)
└── 狀態管理: Zustand (輕量狀態管理)
```

#### 3.1.3 節點庫和工具欄
- **節點調色板**: 按類型分組的可拖拽節點庫
- **屬性面板**: 選中節點的參數配置界面
- **工具欄**: 撤銷/重做、縮放、對齊、匯出功能
- **迷你地圖**: 大型流程的導航輔助

### Phase 3.2: YAML DSL 代碼生成引擎 (Week 3)
**目標**: 實現視覺化流程到 YAML DSL 的完整轉換

#### 3.2.1 DSL 代碼生成器
```typescript
class DSLCodeGenerator {
  generateDSLFromFlow(flowData: FlowData): string {
    // 1. 生成 metadata 部分
    // 2. 生成 variables 部分  
    // 3. 生成 steps 部分
    // 4. 優化和格式化輸出
  }
  
  generateMetadata(flow: FlowData): DSLMetadata
  generateVariables(nodes: FlowNode[]): DSLVariables  
  generateSteps(nodes: FlowNode[], connections: Connection[]): DSLStep[]
}
```

#### 3.2.2 節點到 DSL 步驟映射
- **Condition Node → condition_nodes**: 
  ```yaml
  - id: "check_rack_1"
    type: "condition_nodes"
    function: "rack_at_location_exists"
    source: "enhanced_database_client"
  ```

- **Logic Node → logic_nodes**:
  ```yaml
  - id: "get_inlet_1" 
    type: "logic_nodes"
    function: "get_room_inlet_point"
    source: "location_manager"
  ```

- **Action Node → action_nodes**:
  ```yaml
  - id: "create_task_1"
    type: "action_nodes" 
    function: "create_task_from_decision"
    source: "unified_task_manager"
  ```

#### 3.2.3 控制流程生成
- **條件分支**: if/else 結構生成
- **循環結構**: for/while 循環處理
- **變數管理**: 變數定義、賦值、作用域

### Phase 3.3: 雙向同步機制 (Week 4)
**目標**: 實現視覺化界面與 YAML 代碼的即時雙向同步

#### 3.3.1 YAML 到視覺化解析
```typescript
class YAMLToVisualParser {
  parseYAMLToFlow(yamlContent: string): FlowData {
    // 1. 解析 YAML 結構
    // 2. 創建節點對象
    // 3. 建立連線關係
    // 4. 設置佈局位置
  }
  
  createNodesFromSteps(steps: DSLStep[]): FlowNode[]
  createConnectionsFromFlow(steps: DSLStep[]): Connection[]
  autoLayoutNodes(nodes: FlowNode[]): LayoutResult
}
```

#### 3.3.2 即時同步引擎
```typescript
class SyncEngine {
  private yamlEditor: MonacoEditor;
  private flowCanvas: ReactFlow;
  
  syncVisualToYAML(flowData: FlowData): void {
    const yamlCode = this.codeGenerator.generateDSLFromFlow(flowData);
    this.yamlEditor.setValue(yamlCode);
  }
  
  syncYAMLToVisual(yamlContent: string): void {
    const flowData = this.yamlParser.parseYAMLToFlow(yamlContent);
    this.flowCanvas.setFlowData(flowData);
  }
}
```

#### 3.3.3 衝突解決機制
- **變更檢測**: 檢測 YAML 和視覺化的變更衝突
- **衝突解決**: 提供用戶選擇解決策略的界面
- **版本控制**: 支援流程版本的比較和回滾

## 🎨 用戶界面設計

### 主界面佈局
```
┌─────────────────────────────────────────────────┐
│ Header: Flow Designer - [Flow Name]             │
├──────────┬────────────────────────────┬─────────┤
│ 節點庫   │                           │ 屬性面板 │
│ ┌──────┐ │                           │ ┌──────┐ │
│ │Cond. │ │         流程畫布            │ │Node  │ │
│ │Logic │ │      (React Flow)         │ │Props │ │
│ │Action│ │                           │ │Panel │ │
│ │Script│ │                           │ │      │ │
│ └──────┘ │                           │ └──────┘ │
├──────────┼────────────────────────────┼─────────┤
│ 工具欄   │     YAML 代碼編輯器          │ 迷你地圖│
│          │   (Monaco Editor)          │         │
└──────────┴────────────────────────────┴─────────┘
```

### 節點設計規範
- **一致性**: 所有節點遵循統一的視覺語言
- **直觀性**: 節點圖示和顏色直接反映功能類型
- **資訊密度**: 在節點上顯示關鍵資訊而不臃腫
- **互動性**: 豐富的滑鼠懸停和選中狀態

## 🔧 技術實作規範

### 前端技術棧
```json
{
  "framework": "React 18 + TypeScript",
  "styling": "Tailwind CSS",
  "canvas": "React Flow",
  "editor": "Monaco Editor", 
  "forms": "React Hook Form + Zod",
  "state": "Zustand",
  "icons": "Lucide React",
  "build": "Vite"
}
```

### 與 AGVCUI 整合
- **路由整合**: 在 AGVCUI 中添加 `/flow-designer` 路由
- **樣式一致**: 遵循現有的 AGVCUI 設計系統
- **權限控制**: 整合現有的用戶權限系統
- **資料整合**: 與 Simple WCS 配置資料同步

### API 端點設計
```python
# FastAPI 端點
@app.get("/api/flows")
async def list_flows() -> List[FlowMetadata]

@app.get("/api/flows/{flow_id}")  
async def get_flow(flow_id: str) -> FlowData

@app.post("/api/flows")
async def create_flow(flow: FlowData) -> FlowCreateResult

@app.put("/api/flows/{flow_id}")
async def update_flow(flow_id: str, flow: FlowData) -> FlowUpdateResult

@app.post("/api/flows/{flow_id}/validate")
async def validate_flow(flow_id: str) -> ValidationResult

@app.post("/api/flows/{flow_id}/deploy") 
async def deploy_flow(flow_id: str) -> DeploymentResult
```

## 📊 驗收標準

### Phase 3.1 驗收標準
- ✅ 4種節點類型完整實作 (Condition, Logic, Action, Script)
- ✅ 節點庫支援拖拽創建
- ✅ 基本的節點連線功能
- ✅ 節點屬性編輯面板
- ✅ 流程畫布基本操作 (縮放、平移、選擇)

### Phase 3.2 驗收標準  
- ✅ 視覺化流程完整轉換為 YAML DSL
- ✅ 生成的 YAML 通過 DSL 解析器驗證
- ✅ 支援所有控制流程結構 (條件、循環、變數)
- ✅ 生成的 DSL 可在 Simple WCS 中正常執行

### Phase 3.3 驗收標準
- ✅ YAML 代碼變更即時反映到視覺化界面
- ✅ 視覺化操作即時更新 YAML 代碼
- ✅ 變更衝突檢測和解決機制
- ✅ 複雜流程的雙向同步穩定性

## 🚀 Phase 3 執行時程

### Week 1: 基礎架構
- Day 1-2: React Flow 環境設置，基本畫布
- Day 3-4: 4種節點類型實作和樣式
- Day 5-7: 節點拖拽和屬性編輯功能

### Week 2: 進階功能
- Day 8-10: 節點連線邏輯和驗證
- Day 11-12: 工具欄和迷你地圖
- Day 13-14: 與 AGVCUI 整合和測試

### Week 3: 代碼生成
- Day 15-17: DSL 代碼生成引擎實作
- Day 18-19: 控制流程生成邏輯
- Day 20-21: 代碼生成測試和優化

### Week 4: 雙向同步
- Day 22-24: YAML 到視覺化解析器
- Day 25-26: 即時同步機制實作
- Day 27-28: 衝突解決和完整測試

## 📋 風險評估和緩解

### 技術風險
- **React Flow 複雜度**: 預先進行技術調研和 POC
- **YAML 解析精度**: 建立完整的測試用例集
- **效能問題**: 大型流程的渲染最佳化

### 用戶體驗風險
- **學習曲線**: 提供豐富的教學和範例
- **操作複雜度**: 簡化常用操作的用戶流程
- **視覺混亂**: 良好的資訊架構和視覺層次

### 整合風險
- **AGVCUI 相容性**: 早期整合測試
- **Simple WCS 同步**: 確保生成的 DSL 完全相容
- **瀏覽器相容性**: 支援主流瀏覽器版本

## 🎯 成功指標

### 功能指標
- 支援創建包含 10+ 節點的複雜流程
- YAML ↔ 視覺化轉換準確率 > 99%
- 雙向同步延遲 < 100ms
- 支援所有 38 個 WCS 函數

### 用戶體驗指標
- 新用戶 15 分鐘內能創建基本流程
- 專家用戶相比手寫 YAML 效率提升 3x
- 用戶滿意度評分 > 4.5/5.0

### 技術指標
- 前端載入時間 < 2 秒
- 大型流程 (50+ 節點) 渲染 < 1 秒
- 記憶體使用 < 100MB
- 支援主流瀏覽器 (Chrome, Firefox, Safari, Edge)

---

**Phase 3 準備就緒 🚀**

所有 Phase 2 基礎設施已完成，Flow Designer 視覺化編輯器開發計劃已詳細規劃。我們已具備開始 Phase 3 開發的所有條件。