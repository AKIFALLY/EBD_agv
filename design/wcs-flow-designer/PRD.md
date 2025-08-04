# WCS 流程設計器產品需求文檔 (PRD)

## 🎯 產品概述

### 專案目標
在現有的 AGVCUI 管理界面中整合基於 Rete.js 的視覺化流程設計器，讓用戶可以透過拖拽節點的方式設計 Simple WCS 流程，完全替代手寫 YAML 配置的複雜性。

### 核心價值主張
- **視覺化設計** - 拖拽節點替代複雜的 YAML 語法
- **即時生效** - 設計完成後立即儲存為 JSON，Simple WCS 自動載入
- **保持 Simple** - 不增加 Simple WCS 的複雜度，維持檔案驅動架構
- **AI Agent 友好** - 生成的 JSON 格式便於 AI Agent 理解和修改

## 🏗️ 技術架構

### 技術選型
```
前端技術棧:
├── 🎨 UI 框架: AGVCUI (FastAPI + Jinja2 + Bulma CSS)
├── ⚡ 響應式框架: Vue.js 3 (CDN 引入，漸進式整合)
├── 📦 節點編輯器: Rete.js v2 (與 Vue.js 整合)
├── 🌐 HTTP 請求: Vanilla JavaScript Fetch API
├── 🎨 樣式: Bulma CSS + 自定義 CSS
└── 📄 模板引擎: Jinja2

後端技術棧:
├── 🔌 Web 框架: FastAPI (現有 AGVCUI 伺服器)
├── 📁 資料儲存: JSON 檔案 (/app/config/wcs/flows/*.json)
├── 🔄 Simple WCS: 擴展支援 JSON 檔案讀取
└── 🤖 AI Agent: 直接操作 JSON 檔案
```

### 系統架構圖
```
🎨 Rete.js 視覺化編輯器
    ↓ (儲存 JSON)
📁 /app/config/wcs/flows/*.json  
    ↓ (自動載入)
🔧 Simple WCS (保持輕量級)
    ↓ (執行決策)
⚡ AGV 任務執行

🤖 AI Agent
    ↓ (直接讀寫 JSON)
📁 /app/config/wcs/flows/*.json
```

## 🎨 前端設計

### 整合方式
- **位置**: AGVCUI 管理界面 (Port 8001)
- **新增頁面**: `/flow_designer`
- **導航**: 在現有導航選單中新增「流程設計器」標籤

### 頁面佈局
```
┌─────────────────────────────────────────────┐
│ AGVCUI 導航欄                                │
├─────────────────────────────────────────────┤
│ 工具列: [儲存] [測試] [範本選擇器]              │
├──────┬─────────────────────────────┬────────┤
│ 節點 │ Rete.js 編輯器畫布           │ 屬性   │
│ 工具 │ (拖拽、連接、配置)             │ 面板   │
│ 箱   │                            │        │
└──────┴─────────────────────────────┴────────┘
```

### 核心組件設計

#### 1. 條件節點 (ConditionComponent)
```javascript
功能: 檢查觸發條件
輸入: 觸發信號 (可選)
輸出: 布林值結果
配置選項:
- 檢查類型: 架台狀態 | AGV 狀態 | 位置狀態 | 時間條件
- 檢查欄位: 根據類型動態顯示
- 比較運算: 等於 | 大於 | 小於 | 在範圍內
- 比較值: 用戶輸入
```

#### 2. 邏輯節點 (LogicComponent)
```javascript
功能: 邏輯運算 (AND/OR/NOT)
輸入: 多個條件結果 (動態數量)
輸出: 邏輯運算結果
配置選項:
- 邏輯類型: AND | OR | NOT
- 輸入數量: 2-10 個 (動態調整)
```

#### 3. 動作節點 (ActionComponent)
```javascript
功能: 執行具體動作
輸入: 觸發條件
輸出: 無 (終端節點)
配置選項:
- 動作類型: 創建任務 | 發送通知 | 更新狀態
- 任務參數: work_id, agv_model, 起終點, 優先級
```

### CDN 資源引入 (基於 Context7 驗證)
```html
<!-- Vue.js 3 (優選響應式框架，CDN 整合) -->
<script src="https://cdn.jsdelivr.net/npm/vue@3/dist/vue.global.js"></script>

<!-- Rete.js v2 核心庫 (經驗證的 CDN 路徑) -->
<script src="https://cdn.jsdelivr.net/npm/rete/rete.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-area-plugin/rete-area-plugin.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-connection-plugin/rete-connection-plugin.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-render-utils/rete-render-utils.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-vue-plugin/rete-vue-plugin.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-context-menu-plugin@2.0.1/build/context-menu-plugin.min.js"></script>

<!-- 備選方案：React.js (如需要) -->
<!-- <script src="https://unpkg.com/react@18/umd/react.production.min.js"></script> -->
<!-- <script src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script> -->
<!-- <script src="https://cdn.jsdelivr.net/npm/rete-react-plugin/rete-react-plugin.min.js"></script> -->
```

### CDN 全域變數 (Vue.js + Rete.js v2)
```javascript
// Vue.js 3 全域變數
const { createApp, ref, reactive, computed, onMounted } = Vue;

// 經 Context7 驗證的 Rete.js 全域命名空間
const {
    Rete,
    ReteAreaPlugin,
    ReteConnectionPlugin,
    ReteVuePlugin,    // Vue.js 渲染插件
    ReteRenderUtils
} = window;
```

### Vue.js 技術選型說明

#### 為什麼選擇 Vue.js 而不是 React？
基於 RosAGV 現有系統架構分析，Vue.js 是更適合的技術選擇：

**✅ Vue.js 優勢**
- **漸進式整合**: 與現有 Pure JavaScript 架構完美融合
- **CDN 友好**: 單檔案引入，無需複雜的建置工具
- **模板語法相容**: 與現有 Jinja2 模板語法相似
- **學習曲線**: 對現有 JavaScript 開發團隊更友好
- **檔案大小**: 約 34KB，比 React + ReactDOM 更輕量
- **漸進式採用**: 可以僅在流程設計器使用，不影響現有代碼

**❌ React 限制**
- **複雜 CDN 整合**: 需要 React + ReactDOM + Babel 多個檔案
- **JSX 轉換**: 需要建置步驟或線上轉換
- **學習成本**: 對現有 Pure JavaScript 團隊學習曲線較陡
- **檔案大小**: React + ReactDOM 約 42KB + 額外轉換工具

#### Vue.js 整合示例
```javascript
// Vue.js 組件式 Rete.js 整合
const FlowDesignerApp = createApp({
  data() {
    return {
      editor: null,
      currentFlow: null,
      nodeCount: 0
    }
  },
  mounted() {
    this.initializeReteEditor();
    // 整合現有 Socket.IO
    socket.on('flow_update', this.handleFlowUpdate);
  },
  methods: {
    async initializeReteEditor() {
      const container = this.$refs.reteContainer;
      this.editor = new Rete.NodeEditor('wcs-flow@1.0.0', container);
      
      // 使用 Vue 渲染插件
      this.editor.use(ReteVuePlugin.default);
      
      // 整合現有存儲系統
      this.loadExistingFlows();
    },
    handleFlowUpdate(data) {
      // 響應式更新
      this.currentFlow = data;
      this.nodeCount = data.nodes.length;
    }
  }
});
```

## 📊 資料格式設計

### AI Agent 友好的 JSON 格式
```json
{
  "flow_info": {
    "id": "full_rack_transport",
    "name": "滿架運輸流程",
    "description": "當架台滿載時自動運輸到人工區域",
    "priority": 80,
    "created_by": "visual_editor",
    "tags": ["transport", "rack", "automatic"]
  },
  
  "trigger_logic": {
    "type": "AND",
    "conditions": [
      {
        "description": "檢查架台是否滿載",
        "check_type": "rack_status",
        "field": "full_status",
        "operator": "equals",
        "value": true
      },
      {
        "description": "檢查架台位置",
        "check_type": "location_check",
        "field": "location_id",
        "operator": "in",
        "value": [1, 2, 3]
      }
    ]
  },
  
  "actions": [
    {
      "description": "創建運輸任務",
      "type": "create_task",
      "parameters": {
        "work_id": "220001",
        "agv_model": "KUKA400i",
        "from_location": "${context.rack_location}",
        "to_location": "manual_area_01",
        "priority": 80
      }
    }
  ]
}
```

### Rete.js 設計資料格式 (可選儲存)
```json
{
  "rete_design": {
    "nodes": { /* Rete.js 節點位置和視覺化資訊 */ },
    "connections": { /* 節點間的連接關係 */ }
  }
}
```

## 🔌 API 設計

### 檔案操作端點
```python
# 基礎路徑: /api/flows
GET    /list                    # 列出所有流程檔案
GET    /load/{filename}         # 載入特定流程檔案
POST   /save/{filename}         # 儲存流程檔案
DELETE /delete/{filename}       # 刪除流程檔案

# AI Agent 專用端點  
GET    /ai/flows/simple         # AI 友好的流程列表
GET    /ai/flows/read/{filename} # AI 讀取檔案
POST   /ai/flows/write/{filename} # AI 寫入檔案
```

### 請求/回應格式
```javascript
// 儲存流程 API
POST /api/flows/save/my_flow.json
{
  "flow_info": { /* 流程基本資訊 */ },
  "trigger_logic": { /* 觸發條件 */ },
  "actions": [ /* 執行動作 */ ]
}

// 回應
{
  "status": "success",
  "message": "流程已儲存",
  "filename": "my_flow.json"
}
```

## 🔄 Simple WCS 整合

### 檔案掃描擴展
```python
# 現有 YAML 支援保持不變，新增 JSON 支援
class WCSEngine:
    def load_flows(self):
        # 載入 YAML 檔案 (向後相容)
        for yaml_file in self.config_dir.glob("*.yaml"):
            # 現有邏輯保持不變
            
        # 新增: 載入 JSON 檔案  
        for json_file in self.config_dir.glob("*.json"):
            with open(json_file, 'r') as f:
                flow_data = json.load(f)
                self.flows.append(SimpleFlow(flow_data))
```

### 統一流程類別
```python
class SimpleFlow:
    def __init__(self, flow_data: dict):
        # 自動檢測格式並統一處理
        if "flow_info" in flow_data:  # JSON 格式
            self.id = flow_data["flow_info"]["id"]
            self.name = flow_data["flow_info"]["name"]
            self.priority = flow_data["flow_info"]["priority"]
            self.conditions = flow_data["trigger_logic"]
            self.actions = flow_data["actions"]
        else:  # YAML 格式
            self.id = flow_data["flow_id"]
            self.name = flow_data["flow_name"] 
            self.priority = flow_data["priority"]
            # ... 現有邏輯
```

## 🤖 AI Agent 整合

### 優勢分析
```
✅ JSON 格式對 AI Agent 天然友好
✅ 豐富的描述欄位便於 AI 理解業務邏輯
✅ 直接檔案操作，無需理解複雜資料庫結構
✅ 錯誤只影響單一檔案，安全性高
✅ 可以做複雜的語義層面最佳化和重構
```

### 使用場景
- **智能最佳化**: AI Agent 分析執行效能，自動調整優先級和條件
- **衝突檢測**: 自動檢測流程間的邏輯衝突並提供解決建議
- **範本生成**: 根據現有流程生成新的範本
- **文檔生成**: 自動生成流程說明文檔

## 📋 實施計劃 (基於現有架構優化)

### 階段 1: 基礎架構整合 (2-3天)
- [ ] 在 AGVCUI 中新增流程設計器路由 (`/flow_designer`)
- [ ] 創建 FastAPI 路由器 (`agvcui/routers/flow_designer.py`)
- [ ] 整合 Vue.js 3 + Rete.js v2 (經驗證的 CDN 路徑)
- [ ] 建立基礎 HTML 模板 (繼承現有 `base.html`，Vue.js 整合)
- [ ] 配置 nginx 代理路由 (無需修改現有配置)

### 階段 2: 核心編輯器功能 (3-4天)
- [ ] 實現 Vue.js + Rete.js v2 編輯器初始化
- [ ] 開發三種核心 Vue 節點組件：
  - [ ] 條件節點 (ConditionComponent.vue)
  - [ ] 邏輯節點 (LogicComponent.vue)  
  - [ ] 動作節點 (ActionComponent.vue)
- [ ] 實現 Vue.js 響應式拖拽建立節點功能
- [ ] 建立節點連接系統 (Vue + Rete.js 整合)
- [ ] 實現 Vue.js 屬性配置面板

### 階段 3: 檔案管理和 API (2-3天)
- [ ] 建立 `/app/config/wcs/flows/` 目錄結構
- [ ] 實現檔案儲存 API (`/api/flows/save/{filename}`)
- [ ] 實現檔案載入 API (`/api/flows/load/{filename}`)
- [ ] 實現檔案列表 API (`/api/flows/list`)
- [ ] 整合 AGVCUI 現有的認證機制
- [ ] Socket.IO 即時同步支援

### 階段 4: Simple WCS 整合 (2-3天)
- [ ] 擴展 Simple WCS 支援 JSON 檔案格式
- [ ] 實現統一流程類別 (`SimpleFlow`)
- [ ] 確保 YAML 格式向後相容性
- [ ] 測試流程執行和驗證
- [ ] 整合 AI Agent 友好的 API 端點

### 階段 5: 測試和品質保證 (3-4天)
- [ ] 建立 Playwright E2E 測試套件
- [ ] 實現核心功能自動化測試
- [ ] 響應式設計測試 (桌面版和移動版)
- [ ] 效能測試 (CDN 載入時間、大型流程渲染)
- [ ] 跨瀏覽器相容性測試
- [ ] 整合測試 (與 Simple WCS 的端到端測試)

### 階段 6: 使用者體驗和文檔 (2-3天)
- [ ] 界面美化和 Bulma CSS 整合
- [ ] 響應式設計實施
- [ ] 錯誤處理和用戶友好的提示
- [ ] 建立流程範本庫
- [ ] 撰寫用戶操作手冊
- [ ] API 文檔自動生成 (FastAPI docs)

## 🎯 成功指標

### 功能性指標
- [ ] 可以透過拖拽創建完整的流程邏輯
- [ ] 生成的 JSON 檔案 Simple WCS 可以正確執行
- [ ] 支援複雜的多層 AND/OR 條件邏輯
- [ ] AI Agent 可以直接讀寫和修改流程檔案

### 效能指標  
- [ ] 頁面載入時間 < 3 秒
- [ ] 節點拖拽響應時間 < 100ms
- [ ] 流程儲存時間 < 1 秒
- [ ] 支援同時編輯 10+ 個節點的複雜流程

### 使用者體驗指標
- [ ] 10 分鐘內學會基本操作
- [ ] 相比手寫 YAML 減少 80% 的配置時間
- [ ] 錯誤率降低 90% (語法錯誤、邏輯錯誤)

## 🌐 Nginx 部署整合

### 基於現有 nginx 配置的整合策略
基於 `/nginx/default.conf` 的實際配置，流程設計器將完全整合到現有的 nginx 架構中：

```nginx
# 現有 AGVCUI 配置 (localhost 和 agvc.ui)
server {
    listen 80;
    server_name localhost agvc.ui;
    
    # 流程設計器路由 (無需修改現有配置)
    location /flow_designer {
        proxy_pass http://192.168.100.100:8001/flow_designer;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        
        # WebSocket 支援 (Socket.IO)
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
    
    # 現有的預設代理保持不變
    location / {
        proxy_pass http://192.168.100.100:8001;
        # ... 現有配置 ...
    }
}
```

### CDN 快取策略整合
利用現有 nginx 的 CDN 快取配置：

```nginx
# 現有配置已支援 JavaScript 和 CSS 快取
location ~* \.(js|css)$ {
    expires 24h;
    add_header Cache-Control "public";
}

# Rete.js CDN 資源本地快取 (可選優化)
location /cdn/rete/ {
    proxy_pass https://cdn.jsdelivr.net/npm/rete/;
    proxy_cache_valid 200 7d;
    expires 7d;
}
```

## 🧪 Playwright E2E 測試架構

### 基於現有測試框架擴展
利用 `/app/web_api_ws/src/agvcui/agvcui/testing/` 的現有 Playwright 配置：

```javascript
// playwright.config.js (流程設計器專用測試)
module.exports = {
  // 繼承現有配置
  ...require('./app/web_api_ws/src/agvcui/agvcui/testing/playwright.config.js'),
  
  // 流程設計器專用設定
  testDir: './design/wcs-flow-designer/tests',
  
  projects: [
    {
      name: 'flow-designer-chrome',
      use: { 
        ...devices['Desktop Chrome'],
        baseURL: 'http://localhost:8001/flow_designer'
      }
    },
    {
      name: 'flow-designer-mobile',
      use: { 
        ...devices['iPhone 12'],
        baseURL: 'http://localhost:8001/flow_designer'
      }
    }
  ]
};
```

### E2E 測試用例設計

#### 核心功能測試
```javascript
// tests/flow-designer-core.spec.js
import { test, expect } from '@playwright/test';

test.describe('流程設計器核心功能', () => {
  test('拖拽節點創建流程', async ({ page }) => {
    await page.goto('/flow_designer');
    
    // 等待 Rete.js 編輯器載入
    await page.waitForSelector('.rete-editor');
    
    // 拖拽條件節點到畫布
    await page.dragAndDrop('.node-toolbox .condition-node', '.rete-editor');
    
    // 驗證節點已創建
    const nodes = await page.locator('.rete-node').count();
    expect(nodes).toBeGreaterThan(0);
  });
  
  test('節點連接建立', async ({ page }) => {
    await page.goto('/flow_designer');
    
    // 創建兩個節點
    await page.dragAndDrop('.condition-node', '.rete-editor', { 
      targetPosition: { x: 100, y: 100 } 
    });
    await page.dragAndDrop('.action-node', '.rete-editor', { 
      targetPosition: { x: 300, y: 100 } 
    });
    
    // 建立連接
    await page.locator('.rete-output').first().dragTo(page.locator('.rete-input').first());
    
    // 驗證連接已建立
    const connections = await page.locator('.rete-connection').count();
    expect(connections).toBe(1);
  });
  
  test('流程儲存和載入', async ({ page }) => {
    await page.goto('/flow_designer');
    
    // 創建簡單流程
    await page.dragAndDrop('.condition-node', '.rete-editor');
    
    // 點擊儲存按鈕
    await page.click('#save-flow-btn');
    
    // 填入流程名稱
    await page.fill('#flow-name-input', 'test-flow');
    await page.click('#confirm-save-btn');
    
    // 驗證儲存成功提示
    await expect(page.locator('.notification.is-success')).toBeVisible();
    
    // 重新載入頁面
    await page.reload();
    
    // 載入剛儲存的流程
    await page.selectOption('#flow-selector', 'test-flow.json');
    
    // 驗證流程已載入
    const nodes = await page.locator('.rete-node').count();
    expect(nodes).toBeGreaterThan(0);
  });
});
```

#### 響應式設計測試
```javascript
// tests/flow-designer-responsive.spec.js
test.describe('響應式設計測試', () => {
  test('移動裝置適配', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto('/flow_designer');
    
    // 驗證移動版工具列
    await expect(page.locator('.mobile-toolbar')).toBeVisible();
    
    // 驗證觸控拖拽功能
    await page.touchscreen.tap(200, 200);
    await page.touchscreen.tap(400, 200);
  });
});
```

#### 效能測試
```javascript
// tests/flow-designer-performance.spec.js
test.describe('效能測試', () => {
  test('大型流程渲染效能', async ({ page }) => {
    await page.goto('/flow_designer');
    
    const startTime = Date.now();
    
    // 創建 50 個節點的大型流程
    for (let i = 0; i < 50; i++) {
      await page.dragAndDrop('.condition-node', '.rete-editor', {
        targetPosition: { x: (i % 10) * 200, y: Math.floor(i / 10) * 150 }
      });
    }
    
    const loadTime = Date.now() - startTime;
    expect(loadTime).toBeLessThan(5000); // 5秒內完成
  });
  
  test('CDN 資源載入時間', async ({ page }) => {
    const startTime = Date.now();
    await page.goto('/flow_designer');
    
    // 等待 Rete.js 完全載入
    await page.waitForFunction(() => window.Rete !== undefined);
    
    const loadTime = Date.now() - startTime;
    expect(loadTime).toBeLessThan(3000); // 3秒內載入完成
  });
});
```

### 測試資料管理
```javascript
// tests/test-data/sample-flows.js
export const sampleFlows = {
  simpleFlow: {
    flow_info: {
      id: "test_simple",
      name: "測試簡單流程",
      description: "用於自動化測試的簡單流程"
    },
    trigger_logic: {
      type: "AND",
      conditions: [
        {
          check_type: "rack_status",
          field: "full_status", 
          operator: "equals",
          value: true
        }
      ]
    },
    actions: [
      {
        type: "create_task",
        parameters: {
          work_id: "220001",
          agv_model: "KUKA400i"
        }
      }
    ]
  }
};
```

## 🏗️ AGVCUI 深度整合

### 基於現有 AGVCUI 架構的無縫整合

#### 路由整合
```python
# app/web_api_ws/src/agvcui/agvcui/routers/flow_designer.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates

router = APIRouter(prefix="/flow_designer", tags=["流程設計器"])
templates = Jinja2Templates(directory="templates")

@router.get("/", response_class=HTMLResponse)
async def flow_designer_page(request: Request):
    """流程設計器主頁面"""
    return templates.TemplateResponse("flow_designer.html", {
        "request": request,
        "title": "WCS 流程設計器",
        "current_page": "flow_designer"
    })

@router.get("/api/flows")
async def list_flows():
    """列出所有流程檔案"""
    flow_dir = Path("/app/config/wcs/flows")
    flows = []
    for file in flow_dir.glob("*.json"):
        with open(file, 'r', encoding='utf-8') as f:
            flow_data = json.load(f)
            flows.append({
                "filename": file.name,
                "name": flow_data.get("flow_info", {}).get("name", file.stem),
                "description": flow_data.get("flow_info", {}).get("description", "")
            })
    return flows

@router.post("/api/flows/save/{filename}")
async def save_flow(filename: str, flow_data: dict):
    """儲存流程檔案"""
    flow_dir = Path("/app/config/wcs/flows")
    flow_dir.mkdir(parents=True, exist_ok=True)
    
    file_path = flow_dir / filename
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(flow_data, f, indent=2, ensure_ascii=False)
    
    return {"status": "success", "message": "流程已儲存"}
```

#### 模板整合
```html
<!-- app/web_api_ws/src/agvcui/agvcui/templates/flow_designer.html -->
{% extends "base.html" %}
{% block title %}WCS 流程設計器{% endblock %}

{% block head %}
<!-- 繼承現有的 Bulma CSS 和基礎樣式 -->
<link rel="stylesheet" href="{{ url_for('static', path='css/flow_designer.css') }}">

<!-- Vue.js 3 CDN 資源 -->
<script src="https://cdn.jsdelivr.net/npm/vue@3/dist/vue.global.js"></script>

<!-- Rete.js v2 CDN 資源 -->
<script src="https://cdn.jsdelivr.net/npm/rete/rete.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-area-plugin/rete-area-plugin.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-connection-plugin/rete-connection-plugin.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-render-utils/rete-render-utils.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/rete-vue-plugin/rete-vue-plugin.min.js"></script>
{% endblock %}

{% block content %}
<div class="container is-fluid">
    <!-- 工具列 -->
    <div class="level mb-4">
        <div class="level-left">
            <div class="level-item">
                <h1 class="title is-4">WCS 流程設計器</h1>
            </div>
        </div>
        <div class="level-right">
            <div class="level-item">
                <div class="field is-grouped">
                    <div class="control">
                        <button id="save-flow-btn" class="button is-primary">
                            <span class="icon">
                                <i class="fas fa-save"></i>
                            </span>
                            <span>儲存</span>
                        </button>
                    </div>
                    <div class="control">
                        <div class="select">
                            <select id="flow-selector">
                                <option value="">選択流程...</option>
                            </select>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>
    
    <!-- 主要編輯區域 -->
    <div class="columns is-gapless" style="height: calc(100vh - 200px);">
        <!-- 節點工具箱 -->
        <div class="column is-2">
            <div class="box node-toolbox" style="height: 100%; overflow-y: auto;">
                <h2 class="subtitle is-6">節點工具箱</h2>
                
                <div class="node-category">
                    <h3 class="has-text-weight-semibold">條件節點</h3>
                    <div class="node-item condition-node" draggable="true">
                        <i class="fas fa-question-circle"></i>
                        架台狀態檢查
                    </div>
                    <div class="node-item condition-node" draggable="true">
                        <i class="fas fa-robot"></i>
                        AGV 狀態檢查
                    </div>
                </div>
                
                <div class="node-category">
                    <h3 class="has-text-weight-semibold">邏輯節點</h3>
                    <div class="node-item logic-node" draggable="true">
                        <i class="fas fa-plus"></i>
                        AND 邏輯
                    </div>
                    <div class="node-item logic-node" draggable="true">
                        <i class="fas fa-divide"></i>
                        OR 邏輯
                    </div>
                </div>
                
                <div class="node-category">
                    <h3 class="has-text-weight-semibold">動作節點</h3>
                    <div class="node-item action-node" draggable="true">
                        <i class="fas fa-tasks"></i>
                        創建任務
                    </div>
                    <div class="node-item action-node" draggable="true">
                        <i class="fas fa-bell"></i>
                        發送通知
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Rete.js 編輯器畫布 (Vue.js 整合) -->
        <div class="column is-8">
            <div id="flow-designer-app">
                <div ref="reteContainer" class="rete-editor" style="height: 100%; background: #f8f9fa;">
                    <!-- Vue.js 控制的狀態顯示 -->
                    <div v-if="nodeCount > 0" class="flow-status">
                        <span class="tag is-info">節點數量: {{ nodeCount }}</span>
                        <span v-if="currentFlow" class="tag is-success">{{ currentFlow.name }}</span>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- 屬性面板 -->
        <div class="column is-2">
            <div class="box properties-panel" style="height: 100%; overflow-y: auto;">
                <h2 class="subtitle is-6">屬性設定</h2>
                <div id="node-properties">
                    <p class="has-text-grey">請選擇一個節點來查看其屬性</p>
                </div>
            </div>
        </div>
    </div>
</div>

<!-- Socket.IO 整合 (使用現有的 Socket.IO 連接) -->
<script src="/socket.io/socket.io.js"></script>

<!-- Vue.js 流程設計器應用初始化 -->
<script>
// 初始化 Vue.js 流程設計器應用
const { createApp, ref, reactive } = Vue;

const flowDesignerApp = createApp({
  data() {
    return {
      editor: null,
      currentFlow: reactive({ name: '', id: '' }),
      nodeCount: 0,
      isLoading: false
    }
  },
  mounted() {
    this.initializeFlowDesigner();
  },
  methods: {
    async initializeFlowDesigner() {
      try {
        this.isLoading = true;
        await this.initializeReteEditor();
        this.setupSocketIO();
        this.loadAvailableFlows();
      } catch (error) {
        console.error('流程設計器初始化失敗:', error);
      } finally {
        this.isLoading = false;
      }
    },
    async initializeReteEditor() {
      const container = this.$refs.reteContainer;
      this.editor = new Rete.NodeEditor('wcs-flow@1.0.0', container);
      
      // 使用 Vue 渲染插件
      this.editor.use(ReteVuePlugin.default);
      this.editor.use(ReteAreaPlugin.default);
      this.editor.use(ReteConnectionPlugin.default);
      
      // 綁定事件
      this.editor.on('nodeselected', this.onNodeSelected);
      this.editor.on('noderemoved', this.onNodeRemoved);
    },
    setupSocketIO() {
      const socket = io();
      socket.on('flow_update', this.handleFlowUpdate);
      socket.on('flow_execution_status', this.handleExecutionStatus);
    },
    handleFlowUpdate(data) {
      this.currentFlow = data;
      this.nodeCount = Object.keys(this.editor.nodes).length;
    },
    onNodeSelected(node) {
      // 更新屬性面板
      this.$nextTick(() => {
        this.updatePropertiesPanel(node);
      });
    },
    onNodeRemoved() {
      this.nodeCount = Object.keys(this.editor.nodes).length;
    }
  }
});

// 掛載 Vue 應用
flowDesignerApp.mount('#flow-designer-app');
</script>

<script src="{{ url_for('static', path='js/flow_designer.js') }}"></script>
{% endblock %}
```

#### 導航整合
```html
<!-- 在現有的 navbar.html 中加入流程設計器連結 -->
<div class="navbar-item has-dropdown is-hoverable">
    <a class="navbar-link">
        系統管理
    </a>
    <div class="navbar-dropdown">
        <!-- 現有的選單項目 -->
        <a class="navbar-item" href="/users">用戶管理</a>
        <a class="navbar-item" href="/settings">系統設定</a>
        <hr class="navbar-divider">
        <!-- 新增：流程設計器 -->
        <a class="navbar-item" href="/flow_designer">
            <span class="icon">
                <i class="fas fa-project-diagram"></i>
            </span>
            <span>流程設計器</span>
        </a>
    </div>
</div>
```

### Socket.IO 即時同步
```javascript
// static/js/flow_designer.js 中的 Socket.IO 整合
const socket = io(); // 使用現有的 Socket.IO 連接

// 監聽流程執行狀態
socket.on('flow_execution_status', (data) => {
    const { flow_id, status, message } = data;
    updateFlowStatus(flow_id, status, message);
});

// 廣播流程變更
function broadcastFlowChange(flowData) {
    socket.emit('flow_designer_change', {
        flow_id: flowData.flow_info.id,
        user: getCurrentUser(),
        timestamp: new Date().toISOString(),
        changes: flowData
    });
}

// 協同編輯支援
socket.on('flow_designer_change', (data) => {
    if (data.user !== getCurrentUser()) {
        showCollaborativeNotification(data);
        // 可選：實時同步其他用戶的變更
    }
});
```

## 🔗 相關文檔

- **Simple WCS 架構**: @docs-ai/knowledge/system/simple-wcs-system.md
- **AGVCUI 開發指導**: `app/web_api_ws/src/agvcui/CLAUDE.md`
- **Web API 工作空間**: `app/web_api_ws/CLAUDE.md`
- **技術棧文檔**: @docs-ai/context/system/technology-stack.md
- **開發環境設定**: @docs-ai/operations/development/docker-development.md
- **Playwright 測試指導**: @docs-ai/operations/development/testing-procedures.md

## 🏗️ 容器部署架構

### 基於現有 RosAGV 雙環境的部署策略

#### AGVC 管理容器整合
流程設計器將完全整合到現有的 AGVC 管理系統中：

```yaml
# docker-compose.agvc.yml (無需修改現有配置)
services:
  agvc_server:
    # 現有配置保持不變
    container_name: agvc_server
    ports:
      - "8001:8001"  # 流程設計器通過此端口提供服務
    networks:
      agvc_network:
        ipv4_address: 192.168.100.100
    volumes:
      - ./app:/app  # 現有掛載包含流程設計器代碼
      - ./app/config/wcs/flows:/app/config/wcs/flows  # 流程檔案持久化
```

#### 檔案系統整合
```bash
# 容器內檔案結構 (基於現有架構)
/app/
├── web_api_ws/src/agvcui/
│   ├── agvcui/routers/flow_designer.py     # 新增路由器
│   ├── agvcui/templates/flow_designer.html # 新增模板
│   └── agvcui/static/
│       ├── css/flow_designer.css           # 新增樣式
│       └── js/flow_designer.js             # 新增腳本
├── config/wcs/flows/                       # 流程檔案目錄
│   ├── sample_rack_rotation.json           # 範例流程
│   └── *.json                              # 其他流程檔案
└── simple_wcs_ws/                          # Simple WCS 整合
    └── src/simple_wcs/engine.py            # 擴展支援 JSON
```

#### 網路架構整合
```
現有網路架構 (無需變更)
nginx (192.168.100.200:80)
  ↓ proxy_pass
agvc_server (192.168.100.100:8001)
  ├── /flow_designer → 流程設計器
  ├── /api/flows → 流程管理 API
  └── / → 現有 AGVCUI 功能
```

### 環境變數和配置
```bash
# 現有環境變數保持不變，新增流程設計器專用配置
export FLOW_DESIGNER_ENABLED=true
export FLOW_CONFIG_DIR="/app/config/wcs/flows"
export FLOW_BACKUP_DIR="/app/config/wcs/flows/backup"
export FLOW_TEMPLATE_DIR="/app/config/wcs/templates"
```

## 🔧 開發環境設定

### 基於現有開發流程的整合
```bash
# 使用現有的 AGVC 開發環境
# 1. 啟動 AGVC 系統
source scripts/docker-tools/docker-tools.sh
agvc_start

# 2. 進入開發容器
agvc_enter  # 自動載入 agvc_source

# 3. 開發流程設計器
cd /app/web_api_ws/src/agvcui
python3 agvcui/agvc_ui_server.py

# 4. 訪問流程設計器
curl http://localhost:8001/flow_designer
```

### 開發工具整合
```bash
# 使用現有的開發工具集
source scripts/dev-tools/dev-tools.sh

# 建置和測試
dev_build --workspace web_api_ws
dev_test --workspace web_api_ws --focus flow_designer

# 專用的流程設計器測試指令
scripts/dev-tools/test-flow-designer.sh
```

## 🧪 測試執行指南

### 基於現有測試框架的執行
```bash
# 使用現有的 Playwright 測試基礎設施
cd /app/web_api_ws/src/agvcui/agvcui/testing

# 執行流程設計器專用測試
npx playwright test --project=flow-designer-chrome
npx playwright test tests/flow-designer-*.spec.js

# 生成測試報告
npx playwright show-report

# 整合到現有的測試流程
scripts/dev-tools/run-all-tests.sh --include-flow-designer
```

### CI/CD 整合
```yaml
# 整合到現有的 GitHub Actions 或 CI 流程
name: Flow Designer Tests
on: [push, pull_request]
jobs:
  flow-designer-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Start AGVC System
        run: |
          source scripts/docker-tools/docker-tools.sh
          agvc_start
          sleep 30  # 等待系統完全啟動
      - name: Run Flow Designer Tests
        run: |
          cd app/web_api_ws/src/agvcui/agvcui/testing
          npx playwright test --project=flow-designer-chrome
```

## 📊 監控和維護

### 基於現有監控工具的擴展
```bash
# 使用現有的系統診斷工具
r agvc-check                    # 包含流程設計器健康檢查
r system-health                 # 完整系統健康狀態

# 流程設計器專用診斷
curl http://localhost:8001/flow_designer/api/health
tail -f /app/logs/flow_designer.log
```

### 日誌整合
```python
# 整合到現有的日誌系統
import logging
from agvcui.logging_config import get_logger

logger = get_logger('flow_designer')

# 流程操作日誌
logger.info(f"Flow saved: {flow_name} by {user_id}")
logger.warning(f"Flow validation failed: {validation_errors}")
logger.error(f"Flow execution error: {error_message}")
```

## 📝 實施考量和最佳實踐

### 設計理念 (基於現有架構)
- **無侵入性整合** - 完全基於現有 AGVCUI 架構，無需修改核心系統
- **保持 Simple WCS 簡潔性** - 不增加資料庫依賴，維持檔案驅動架構  
- **檔案即真理** - JSON 檔案是唯一資料來源，便於版本控制和備份
- **AI Agent 友好** - 豐富描述欄位和結構化格式便於 AI 理解和操作
- **漸進式採用** - 與現有 YAML 流程並存，支援逐步遷移

### 風險控制 (基於生產環境考量)
- **備份機制**: 整合到現有的檔案備份系統
- **版本控制**: 利用現有的 git 工作流程追蹤變更
- **權限控制**: 整合 AGVCUI 的現有用戶認證和權限系統
- **錯誤隔離**: 單一流程錯誤不影響系統運行或其他流程
- **回滾能力**: 基於 git 的快速回復機制

### 效能考量 (基於實際負載)
- **CDN 快取**: 利用現有 nginx 配置的靜態資源快取
- **檔案大小限制**: 單個流程檔案限制在 1MB 以內
- **並發處理**: 基於現有 FastAPI 的異步處理能力
- **記憶體使用**: Rete.js 編輯器記憶體使用監控和限制

### 安全考量 (基於企業級需求)
- **輸入驗證**: 嚴格的 JSON 格式驗證和檔案大小限制
- **路徑安全**: 防止目錄遍歷攻擊的檔案路徑驗證
- **權限檢查**: 基於現有 AGVCUI 的用戶權限系統
- **審計日誌**: 完整的流程變更審計追蹤