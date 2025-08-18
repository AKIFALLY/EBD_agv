# Flow Designer 故障排除手冊

## 🎯 手冊目標

為 Flow Designer 和 YAML DSL 系統提供完整的故障診斷和解決指導，幫助系統管理員和技術支援人員快速識別、診斷和解決常見問題。

## 📋 先決條件

- 熟悉 Flow Designer 基本操作
- 了解 AGVC 系統架構
- 具備基本的 Web 開發和系統管理知識

## 🚨 快速診斷流程

### 第一階段：問題識別（1-2分鐘）
```bash
# 1. 檢查系統基礎狀態
r agvc-check

# 2. 檢查 Web 服務
curl http://localhost:8001/flows/create

# 3. 檢查瀏覽器控制台
# 開啟瀏覽器開發工具 → Console 分頁 → 查看錯誤訊息
```

### 第二階段：問題分類（2-3分鐘）
根據症狀進行問題分類：
- 🌐 **系統存取問題**: 無法開啟 Flow Designer
- 🎨 **視覺化問題**: 節點無法顯示或操作異常
- 📝 **YAML DSL 問題**: 生成或載入 YAML 失敗
- ⚡ **效能問題**: 系統回應緩慢或卡頓
- 🔗 **整合問題**: WCS 函數無法正常運作

### 第三階段：具體解決（5-15分鐘）
依據問題分類執行對應的解決方案

## 🌐 系統存取問題

### 問題 1: 無法開啟 Flow Designer
**症狀**: 瀏覽器顯示連接錯誤或頁面無法載入

**診斷步驟**:
```bash
# 1. 檢查 AGVC 系統狀態
r agvc-check

# 2. 檢查 Web 服務端口
curl http://localhost:8001/
curl http://localhost:8001/flows/create

# 3. 检查容器狀態
docker compose -f docker-compose.agvc.yml ps
```

**解決方案**:
```bash
# 方案 1: 重啟 AGVC 系統
agvc_restart

# 方案 2: 檢查端口衝突
ss -tulpn | grep 8001
# 如果端口被佔用，停止衝突進程

# 方案 3: 檢查防火牆設定
sudo ufw status
sudo ufw allow 8001

# 方案 4: 檢查容器日誌
docker compose -f docker-compose.agvc.yml logs agvc_server
```

### 問題 2: 頁面載入但功能異常
**症狀**: 頁面可以開啟但按鈕無回應或部分功能缺失

**診斷步驟**:
```javascript
// 在瀏覽器控制台執行
console.log('Flow Designer 實例:', window.flowDesigner);
console.log('Node Types:', window.flowDesigner?.nodeTypes);
console.log('YAML 解析器:', typeof jsyaml);
```

**解決方案**:
```bash
# 方案 1: 清除瀏覽器快取
# Ctrl+Shift+Delete → 清除快取和 Cookie

# 方案 2: 檢查 JavaScript 載入
# 瀏覽器 F12 → Network → 重新載入頁面 → 檢查 JS 檔案載入狀態

# 方案 3: 檢查靜態檔案
ls -la /app/web_api_ws/src/agvcui/agvcui/static/js/
curl http://localhost:8001/static/js/flowDesignerPage.js
```

## 🎨 視覺化問題

### 問題 3: 節點選板空白或節點無法拖拽
**症狀**: 節點選板沒有顯示節點，或節點無法拖拽到編輯器

**診斷步驟**:
```javascript
// 檢查節點類型載入
console.log('可用節點類型:', Object.keys(window.flowDesigner.nodeTypes));
console.log('節點選板元素:', document.querySelector('.flow-node-palette'));

// 檢查拖拽事件
document.addEventListener('dragstart', (e) => {
    console.log('拖拽開始:', e.target);
});
```

**解決方案**:
```javascript
// 方案 1: 重新初始化節點選板
if (window.flowDesigner) {
    window.flowDesigner.initializeNodePalette();
}

// 方案 2: 檢查 CSS 樣式
const palette = document.querySelector('.flow-node-palette');
if (palette) {
    console.log('選板樣式:', window.getComputedStyle(palette));
}

// 方案 3: 手動添加節點
window.flowDesigner.addNode('check_agv_rotation_flow', 100, 100);
```

### 問題 4: 節點連接線無法建立
**症狀**: 拖拽節點連接點時無法建立連接線

**診斷步驟**:
```javascript
// 檢查連接邏輯
console.log('SVG 容器:', document.querySelector('#rete-editor svg'));
console.log('連接管理器:', window.flowDesigner.connectionManager);

// 檢查節點輸入輸出
const nodes = document.querySelectorAll('.flow-node');
nodes.forEach(node => {
    console.log('節點 ID:', node.id);
    console.log('輸入接點:', node.querySelectorAll('.input-socket'));
    console.log('輸出接點:', node.querySelectorAll('.output-socket'));
});
```

**解決方案**:
```javascript
// 方案 1: 重新初始化連接管理器
if (window.flowDesigner.connectionManager) {
    window.flowDesigner.connectionManager.initialize();
}

// 方案 2: 檢查事件監聽器
document.querySelectorAll('.output-socket').forEach(socket => {
    socket.addEventListener('mousedown', (e) => {
        console.log('輸出接點點擊:', e.target);
    });
});

// 方案 3: 手動建立連接
window.flowDesigner.createConnection('node1', 'output1', 'node2', 'input1');
```

## 📝 YAML DSL 問題

### 問題 5: YAML 生成失敗
**症狀**: 點擊「生成 YAML」按鈕後沒有反應或出現錯誤

**診斷步驟**:
```javascript
// 檢查生成函數
console.log('生成 YAML 函數:', typeof window.flowDesigner.generateYamlDsl);

// 測試生成過程
try {
    const yaml = window.flowDesigner.generateYamlDsl();
    console.log('生成的 YAML:', yaml);
} catch (error) {
    console.error('生成錯誤:', error);
}

// 檢查流程數據
console.log('當前流程:', window.flowDesigner.extractCurrentFlow());
```

**解決方案**:
```javascript
// 方案 1: 檢查流程完整性
const flow = window.flowDesigner.extractCurrentFlow();
if (!flow.nodes || flow.nodes.length === 0) {
    console.warn('流程中沒有節點，無法生成 YAML');
    // 添加至少一個節點後重試
}

// 方案 2: 逐步生成測試
try {
    // 測試各個生成步驟
    const nodes = window.flowDesigner.extractCurrentFlow().nodes;
    const variables = window.flowDesigner.extractVariables(nodes);
    const steps = window.flowDesigner.convertNodesToSteps(nodes);
    console.log('變數:', variables);
    console.log('步驟:', steps);
} catch (error) {
    console.error('生成步驟錯誤:', error);
}

// 方案 3: 使用備用生成方法
const backupYaml = `
flow_id: "manual_flow"
description: "手動創建的流程"
variables:
  test_var: "test_value"
steps:
  - step: 1
    function: "check_agv_rotation_flow"
    type: "condition_nodes"
`;
console.log('備用 YAML:', backupYaml);
```

### 問題 6: YAML 載入失敗
**症狀**: 載入 YAML 檔案後流程圖沒有顯示或顯示錯誤

**診斷步驟**:
```javascript
// 檢查載入函數
console.log('載入 YAML 函數:', typeof window.flowDesigner.parseDslToFlow);

// 檢查 YAML 解析器
console.log('YAML 解析器:', typeof jsyaml);

// 測試 YAML 解析
const testYaml = `
flow_id: "test_flow"
steps:
  - step: 1
    function: "test_function"
`;

try {
    const parsed = jsyaml.load(testYaml);
    console.log('解析結果:', parsed);
} catch (error) {
    console.error('YAML 解析錯誤:', error);
}
```

**解決方案**:
```javascript
// 方案 1: 驗證 YAML 格式
function validateYaml(yamlText) {
    try {
        const parsed = jsyaml.load(yamlText);
        
        // 檢查必要欄位
        if (!parsed.steps) {
            throw new Error('缺少 steps 欄位');
        }
        
        if (!Array.isArray(parsed.steps)) {
            throw new Error('steps 必須是陣列');
        }
        
        return { valid: true, data: parsed };
    } catch (error) {
        return { valid: false, error: error.message };
    }
}

// 方案 2: 分步載入
async function debugLoadYaml(yamlText) {
    console.log('1. 開始解析 YAML');
    const parseResult = validateYaml(yamlText);
    
    if (!parseResult.valid) {
        console.error('YAML 格式錯誤:', parseResult.error);
        return;
    }
    
    console.log('2. YAML 解析成功');
    console.log('3. 開始轉換為流程圖');
    
    try {
        await window.flowDesigner.parseDslToFlow(yamlText);
        console.log('4. 轉換完成');
    } catch (error) {
        console.error('轉換錯誤:', error);
    }
}

// 方案 3: 檢查節點映射
function checkNodeMapping(yamlData) {
    const unmappedFunctions = [];
    
    yamlData.steps.forEach(step => {
        const nodeTypeId = window.flowDesigner.findNodeTypeByFunction(step.function, step.type);
        if (!nodeTypeId) {
            unmappedFunctions.push({
                function: step.function,
                type: step.type,
                step: step.step
            });
        }
    });
    
    if (unmappedFunctions.length > 0) {
        console.warn('未映射的函數:', unmappedFunctions);
    }
    
    return unmappedFunctions;
}
```

## ⚡ 效能問題

### 問題 7: 系統回應緩慢
**症狀**: 節點操作、YAML 生成或載入需要很長時間

**診斷步驟**:
```javascript
// 檢查效能監控器
if (window.PerformanceOptimizer) {
    const monitor = window.PerformanceOptimizer.monitor;
    console.log('FPS:', monitor.getCurrentFPS());
    console.log('記憶體使用:', monitor.getMemoryUsage());
}

// 檢查節點數量
const nodeCount = document.querySelectorAll('.flow-node').length;
console.log('節點數量:', nodeCount);

// 檢查瀏覽器效能
console.log('效能記憶體:', performance.memory);
```

**解決方案**:
```javascript
// 方案 1: 啟用效能最佳化
if (window.PerformanceOptimizer) {
    // 啟用批量渲染
    window.PerformanceOptimizer.batchRenderer.enable();
    
    // 啟用記憶體管理
    window.PerformanceOptimizer.memoryManager.enableAutoCleanup();
    
    // 啟用視窗剔除
    if (nodeCount > 20) {
        window.PerformanceOptimizer.viewportCulling.enable();
    }
}

// 方案 2: 手動記憶體清理
function cleanupMemory() {
    // 清理 DOM 事件監聽器
    document.querySelectorAll('.flow-node').forEach(node => {
        const clone = node.cloneNode(true);
        node.parentNode.replaceChild(clone, node);
    });
    
    // 觸發垃圾回收（如果可用）
    if (window.gc) {
        window.gc();
    }
}

// 方案 3: 分批處理大型流程
function processBatchFlow(yamlData, batchSize = 10) {
    const steps = yamlData.steps;
    const batches = [];
    
    for (let i = 0; i < steps.length; i += batchSize) {
        batches.push(steps.slice(i, i + batchSize));
    }
    
    batches.forEach((batch, index) => {
        setTimeout(() => {
            batch.forEach(step => {
                window.flowDesigner.addNodeFromStep(step);
            });
        }, index * 100); // 每批間隔 100ms
    });
}
```

### 問題 8: 記憶體洩漏
**症狀**: 長時間使用後瀏覽器變慢或崩潰

**診斷步驟**:
```javascript
// 監控記憶體使用
function monitorMemory() {
    setInterval(() => {
        if (performance.memory) {
            console.log('記憶體使用:', {
                used: Math.round(performance.memory.usedJSHeapSize / 1024 / 1024) + ' MB',
                total: Math.round(performance.memory.totalJSHeapSize / 1024 / 1024) + ' MB',
                limit: Math.round(performance.memory.jsHeapSizeLimit / 1024 / 1024) + ' MB'
            });
        }
    }, 10000); // 每 10 秒檢查一次
}

monitorMemory();

// 檢查 DOM 節點數量
function checkDOMNodes() {
    console.log('DOM 節點總數:', document.querySelectorAll('*').length);
    console.log('Flow 節點數:', document.querySelectorAll('.flow-node').length);
    console.log('SVG 元素數:', document.querySelectorAll('svg *').length);
}

checkDOMNodes();
```

**解決方案**:
```javascript
// 方案 1: 實施自動清理
function implementAutoCleanup() {
    // 清理未使用的節點
    setInterval(() => {
        const unusedNodes = document.querySelectorAll('.flow-node[data-unused="true"]');
        unusedNodes.forEach(node => node.remove());
    }, 30000); // 每 30 秒清理一次
    
    // 清理事件監聽器
    function cleanupEventListeners() {
        const nodes = document.querySelectorAll('.flow-node');
        nodes.forEach(node => {
            const newNode = node.cloneNode(true);
            node.parentNode.replaceChild(newNode, node);
        });
    }
    
    // 定期清理
    setInterval(cleanupEventListeners, 300000); // 每 5 分鐘清理一次
}

// 方案 2: 實施對象池
class NodePool {
    constructor() {
        this.pool = [];
        this.maxSize = 50;
    }
    
    getNode() {
        if (this.pool.length > 0) {
            return this.pool.pop();
        }
        return this.createNewNode();
    }
    
    returnNode(node) {
        if (this.pool.length < this.maxSize) {
            this.resetNode(node);
            this.pool.push(node);
        } else {
            node.remove();
        }
    }
    
    createNewNode() {
        return document.createElement('div');
    }
    
    resetNode(node) {
        node.innerHTML = '';
        node.className = 'flow-node';
    }
}

const nodePool = new NodePool();
```

## 🔗 整合問題

### 問題 9: WCS 函數無法正常運作
**症狀**: 特定的 WCS 函數節點無法找到或執行異常

**診斷步驟**:
```javascript
// 檢查 WCS 函數註冊
console.log('已註冊的 WCS 函數:');
Object.keys(window.flowDesigner.nodeTypes).forEach(key => {
    const node = window.flowDesigner.nodeTypes[key];
    console.log(`${node.dslType}: ${node.id} (${node.source})`);
});

// 檢查特定函數
function checkWCSFunction(functionName, nodeType) {
    const nodeTypeId = window.flowDesigner.findNodeTypeByFunction(functionName, nodeType);
    if (nodeTypeId) {
        const nodeData = window.flowDesigner.nodeTypes[nodeTypeId];
        console.log('函數詳情:', nodeData);
    } else {
        console.error(`函數未找到: ${functionName} (${nodeType})`);
    }
}

// 測試所有 38 個 WCS 函數
const wcsFunctions = [
    // condition_nodes
    'check_agv_rotation_flow', 'is_agv_at_location', 'check_rack_availability',
    // logic_nodes  
    'get_room_inlet_point', 'get_agv_current_location',
    // action_nodes
    'create_task_from_decision', 'update_task_status'
    // ... 其他函數
];

wcsFunctions.forEach(func => {
    checkWCSFunction(func, 'condition_nodes');
});
```

**解決方案**:
```javascript
// 方案 1: 重新載入節點類型
async function reloadNodeTypes() {
    try {
        // 重新初始化節點類型
        await window.flowDesigner.initializeNodeTypes();
        console.log('節點類型重新載入完成');
    } catch (error) {
        console.error('節點類型載入失敗:', error);
    }
}

// 方案 2: 手動註冊缺失函數
function registerMissingFunction(functionName, nodeType, source) {
    const nodeId = `${functionName}_${nodeType}`;
    
    window.flowDesigner.nodeTypes[nodeId] = {
        id: functionName,
        name: functionName.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase()),
        dslType: nodeType,
        source: source,
        inputs: [],
        outputs: [],
        color: getNodeColor(nodeType)
    };
    
    console.log(`已註冊函數: ${functionName}`);
}

function getNodeColor(nodeType) {
    const colors = {
        'condition_nodes': '#e74c3c',
        'logic_nodes': '#3498db', 
        'action_nodes': '#2ecc71',
        'script_nodes': '#f39c12'
    };
    return colors[nodeType] || '#95a5a6';
}

// 方案 3: 函數映射修復
function fixFunctionMapping() {
    const missingFunctions = [
        { name: 'check_agv_rotation_flow', type: 'condition_nodes', source: 'unified_decision_engine' },
        { name: 'get_room_inlet_point', type: 'logic_nodes', source: 'location_manager' }
        // 添加其他缺失的函數
    ];
    
    missingFunctions.forEach(func => {
        registerMissingFunction(func.name, func.type, func.source);
    });
    
    // 重新初始化選板
    window.flowDesigner.initializeNodePalette();
}
```

### 問題 10: Simple WCS 整合異常
**症狀**: Flow Designer 生成的 YAML 無法被 Simple WCS 正確解析

**診斷步驟**:
```bash
# 檢查 Simple WCS 服務狀態
curl http://localhost:8000/api/wcs/status

# 檢查 DSL 解析器
curl -X POST http://localhost:8000/api/wcs/parse-dsl \
  -H "Content-Type: application/json" \
  -d '{"dsl_content": "flow_id: test\nsteps: []"}'

# 檢查錯誤日誌
docker compose -f docker-compose.agvc.yml logs agvc_server | grep -i "wcs\|dsl"
```

**解決方案**:
```bash
# 方案 1: 重啟相關服務
docker compose -f docker-compose.agvc.yml restart agvc_server

# 方案 2: 檢查 DSL 格式相容性
# 在瀏覽器控制台執行
const yamlContent = window.flowDesigner.generateYamlDsl();
console.log('生成的 YAML:', yamlContent);

// 驗證格式
const validation = {
    hasFlowId: yamlContent.includes('flow_id:'),
    hasSteps: yamlContent.includes('steps:'),
    hasValidStructure: true
};
console.log('格式驗證:', validation);

# 方案 3: 手動測試 WCS 整合
fetch('/api/wcs/parse-dsl', {
    method: 'POST',
    headers: {
        'Content-Type': 'application/json'
    },
    body: JSON.stringify({
        dsl_content: yamlContent
    })
})
.then(response => response.json())
.then(data => console.log('WCS 解析結果:', data))
.catch(error => console.error('WCS 整合錯誤:', error));
```

## 🧪 診斷工具

### 內建診斷工具
```javascript
// Flow Designer 診斷工具
window.FlowDesignerDiagnostics = {
    // 系統狀態檢查
    checkSystemStatus() {
        const status = {
            flowDesigner: typeof window.flowDesigner !== 'undefined',
            yamlParser: typeof jsyaml !== 'undefined',
            nodeTypes: window.flowDesigner?.nodeTypes ? Object.keys(window.flowDesigner.nodeTypes).length : 0,
            performanceOptimizer: typeof window.PerformanceOptimizer !== 'undefined'
        };
        
        console.log('系統狀態:', status);
        return status;
    },
    
    // 效能檢查
    checkPerformance() {
        const perf = {
            memoryUsage: performance.memory ? Math.round(performance.memory.usedJSHeapSize / 1024 / 1024) + ' MB' : 'N/A',
            nodeCount: document.querySelectorAll('.flow-node').length,
            connectionCount: document.querySelectorAll('.connection-line').length,
            fps: window.PerformanceOptimizer?.monitor?.getCurrentFPS() || 'N/A'
        };
        
        console.log('效能狀態:', perf);
        return perf;
    },
    
    // WCS 函數檢查
    checkWCSFunctions() {
        const functions = {
            condition_nodes: [],
            logic_nodes: [],
            action_nodes: [],
            script_nodes: []
        };
        
        Object.values(window.flowDesigner.nodeTypes || {}).forEach(node => {
            if (functions[node.dslType]) {
                functions[node.dslType].push(node.id);
            }
        });
        
        console.log('WCS 函數分布:', functions);
        return functions;
    },
    
    // 完整診斷
    runFullDiagnostics() {
        console.log('🔍 開始完整診斷...');
        
        const results = {
            system: this.checkSystemStatus(),
            performance: this.checkPerformance(),
            wcsFunctions: this.checkWCSFunctions(),
            timestamp: new Date().toISOString()
        };
        
        console.log('📊 診斷完成:', results);
        return results;
    }
};

// 使用方式
console.log('Flow Designer 診斷工具已載入');
console.log('使用 FlowDesignerDiagnostics.runFullDiagnostics() 開始診斷');
```

### 自動化測試工具
```javascript
// 自動化問題檢測
window.FlowDesignerAutoTest = {
    async runBasicTests() {
        const results = [];
        
        // 測試 1: 節點創建
        try {
            window.flowDesigner.addNode('check_agv_rotation_flow', 100, 100);
            results.push({ test: '節點創建', status: 'PASS' });
        } catch (error) {
            results.push({ test: '節點創建', status: 'FAIL', error: error.message });
        }
        
        // 測試 2: YAML 生成
        try {
            const yaml = window.flowDesigner.generateYamlDsl();
            results.push({ 
                test: 'YAML 生成', 
                status: yaml.length > 0 ? 'PASS' : 'FAIL',
                details: `生成 ${yaml.length} 個字符`
            });
        } catch (error) {
            results.push({ test: 'YAML 生成', status: 'FAIL', error: error.message });
        }
        
        // 測試 3: YAML 載入
        try {
            const testYaml = `
flow_id: "test_flow"
steps:
  - step: 1
    function: "check_agv_rotation_flow"
    type: "condition_nodes"
`;
            await window.flowDesigner.parseDslToFlow(testYaml);
            results.push({ test: 'YAML 載入', status: 'PASS' });
        } catch (error) {
            results.push({ test: 'YAML 載入', status: 'FAIL', error: error.message });
        }
        
        console.log('自動化測試結果:', results);
        return results;
    }
};
```

## 📞 技術支援流程

### 問題報告格式
```
問題報告模板:
─────────────────
📋 基本資訊
• 時間: 2025-08-11 14:30:00
• 使用者: admin
• 瀏覽器: Chrome 120.0.0.0
• 系統: Windows 11

🚨 問題描述
• 症狀: 無法生成 YAML DSL
• 重現步驟: 
  1. 開啟 Flow Designer
  2. 添加 check_agv_rotation_flow 節點
  3. 點擊生成 YAML 按鈕
  4. 沒有任何反應

🔍 診斷資訊
• 系統狀態: [FlowDesignerDiagnostics 結果]
• 控制台錯誤: [瀏覽器控制台截圖]
• 網路狀態: [Network 面板資訊]

📊 影響範圍
• 影響功能: YAML DSL 生成
• 影響用戶: 所有用戶
• 緊急程度: 中等
─────────────────
```

### 升級路徑
```
Level 1: 使用者自助 (本手冊)
↓ (15分鐘內無法解決)
Level 2: 技術支援 (系統管理員)
↓ (1小時內無法解決)  
Level 3: 開發團隊 (系統開發者)
↓ (需要代碼修改)
Level 4: 架構審查 (技術架構師)
```

## 🔗 相關資源

- **Flow Designer 完整使用手冊**: 基礎操作指導
- **最佳實踐指南**: 避免常見問題的設計原則
- **系統架構文檔**: 深入理解系統工作原理
- **API 參考文檔**: 開發者級別的技術資訊
- **測試套件**: 自動化測試和驗證工具

## 📝 常見問題快速索引

| 問題類型 | 快速解決方案 | 參考章節 |
|---------|-------------|---------|
| 頁面無法開啟 | `agvc_restart` | 系統存取問題 |
| 節點選板空白 | 清除瀏覽器快取 | 視覺化問題 |
| YAML 生成失敗 | 檢查流程節點數量 | YAML DSL 問題 |
| 系統回應緩慢 | 啟用效能最佳化 | 效能問題 |
| WCS 函數錯誤 | 重新載入節點類型 | 整合問題 |

---

📝 **文檔版本**: v1.0  
📅 **更新日期**: 2025-08-11  
👥 **目標用戶**: 系統管理員、技術支援人員、開發者