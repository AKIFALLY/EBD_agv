# Flow Designer 開發者文檔

## 🎯 文檔目標

為系統開發者和技術集成人員提供完整的技術參考文檔，涵蓋 Flow Designer + YAML DSL 系統的架構設計、API 接口、擴展開發和集成測試指導。

## 📋 先決條件

- 熟悉 JavaScript/TypeScript 開發
- 了解 Web 前端架構設計
- 掌握 ROS 2 和 WCS 系統概念
- 具備 RESTful API 和 WebSocket 開發經驗

## 🏗️ 系統技術架構

### 架構概覽
```
Flow Designer 技術架構
├── 前端層 (Client-side)
│   ├── 視覺化編輯器 (Rete.js + Custom)
│   ├── YAML DSL 引擎 (JS-YAML + Custom Parser)
│   ├── 效能最佳化層 (Performance Optimizer)
│   └── UI 互動層 (Vanilla JS + Bulma CSS)
├── 服務層 (Server-side)
│   ├── FastAPI Web 服務
│   ├── Socket.IO 即時通訊
│   └── PostgreSQL 資料持久化
├── 整合層 (Integration)
│   ├── Simple WCS DSL 解析器
│   ├── WCS 函數註冊器
│   └── ROS 2 服務橋接
└── 基礎設施層 (Infrastructure)
    ├── Docker 容器化
    ├── Nginx 反向代理
    └── Zenoh RMW 通訊
```

### 核心模組依賴關係
```javascript
// 核心依賴樹
flowDesignerPage.js
├── node-types.js              // 節點類型定義
├── yaml-dsl-generator.js      // YAML DSL 生成引擎
├── yaml-dsl-parser.js         // YAML DSL 解析引擎
├── performance-optimizer.js   // 效能最佳化
├── connection-manager.js      // 連接管理
└── flow-designer/
    ├── test-suite.js          // 測試套件
    └── utils.js               // 工具函數
```

## 🔧 核心 API 參考

### FlowDesigner 主類
```javascript
/**
 * Flow Designer 主類
 * 負責整個流程設計系統的初始化和管理
 */
class FlowDesigner {
    constructor(containerId, options = {}) {
        this.containerId = containerId;
        this.options = {
            enablePerformanceOptimizer: true,
            maxNodes: 100,
            autoSave: false,
            ...options
        };
        
        // 核心組件
        this.nodeTypes = {};
        this.connectionManager = null;
        this.performanceOptimizer = null;
        
        this.initialize();
    }
    
    /**
     * 初始化系統
     */
    async initialize() {
        await this.loadNodeTypes();
        this.initializeEditor();
        this.initializeNodePalette();
        this.initializePerformanceOptimizer();
        this.bindEvents();
    }
    
    /**
     * 載入節點類型定義
     */
    async loadNodeTypes() {
        // 從服務器載入或使用內建定義
        this.nodeTypes = await this.fetchNodeTypes();
    }
    
    /**
     * 添加節點到編輯器
     * @param {string} nodeTypeId - 節點類型 ID
     * @param {number} x - X 座標
     * @param {number} y - Y 座標
     * @returns {string} 節點 ID
     */
    addNode(nodeTypeId, x, y) {
        const nodeType = this.nodeTypes[nodeTypeId];
        if (!nodeType) {
            throw new Error(`節點類型不存在: ${nodeTypeId}`);
        }
        
        const nodeId = this.generateNodeId();
        const nodeElement = this.createNodeElement(nodeType, x, y);
        
        this.getEditorContainer().appendChild(nodeElement);
        
        // 觸發事件
        this.emit('nodeAdded', { nodeId, nodeType, x, y });
        
        return nodeId;
    }
    
    /**
     * 創建節點間連接
     * @param {string} sourceNodeId - 源節點 ID
     * @param {string} sourceOutput - 源輸出名稱
     * @param {string} targetNodeId - 目標節點 ID  
     * @param {string} targetInput - 目標輸入名稱
     */
    createConnection(sourceNodeId, sourceOutput, targetNodeId, targetInput) {
        const connection = {
            id: this.generateConnectionId(),
            source: { nodeId: sourceNodeId, output: sourceOutput },
            target: { nodeId: targetNodeId, input: targetInput }
        };
        
        // 驗證連接有效性
        if (!this.validateConnection(connection)) {
            throw new Error('無效的節點連接');
        }
        
        // 創建視覺化連接線
        this.connectionManager.createConnectionLine(connection);
        
        // 觸發事件
        this.emit('connectionCreated', connection);
        
        return connection.id;
    }
    
    /**
     * 生成 YAML DSL
     * @returns {string} YAML DSL 內容
     */
    generateYamlDsl() {
        const flowData = this.extractCurrentFlow();
        const yamlGenerator = new YAMLDSLGenerator();
        
        return yamlGenerator.generateFromFlow(flowData);
    }
    
    /**
     * 解析 YAML DSL 並創建流程圖
     * @param {string} yamlContent - YAML DSL 內容
     */
    async parseDslToFlow(yamlContent) {
        const yamlParser = new YAMLDSLParser();
        const flowData = yamlParser.parse(yamlContent);
        
        // 清空當前流程
        this.clearFlow();
        
        // 創建節點
        await this.createNodesFromFlow(flowData);
        
        // 重建連接
        this.rebuildConnections(flowData);
        
        // 觸發事件
        this.emit('flowLoaded', flowData);
    }
}
```

### 節點類型系統
```javascript
/**
 * 節點類型定義接口
 */
interface NodeType {
    id: string;                 // 節點唯一標識
    name: string;              // 顯示名稱
    description?: string;      // 節點描述
    dslType: 'condition_nodes' | 'logic_nodes' | 'action_nodes' | 'script_nodes';
    source: string;            // 函數來源 (如 'unified_decision_engine')
    category: string;          // 節點分類
    icon?: string;             // 節點圖標
    color: string;             // 節點顏色
    inputs: NodeSocket[];      // 輸入接口
    outputs: NodeSocket[];     // 輸出接口
    parameters: NodeParameter[]; // 配置參數
}

/**
 * 節點接口定義
 */
interface NodeSocket {
    name: string;              // 接口名稱
    type: 'data' | 'control';  // 接口類型
    dataType: string;          // 數據類型
    required: boolean;         // 是否必需
    description?: string;      // 接口描述
}

/**
 * 節點參數定義
 */
interface NodeParameter {
    name: string;              // 參數名稱
    type: 'string' | 'number' | 'boolean' | 'select' | 'json';
    required: boolean;         // 是否必需
    defaultValue?: any;        // 預設值
    options?: string[];        // 選項列表 (type='select' 時)
    validation?: {             // 驗證規則
        min?: number;
        max?: number;
        pattern?: string;
    };
    description?: string;      // 參數描述
}

/**
 * 節點類型註冊器
 */
class NodeTypeRegistry {
    constructor() {
        this.nodeTypes = new Map();
        this.categories = new Set();
    }
    
    /**
     * 註冊節點類型
     * @param {NodeType} nodeType - 節點類型定義
     */
    register(nodeType) {
        // 驗證節點類型
        this.validateNodeType(nodeType);
        
        // 註冊到映射表
        this.nodeTypes.set(nodeType.id, nodeType);
        this.categories.add(nodeType.category);
        
        console.log(`已註冊節點類型: ${nodeType.id}`);
    }
    
    /**
     * 獲取節點類型
     * @param {string} nodeId - 節點 ID
     * @returns {NodeType|null}
     */
    get(nodeId) {
        return this.nodeTypes.get(nodeId) || null;
    }
    
    /**
     * 根據 DSL 類型和函數名查找節點類型
     * @param {string} functionName - 函數名
     * @param {string} dslType - DSL 類型
     * @returns {NodeType|null}
     */
    findByFunction(functionName, dslType) {
        for (const [id, nodeType] of this.nodeTypes) {
            if (nodeType.id === functionName && nodeType.dslType === dslType) {
                return nodeType;
            }
        }
        return null;
    }
    
    /**
     * 獲取分類下的所有節點類型
     * @param {string} category - 分類名稱
     * @returns {NodeType[]}
     */
    getByCategory(category) {
        const result = [];
        for (const nodeType of this.nodeTypes.values()) {
            if (nodeType.category === category) {
                result.push(nodeType);
            }
        }
        return result;
    }
}
```

### YAML DSL 生成引擎
```javascript
/**
 * YAML DSL 生成引擎
 */
class YAMLDSLGenerator {
    constructor(options = {}) {
        this.options = {
            includeComments: true,
            sortSteps: true,
            validateOutput: true,
            ...options
        };
    }
    
    /**
     * 從流程數據生成 YAML DSL
     * @param {Object} flowData - 流程數據
     * @returns {string} YAML DSL 內容
     */
    generateFromFlow(flowData) {
        const yamlData = {
            flow_id: flowData.flowId || this.generateFlowId(),
            description: flowData.description || "Flow Designer 生成的流程",
            variables: this.extractVariables(flowData.nodes),
            steps: this.convertNodesToSteps(flowData.nodes, flowData.connections)
        };
        
        // 生成 YAML 字符串
        let yamlContent = this.generateYamlContent(yamlData);
        
        // 添加註解
        if (this.options.includeComments) {
            yamlContent = this.addComments(yamlContent, yamlData);
        }
        
        // 驗證輸出
        if (this.options.validateOutput) {
            this.validateYamlOutput(yamlContent);
        }
        
        return yamlContent;
    }
    
    /**
     * 提取變數定義
     * @param {Array} nodes - 節點列表
     * @returns {Object} 變數對象
     */
    extractVariables(nodes) {
        const variables = {};
        const usedVariables = new Set();
        
        // 掃描所有節點的輸入參數
        nodes.forEach(node => {
            this.scanNodeForVariables(node, usedVariables);
        });
        
        // 為每個變數生成預設值
        usedVariables.forEach(varName => {
            variables[varName] = this.inferVariableDefaultValue(varName);
        });
        
        return variables;
    }
    
    /**
     * 將節點轉換為步驟
     * @param {Array} nodes - 節點列表
     * @param {Array} connections - 連接列表
     * @returns {Array} 步驟列表
     */
    convertNodesToSteps(nodes, connections) {
        const steps = [];
        const nodeOrder = this.calculateExecutionOrder(nodes, connections);
        
        nodeOrder.forEach((nodeId, index) => {
            const node = nodes.find(n => n.id === nodeId);
            if (node) {
                const step = this.convertNodeToStep(node, index + 1);
                steps.push(step);
            }
        });
        
        return steps;
    }
    
    /**
     * 轉換單個節點為步驟
     * @param {Object} node - 節點對象
     * @param {number} stepNumber - 步驟編號
     * @returns {Object} 步驟對象
     */
    convertNodeToStep(node, stepNumber) {
        const nodeType = window.flowDesigner.nodeTypes[node.typeId];
        
        const step = {
            step: stepNumber,
            function: nodeType.id,
            type: nodeType.dslType,
            source: nodeType.source
        };
        
        // 添加輸入參數
        if (node.inputs && Object.keys(node.inputs).length > 0) {
            step.inputs = this.processNodeInputs(node.inputs);
        }
        
        // 添加輸出參數
        if (node.outputs && Object.keys(node.outputs).length > 0) {
            step.outputs = this.processNodeOutputs(node.outputs);
        }
        
        // 添加條件邏輯 (用於 script_nodes)
        if (nodeType.dslType === 'script_nodes') {
            this.addScriptNodeLogic(step, node);
        }
        
        return step;
    }
    
    /**
     * 生成 YAML 內容
     * @param {Object} yamlData - YAML 數據對象
     * @returns {string} YAML 字符串
     */
    generateYamlContent(yamlData) {
        // 使用 js-yaml 庫生成基礎 YAML
        let yaml = jsyaml.dump(yamlData, {
            indent: 2,
            lineWidth: 120,
            noRefs: true,
            sortKeys: false
        });
        
        // 自定義格式化
        yaml = this.formatYamlOutput(yaml);
        
        return yaml;
    }
}
```

### YAML DSL 解析引擎
```javascript
/**
 * YAML DSL 解析引擎
 */
class YAMLDSLParser {
    constructor(options = {}) {
        this.options = {
            strictMode: false,
            allowUnknownFunctions: true,
            validateConnections: true,
            ...options
        };
        
        this.errors = [];
        this.warnings = [];
    }
    
    /**
     * 解析 YAML DSL
     * @param {string} yamlContent - YAML 內容
     * @returns {Object} 解析後的流程數據
     */
    parse(yamlContent) {
        this.clearMessages();
        
        try {
            // 解析 YAML
            const yamlData = jsyaml.load(yamlContent);
            
            // 驗證基本結構
            this.validateBasicStructure(yamlData);
            
            // 解析為流程數據
            const flowData = this.convertToFlowData(yamlData);
            
            // 驗證流程數據
            if (this.options.validateConnections) {
                this.validateFlowData(flowData);
            }
            
            return flowData;
            
        } catch (error) {
            this.addError('YAML 解析失敗', error.message);
            throw new Error(`YAML DSL 解析錯誤: ${error.message}`);
        }
    }
    
    /**
     * 驗證基本結構
     * @param {Object} yamlData - YAML 數據
     */
    validateBasicStructure(yamlData) {
        // 檢查必需欄位
        if (!yamlData.steps) {
            this.addError('缺少必需欄位', 'steps 欄位不存在');
        }
        
        if (!Array.isArray(yamlData.steps)) {
            this.addError('欄位類型錯誤', 'steps 必須是陣列');
        }
        
        // 檢查步驟結構
        yamlData.steps.forEach((step, index) => {
            this.validateStepStructure(step, index);
        });
    }
    
    /**
     * 驗證步驟結構
     * @param {Object} step - 步驟對象
     * @param {number} index - 步驟索引
     */
    validateStepStructure(step, index) {
        const requiredFields = ['step', 'function', 'type'];
        
        requiredFields.forEach(field => {
            if (!(field in step)) {
                this.addError(`步驟 ${index + 1} 缺少必需欄位`, `${field} 欄位不存在`);
            }
        });
        
        // 檢查函數是否存在
        if (!this.options.allowUnknownFunctions) {
            const nodeType = window.flowDesigner.nodeTypes.findByFunction(step.function, step.type);
            if (!nodeType) {
                this.addWarning(`步驟 ${index + 1} 未知函數`, `${step.function} (${step.type})`);
            }
        }
    }
    
    /**
     * 轉換為流程數據
     * @param {Object} yamlData - YAML 數據
     * @returns {Object} 流程數據
     */
    convertToFlowData(yamlData) {
        const flowData = {
            flowId: yamlData.flow_id || 'unnamed_flow',
            description: yamlData.description || '',
            variables: yamlData.variables || {},
            nodes: [],
            connections: []
        };
        
        // 轉換步驟為節點
        const nodePositions = this.calculateNodePositions(yamlData.steps.length);
        
        yamlData.steps.forEach((step, index) => {
            const node = this.convertStepToNode(step, nodePositions[index]);
            flowData.nodes.push(node);
        });
        
        // 推斷連接關係
        flowData.connections = this.inferConnections(yamlData.steps);
        
        return flowData;
    }
    
    /**
     * 轉換步驟為節點
     * @param {Object} step - 步驟對象
     * @param {Object} position - 節點位置
     * @returns {Object} 節點對象
     */
    convertStepToNode(step, position) {
        // 查找節點類型
        const nodeTypeId = this.findNodeTypeId(step.function, step.type);
        
        const node = {
            id: this.generateNodeId(step),
            typeId: nodeTypeId,
            position: position,
            inputs: step.inputs || {},
            outputs: step.outputs || {},
            parameters: this.extractStepParameters(step)
        };
        
        // 處理腳本節點的特殊邏輯
        if (step.type === 'script_nodes') {
            this.processScriptNodeLogic(node, step);
        }
        
        return node;
    }
    
    /**
     * 推斷連接關係
     * @param {Array} steps - 步驟列表
     * @returns {Array} 連接列表
     */
    inferConnections(steps) {
        const connections = [];
        
        // 基於步驟順序和輸出輸入匹配推斷連接
        for (let i = 0; i < steps.length - 1; i++) {
            const currentStep = steps[i];
            const nextStep = steps[i + 1];
            
            // 查找輸出輸入匹配
            const connection = this.findOutputInputMatch(currentStep, nextStep);
            if (connection) {
                connections.push(connection);
            }
        }
        
        return connections;
    }
}
```

### 效能最佳化 API
```javascript
/**
 * 效能最佳化管理器
 */
class PerformanceOptimizer {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.isEnabled = false;
        
        // 初始化子系統
        this.batchRenderer = new BatchNodeRenderer();
        this.memoryManager = new MemoryManager();
        this.monitor = new PerformanceMonitor();
        this.resourcePreloader = new ResourcePreloader();
        this.progressiveLoader = new ProgressiveLoader();
        this.viewportCulling = new ViewportCullingManager();
    }
    
    /**
     * 啟用效能最佳化
     */
    enable() {
        if (this.isEnabled) return;
        
        this.batchRenderer.enable();
        this.memoryManager.enableAutoCleanup();
        this.monitor.startMonitoring();
        this.resourcePreloader.preloadCriticalResources();
        
        this.isEnabled = true;
        console.log('✅ 效能最佳化已啟用');
    }
    
    /**
     * 停用效能最佳化
     */
    disable() {
        if (!this.isEnabled) return;
        
        this.batchRenderer.disable();
        this.memoryManager.disableAutoCleanup();
        this.monitor.stopMonitoring();
        
        this.isEnabled = false;
        console.log('❌ 效能最佳化已停用');
    }
    
    /**
     * 獲取效能統計
     * @returns {Object} 效能統計數據
     */
    getPerformanceStats() {
        return {
            fps: this.monitor.getCurrentFPS(),
            memoryUsage: this.monitor.getMemoryUsage(),
            nodeCount: this.flowDesigner.getNodeCount(),
            renderTime: this.batchRenderer.getLastRenderTime(),
            cacheHitRate: this.resourcePreloader.getCacheHitRate()
        };
    }
}

/**
 * 批量節點渲染器
 */
class BatchNodeRenderer {
    constructor(batchSize = 10, renderDelay = 16) {
        this.batchSize = batchSize;
        this.renderDelay = renderDelay;
        this.renderQueue = [];
        this.isRendering = false;
        this.enabled = false;
    }
    
    /**
     * 批量渲染節點
     * @param {Array} nodes - 節點列表
     */
    async batchRenderNodes(nodes) {
        if (!this.enabled) {
            // 直接渲染
            return this.directRender(nodes);
        }
        
        console.log(`🔄 批量渲染 ${nodes.length} 個節點...`);
        
        // 分批處理
        for (let i = 0; i < nodes.length; i += this.batchSize) {
            const batch = nodes.slice(i, i + this.batchSize);
            this.renderQueue.push(batch);
        }
        
        if (!this.isRendering) {
            await this.processRenderQueue();
        }
        
        console.log('✅ 批量渲染完成');
    }
    
    /**
     * 處理渲染隊列
     */
    async processRenderQueue() {
        this.isRendering = true;
        
        while (this.renderQueue.length > 0) {
            const batch = this.renderQueue.shift();
            
            // 渲染當前批次
            await this.renderBatch(batch);
            
            // 等待下一個動畫幀
            await this.waitForNextFrame();
        }
        
        this.isRendering = false;
    }
    
    /**
     * 等待下一個動畫幀
     */
    waitForNextFrame() {
        return new Promise(resolve => {
            requestAnimationFrame(resolve);
        });
    }
}
```

## 🔌 擴展開發指南

### 自定義節點類型開發
```javascript
/**
 * 自定義節點類型範例
 */
class CustomNodeType {
    static create() {
        return {
            id: 'custom_business_logic',
            name: '自定義業務邏輯',
            description: '執行特定的業務邏輯處理',
            dslType: 'action_nodes',
            source: 'custom_business_engine',
            category: 'business',
            icon: '🔧',
            color: '#9b59b6',
            
            inputs: [
                {
                    name: 'business_data',
                    type: 'data',
                    dataType: 'object',
                    required: true,
                    description: '業務數據輸入'
                }
            ],
            
            outputs: [
                {
                    name: 'processed_result',
                    type: 'data', 
                    dataType: 'object',
                    description: '處理結果'
                }
            ],
            
            parameters: [
                {
                    name: 'processing_mode',
                    type: 'select',
                    required: true,
                    defaultValue: 'standard',
                    options: ['standard', 'advanced', 'custom'],
                    description: '處理模式'
                },
                {
                    name: 'timeout_seconds',
                    type: 'number',
                    required: false,
                    defaultValue: 30,
                    validation: { min: 1, max: 300 },
                    description: '超時時間 (秒)'
                }
            ],
            
            // 自定義渲染邏輯
            render(node, container) {
                const element = document.createElement('div');
                element.className = 'custom-node';
                element.innerHTML = `
                    <div class="node-header">
                        <span class="node-icon">${this.icon}</span>
                        <span class="node-title">${this.name}</span>
                    </div>
                    <div class="node-body">
                        <div class="node-inputs"></div>
                        <div class="node-outputs"></div>
                    </div>
                `;
                
                container.appendChild(element);
                return element;
            },
            
            // 自定義驗證邏輯
            validate(node) {
                const errors = [];
                
                // 檢查必需參數
                if (!node.parameters.processing_mode) {
                    errors.push('處理模式為必需參數');
                }
                
                // 檢查輸入連接
                if (!node.inputs || !node.inputs.business_data) {
                    errors.push('缺少業務數據輸入');
                }
                
                return {
                    isValid: errors.length === 0,
                    errors: errors
                };
            },
            
            // YAML DSL 生成自定義邏輯
            toYamlStep(node, stepNumber) {
                return {
                    step: stepNumber,
                    function: this.id,
                    type: this.dslType,
                    source: this.source,
                    inputs: {
                        business_data: node.inputs.business_data || '${business_input}'
                    },
                    outputs: {
                        processed_result: node.outputs.processed_result || 'processing_result'
                    },
                    parameters: {
                        processing_mode: node.parameters.processing_mode,
                        timeout_seconds: node.parameters.timeout_seconds || 30
                    }
                };
            }
        };
    }
}

// 註冊自定義節點類型
window.flowDesigner.nodeTypes.register(CustomNodeType.create());
```

### 自定義 DSL 擴展
```javascript
/**
 * 自定義 DSL 擴展
 */
class CustomDSLExtension {
    constructor() {
        this.name = 'CustomBusinessDSL';
        this.version = '1.0.0';
    }
    
    /**
     * 擴展 YAML 生成器
     */
    extendYamlGenerator(generator) {
        // 添加自定義區塊
        generator.addCustomBlock('business_rules', (flowData) => {
            return this.generateBusinessRules(flowData);
        });
        
        // 添加自定義驗證
        generator.addValidator('business_logic', (yamlData) => {
            return this.validateBusinessLogic(yamlData);
        });
    }
    
    /**
     * 擴展 YAML 解析器
     */
    extendYamlParser(parser) {
        // 添加自定義解析器
        parser.addCustomParser('business_rules', (ruleData) => {
            return this.parseBusinessRules(ruleData);
        });
        
        // 添加自定義節點轉換器
        parser.addNodeConverter('custom_business_logic', (step) => {
            return this.convertCustomBusinessNode(step);
        });
    }
    
    /**
     * 生成業務規則
     */
    generateBusinessRules(flowData) {
        const businessNodes = flowData.nodes.filter(node => 
            node.category === 'business'
        );
        
        const rules = businessNodes.map(node => ({
            rule_id: node.id,
            rule_type: node.parameters.processing_mode,
            conditions: this.extractConditions(node),
            actions: this.extractActions(node)
        }));
        
        return { business_rules: rules };
    }
    
    /**
     * 驗證業務邏輯
     */
    validateBusinessLogic(yamlData) {
        const errors = [];
        
        // 檢查業務規則一致性
        if (yamlData.business_rules) {
            yamlData.business_rules.forEach((rule, index) => {
                if (!this.isValidBusinessRule(rule)) {
                    errors.push(`業務規則 ${index + 1} 無效`);
                }
            });
        }
        
        return {
            isValid: errors.length === 0,
            errors: errors
        };
    }
}

// 註冊 DSL 擴展
const customExtension = new CustomDSLExtension();
window.flowDesigner.yamlGenerator.registerExtension(customExtension);
window.flowDesigner.yamlParser.registerExtension(customExtension);
```

### 插件系統開發
```javascript
/**
 * Flow Designer 插件系統
 */
class FlowDesignerPlugin {
    constructor() {
        this.plugins = new Map();
        this.hooks = new Map();
    }
    
    /**
     * 註冊插件
     * @param {Object} plugin - 插件對象
     */
    register(plugin) {
        if (!plugin.name || !plugin.version) {
            throw new Error('插件必須包含 name 和 version');
        }
        
        // 驗證插件接口
        this.validatePlugin(plugin);
        
        // 註冊插件
        this.plugins.set(plugin.name, plugin);
        
        // 初始化插件
        if (typeof plugin.initialize === 'function') {
            plugin.initialize(window.flowDesigner);
        }
        
        // 註冊鉤子
        if (plugin.hooks) {
            Object.entries(plugin.hooks).forEach(([hookName, handler]) => {
                this.registerHook(hookName, handler);
            });
        }
        
        console.log(`插件已註冊: ${plugin.name} v${plugin.version}`);
    }
    
    /**
     * 執行鉤子
     * @param {string} hookName - 鉤子名稱
     * @param {any} data - 鉤子數據
     */
    executeHook(hookName, data) {
        const hooks = this.hooks.get(hookName) || [];
        
        return hooks.reduce((result, hook) => {
            try {
                return hook(result);
            } catch (error) {
                console.error(`鉤子執行錯誤: ${hookName}`, error);
                return result;
            }
        }, data);
    }
    
    /**
     * 註冊鉤子
     * @param {string} hookName - 鉤子名稱
     * @param {Function} handler - 處理函數
     */
    registerHook(hookName, handler) {
        if (!this.hooks.has(hookName)) {
            this.hooks.set(hookName, []);
        }
        
        this.hooks.get(hookName).push(handler);
    }
}

/**
 * 插件範例：自動保存
 */
class AutoSavePlugin {
    constructor() {
        this.name = 'AutoSave';
        this.version = '1.0.0';
        this.description = '自動保存流程圖';
        
        this.saveInterval = 30000; // 30秒
        this.saveTimer = null;
    }
    
    initialize(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.startAutoSave();
        
        // 註冊事件監聽
        flowDesigner.on('nodeAdded', () => this.markAsModified());
        flowDesigner.on('nodeDeleted', () => this.markAsModified());
        flowDesigner.on('connectionCreated', () => this.markAsModified());
    }
    
    startAutoSave() {
        this.saveTimer = setInterval(() => {
            this.autoSave();
        }, this.saveInterval);
    }
    
    async autoSave() {
        if (!this.isModified) return;
        
        try {
            const flowData = this.flowDesigner.extractCurrentFlow();
            const yamlContent = this.flowDesigner.generateYamlDsl();
            
            // 保存到本地存儲
            localStorage.setItem('autosave_flow', yamlContent);
            localStorage.setItem('autosave_timestamp', Date.now().toString());
            
            this.isModified = false;
            console.log('✅ 自動保存完成');
            
        } catch (error) {
            console.error('❌ 自動保存失敗:', error);
        }
    }
    
    markAsModified() {
        this.isModified = true;
    }
    
    // 插件鉤子
    hooks = {
        'before_yaml_generate': (yamlData) => {
            // 在 YAML 生成前添加自動保存標記
            yamlData._autosaved = true;
            yamlData._autosave_timestamp = new Date().toISOString();
            return yamlData;
        },
        
        'after_flow_load': (flowData) => {
            // 流程載入後重置修改標記
            this.isModified = false;
            return flowData;
        }
    };
}

// 註冊插件
window.flowDesigner.plugins.register(new AutoSavePlugin());
```

## 🧪 集成測試指導

### 測試架構
```javascript
/**
 * Flow Designer 測試框架
 */
class FlowDesignerTestFramework {
    constructor() {
        this.testSuites = new Map();
        this.mockData = new Map();
        this.testResults = [];
    }
    
    /**
     * 註冊測試套件
     * @param {string} suiteName - 測試套件名稱
     * @param {Object} testSuite - 測試套件對象
     */
    registerTestSuite(suiteName, testSuite) {
        this.testSuites.set(suiteName, testSuite);
    }
    
    /**
     * 運行所有測試
     */
    async runAllTests() {
        console.log('🚀 開始運行所有測試套件...');
        
        for (const [suiteName, testSuite] of this.testSuites) {
            console.log(`📋 運行測試套件: ${suiteName}`);
            
            try {
                const result = await this.runTestSuite(testSuite);
                this.testResults.push({
                    suite: suiteName,
                    ...result
                });
            } catch (error) {
                console.error(`❌ 測試套件失敗: ${suiteName}`, error);
                this.testResults.push({
                    suite: suiteName,
                    passed: false,
                    error: error.message
                });
            }
        }
        
        return this.generateTestReport();
    }
    
    /**
     * 運行單個測試套件
     * @param {Object} testSuite - 測試套件
     */
    async runTestSuite(testSuite) {
        const results = {
            passed: 0,
            failed: 0,
            tests: []
        };
        
        // 設置測試環境
        if (testSuite.setup) {
            await testSuite.setup();
        }
        
        // 運行測試
        for (const [testName, testFunction] of Object.entries(testSuite.tests)) {
            try {
                await testFunction();
                results.passed++;
                results.tests.push({ name: testName, status: 'PASS' });
                console.log(`  ✅ ${testName}`);
            } catch (error) {
                results.failed++;
                results.tests.push({ 
                    name: testName, 
                    status: 'FAIL', 
                    error: error.message 
                });
                console.error(`  ❌ ${testName}: ${error.message}`);
            }
        }
        
        // 清理測試環境
        if (testSuite.teardown) {
            await testSuite.teardown();
        }
        
        return results;
    }
}

/**
 * 節點操作測試套件
 */
class NodeOperationTestSuite {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
    }
    
    async setup() {
        // 初始化測試環境
        this.flowDesigner.clearFlow();
        this.originalNodeCount = this.flowDesigner.getNodeCount();
    }
    
    async teardown() {
        // 清理測試環境
        this.flowDesigner.clearFlow();
    }
    
    tests = {
        async 'test_add_condition_node'() {
            const nodeId = this.flowDesigner.addNode('check_agv_rotation_flow', 100, 100);
            
            // 驗證節點已添加
            const nodeElement = document.getElementById(nodeId);
            if (!nodeElement) {
                throw new Error('節點元素未創建');
            }
            
            // 驗證節點類型
            const nodeType = this.flowDesigner.nodeTypes['check_agv_rotation_flow_condition_nodes'];
            if (!nodeType) {
                throw new Error('節點類型未找到');
            }
            
            console.log('✅ 條件節點添加成功');
        },
        
        async 'test_create_node_connection'() {
            // 添加兩個節點
            const node1Id = this.flowDesigner.addNode('check_agv_rotation_flow', 100, 100);
            const node2Id = this.flowDesigner.addNode('get_room_inlet_point', 300, 100);
            
            // 創建連接
            const connectionId = this.flowDesigner.createConnection(
                node1Id, 'output', 
                node2Id, 'input'
            );
            
            // 驗證連接已創建
            const connectionElement = document.querySelector(`[data-connection-id="${connectionId}"]`);
            if (!connectionElement) {
                throw new Error('連接元素未創建');
            }
            
            console.log('✅ 節點連接創建成功');
        },
        
        async 'test_yaml_generation'() {
            // 創建測試流程
            const node1Id = this.flowDesigner.addNode('check_agv_rotation_flow', 100, 100);
            const node2Id = this.flowDesigner.addNode('create_task_from_decision', 300, 100);
            this.flowDesigner.createConnection(node1Id, 'output', node2Id, 'input');
            
            // 生成 YAML
            const yamlContent = this.flowDesigner.generateYamlDsl();
            
            // 驗證 YAML 內容
            if (!yamlContent || yamlContent.trim().length === 0) {
                throw new Error('YAML 內容為空');  
            }
            
            // 驗證 YAML 格式
            const parsedYaml = jsyaml.load(yamlContent);
            if (!parsedYaml.steps || parsedYaml.steps.length !== 2) {
                throw new Error('YAML 結構不正確');
            }
            
            console.log('✅ YAML 生成測試通過');
        }
    };  
}

// 註冊測試套件
const testFramework = new FlowDesignerTestFramework();
testFramework.registerTestSuite('NodeOperations', new NodeOperationTestSuite(window.flowDesigner));
```

### 性能測試
```javascript
/**
 * 效能測試套件
 */
class PerformanceTestSuite {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.performanceData = [];
    }
    
    async setup() {
        // 啟用效能監控
        if (window.PerformanceOptimizer) {
            window.PerformanceOptimizer.enable();
        }
        
        this.flowDesigner.clearFlow();
    }
    
    tests = {
        async 'test_large_flow_rendering'() {
            const startTime = performance.now();
            const startMemory = this.getMemoryUsage();
            
            // 創建大型流程 (50個節點)
            const nodeIds = [];
            for (let i = 0; i < 50; i++) {
                const x = (i % 10) * 150 + 100;
                const y = Math.floor(i / 10) * 150 + 100;
                const nodeType = ['check_agv_rotation_flow', 'get_room_inlet_point', 'create_task_from_decision'][i % 3];
                
                const nodeId = this.flowDesigner.addNode(nodeType, x, y);
                nodeIds.push(nodeId);
            }
            
            // 創建連接
            for (let i = 0; i < nodeIds.length - 1; i++) {
                this.flowDesigner.createConnection(nodeIds[i], 'output', nodeIds[i + 1], 'input');
            }
            
            const endTime = performance.now();
            const endMemory = this.getMemoryUsage();
            
            // 效能指標
            const renderTime = endTime - startTime;
            const memoryIncrease = endMemory - startMemory;
            
            console.log(`渲染時間: ${renderTime.toFixed(2)}ms`);
            console.log(`記憶體增長: ${memoryIncrease.toFixed(2)}MB`);
            
            // 驗證效能標準
            if (renderTime > 5000) { // 5秒
                throw new Error(`渲染時間過長: ${renderTime.toFixed(2)}ms`);
            }
            
            if (memoryIncrease > 100) { // 100MB
                throw new Error(`記憶體使用過多: ${memoryIncrease.toFixed(2)}MB`);
            }
        },
        
        async 'test_yaml_generation_performance'() {
            // 創建複雜流程
            for (let i = 0; i < 20; i++) {
                const nodeType = ['check_agv_rotation_flow', 'get_room_inlet_point'][i % 2];
                this.flowDesigner.addNode(nodeType, i * 100, 100);
            }
            
            // 測試 YAML 生成效能
            const iterations = 10;
            const times = [];
            
            for (let i = 0; i < iterations; i++) {
                const startTime = performance.now();
                const yamlContent = this.flowDesigner.generateYamlDsl();
                const endTime = performance.now();
                
                times.push(endTime - startTime);
            }
            
            const avgTime = times.reduce((a, b) => a + b, 0) / times.length;
            console.log(`YAML 生成平均時間: ${avgTime.toFixed(2)}ms`);
            
            if (avgTime > 1000) { // 1秒
                throw new Error(`YAML 生成時間過長: ${avgTime.toFixed(2)}ms`);
            }
        }
    };
    
    getMemoryUsage() {
        if (performance.memory) {
            return performance.memory.usedJSHeapSize / 1024 / 1024;
        }
        return 0;
    }
}

// 註冊效能測試
testFramework.registerTestSuite('Performance', new PerformanceTestSuite(window.flowDesigner));
```

## 📚 API 參考速查

### 主要類別和方法
```javascript
// FlowDesigner 主類
window.flowDesigner = new FlowDesigner('editor-container')
window.flowDesigner.addNode(nodeTypeId, x, y)
window.flowDesigner.createConnection(sourceId, output, targetId, input)
window.flowDesigner.generateYamlDsl()
window.flowDesigner.parseDslToFlow(yamlContent)
window.flowDesigner.clearFlow()

// 節點類型系統
window.flowDesigner.nodeTypes.register(nodeType)
window.flowDesigner.nodeTypes.get(nodeId)
window.flowDesigner.nodeTypes.findByFunction(functionName, dslType)

// 效能最佳化
window.PerformanceOptimizer.enable()
window.PerformanceOptimizer.disable()
window.PerformanceOptimizer.getPerformanceStats()

// 測試系統
window.runFlowDesignerTests()
window.FlowDesignerTestSuite
window.FlowDesignerDiagnostics.runFullDiagnostics()

// 插件系統
window.flowDesigner.plugins.register(plugin)
window.flowDesigner.plugins.executeHook(hookName, data)
```

### 事件系統
```javascript
// 監聽事件
window.flowDesigner.on('nodeAdded', (data) => {
    console.log('節點已添加:', data);
});

window.flowDesigner.on('connectionCreated', (connection) => {
    console.log('連接已創建:', connection);
});

window.flowDesigner.on('flowLoaded', (flowData) => {
    console.log('流程已載入:', flowData);
});

// 觸發自定義事件
window.flowDesigner.emit('customEvent', { data: 'example' });
```

## 🔗 相關資源

- **完整使用手冊**: 用戶操作指導
- **最佳實踐指南**: 企業級設計模式
- **故障排除手冊**: 問題診斷和解決
- **系統架構文檔**: 深入技術細節
- **YAML DSL 語法規範**: DSL 語法完整參考

---

📝 **文檔版本**: v1.0  
📅 **更新日期**: 2025-08-15  
👥 **目標用戶**: 系統開發者、技術集成人員、架構師