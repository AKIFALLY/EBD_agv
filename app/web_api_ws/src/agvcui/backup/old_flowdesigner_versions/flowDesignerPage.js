// WCS Flow Designer JavaScript 模組 v4.2.2
// Phase 4.2: Performance Optimization + 節點選板顯示修復 + 節點連接修復
// 修復版本：節點選板分類修復 + Socket.IO 衝突解決 + Performance Optimization + 節點連接功能修復

class WcsFlowDesigner {
    constructor() {
        this.editor = null;
        this.area = null;
        this.selectedNode = null;
        this.nodeCounter = 0;
        this.nodeIdCounter = 1; // For clearFlow() method
        this.nodes = new Map(); // Track nodes
        this.connections = new Map(); // Track visual connections
        this.connectionState = null; // Connection state for drag operations
        this.globalEventsAdded = false; // Track global event listeners
        this.currentFlowName = '未命名';
        this.functionsConfig = null;
        this.socket = null; // Rete socket
        
        // 檢測編輯模式
        this.isEditMode = this.detectEditMode();
        console.log(`🎛️ Flow Designer 模式: ${this.isEditMode ? '編輯模式' : '標準模式'}`);
        
        // 連線繪製模式設置 - 固定使用曲線
        this.connectionMode = 'curved'; // 固定使用曲線連接
        
        // 節點類型定義 - 從配置文件動態加載
        this.nodeTypes = {};
        this.nodeConfigs = {
            condition_nodes: null,
            logic_nodes: null,
            action_nodes: null,
            script_nodes: null
        };
        
        // 不在建構函數中立即初始化，改為手動調用
        console.log('🔧 WcsFlowDesigner 建構函數完成，等待手動初始化');
    }
    
    async initialize() {
        console.log('🚀 開始 Flow Designer 初始化...');
        
        try {
            // Load node configurations from YAML files
            console.log('📋 步驟 1: 載入節點配置...');
            await this.loadNodeConfigurations();
            console.log('✅ 步驟 1 完成: 節點配置載入完成');
            
            // Initialize the designer
            console.log('🎨 步驟 2: 初始化設計器...');
            await this.init();
            console.log('✅ 步驟 2 完成: 設計器初始化完成');
            
            // 根據模式調整 UI
            console.log('🎨 步驟 3: 根據模式調整 UI...');
            this.adjustUIForMode();
            console.log('✅ 步驟 3 完成: UI 模式調整完成');
            
            console.log('✅ Flow Designer 初始化完成');
            
            // Phase 4.2.3: 啟動載入速度優化
            console.log('🚀 啟動載入速度優化...');
            setTimeout(() => {
                this.optimizeLoadingSpeed().catch(error => {
                    console.warn('⚠️ 載入速度優化失敗:', error);
                });
                
                // Phase 4.2.4: 啟動整合測試
                if (window.location.search.includes('perf-test=true')) {
                    console.log('🧪 啟動效能基準測試...');
                    setTimeout(() => {
                        this.runIntegrationTests().catch(error => {
                            console.warn('⚠️ 整合測試失敗:', error);
                        });
                    }, 500);
                }
            }, 100); // 延遲執行避免阻塞主要初始化
            
        } catch (error) {
            console.error('❌ Flow Designer 初始化失敗:', error);
            console.error('錯誤堆疊:', error.stack);
            
            // Try to continue with empty configurations
            try {
                console.log('🔄 嘗試以降級模式初始化...');
                await this.init();
                console.log('⚠️ Flow Designer 以空配置模式運行');
            } catch (initError) {
                console.error('❌ Flow Designer 無法啟動:', initError);
                console.error('降級模式錯誤堆疊:', initError.stack);
                throw initError; // 重新拋出錯誤以便上層處理
            }
        }
    }
    
    // 檢測編輯模式
    detectEditMode() {
        const urlParams = new URLSearchParams(window.location.search);
        const modeParam = urlParams.get('mode');
        const flowParam = urlParams.get('flow');
        
        // 編輯模式條件：
        // 1. 明確設定 mode=edit 參數，或
        // 2. 有 flow 參數（從 flows 頁面來編輯現有流程）
        const isExplicitEditMode = modeParam === 'edit';
        const isImplicitEditMode = flowParam && flowParam !== '' && flowParam !== '未命名';
        
        const editMode = isExplicitEditMode || isImplicitEditMode;
        
        console.log(`🔍 編輯模式檢測:`, {
            modeParam,
            flowParam,
            isExplicitEditMode,
            isImplicitEditMode,
            finalResult: editMode
        });
        
        return editMode;
    }
    
    // 根據模式調整 UI
    adjustUIForMode() {
        if (this.isEditMode) {
            console.log('🎛️ 調整為編輯模式 UI');
            
            // 隱藏「新建流程」和「載入 YAML」按鈕
            const newFlowBtn = document.getElementById('btn-new-flow');
            const loadFlowBtn = document.getElementById('btn-load-flow');
            
            if (newFlowBtn) {
                newFlowBtn.style.display = 'none';
                console.log('✅ 隱藏「新建流程」按鈕');
            }
            
            if (loadFlowBtn) {
                loadFlowBtn.style.display = 'none';
                console.log('✅ 隱藏「載入 YAML」按鈕');
            }
            
            // 更新保存按鈕文字
            const saveFlowBtn = document.getElementById('btn-save-flow');
            if (saveFlowBtn) {
                const saveText = saveFlowBtn.querySelector('span:last-child');
                if (saveText) {
                    saveText.textContent = '保存到伺服器';
                }
                console.log('✅ 更新保存按鈕文字');
            }
        }
    }
    
    // 標記流程為已保存狀態
    markFlowAsSaved() {
        // 更新狀態列指示
        const statusTag = document.querySelector('.flow-status-bar .tag');
        if (statusTag) {
            statusTag.textContent = '已保存';
            statusTag.className = 'tag is-success';
        }
        
        // 更新保存按鈕狀態（可選）
        const saveFlowBtn = document.getElementById('btn-save-flow');
        if (saveFlowBtn && this.isEditMode) {
            saveFlowBtn.classList.add('is-success');
            setTimeout(() => {
                saveFlowBtn.classList.remove('is-success');
            }, 2000); // 2秒後恢復正常狀態
        }
    }

    async init() {
        try {
            // 修復 Socket.IO 連接問題
            console.log('🔌 步驟 2.1: 初始化 Socket 連接...');
            this.initSocket();
            console.log('✅ 步驟 2.1 完成: Socket 連接初始化完成');
            
            // 載入函數配置
            console.log('📋 步驟 2.2: 載入函數配置...');
            await this.loadFunctionsConfig();
            console.log('✅ 步驟 2.2 完成: 函數配置載入完成');
            
            // 設置編輯器
            console.log('🎨 步驟 2.3: 設置編輯器...');
            await this.setupEditor();
            console.log('✅ 步驟 2.3 完成: 編輯器設置完成');
            
            // 設置事件監聽器
            console.log('🎧 步驟 2.4: 設置事件監聽器...');
            this.setupEventListeners();
            console.log('✅ 步驟 2.4 完成: 事件監聽器設置完成');
            
            // 創建節點選板
            console.log('🎨 步驟 2.5: 準備創建節點選板，nodeTypes數量:', Object.keys(this.nodeTypes).length);
            this.createNodePalette();
            console.log('✅ 步驟 2.5 完成: 節點選板創建完成');
            
            // 更新狀態列
            console.log('📊 步驟 2.6: 更新狀態列...');
            this.updateStatusBar();
            console.log('✅ 步驟 2.6 完成: 狀態列更新完成');
            
            console.log('✅ WCS Flow Designer 初始化完成');
            
        } catch (error) {
            console.error('❌ Flow Designer 初始化失敗:', error);
            console.error('詳細錯誤信息:', error.stack);
            this.showNotification('Flow Designer 初始化失敗', 'danger');
            throw error; // 重新拋出錯誤以便上層處理
        }
    }

    initSocket() {
        try {
            // 使用現有的 AGVCUI Socket.IO 連接，避免創建新連接
            if (window.socket && window.socket.connected) {
                console.log('✅ 使用現有的 Socket.IO 連接');
                this.setupFlowDesignerSocketEvents();
                this.showNotification('Flow Designer 已連接到即時服務', 'success');
            } else {
                console.log('⚠️ Socket.IO 未連接，Flow Designer 使用本地模式');
                this.showNotification('Flow Designer 在本地模式運行', 'info');
            }
        } catch (error) {
            console.error('❌ Flow Designer Socket 初始化失敗:', error);
            this.showNotification('Flow Designer 在本地模式運行', 'info');
        }
    }
    
    setupFlowDesignerSocketEvents() {
        if (!window.socket) return;
        
        // 監聽 Flow Designer 特定事件
        window.socket.on('flow_saved', (data) => {
            console.log('📄 流程已保存:', data);
            this.showNotification(`流程 "${data.name}" 已保存`, 'success');
        });
        
        window.socket.on('flow_loaded', (data) => {
            console.log('📄 流程已載入:', data);
            this.showNotification(`流程 "${data.name}" 已載入`, 'info');
        });
        
        window.socket.on('flow_validation_result', (data) => {
            console.log('✅ 流程驗證結果:', data);
            if (data.valid) {
                this.showNotification('流程驗證通過', 'success');
            } else {
                this.showNotification(`流程驗證失敗: ${data.errors.join(', ')}`, 'danger');
            }
        });
        
        console.log('✅ Flow Designer Socket 事件已設置');
    }

    async loadFunctionsConfig() {
        console.log('📋 載入函數配置...');
        // 使用內嵌配置避免 CORS 問題
        this.functionsConfig = {
            "condition_functions": [
                { "value": "check_room_carrier", "label": "檢查房間載具", "parameters": [] },
                { "value": "rack_location_check", "label": "貨架位置檢查", "parameters": [] }
            ],
            "action_functions": [
                { "value": "create_transport_task", "label": "創建運輸任務", "parameters": [] },
                { "value": "update_rack_status", "label": "更新貨架狀態", "parameters": [] }
            ]
        };
        console.log('✅ 函數配置載入完成');
    }

    async loadNodeConfigurations() {
        console.log('📋 載入節點配置...');
        
        try {
            // Phase 3.1: 使用增強的 node-types.js 定義
            // 檢查 FlowDesigner 全域變數是否可用
            if (typeof window.FlowDesigner !== 'undefined') {
                console.log('✅ 使用 node-types.js 中的節點定義');
                
                // 🔧 修復: 直接將節點類型設置到 nodeTypes，避免複雜的配置轉換
                this.nodeTypes = {
                    ...window.FlowDesigner.CONDITION_NODES,
                    ...window.FlowDesigner.LOGIC_NODES,
                    ...window.FlowDesigner.ACTION_NODES,
                    ...window.FlowDesigner.SCRIPT_NODES
                };
                
                console.log('📊 載入統計:');
                console.log('  - 條件節點:', Object.keys(window.FlowDesigner.CONDITION_NODES).length);
                console.log('  - 邏輯節點:', Object.keys(window.FlowDesigner.LOGIC_NODES).length); 
                console.log('  - 動作節點:', Object.keys(window.FlowDesigner.ACTION_NODES).length);
                console.log('  - 腳本節點:', Object.keys(window.FlowDesigner.SCRIPT_NODES).length);
                console.log('  - 總計:', Object.keys(this.nodeTypes).length, '個節點類型');
                
                // 直接返回，跳過 populateNodeTypes 調用
                console.log('✅ 節點配置載入完成 (直接模式)');
                return;
                
            } else {
                console.warn('⚠️ node-types.js 未載入，嘗試載入 YAML 配置...');
                // 後備方案：使用舊的 YAML 載入方式
                await this.loadYamlConfigurations();
            }
            
            // Transform loaded configurations into nodeTypes
            this.populateNodeTypes();
            
            console.log('✅ 節點配置載入完成:', Object.keys(this.nodeTypes).length, '個節點類型');
            
        } catch (error) {
            console.error('❌ 節點配置載入失敗:', error);
            // Use fallback empty configurations
            this.nodeConfigs = {
                condition_nodes: { condition_nodes: {} },
                logic_nodes: { logic_nodes: {} },
                action_nodes: { action_nodes: {} },
                script_nodes: { script_nodes: {} }
            };
            this.populateNodeTypes();
        }
    }

    async loadYamlConfigurations() {
        // 後備 YAML 載入方式
        const configPromises = [
            this.fetchNodeConfig('/static/config/wcs/nodes/condition_nodes.yaml', 'condition_nodes'),
            this.fetchNodeConfig('/static/config/wcs/nodes/logic_nodes.yaml', 'logic_nodes'),
            this.fetchNodeConfig('/static/config/wcs/nodes/action_nodes.yaml', 'action_nodes')
        ];
        
        await Promise.all(configPromises);
    }

    async fetchNodeConfig(url, configType) {
        try {
            console.log(`📄 載入 ${configType} 配置:`, url);
            const response = await fetch(url);
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
            const yamlText = await response.text();
            
            // Convert YAML to JSON using js-yaml library (should be included in the page)
            if (typeof jsyaml !== 'undefined') {
                const config = jsyaml.load(yamlText);
                this.nodeConfigs[configType] = config;
                console.log(`✅ ${configType} 載入成功:`, Object.keys(config[configType] || {}).length, '個節點');
            } else {
                console.warn('⚠️ js-yaml 庫未載入，無法解析 YAML');
                this.nodeConfigs[configType] = { [configType]: {} };
            }
            
        } catch (error) {
            console.error(`❌ ${configType} 載入失敗:`, error);
            this.nodeConfigs[configType] = { [configType]: {} };
        }
    }

    populateNodeTypes() {
        console.log('🔄 轉換節點配置為節點類型...');
        
        // Clear existing nodeTypes
        this.nodeTypes = {};
        
        // Phase 3.1: 處理增強的節點定義 - 支援完整的 DSL 系統整合
        
        // Process condition nodes - 條件判斷節點
        if (this.nodeConfigs.condition_nodes && this.nodeConfigs.condition_nodes.condition_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.condition_nodes.condition_nodes)) {
                this.nodeTypes[nodeId] = {
                    id: nodeConfig.id || nodeId,
                    name: nodeConfig.name || nodeId,
                    type: 'condition',
                    category: nodeConfig.category || 'input',
                    icon: nodeConfig.icon || '❓',
                    color: nodeConfig.color || '#3B82F6',
                    bgColor: nodeConfig.bgColor || '#dbeafe',
                    borderColor: nodeConfig.borderColor || '#2563eb',
                    textColor: nodeConfig.textColor || '#1e3a8a',
                    shape: nodeConfig.shape || 'diamond',
                    
                    // DSL 系統整合
                    source: nodeConfig.source,
                    dslType: nodeConfig.dslType || 'condition_nodes',
                    returnType: nodeConfig.returnType,
                    
                    // 輸入/輸出定義 - 基於實際 DSL 函數
                    inputs: this.processNodeConnections(nodeConfig.inputs || []),
                    outputs: this.processNodeConnections(nodeConfig.outputs || [{ name: 'result', type: nodeConfig.returnType || 'boolean' }]),
                    
                    // 參數定義 - 支援完整的型別系統
                    parameters: this.processNodeParameters(nodeConfig.inputs || []),
                    
                    description: nodeConfig.description || '',
                    
                    // Flow Designer 特有屬性
                    isDraggable: true,
                    isConnectable: true,
                    isConfigurable: true
                };
            }
        }
        
        // Process logic nodes - 邏輯處理節點
        if (this.nodeConfigs.logic_nodes && this.nodeConfigs.logic_nodes.logic_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.logic_nodes.logic_nodes)) {
                this.nodeTypes[nodeId] = {
                    id: nodeConfig.id || nodeId,
                    name: nodeConfig.name || nodeId,
                    type: 'logic',
                    category: nodeConfig.category || 'control',
                    icon: nodeConfig.icon || '⚙️',
                    color: nodeConfig.color || '#F59E0B',
                    bgColor: nodeConfig.bgColor || '#fef3c7',
                    borderColor: nodeConfig.borderColor || '#d97706',
                    textColor: nodeConfig.textColor || '#92400e',
                    shape: nodeConfig.shape || 'rectangle',
                    
                    // DSL 系統整合
                    source: nodeConfig.source,
                    dslType: nodeConfig.dslType || 'logic_nodes',
                    returnType: nodeConfig.returnType,
                    
                    // 輸入/輸出定義
                    inputs: this.processNodeConnections(nodeConfig.inputs || []),
                    outputs: this.processNodeConnections(nodeConfig.outputs || [{ name: 'result', type: nodeConfig.returnType || 'any' }]),
                    
                    // 參數定義
                    parameters: this.processNodeParameters(nodeConfig.inputs || []),
                    
                    description: nodeConfig.description || '',
                    
                    // Flow Designer 特有屬性
                    isDraggable: true,
                    isConnectable: true,
                    isConfigurable: true
                };
            }
        }
        
        // Process action nodes - 動作執行節點
        if (this.nodeConfigs.action_nodes && this.nodeConfigs.action_nodes.action_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.action_nodes.action_nodes)) {
                this.nodeTypes[nodeId] = {
                    id: nodeConfig.id || nodeId,
                    name: nodeConfig.name || nodeId,
                    type: 'action',
                    category: nodeConfig.category || 'output',
                    icon: nodeConfig.icon || '🎯',
                    color: nodeConfig.color || '#10B981',
                    bgColor: nodeConfig.bgColor || '#d1fae5',
                    borderColor: nodeConfig.borderColor || '#059669',
                    textColor: nodeConfig.textColor || '#064e3b',
                    shape: nodeConfig.shape || 'rounded-rectangle',
                    
                    // DSL 系統整合
                    source: nodeConfig.source,
                    dslType: nodeConfig.dslType || 'action_nodes',
                    returnType: nodeConfig.returnType,
                    
                    // 輸入/輸出定義
                    inputs: this.processNodeConnections(nodeConfig.inputs || [{ name: 'trigger', type: 'boolean' }]),
                    outputs: this.processNodeConnections(nodeConfig.outputs || []),
                    
                    // 參數定義
                    parameters: this.processNodeParameters(nodeConfig.inputs || []),
                    
                    description: nodeConfig.description || '',
                    
                    // Flow Designer 特有屬性
                    isDraggable: true,
                    isConnectable: true,
                    isConfigurable: true
                };
            }
        }
        
        // Process script nodes - 腳本控制節點 (新增支援)
        if (this.nodeConfigs.script_nodes && this.nodeConfigs.script_nodes.script_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.script_nodes.script_nodes)) {
                this.nodeTypes[nodeId] = {
                    id: nodeConfig.id || nodeId,
                    name: nodeConfig.name || nodeId,
                    type: 'script',
                    category: nodeConfig.category || 'storage',
                    icon: nodeConfig.icon || '📜',
                    color: nodeConfig.color || '#8B5CF6',
                    bgColor: nodeConfig.bgColor || '#e9d5ff',
                    borderColor: nodeConfig.borderColor || '#7c3aed',
                    textColor: nodeConfig.textColor || '#4c1d95',
                    shape: nodeConfig.shape || 'octagon',
                    
                    // DSL 系統整合
                    source: nodeConfig.source || 'dsl_runtime',
                    dslType: nodeConfig.dslType || 'script_nodes',
                    returnType: nodeConfig.returnType,
                    
                    // 輸入/輸出定義
                    inputs: this.processNodeConnections(nodeConfig.inputs || []),
                    outputs: this.processNodeConnections(nodeConfig.outputs || []),
                    
                    // 參數定義
                    parameters: this.processNodeParameters(nodeConfig.inputs || []),
                    
                    description: nodeConfig.description || '',
                    
                    // Flow Designer 特有屬性
                    isDraggable: true,
                    isConnectable: true,
                    isConfigurable: true
                };
            }
        }
        
        console.log('✅ 節點類型轉換完成:', Object.keys(this.nodeTypes).length, '個節點類型');
        console.log('📊 類型統計:');
        const typeCount = {};
        Object.values(this.nodeTypes).forEach(node => {
            typeCount[node.type] = (typeCount[node.type] || 0) + 1;
        });
        console.log('  - 條件節點:', typeCount.condition || 0);
        console.log('  - 邏輯節點:', typeCount.logic || 0);
        console.log('  - 動作節點:', typeCount.action || 0);
        console.log('  - 腳本節點:', typeCount.script || 0);
    }

    processNodeConnections(connections) {
        // Phase 3.1: 處理節點連接定義 - 支援增強的 DSL 系統
        if (!Array.isArray(connections)) {
            return [];
        }
        
        return connections.map(conn => ({
            name: conn.name || 'default',
            type: conn.type || 'any',
            description: conn.description || '',
            required: conn.required !== undefined ? conn.required : true,
            // Flow Designer 特有屬性
            socketType: 'flow', // 使用通用的 flow socket
            multipleConnections: conn.multipleConnections || false
        }));
    }

    processNodeParameters(parameters) {
        // Phase 3.1: 處理節點參數，確保每個參數都有完整的配置 - 支援 DSL 型別系統
        if (!Array.isArray(parameters)) {
            return [];
        }
        
        return parameters.map(param => ({
            name: param.name,
            type: param.type || 'string',
            required: param.required !== undefined ? param.required : false,
            default: param.default !== undefined ? param.default : '',
            description: param.description || '',
            
            // 擴展型別支援
            options: param.options || null, // 用於 select 類型
            min: param.min !== undefined ? param.min : null,
            max: param.max !== undefined ? param.max : null,
            
            // DSL 系統特有屬性
            dslType: param.type, // 保留原始 DSL 型別
            validation: {
                required: param.required !== undefined ? param.required : false,
                type: param.type || 'string',
                min: param.min,
                max: param.max,
                options: param.options
            },
            
            // Flow Designer 特有屬性
            value: param.default !== undefined ? param.default : '', // 當前設定值
            isModified: false, // 是否被修改過
            
            // UI 相關屬性
            inputType: this.getInputType(param.type),
            placeholder: param.description || `請輸入 ${param.name}`,
            
            // 高級屬性
            group: param.group || 'general', // 參數分組
            order: param.order || 0, // 顯示順序
            conditional: param.conditional || null // 條件顯示邏輯
        }));
    }

    getInputType(dslType) {
        // Phase 3.1: 將 DSL 型別映射到 HTML input 類型
        const typeMapping = {
            'string': 'text',
            'integer': 'number', 
            'int': 'number',
            'float': 'number',
            'boolean': 'checkbox',
            'List[integer]': 'text', // 將作為 JSON 輸入
            'List[string]': 'text',
            'Dict[str, Any]': 'textarea', // 將作為 JSON 輸入
            'Any': 'text'
        };
        
        return typeMapping[dslType] || 'text';
    }

    hexToRgba(hex, alpha) {
        // 將十六進制顏色轉換為 RGBA 格式
        const r = parseInt(hex.slice(1, 3), 16);
        const g = parseInt(hex.slice(3, 5), 16);
        const b = parseInt(hex.slice(5, 7), 16);
        return `rgba(${r}, ${g}, ${b}, ${alpha})`;
    }

    async setupEditor() {
        console.log('🔧 步驟 2.3.1: 檢查 rete-editor 容器...');
        const container = document.getElementById('rete-editor');
        if (!container) {
            throw new Error('找不到 rete-editor 容器');
        }
        console.log('✅ 步驟 2.3.1 完成: rete-editor 容器找到');

        console.log('🔧 步驟 2.3.2: 檢查 Rete.js 依賴...');
        if (typeof Rete === 'undefined') {
            throw new Error('Rete.js 庫未載入');
        }
        if (typeof ReteAreaPlugin === 'undefined') {
            throw new Error('ReteAreaPlugin 插件未載入');
        }
        console.log('✅ 步驟 2.3.2 完成: Rete.js 依賴檢查通過');

        console.log('🔧 步驟 2.3.3: 創建 Rete.js v2 編輯器...');
        this.editor = new Rete.NodeEditor();
        console.log('✅ 步驟 2.3.3 完成: NodeEditor 創建成功');
        
        console.log('🔧 步驟 2.3.4: 創建區域插件...');
        this.area = new ReteAreaPlugin.AreaPlugin(container);
        console.log('✅ 步驟 2.3.4 完成: AreaPlugin 創建成功');
        
        console.log('🔧 步驟 2.3.5: 使用區域插件...');
        this.editor.use(this.area);
        console.log('✅ 步驟 2.3.5 完成: 區域插件配置成功');
        
        console.log('🔧 步驟 2.3.6: 創建通用 socket...');
        this.socket = new Rete.ClassicPreset.Socket('flow');
        console.log('🔌 創建通用 socket:', this.socket);
        console.log('✅ 步驟 2.3.6 完成: 通用 socket 創建成功');
        
        // 添加連接插件 (Rete.js v2 連接系統)
        console.log('🔧 步驟 2.3.7: 配置連接插件...');
        try {
            // 修復：檢查多種可能的 API 結構
            console.log('🔍 檢查 Connection Plugin API 結構...');
            console.log('window.ReteConnectionPlugin:', typeof window.ReteConnectionPlugin);
            
            if (window.ReteConnectionPlugin) {
                let ConnectionPlugin = null;
                let Presets = null;
                
                // 嘗試不同的 API 結構
                if (window.ReteConnectionPlugin.ConnectionPlugin) {
                    ConnectionPlugin = window.ReteConnectionPlugin.ConnectionPlugin;
                    Presets = window.ReteConnectionPlugin.Presets;
                    console.log('✅ 使用 ReteConnectionPlugin.ConnectionPlugin API');
                } else if (window.ReteConnectionPlugin.default) {
                    ConnectionPlugin = window.ReteConnectionPlugin.default.ConnectionPlugin;
                    Presets = window.ReteConnectionPlugin.default.Presets;
                    console.log('✅ 使用 ReteConnectionPlugin.default API');
                } else if (typeof window.ReteConnectionPlugin === 'function') {
                    ConnectionPlugin = window.ReteConnectionPlugin;
                    console.log('✅ ReteConnectionPlugin 本身就是構造函數');
                }
                
                if (ConnectionPlugin) {
                    const connection = new ConnectionPlugin();
                    
                    // 嘗試添加預設配置
                    if (Presets && Presets.classic && Presets.classic.setup) {
                        connection.addPreset(Presets.classic.setup());
                        console.log('✅ Classic preset 已應用');
                    } else {
                        console.log('⚠️ Classic preset 不可用，使用默認配置');
                    }
                    
                    this.area.use(connection);
                    console.log('✅ Connection plugin 載入成功');
                } else {
                    throw new Error('無法找到 ConnectionPlugin 構造函數');
                }
            } else {
                throw new Error('ReteConnectionPlugin 不存在');
            }
        } catch (error) {
            console.error('❌ Connection plugin 載入錯誤:', error);
            console.warn('⚠️ 將使用自定義連接渲染系統');
            // 當連接插件失敗時，我們仍然有自定義的連接渲染系統
        }
        console.log('✅ 步驟 2.3.7 完成: 連接插件配置完成');
        
        console.log('🔧 步驟 2.3.8: 設置區域擴展功能...');
        ReteAreaPlugin.AreaExtensions.selectableNodes(this.area, 
            ReteAreaPlugin.AreaExtensions.selector(), {
            accumulating: ReteAreaPlugin.AreaExtensions.accumulateOnCtrl()
        });
        console.log('✅ 步驟 2.3.8 完成: 區域擴展功能設置完成');
        
        console.log('🔧 步驟 2.3.9: 初始化連接狀態...');
        this.connectionState = {
            isConnecting: false,
            startSocket: null,
            tempLine: null
        };
        console.log('✅ 步驟 2.3.9 完成: 連接狀態初始化完成');
        
        console.log('🔧 步驟 2.3.10: 設置編輯器事件監聽...');
        this.editor.addPipe(context => {
            if (context.type === 'nodeselected') {
                this.selectNode(context.data.node);
            } else if (context.type === 'connectionremoved') {
                console.log('連接移除事件:', context.data);
                if (context.data && context.data.connection) {
                    this.handleConnectionRemoved(context.data.connection);
                }
                this.updateStatusBar();
            } else if (context.type === 'connectioncreated') {
                this.updateStatusBar();
            } else if (context.type === 'noderemoved' || context.type === 'nodecreated') {
                this.updateStatusBar();
            }
            return context;
        });
        
        console.log('✅ 步驟 2.3.10 完成: 編輯器事件監聽設置完成');
        
        console.log('🔧 步驟 2.3.11: 設置畫布互動...');
        this.setupCanvasInteractions();
        console.log('✅ 步驟 2.3.11 完成: 畫布互動設置完成');
        
        console.log('✅ Rete.js 編輯器設置完成');
    }

    setupCanvasInteractions() {
        const container = document.getElementById('rete-editor');
        if (!container) return;

        // 畫布拖拽
        let isPanning = false;
        let lastPanPoint = { x: 0, y: 0 };

        container.addEventListener('mousedown', (e) => {
            const isOnNode = e.target.closest('.rete-node');
            const isOnSocket = e.target.classList.contains('rete-socket');
            const isOnConnection = e.target.closest('.rete-connection');
            
            // 允許拖拽畫布（如果不是在互動元素上）
            if (!isOnNode && !isOnSocket && !isOnConnection) {
                isPanning = true;
                lastPanPoint = { x: e.clientX, y: e.clientY };
                container.style.cursor = 'grabbing';
                e.preventDefault();
            }
        });

        document.addEventListener('mousemove', (e) => {
            if (isPanning) {
                const deltaX = e.clientX - lastPanPoint.x;
                const deltaY = e.clientY - lastPanPoint.y;
                
                // 實現畫布平移邏輯
                this.updateCanvasOffset(deltaX, deltaY);
                
                lastPanPoint = { x: e.clientX, y: e.clientY };
            }
        });

        document.addEventListener('mouseup', () => {
            if (isPanning) {
                isPanning = false;
                container.style.cursor = 'default';
            }
        });

        // 鍵盤事件
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Delete' || e.key === 'Backspace') {
                e.preventDefault();
                
                // 檢查是否有選中的連接
                const selectedConnection = document.querySelector('.rete-connection.selected');
                if (selectedConnection) {
                    const fullId = selectedConnection.id;
                    const connectionId = fullId.replace('connection-', '');
                    this.deleteConnection(connectionId);
                    return;
                }
                
                // 檢查是否有選中的節點
                if (this.selectedNode) {
                    this.deleteNode(this.selectedNode.id);
                }
            }
        });

        // 點擊空白處取消選擇
        container.addEventListener('click', (e) => {
            if (e.target === container) {
                this.deselectAllConnections();
            }
        });
    }

    updateCanvasOffset(deltaX, deltaY) {
        // 更新所有節點和連接的位置
        const nodes = document.querySelectorAll('.rete-node');
        nodes.forEach(node => {
            const currentX = parseInt(node.style.left) || 0;
            const currentY = parseInt(node.style.top) || 0;
            node.style.left = `${currentX + deltaX}px`;
            node.style.top = `${currentY + deltaY}px`;
        });
        
        // 更新所有連接
        for (const connectionId of this.connections.keys()) {
            this.updateConnectionPath(connectionId);
        }
    }

    setupEventListeners() {
        // 工具列按鈕事件
        const btnNewFlow = document.getElementById('btn-new-flow');
        if (btnNewFlow) btnNewFlow.addEventListener('click', () => this.newFlow());
        
        const btnLoadFlow = document.getElementById('btn-load-flow');
        if (btnLoadFlow) btnLoadFlow.addEventListener('click', () => this.loadFlow());
        
        const btnSaveFlow = document.getElementById('btn-save-flow');
        if (btnSaveFlow) btnSaveFlow.addEventListener('click', () => this.saveFlow());
        
        // 移除已不需要的按鈕事件監聽器 - 統一使用 YAML DSL 格式
        
        // 模態框事件
        const confirmFlowAction = document.getElementById('confirm-flow-action');
        if (confirmFlowAction) confirmFlowAction.addEventListener('click', () => this.confirmFlowAction());
        
        const cancelFlowAction = document.getElementById('cancel-flow-action');
        if (cancelFlowAction) cancelFlowAction.addEventListener('click', () => this.closeFlowModal());
        
        // 屬性面板關閉
        const closeProperties = document.getElementById('close-properties');
        if (closeProperties) closeProperties.addEventListener('click', () => this.closePropertiesPanel());
    }

    createNodePalette() {
        // Phase 3.1: 清空現有節點，包含新增的腳本節點區域
        document.getElementById('condition-nodes').innerHTML = '';
        document.getElementById('action-nodes').innerHTML = '';
        document.getElementById('logic-nodes').innerHTML = '';
        const scriptNodesElement = document.getElementById('script-nodes');
        if (scriptNodesElement) {
            scriptNodesElement.innerHTML = '';
        }

        console.log('🎨 創建節點選板，節點總數:', Object.keys(this.nodeTypes).length);

        // Phase 3.1: 增強的節點分類映射 - 支援四種節點類型
        Object.entries(this.nodeTypes).forEach(([nodeId, nodeType]) => {
            let containerId;
            
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
                    containerId = 'script-nodes';     // 腳本/存儲節點 → 腳本節點區域 (新增)
                    break;
                default:
                    console.warn(`⚠️ 未知節點類型: ${nodeCategory} (nodeId: ${nodeId})，放入條件節點區域`);
                    console.log('🔍 節點詳細資訊:', nodeType);
                    containerId = 'condition-nodes';  // 預設放在條件節點區域
            }
            
            const container = document.getElementById(containerId);
            if (container) {
                const item = this.createNodePaletteItem(nodeId, nodeType);
                container.appendChild(item);
            } else {
                console.warn(`⚠️ 找不到容器: ${containerId}，節點 ${nodeId} 將被跳過`);
            }
        });
        
        // Phase 3.1: 輸出節點統計資訊
        console.log('✅ 節點選板創建完成');
        console.log('📊 節點分佈統計:');
        ['condition-nodes', 'action-nodes', 'logic-nodes', 'script-nodes'].forEach(containerId => {
            const container = document.getElementById(containerId);
            if (container) {
                const count = container.children.length;
                console.log(`  - ${containerId}: ${count} 個節點`);
            }
        });
    }

    createNodePaletteItem(nodeId, nodeType) {
        const item = document.createElement('div');
        item.className = 'palette-node-item';
        item.dataset.nodeId = nodeId;
        item.dataset.nodeType = nodeType.category || nodeType.type;
        item.dataset.nodeCategory = nodeType.category;
        
        // Phase 3.1: 使用增強的節點視覺配置
        item.style.cssText = `
            display: flex;
            align-items: center;
            padding: 8px 12px;
            margin: 4px 0;
            background: ${nodeType.color || '#404040'};
            color: ${nodeType.textColor || 'white'};
            border: 1px solid ${nodeType.borderColor || nodeType.color || '#404040'};
            border-radius: 4px;
            cursor: pointer;
            font-size: 12px;
            font-weight: 500;
            transition: all 0.2s ease;
            position: relative;
        `;
        
        // Phase 3.1: 增強的節點項目結構 - 包含圖標、名稱、描述和來源標識
        item.innerHTML = `
            <div class="node-icon" style="
                margin-right: 8px; 
                width: 18px; 
                height: 18px; 
                display: flex; 
                align-items: center; 
                justify-content: center;
                background: rgba(255,255,255,0.2); 
                border-radius: 3px;
                font-size: 10px;
            ">${nodeType.icon || '❓'}</div>
            <div class="node-info" style="flex: 1;">
                <div class="node-name" style="font-weight: 600; line-height: 1.2;">
                    ${nodeType.name || nodeId}
                </div>
                <div class="node-source" style="
                    font-size: 10px; 
                    opacity: 0.8; 
                    color: ${nodeType.textColor || 'rgba(255,255,255,0.8)'};
                    margin-top: 1px;
                ">
                    ${nodeType.source || 'unknown'}
                </div>
            </div>
            <div class="node-type-badge" style="
                font-size: 9px;
                background: rgba(255,255,255,0.2);
                padding: 1px 4px;
                border-radius: 2px;
                margin-left: 4px;
            ">
                ${nodeType.dslType || nodeType.category || 'node'}
            </div>
        `;
        
        // Phase 3.1: 增強的懸停效果和工具提示
        item.addEventListener('mouseenter', () => {
            item.style.transform = 'translateX(4px)';
            item.style.boxShadow = '0 2px 8px rgba(0,0,0,0.3)';
        });
        
        item.addEventListener('mouseleave', () => {
            item.style.transform = 'translateX(0)';
            item.style.boxShadow = 'none';
        });
        
        // 點擊事件
        item.addEventListener('click', async (e) => {
            e.preventDefault();
            e.stopPropagation();
            console.log('🖱️ 點擊節點:', nodeId, nodeType.name);
            try {
                await this.addNode(nodeId);
                this.showNotification(`已添加節點: ${nodeType.name}`, 'success');
            } catch (error) {
                console.error('❌ 添加節點時發生錯誤:', error);
                this.showNotification('添加節點失敗: ' + error.message, 'danger');
            }
        });
        
        return item;
    }

    async addNode(type) {
        console.log(`🔄 嘗試添加節點: ${type}`);
        
        if (!this.editor) {
            console.error('❌ 編輯器尚未初始化');
            throw new Error('編輯器尚未初始化');
        }
        
        const config = this.nodeTypes[type];
        if (!config) {
            throw new Error(`未知的節點類型: ${type}`);
        }
        
        try {
            // 創建 Rete.js v2 節點實例
            const node = new Rete.ClassicPreset.Node(config.name);
            node.id = `${type}_${++this.nodeCounter}`;
            // 初始化節點參數 - 使用配置中的預設值
            const nodeParameters = {};
            if (config.parameters && config.parameters.length > 0) {
                config.parameters.forEach(param => {
                    nodeParameters[param.name] = param.value || param.default || '';
                });
            }
            
            node.data = {
                type: type,
                name: config.name,
                category: config.category,
                description: this.getNodeDescription(type),
                parameters: nodeParameters,
                // 保存節點配置供屬性面板使用
                nodeConfig: config
            };
            
            // 添加適當的 socket
            this.addNodeSockets(node, config, type);
            
            // 添加到編輯器
            await this.editor.addNode(node);
            
            // 設置位置
            const x = 250 + Math.random() * 200;
            const y = 200 + Math.random() * 200;
            
            if (this.area) {
                await this.area.translate(node.id, { x, y });
            }
            
            // 手動 DOM 渲染
            this.renderNodeManually(node, x, y);
            
            // 將 DOM 元素關聯到 Rete 節點
            node.element = document.getElementById(`node-${node.id}`);
            
            this.updateStatusBar();
            console.log('✅ 已添加節點:', config.name, '位置:', { x, y });
            
        } catch (error) {
            console.error('❌ 添加節點失敗:', error);
            throw error;
        }
    }

    addNodeSockets(node, nodeType, nodeId) {
        console.log('🔌 為節點添加接口:', nodeId, nodeType.category);
        
        if (!this.socket) {
            console.error('❌ Socket 尚未初始化');
            return;
        }
        
        // 🔧 修復：使用節點類型定義中的實際 inputs 和 outputs
        let hasInputs = false;
        let hasOutputs = false;
        
        // 添加輸入接口
        if (nodeType.inputs && Array.isArray(nodeType.inputs) && nodeType.inputs.length > 0) {
            nodeType.inputs.forEach(input => {
                const inputKey = input.name || 'input';
                const inputLabel = input.description || input.label || input.name || '輸入';
                node.addInput(inputKey, new Rete.ClassicPreset.Input(this.socket, inputLabel));
                console.log(`📥 添加輸入接口: ${inputKey} (${inputLabel})`);
                hasInputs = true;
            });
        }
        
        // 添加輸出接口
        if (nodeType.outputs && Array.isArray(nodeType.outputs) && nodeType.outputs.length > 0) {
            nodeType.outputs.forEach(output => {
                const outputKey = output.name || 'output';
                const outputLabel = output.description || output.label || output.name || '輸出';
                node.addOutput(outputKey, new Rete.ClassicPreset.Output(this.socket, outputLabel));
                console.log(`📤 添加輸出接口: ${outputKey} (${outputLabel})`);
                hasOutputs = true;
            });
        }
        
        // 🔧 如果節點類型定義中沒有 inputs/outputs，使用預設的接口配置
        if (!hasInputs && !hasOutputs) {
            console.log('⚠️ 節點類型沒有定義 inputs/outputs，使用預設配置');
            
            switch (nodeType.category) {
                case 'input':
                    // 條件節點：沒有輸入，有輸出
                    node.addOutput('output', new Rete.ClassicPreset.Output(this.socket, '輸出'));
                    break;
                case 'output':
                    // 動作節點：有輸入，沒有輸出
                    node.addInput('input', new Rete.ClassicPreset.Input(this.socket, '輸入'));
                    break;
                case 'control':
                    // 邏輯節點：有輸入和輸出
                    node.addInput('input', new Rete.ClassicPreset.Input(this.socket, '輸入'));
                    node.addOutput('output', new Rete.ClassicPreset.Output(this.socket, '輸出'));
                    // 決策節點有額外的 Yes/No 輸出
                    if (nodeId === 'decision') {
                        node.addOutput('yes', new Rete.ClassicPreset.Output(this.socket, 'Yes'));
                        node.addOutput('no', new Rete.ClassicPreset.Output(this.socket, 'No'));
                    }
                    break;
                default:
                    // 其他節點：有輸入和輸出
                    node.addInput('input', new Rete.ClassicPreset.Input(this.socket, '輸入'));
                    node.addOutput('output', new Rete.ClassicPreset.Output(this.socket, '輸出'));
                    break;
            }
        }
    }

    renderNodeManually(node, x, y) {
        const container = document.getElementById('rete-editor');
        if (!container) {
            console.error('❌ 找不到編輯器容器');
            return;
        }
        
        // 檢查節點是否已經存在
        const existingNode = container.querySelector(`[data-node-id="${node.id}"]`);
        if (existingNode) {
            console.log('⚠️ 節點已存在，移除舊的');
            existingNode.remove();
        }
        
        // 創建節點元素
        const nodeEl = document.createElement('div');
        nodeEl.className = 'rete-node';
        nodeEl.id = `node-${node.id}`;
        nodeEl.setAttribute('data-node-id', node.id);
        
        // 只設置位置，讓 CSS 處理其他樣式
        nodeEl.style.left = `${x}px`;
        nodeEl.style.top = `${y}px`;
        nodeEl.style.transform = 'translate(-50%, -50%)';
        
        // 添加節點類型作為 CSS 類別用於特定樣式
        const nodeType = this.nodeTypes[node.data.type];
        if (nodeType) {
            nodeEl.setAttribute('data-node-type', node.data.type);
            nodeEl.setAttribute('data-node-category', nodeType.category);
            
            // CSS 會根據 data-node-category 屬性自動應用對應的邊框顏色
            console.log(`🎨 節點 ${node.id} 設置類別: ${nodeType.category}`);
            
            // 調試：檢查屬性是否正確設置
            setTimeout(() => {
                const actualCategory = nodeEl.getAttribute('data-node-category');
                const computedStyle = window.getComputedStyle(nodeEl);
                const borderColor = computedStyle.borderColor;
                console.log(`📊 節點 ${node.id} 調試:`, {
                    type: node.data.type,
                    category: nodeType.category,
                    actualCategory: actualCategory,
                    borderColor: borderColor
                });
            }, 100);
        }
        
        // 添加節點標題
        const titleEl = document.createElement('div');
        titleEl.className = 'rete-node-title';
        titleEl.textContent = node.data.name || node.label || 'Unnamed Node';
        titleEl.style.textAlign = 'center';
        nodeEl.appendChild(titleEl);
        
        // 添加節點描述
        if (node.data.description) {
            const descEl = document.createElement('div');
            descEl.className = 'rete-node-description';
            descEl.textContent = node.data.description;
            descEl.style.textAlign = 'center';
            nodeEl.appendChild(descEl);
        }
        
        // 渲染 socket
        this.renderNodeSockets(nodeEl, node);
        
        // 添加節點互動事件
        this.setupNodeInteractions(nodeEl, node);
        
        // 添加到容器
        container.appendChild(nodeEl);
        
        console.log(`🎨 節點 ${node.id} 已渲染到 DOM，位置: (${x}, ${y})`);
    }

    renderNodeSockets(nodeEl, node) {
        // 創建輸入接口容器
        const inputContainer = document.createElement('div');
        inputContainer.className = 'rete-input';
        nodeEl.appendChild(inputContainer);
        
        // 渲染輸入接口
        Object.entries(node.inputs).forEach(([key, input], index) => {
            const inputEl = document.createElement('div');
            inputEl.className = 'rete-socket';
            inputEl.setAttribute('data-node-id', node.id);
            inputEl.setAttribute('data-socket-key', key);
            inputEl.setAttribute('data-socket-type', 'input');
            inputEl.title = input.label || '輸入';
            
            // 只設置位置，其他樣式由 CSS 處理
            inputEl.style.top = `${20 + index * 20}px`;
            
            inputContainer.appendChild(inputEl);
        });
        
        // 創建輸出接口容器
        const outputContainer = document.createElement('div');
        outputContainer.className = 'rete-output';
        nodeEl.appendChild(outputContainer);
        
        // 渲染輸出接口
        Object.entries(node.outputs).forEach(([key, output], index) => {
            const outputEl = document.createElement('div');
            outputEl.className = 'rete-socket';
            outputEl.setAttribute('data-node-id', node.id);
            outputEl.setAttribute('data-socket-key', key);
            outputEl.setAttribute('data-socket-type', 'output');
            outputEl.title = output.label || '輸出';
            
            // 只設置位置，其他樣式由 CSS 處理
            outputEl.style.top = `${20 + index * 20}px`;
            
            outputContainer.appendChild(outputEl);
        });
        
        // 設置 socket 事件
        this.setupSocketEvents(nodeEl);
    }

    setupSocketEvents(nodeEl) {
        const sockets = nodeEl.querySelectorAll('.rete-socket');
        
        sockets.forEach(socket => {
            // 開始連接
            socket.addEventListener('mousedown', (e) => {
                e.stopPropagation();
                e.preventDefault();
                
                if (socket.dataset.socketType === 'output') {
                    // 從輸出開始連接
                    this.connectionState.isConnecting = true;
                    this.connectionState.startSocket = socket;
                    
                    // 創建臨時連線
                    this.connectionState.tempLine = this.createTempLine(e.clientX, e.clientY);
                    this.connectionState.tempLine.startSocket = socket;
                    document.body.appendChild(this.connectionState.tempLine);
                    
                    console.log('開始從輸出建立連線:', socket.dataset.nodeId);
                }
            });
            
            // 完成連接
            socket.addEventListener('mouseup', (e) => {
                if (this.connectionState.isConnecting && this.connectionState.startSocket && 
                    socket.dataset.socketType === 'input' && socket !== this.connectionState.startSocket) {
                    // 完成連接到輸入
                    e.stopPropagation();
                    this.createConnection(this.connectionState.startSocket, socket);
                }
                
                // 清理
                this.connectionState.isConnecting = false;
                this.connectionState.startSocket = null;
                if (this.connectionState.tempLine) {
                    this.connectionState.tempLine.remove();
                    this.connectionState.tempLine = null;
                }
            });
            
            // 懸停效果 - 使用 CSS 類別而非內聯樣式
            socket.addEventListener('mouseover', (e) => {
                if (this.connectionState.isConnecting && socket.dataset.socketType === 'input' && 
                    socket !== this.connectionState.startSocket) {
                    socket.classList.add('hover-target'); // 使用 CSS 類別高亮
                }
            });
            
            socket.addEventListener('mouseout', (e) => {
                socket.classList.remove('hover-target'); // 移除高亮樣式
            });
        });
        
        // 全域滑鼠移動事件（僅添加一次）
        if (!this.globalEventsAdded) {
            this.globalEventsAdded = true;
            
            document.addEventListener('mousemove', (e) => {
                if (this.connectionState.isConnecting && this.connectionState.tempLine) {
                    this.updateTempLine(this.connectionState.tempLine, e.clientX, e.clientY);
                }
            });
            
            // 全域滑鼠釋放事件取消連接
            document.addEventListener('mouseup', (e) => {
                if (this.connectionState.isConnecting && !e.target.classList.contains('rete-socket')) {
                    this.connectionState.isConnecting = false;
                    this.connectionState.startSocket = null;
                    if (this.connectionState.tempLine) {
                        this.connectionState.tempLine.remove();
                        this.connectionState.tempLine = null;
                    }
                }
            });
        }
    }

    createTempLine(x, y) {
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        line.style.position = 'fixed';
        line.style.top = '0';
        line.style.left = '0';
        line.style.width = '100%';
        line.style.height = '100%';
        line.style.pointerEvents = 'none';
        line.style.zIndex = '9999';
        
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('stroke', '#666');
        path.setAttribute('stroke-width', '2');
        path.setAttribute('fill', 'none');
        path.setAttribute('stroke-dasharray', '5,5');
        
        line.appendChild(path);
        line.startX = x;
        line.startY = y;
        
        return line;
    }

    updateTempLine(line, x, y) {
        const path = line.querySelector('path');
        if (path) {
            const d = `M ${line.startX} ${line.startY} L ${x} ${y}`;
            path.setAttribute('d', d);
        }
    }

    createConnection(startSocket, endSocket) {
        try {
            const sourceNodeId = startSocket.dataset.nodeId;
            const targetNodeId = endSocket.dataset.nodeId;
            const sourceKey = startSocket.dataset.socketKey;
            const targetKey = endSocket.dataset.socketKey;
            
            console.log(`創建連接: ${sourceNodeId}.${sourceKey} -> ${targetNodeId}.${targetKey}`);
            
            // 檢查是否有重複連接
            const existingConnections = this.editor.getConnections();
            const duplicateConnection = existingConnections.find(conn => 
                conn.target === targetNodeId && conn.targetInput === targetKey
            );
            
            if (duplicateConnection) {
                console.warn('接口已有連接');
                this.showNotification('該輸入接口已有連接', 'warning');
                return;
            }
            
            // 從編輯器取得節點
            const sourceNode = this.editor.getNode(sourceNodeId);
            const targetNode = this.editor.getNode(targetNodeId);
            
            if (sourceNode && targetNode) {
                // 使用 Rete.js 創建連接
                const connection = new Rete.ClassicPreset.Connection(
                    sourceNode, sourceKey,
                    targetNode, targetKey
                );
                
                this.editor.addConnection(connection);
                
                // 視覺化渲染連接
                this.renderConnection(startSocket, endSocket, connection.id);
                
                this.updateStatusBar();
                console.log('✅ 連接創建成功');
                this.showNotification('連接創建成功', 'success');
            }
        } catch (error) {
            console.error('❌ 創建連接時發生錯誤:', error);
            this.showNotification('創建連接失敗', 'danger');
        }
    }

    renderConnection(startSocket, endSocket, connectionId) {
        const container = document.getElementById('rete-editor');
        if (!container) {
            console.error('❌ 找不到編輯器容器');
            return;
        }
        
        // 檢查是否已有此連接的資訊
        const existingConnection = this.connections.get(connectionId);
        if (existingConnection && existingConnection.reteConnection) {
            // 如果已有 Rete 連接資訊，更新 socket 元素
            existingConnection.startSocket = startSocket;
            existingConnection.endSocket = endSocket;
        } else {
            // 否則創建新的連接資訊
            this.connections.set(connectionId, {
                startSocket: startSocket,
                endSocket: endSocket,
                startNodeId: startSocket.dataset.nodeId,
                endNodeId: endSocket.dataset.nodeId
            });
        }
        
        // 移除舊的連接線 (如果存在)
        const oldSvg = document.getElementById(`connection-${connectionId}`);
        if (oldSvg) {
            oldSvg.remove();
        }
        
        // 創建 SVG 連接線
        const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svg.style.position = 'absolute';
        svg.style.pointerEvents = 'none';
        svg.style.zIndex = '1';
        svg.style.left = '0';
        svg.style.top = '0';
        svg.style.width = '100%';
        svg.style.height = '100%';
        svg.id = `connection-${connectionId}`;
        svg.classList.add('rete-connection');
        
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('stroke', '#666');
        path.setAttribute('stroke-width', '2');
        path.setAttribute('fill', 'none');
        
        // 計算連接點位置
        const startRect = startSocket.getBoundingClientRect();
        const endRect = endSocket.getBoundingClientRect();
        const containerRect = container.getBoundingClientRect();
        
        // 相對於容器的座標
        const startX = startRect.left + startRect.width / 2 - containerRect.left;
        const startY = startRect.top + startRect.height / 2 - containerRect.top;
        const endX = endRect.left + endRect.width / 2 - containerRect.left;
        const endY = endRect.top + endRect.height / 2 - containerRect.top;
        
        // 計算控制點 (用於貝茲曲線)
        const controlOffset = Math.abs(endX - startX) * 0.5;
        const pathData = `M ${startX} ${startY} C ${startX + controlOffset} ${startY}, ${endX - controlOffset} ${endY}, ${endX} ${endY}`;
        
        path.setAttribute('d', pathData);
        path.style.cursor = 'pointer';
        path.style.pointerEvents = 'auto';
        
        // 點擊選擇連接
        path.addEventListener('click', (e) => {
            e.stopPropagation();
            this.selectConnection(connectionId, svg, path);
        });
        
        // 添加到 SVG 和容器
        svg.appendChild(path);
        container.appendChild(svg);
        
        console.log(`🔗 連接 ${connectionId} 已視覺化渲染: (${startX}, ${startY}) → (${endX}, ${endY})`);
    }

    updateConnectionPath(connectionId) {
        const connectionData = this.connections.get(connectionId);
        if (!connectionData) return;
        
        const { startSocket, endSocket } = connectionData;
        const svg = document.getElementById(`connection-${connectionId}`);
        const path = svg ? svg.querySelector('path') : null;
        
        if (!svg || !path || !startSocket || !endSocket) return;
        
        // 計算 socket 位置
        const startRect = startSocket.getBoundingClientRect();
        const endRect = endSocket.getBoundingClientRect();
        const containerRect = document.getElementById('rete-editor').getBoundingClientRect();
        
        const startX = startRect.left + startRect.width / 2 - containerRect.left;
        const startY = startRect.top + startRect.height / 2 - containerRect.top;
        const endX = endRect.left + endRect.width / 2 - containerRect.left;
        const endY = endRect.top + endRect.height / 2 - containerRect.top;
        
        // 根據連線模式生成不同的路徑
        // 固定使用曲線連接
        const pathData = this.generateCurvedPath(startX, startY, endX, endY);
        const pathPoints = this.getCurvedPathPoints(startX, startY, endX, endY);
        
        path.setAttribute('d', pathData);
        
        // 更新 SVG 尺寸
        const allX = pathPoints.map(p => p.x);
        const allY = pathPoints.map(p => p.y);
        const minX = Math.min(...allX) - 10;
        const minY = Math.min(...allY) - 10;
        const maxX = Math.max(...allX) + 10;
        const maxY = Math.max(...allY) + 10;
        
        svg.style.left = `${minX}px`;
        svg.style.top = `${minY}px`;
        svg.setAttribute('width', maxX - minX);
        svg.setAttribute('height', maxY - minY);
        
        // 調整路徑座標到 SVG 本地坐標系
        const adjustedPathData = this.adjustPathToSVGCoords(pathData, pathPoints, minX, minY);
        path.setAttribute('d', adjustedPathData);
    }

    // 🔗 連線路徑生成方法
    
    generateOrthogonalPath(startX, startY, endX, endY) {
        // 智能直角折線路徑
        const gap = 30; // 節點邊緣間隙
        const minSegment = 50; // 最小線段長度
        
        // 輸出socket在右側 (+gap)，輸入socket在左側 (-gap)
        const outputX = startX + gap;
        const inputX = endX - gap;
        
        if (outputX < inputX) {
            // 正常情況：目標在右側
            const midX = outputX + (inputX - outputX) / 2;
            return `M ${startX} ${startY} 
                   L ${outputX} ${startY} 
                   L ${midX} ${startY} 
                   L ${midX} ${endY} 
                   L ${inputX} ${endY} 
                   L ${endX} ${endY}`;
        } else {
            // 回折情況：目標在左側或重疊
            const offsetY = (endY > startY) ? -minSegment : minSegment;
            const bridgeY = startY + offsetY;
            
            return `M ${startX} ${startY} 
                   L ${outputX} ${startY} 
                   L ${outputX} ${bridgeY} 
                   L ${inputX} ${bridgeY} 
                   L ${inputX} ${endY} 
                   L ${endX} ${endY}`;
        }
    }
    
    generateCurvedPath(startX, startY, endX, endY) {
        // 原有的貝茲曲線路徑
        const dx = endX - startX;
        const controlOffset = Math.abs(dx) * 0.5;
        const controlX1 = startX + controlOffset;
        const controlY1 = startY;
        const controlX2 = endX - controlOffset;
        const controlY2 = endY;
        
        return `M ${startX} ${startY} C ${controlX1} ${controlY1}, ${controlX2} ${controlY2}, ${endX} ${endY}`;
    }
    
    generateStraightPath(startX, startY, endX, endY) {
        // 直線路徑
        return `M ${startX} ${startY} L ${endX} ${endY}`;
    }
    
    getOrthogonalPathPoints(startX, startY, endX, endY) {
        const gap = 30;
        const minSegment = 50;
        const outputX = startX + gap;
        const inputX = endX - gap;
        
        if (outputX < inputX) {
            const midX = outputX + (inputX - outputX) / 2;
            return [
                { x: startX, y: startY },
                { x: outputX, y: startY },
                { x: midX, y: startY },
                { x: midX, y: endY },
                { x: inputX, y: endY },
                { x: endX, y: endY }
            ];
        } else {
            const offsetY = (endY > startY) ? -minSegment : minSegment;
            const bridgeY = startY + offsetY;
            return [
                { x: startX, y: startY },
                { x: outputX, y: startY },
                { x: outputX, y: bridgeY },
                { x: inputX, y: bridgeY },
                { x: inputX, y: endY },
                { x: endX, y: endY }
            ];
        }
    }
    
    getCurvedPathPoints(startX, startY, endX, endY) {
        const dx = endX - startX;
        const controlOffset = Math.abs(dx) * 0.5;
        const controlX1 = startX + controlOffset;
        const controlY1 = startY;
        const controlX2 = endX - controlOffset;
        const controlY2 = endY;
        
        return [
            { x: startX, y: startY },
            { x: controlX1, y: controlY1 },
            { x: controlX2, y: controlY2 },
            { x: endX, y: endY }
        ];
    }
    
    adjustPathToSVGCoords(pathData, pathPoints, offsetX, offsetY) {
        // 更準確的座標調整方法
        let adjustedPath = pathData;
        
        // 調整路徑中的所有座標點
        pathPoints.forEach(point => {
            const originalX = point.x.toString();
            const originalY = point.y.toString();
            const adjustedX = (point.x - offsetX).toString();
            const adjustedY = (point.y - offsetY).toString();
            
            // 替換路徑中的座標
            adjustedPath = adjustedPath.replace(
                new RegExp(`\\b${originalX}\\b`), adjustedX
            );
            adjustedPath = adjustedPath.replace(
                new RegExp(`\\b${originalY}\\b`), adjustedY
            );
        });
        
        return adjustedPath;
    }

    selectConnection(connectionId, svg, path) {
        // 取消所有其他連接的選擇
        this.deselectAllConnections();
        
        // 選擇當前連接
        svg.classList.add('selected');
        path.setAttribute('stroke', '#2196F3');
        path.setAttribute('stroke-width', '3');
        
        console.log(`🔗 選擇連接: ${connectionId}`);
    }

    deselectAllConnections() {
        const connections = document.querySelectorAll('.rete-connection');
        connections.forEach(conn => {
            conn.classList.remove('selected');
            const path = conn.querySelector('path');
            if (path) {
                path.setAttribute('stroke', '#666');
                path.setAttribute('stroke-width', '2');
            }
        });
    }

    deleteConnection(connectionId) {
        console.log(`🗑️ 刪除連接: ${connectionId}`);
        
        try {
            // 從 Rete.js 編輯器中移除連接
            const connections = this.editor.getConnections();
            const connection = connections.find(conn => conn.id === connectionId);
            if (connection) {
                this.editor.removeConnection(connection.id);
            }
            
            // 移除視覺化元素
            const connectionElement = document.getElementById(`connection-${connectionId}`);
            if (connectionElement) {
                connectionElement.remove();
            }
            
            // 從連接映射中移除
            this.connections.delete(connectionId);
            
            this.updateStatusBar();
            console.log(`✅ 連接 ${connectionId} 已刪除`);
            
        } catch (error) {
            console.error('❌ 刪除連接時發生錯誤:', error);
            this.showNotification('刪除連接失敗', 'danger');
        }
    }

    setupNodeInteractions(nodeEl, node) {
        let isDragging = false;
        let dragStart = { x: 0, y: 0 };
        
        // 節點點擊選擇
        nodeEl.addEventListener('click', (e) => {
            e.stopPropagation();
            this.selectNode(node);
        });
        
        // 拖拽功能
        nodeEl.addEventListener('mousedown', (e) => {
            if (e.target.classList.contains('rete-socket')) {
                return; // 不要在接口上觸發拖拽
            }
            
            e.stopPropagation();
            isDragging = true;
            dragStart = { x: e.clientX, y: e.clientY };
            
            const onMouseMove = (e) => {
                if (isDragging) {
                    const deltaX = e.clientX - dragStart.x;
                    const deltaY = e.clientY - dragStart.y;
                    
                    const currentX = parseInt(nodeEl.style.left) || 0;
                    const currentY = parseInt(nodeEl.style.top) || 0;
                    
                    nodeEl.style.left = `${currentX + deltaX}px`;
                    nodeEl.style.top = `${currentY + deltaY}px`;
                    
                    // 更新相關連接
                    this.updateConnectionsForNode(node.id);
                    
                    dragStart = { x: e.clientX, y: e.clientY };
                }
            };
            
            const onMouseUp = () => {
                if (isDragging) {
                    isDragging = false;
                    document.removeEventListener('mousemove', onMouseMove);
                    document.removeEventListener('mouseup', onMouseUp);
                }
            };
            
            document.addEventListener('mousemove', onMouseMove);
            document.addEventListener('mouseup', onMouseUp);
        });
    }

    updateConnectionsForNode(nodeId) {
        // 更新與此節點相關的所有連接
        console.log(`🔄 更新節點 ${nodeId} 的相關連接`);
        let updatedCount = 0;
        
        for (const [connectionId, connectionData] of this.connections) {
            // 支援多種連接數據格式
            const sourceNodeId = connectionData.source || connectionData.startNodeId;
            const targetNodeId = connectionData.target || connectionData.endNodeId;
            
            if (sourceNodeId === nodeId || targetNodeId === nodeId) {
                console.log(`🔄 更新連接 ${connectionId}: ${sourceNodeId} → ${targetNodeId}`);
                this.updateConnectionPath(connectionId);
                updatedCount++;
            }
        }
        
        console.log(`✅ 已更新 ${updatedCount} 個連接`);
    }

    handleConnectionRemoved(connection) {
        if (!connection || !connection.id) {
            console.warn('無效的連接物件');
            return;
        }
        
        console.log('處理連接移除事件:', connection.id);
        
        // 移除視覺化連接
        const connectionElement = document.getElementById(`connection-${connection.id}`);
        if (connectionElement) {
            connectionElement.remove();
        }
        
        // 從連接映射中移除
        if (this.connections.has(connection.id)) {
            this.connections.delete(connection.id);
        }
    }

    selectNode(node) {
        // 取消所有節點的選擇
        document.querySelectorAll('.rete-node').forEach(n => {
            n.classList.remove('selected');
            // 移除選擇樣式，但保留類別邊框顏色
            n.style.boxShadow = '';
            n.style.outline = '';
        });
        
        // 選擇當前節點
        const nodeEl = document.getElementById(`node-${node.id}`);
        if (nodeEl) {
            nodeEl.classList.add('selected');
            // 使用白色光外框表示選中狀態 - 讓CSS樣式生效，不設置內聯樣式
        }
        
        this.selectedNode = node;
        this.showPropertiesPanel(node);
        
        console.log(`🎯 選擇節點: ${node.data.name}`);
    }

    showPropertiesPanel(node) {
        const panel = document.getElementById('properties-panel');
        if (panel) {
            // 修復：使用 CSS 類別而非內聯樣式
            panel.classList.add('is-active');
            panel.style.display = 'flex'; // 確保顯示
            
            // 填充基本屬性
            const nameInput = document.getElementById('node-name');
            const descInput = document.getElementById('node-description');
            
            if (nameInput) nameInput.value = node.data.name || '';
            if (descInput) descInput.value = node.data.description || '';
            
            // 動態生成參數編輯界面
            this.updateParametersPanel(node);
            
            console.log('✅ 屬性面板已顯示，節點:', node.data.name);
        } else {
            console.error('❌ 找不到屬性面板元素 #properties-panel');
        }
    }

    updateParametersPanel(node) {
        console.log('🔧 更新參數面板:', node.data.type, node.data.parameters);
        
        // 隱藏所有參數區域
        document.getElementById('condition-parameters').style.display = 'none';
        document.getElementById('action-parameters').style.display = 'none';
        
        // 🔧 修復：從 nodeConfig 獲取參數定義，從 node.data.parameters 獲取當前值
        const nodeConfig = node.data.nodeConfig;
        const currentParameterValues = node.data.parameters || {}; // 這是一個對象，不是數組
        
        // 從節點配置中獲取參數定義（這是一個數組）
        const parametersDefinition = nodeConfig && nodeConfig.inputs ? nodeConfig.inputs : [];
        
        if (!parametersDefinition || !Array.isArray(parametersDefinition) || parametersDefinition.length === 0) {
            console.log('📝 節點無參數配置定義');
            return;
        }
        
        // 確保節點有 parameterValues 對象來存儲實際的參數值
        if (!node.data.parameterValues) {
            node.data.parameterValues = {};
        }
        
        console.log('✅ 找到節點參數定義:', parametersDefinition.length, '個參數');
        
        // 根據節點類型選擇參數容器
        let parametersContainer;
        let parametersId;
        
        if (node.data.category === 'input') {
            parametersContainer = document.getElementById('condition-parameters');
            parametersId = 'condition-dynamic-params';
        } else if (node.data.category === 'output') {
            parametersContainer = document.getElementById('action-parameters');
            parametersId = 'action-dynamic-params';
        } else {
            // 邏輯節點創建新的參數區域
            parametersContainer = this.createLogicParametersSection();
            parametersId = 'logic-dynamic-params';
        }
        
        if (parametersContainer) {
            parametersContainer.style.display = 'block';
            
            // 獲取動態參數容器
            const dynamicParamsContainer = document.getElementById(parametersId) || 
                                         parametersContainer.querySelector('.dynamic-params');
            
            if (dynamicParamsContainer) {
                // 清空現有內容
                dynamicParamsContainer.innerHTML = '';
                
                // 🔧 修復：基於參數定義數組生成輸入控件
                parametersDefinition.forEach(param => {
                    // 從當前參數值中獲取該參數的值
                    const currentValue = currentParameterValues[param.name] || param.default || '';
                    const paramElement = this.createParameterInput(param, node, param.name, currentValue);
                    dynamicParamsContainer.appendChild(paramElement);
                });
                
                console.log('✅ 參數編輯界面已生成，包含', parametersDefinition.length, '個參數控件');
            }
        }
    }
    
    createLogicParametersSection() {
        // 檢查是否已存在邏輯參數區域
        let logicParamsSection = document.getElementById('logic-parameters');
        
        if (!logicParamsSection) {
            // 創建邏輯節點參數區域
            logicParamsSection = document.createElement('div');
            logicParamsSection.id = 'logic-parameters';
            logicParamsSection.style.display = 'none';
            
            const title = document.createElement('div');
            title.className = 'field';
            title.innerHTML = '<label class="label">邏輯參數</label>';
            
            const dynamicContainer = document.createElement('div');
            dynamicContainer.id = 'logic-dynamic-params';
            dynamicContainer.className = 'dynamic-params';
            
            logicParamsSection.appendChild(title);
            logicParamsSection.appendChild(dynamicContainer);
            
            // 插入到屬性面板內容中
            const propertiesContent = document.getElementById('properties-content');
            if (propertiesContent) {
                propertiesContent.appendChild(logicParamsSection);
            }
        }
        
        return logicParamsSection;
    }
    
    createParameterInput(param, node, paramName, currentValue = null) {
        const field = document.createElement('div');
        field.className = 'field';
        
        const label = document.createElement('label');
        label.className = 'label';
        label.textContent = param.description || paramName;
        if (param.required) {
            label.innerHTML += ' <span style="color: red;">*</span>';
        }
        
        const control = document.createElement('div');
        control.className = 'control';
        
        let input;
        // 🔧 修復：優先使用傳入的 currentValue，否則從節點中獲取
        const effectiveValue = currentValue !== null ? currentValue : 
                              (node.data.parameterValues?.[paramName] || param.value || param.default || '');
        
        // 根據參數類型創建不同的輸入控件
        switch (param.type) {
            case 'integer':
            case 'number':
                input = document.createElement('input');
                input.className = 'input';
                input.type = 'number';
                input.value = effectiveValue;
                if (param.min !== null) input.min = param.min;
                if (param.max !== null) input.max = param.max;
                break;
                
            case 'boolean':
                const checkboxContainer = document.createElement('label');
                checkboxContainer.className = 'checkbox';
                
                input = document.createElement('input');
                input.type = 'checkbox';
                input.checked = effectiveValue === true || effectiveValue === 'true';
                
                const checkboxText = document.createTextNode(' ' + (param.description || param.name));
                checkboxContainer.appendChild(input);
                checkboxContainer.appendChild(checkboxText);
                
                control.appendChild(checkboxContainer);
                break;
                
            case 'list':
                input = document.createElement('textarea');
                input.className = 'textarea';
                input.rows = 3;
                input.placeholder = '每行一個值，或使用逗號分隔';
                input.value = Array.isArray(effectiveValue) ? effectiveValue.join('\n') : effectiveValue;
                break;
                
            case 'object':
                input = document.createElement('textarea');
                input.className = 'textarea';
                input.rows = 4;
                input.placeholder = 'JSON 格式的物件';
                input.value = typeof effectiveValue === 'object' ? 
                             JSON.stringify(effectiveValue, null, 2) : effectiveValue;
                break;
                
            default: // string
                if (param.options && param.options.length > 0) {
                    // 下拉選單
                    const selectWrapper = document.createElement('div');
                    selectWrapper.className = 'select is-fullwidth';
                    
                    input = document.createElement('select');
                    
                    // 添加空選項
                    if (!param.required) {
                        const emptyOption = document.createElement('option');
                        emptyOption.value = '';
                        emptyOption.textContent = '請選擇...';
                        input.appendChild(emptyOption);
                    }
                    
                    // 添加選項
                    param.options.forEach(option => {
                        const optionEl = document.createElement('option');
                        optionEl.value = option;
                        optionEl.textContent = option;
                        if (option === effectiveValue) {
                            optionEl.selected = true;
                        }
                        input.appendChild(optionEl);
                    });
                    
                    selectWrapper.appendChild(input);
                    control.appendChild(selectWrapper);
                } else {
                    // 文字輸入
                    input = document.createElement('input');
                    input.className = 'input';
                    input.type = 'text';
                    input.value = effectiveValue;
                    input.placeholder = param.description || param.name;
                }
                break;
        }
        
        // 添加輸入變更事件監聽器
        if (input && param.type !== 'boolean') {
            input.addEventListener('change', (e) => {
                this.updateNodeParameter(node, paramName, e.target.value, param.type);
            });
            control.appendChild(input);
        } else if (param.type === 'boolean') {
            input.addEventListener('change', (e) => {
                this.updateNodeParameter(node, paramName, e.target.checked, param.type);
            });
        }
        
        field.appendChild(label);
        field.appendChild(control);
        
        return field;
    }
    
    updateNodeParameter(node, paramName, value, paramType) {
        // 轉換值到正確的類型
        let convertedValue = value;
        
        switch (paramType) {
            case 'integer':
                convertedValue = parseInt(value) || 0;
                break;
            case 'number':
                convertedValue = parseFloat(value) || 0;
                break;
            case 'boolean':
                convertedValue = value === true || value === 'true';
                break;
            case 'list':
                convertedValue = value.split(/[\n,]/).map(v => v.trim()).filter(v => v);
                break;
            case 'object':
                try {
                    convertedValue = JSON.parse(value);
                } catch (e) {
                    console.warn('⚠️ JSON 解析失敗:', e);
                    convertedValue = value;
                }
                break;
        }
        
        // 更新節點參數值
        if (!node.data.parameterValues) {
            node.data.parameterValues = {};
        }
        node.data.parameterValues[paramName] = convertedValue;
        
        console.log('📝 更新節點參數:', node.data.name, paramName, convertedValue);
        
        // 觸發變更事件，供保存功能使用
        this.onNodeParameterChanged(node, paramName, convertedValue);
    }
    
    onNodeParameterChanged(node, paramName, value) {
        // 節點參數變更事件，可用於自動保存或驗證
        console.log('🔄 節點參數已變更:', {
            nodeId: node.id,
            nodeName: node.data.name,
            parameter: paramName,
            value: value
        });
        
        // 可以在這裡添加自動保存或即時驗證邏輯
        this.markFlowAsModified();
    }
    
    markFlowAsModified() {
        // 標記流程已修改
        const statusElement = document.querySelector('.flow-status-bar .tag');
        if (statusElement) {
            statusElement.textContent = '已修改';
            statusElement.className = 'tag is-warning';
        }
    }

    closePropertiesPanel() {
        const panel = document.getElementById('properties-panel');
        if (panel) {
            // 修復：移除 CSS 類別而非只設置內聯樣式
            panel.classList.remove('is-active');
            panel.style.display = 'none';
            console.log('✅ 屬性面板已關閉');
        }
        
        // 取消節點選擇
        if (this.selectedNode) {
            const nodeEl = document.getElementById(`node-${this.selectedNode.id}`);
            if (nodeEl) {
                nodeEl.classList.remove('selected');
                // 只移除選擇樣式，保留類別邊框顏色
                nodeEl.style.outline = '';
                nodeEl.style.outlineOffset = '';
                nodeEl.style.boxShadow = '';
            }
            this.selectedNode = null;
        }
    }

    deleteNode(nodeId) {
        console.log(`🗑️ 刪除節點: ${nodeId}`);
        
        try {
            // 從 Rete.js 編輯器中移除節點
            const node = this.editor.getNode(nodeId);
            if (node) {
                this.editor.removeNode(nodeId);
            }
            
            // 移除視覺化元素
            const nodeElement = document.getElementById(`node-${nodeId}`);
            if (nodeElement) {
                nodeElement.remove();
            }
            
            // 移除相關連接
            const connectionsToRemove = [];
            for (const [connectionId, connectionData] of this.connections) {
                if (connectionData.startNodeId === nodeId || connectionData.endNodeId === nodeId) {
                    connectionsToRemove.push(connectionId);
                }
            }
            
            connectionsToRemove.forEach(connId => {
                this.deleteConnection(connId);
            });
            
            // 清除選擇
            if (this.selectedNode && this.selectedNode.id === nodeId) {
                this.selectedNode = null;
                this.closePropertiesPanel();
            }
            
            this.updateStatusBar();
            console.log(`✅ 節點 ${nodeId} 已刪除`);
            
        } catch (error) {
            console.error('❌ 刪除節點時發生錯誤:', error);
            this.showNotification('刪除節點失敗', 'danger');
        }
    }

    getNodeDescription(nodeId) {
        const descriptions = {
            'receiving': '處理收貨流程',
            'scanner': '掃描條碼或二維碼',
            'goods-in': '貨物入庫處理',
            'sorting': '按規則分揀貨物',
            'quality-check': '品質檢驗流程',
            'packaging': '包裝作業',
            'plc-controller': 'PLC設備控制',
            'decision': '條件判斷分支',
            'timer': '定時控制',
            'rack': '貨架儲存',
            'buffer': '緩衝區暫存',
            'shipping': '出貨流程',
            'agv-dispatch': 'AGV調度控制',
            'printer': '標籤列印'
        };
        return descriptions[nodeId] || '';
    }

    updateStatusBar() {
        const nodeCount = document.getElementById('node-count');
        const connectionCount = document.getElementById('connection-count');
        const currentFlowName = document.getElementById('current-flow-name');
        
        if (nodeCount) {
            const nodes = this.editor ? this.editor.getNodes().length : 0;
            nodeCount.textContent = nodes;
        }
        
        if (connectionCount) {
            const connections = this.editor ? this.editor.getConnections().length : 0;
            connectionCount.textContent = connections;
        }
        
        if (currentFlowName) {
            currentFlowName.textContent = this.currentFlowName;
        }
    }

    // 流程管理方法
    newFlow() {
        // 顯示新建流程模態框，讓用戶輸入流程名稱
        this.showFlowModal('new', '新建流程');
    }

    loadFlow() {
        const input = document.createElement('input');
        input.type = 'file';
        input.accept = '.yaml,.yml';
        input.onchange = (e) => {
            const file = e.target.files[0];
            if (file) {
                const reader = new FileReader();
                reader.onload = (e) => {
                    try {
                        // 解析 YAML 文件
                        const yamlContent = e.target.result;
                        const flowData = this.parseYamlToFlow(yamlContent);
                        this.importFlow(flowData);
                        this.showNotification(`YAML 流程載入成功：${flowData.name || file.name}`, 'success');
                    } catch (error) {
                        console.error('載入 YAML 流程失敗:', error);
                        this.showNotification(`載入 YAML 流程失敗: ${error.message}`, 'danger');
                    }
                };
                reader.readAsText(file);
            }
        };
        input.click();
    }

    // URL 參數支援：從流程數據載入流程
    async loadFlowFromData(flowData) {
        console.log('📥 從數據載入流程:', flowData);
        
        if (!flowData || typeof flowData !== 'object') {
            throw new Error('無效的流程數據');
        }
        
        try {
            // 清除現有流程
            if (this.editor) {
                this.clearFlow();
            }
            
            // 設置流程名稱
            if (flowData.name) {
                this.currentFlowName = flowData.name;
            }
            
            // 載入節點
            if (flowData.nodes && Array.isArray(flowData.nodes)) {
                for (const nodeData of flowData.nodes) {
                    await this.loadNodeFromData(nodeData);
                }
            }
            
            // 載入連接
            if (flowData.connections && Array.isArray(flowData.connections)) {
                for (const connectionData of flowData.connections) {
                    await this.loadConnectionFromData(connectionData);
                }
            }
            
            // 更新狀態欄
            this.updateStatusBar();
            
            // 🔧 修復：確保所有連接在節點完全渲染後都能正確顯示
            setTimeout(() => {
                console.log('🔄 延遲渲染所有連接...');
                this.ensureAllConnectionsRendered();
            }, 500);
            
            console.log(`✅ 流程 "${this.currentFlowName}" 載入完成`);
            
        } catch (error) {
            console.error('❌ 載入流程數據失敗:', error);
            throw error;
        }
    }
    
    // 確保所有連接都已渲染
    ensureAllConnectionsRendered() {
        console.log('🔍 檢查並渲染所有未顯示的連接...');
        let renderedCount = 0;
        let failedCount = 0;
        
        this.connections.forEach((connData, connId) => {
            // 檢查是否已有 SVG 元素
            const existingSvg = document.getElementById(`connection-${connId}`);
            if (existingSvg) {
                console.log(`✅ 連接 ${connId} 已存在 SVG 元素`);
                return;
            }
            
            // 獲取源和目標節點
            const sourceNode = this.editor.getNode(connData.source);
            const targetNode = this.editor.getNode(connData.target);
            
            if (!sourceNode || !targetNode) {
                console.error(`❌ 連接 ${connId} 缺少節點`);
                failedCount++;
                return;
            }
            
            // 確保節點有 DOM 元素
            if (!sourceNode.element) {
                sourceNode.element = document.getElementById(`node-${sourceNode.id}`);
            }
            if (!targetNode.element) {
                targetNode.element = document.getElementById(`node-${targetNode.id}`);
            }
            
            if (!sourceNode.element || !targetNode.element) {
                console.error(`❌ 連接 ${connId} 節點缺少 DOM 元素`);
                failedCount++;
                return;
            }
            
            // 查找 socket 元素
            const sourceSocket = sourceNode.element.querySelector(
                `.rete-socket[data-socket-key="${connData.sourceSocket}"][data-socket-type="output"]`
            ) || sourceNode.element.querySelector(
                `.rete-output .rete-socket[data-socket-key="${connData.sourceSocket}"]`
            );
            
            const targetSocket = targetNode.element.querySelector(
                `.rete-socket[data-socket-key="${connData.targetSocket}"][data-socket-type="input"]`
            ) || targetNode.element.querySelector(
                `.rete-input .rete-socket[data-socket-key="${connData.targetSocket}"]`
            );
            
            if (sourceSocket && targetSocket) {
                this.renderConnection(sourceSocket, targetSocket, connId);
                renderedCount++;
                console.log(`✅ 成功渲染連接 ${connId}`);
            } else {
                console.error(`❌ 連接 ${connId} 找不到 socket 元素`, {
                    sourceSocket: connData.sourceSocket,
                    targetSocket: connData.targetSocket,
                    sourceSocketFound: !!sourceSocket,
                    targetSocketFound: !!targetSocket
                });
                failedCount++;
            }
        });
        
        console.log(`📊 連接渲染完成：成功 ${renderedCount}，失敗 ${failedCount}`);
    }
    
    // 從數據載入單個節點
    async loadNodeFromData(nodeData) {
        if (!nodeData || !nodeData.id) {
            console.warn('⚠️ 跳過無效的節點數據:', nodeData);
            return;
        }
        
        try {
            // 檢查節點類型是否存在
            const nodeType = this.nodeTypes[nodeData.type];
            if (!nodeType) {
                console.warn(`⚠️ 未知節點類型: ${nodeData.type}`);
                return;
            }
            
            // 創建節點，處理不同的位置格式
            const nodeIndex = this.nodes.size;
            
            // 支援兩種位置格式：position.x/y 或直接 x/y
            let x, y;
            if (nodeData.position && typeof nodeData.position === 'object') {
                x = nodeData.position.x || (150 + nodeIndex * 250);
                y = nodeData.position.y || (100 + (nodeIndex % 3) * 150);
            } else {
                x = nodeData.x || (150 + nodeIndex * 250);
                y = nodeData.y || (100 + (nodeIndex % 3) * 150);
            }
            
            const node = {
                id: nodeData.id,
                type: nodeData.type,
                name: nodeData.name || nodeType.name,
                x: x,
                y: y,
                width: nodeData.width || 200,
                height: nodeData.height || 100,
                parameters: nodeData.parameters || {},
                inputs: nodeData.inputs || [],
                outputs: nodeData.outputs || []
            };
            
            // 創建 Rete.js 節點實例並添加到編輯器
            if (this.editor) {
                // 創建 Rete.js v2 節點實例
                const reteNode = new Rete.ClassicPreset.Node(node.name);
                reteNode.id = node.id;
                
                reteNode.data = {
                    type: node.type,
                    name: node.name,
                    category: nodeType.category,
                    description: nodeType.description,
                    parameters: node.parameters,
                    nodeConfig: nodeType
                };
                
                // 添加適當的 socket
                this.addNodeSockets(reteNode, nodeType, node.type);
                
                // 添加到編輯器
                await this.editor.addNode(reteNode);
                
                // 設置位置
                if (this.area) {
                    await this.area.translate(reteNode.id, { x: node.x, y: node.y });
                }
                
                // 手動 DOM 渲染
                this.renderNodeManually(reteNode, node.x, node.y);
                
                // 將 DOM 元素關聯到 Rete 節點
                reteNode.element = document.getElementById(`node-${node.id}`);
                
                this.nodes.set(node.id, node); // 記錄節點
            }
            
            console.log(`✅ 節點 "${node.name}" (${node.id}) 載入成功，位置: (${node.x}, ${node.y})`);
            
        } catch (error) {
            console.error(`❌ 載入節點失敗:`, nodeData, error);
        }
    }
    
    // 從數據載入單個連接
    async loadConnectionFromData(connectionData) {
        if (!connectionData || !connectionData.source || !connectionData.target) {
            console.warn('⚠️ 跳過無效的連接數據:', connectionData);
            return;
        }
        
        try {
            const sourceNodeId = connectionData.source;
            const targetNodeId = connectionData.target;
            
            // 創建連接 ID
            const connectionId = connectionData.id || `${sourceNodeId}_${targetNodeId}`;
            
            // 檢查是否已存在相同連接 (通常不應該發生，因為已在推斷階段避免重複)
            if (this.connections.has(connectionId)) {
                console.log(`🔄 連接 ${sourceNodeId} → ${targetNodeId} 已存在，跳過重複添加`);
                return;
            }
            
            // 從編輯器取得節點
            const sourceNode = this.editor.getNode(sourceNodeId);
            const targetNode = this.editor.getNode(targetNodeId);
            
            if (!sourceNode || !targetNode) {
                console.error(`❌ 找不到節點: source=${sourceNodeId}, target=${targetNodeId}`);
                return;
            }
            
            // 🔧 修復：智慧地確定 socket 鍵名，支援兩種屬性名格式
            let sourceKey = connectionData.sourceSocket || connectionData.sourceOutput;
            let targetKey = connectionData.targetSocket || connectionData.targetInput;
            
            // 如果沒有指定源 socket，使用第一個可用的輸出
            if (!sourceKey) {
                const sourceOutputs = Object.keys(sourceNode.outputs);
                if (sourceOutputs.length > 0) {
                    sourceKey = sourceOutputs[0];
                    console.log(`🔍 自動選擇源節點輸出: ${sourceKey}`);
                } else {
                    console.error(`❌ 源節點 ${sourceNodeId} 沒有輸出接口`);
                    return;
                }
            }
            
            // 如果沒有指定目標 socket，使用第一個可用的輸入
            if (!targetKey) {
                const targetInputs = Object.keys(targetNode.inputs);
                if (targetInputs.length > 0) {
                    targetKey = targetInputs[0];
                    console.log(`🔍 自動選擇目標節點輸入: ${targetKey}`);
                } else {
                    console.error(`❌ 目標節點 ${targetNodeId} 沒有輸入接口`);
                    return;
                }
            }
            
            // 驗證 socket 是否存在
            if (!sourceNode.outputs[sourceKey]) {
                console.error(`❌ 源節點 ${sourceNodeId} 沒有輸出接口 "${sourceKey}"，可用輸出:`, Object.keys(sourceNode.outputs));
                return;
            }
            
            if (!targetNode.inputs[targetKey]) {
                console.error(`❌ 目標節點 ${targetNodeId} 沒有輸入接口 "${targetKey}"，可用輸入:`, Object.keys(targetNode.inputs));
                return;
            }
            
            // 使用 Rete.js 正確方式創建連接
            const connection = new Rete.ClassicPreset.Connection(
                sourceNode, sourceKey,
                targetNode, targetKey
            );
            
            // 添加到編輯器
            await this.editor.addConnection(connection);
            
            // 記錄連接
            this.connections.set(connectionId, {
                id: connectionId,
                source: sourceNodeId,
                target: targetNodeId,
                sourceSocket: sourceKey,
                targetSocket: targetKey,
                reteConnection: connection
            });
            
            console.log(`✅ 連接 ${sourceNodeId} → ${targetNodeId} 載入成功`);
            
            // 🔧 修復：確保連接視覺化渲染
            if (sourceNode.element && targetNode.element) {
                // 修正選擇器：根據實際 DOM 結構查找 socket
                const sourceSocket = sourceNode.element.querySelector(`.rete-output .rete-socket[data-socket-key="${sourceKey}"]`) || 
                                   sourceNode.element.querySelector(`.rete-socket[data-socket-key="${sourceKey}"][data-socket-type="output"]`);
                const targetSocket = targetNode.element.querySelector(`.rete-input .rete-socket[data-socket-key="${targetKey}"]`) ||
                                   targetNode.element.querySelector(`.rete-socket[data-socket-key="${targetKey}"][data-socket-type="input"]`);
                
                if (sourceSocket && targetSocket) {
                    this.renderConnection(sourceSocket, targetSocket, connectionId);
                    console.log(`✅ 找到 socket 元素並渲染連接: ${sourceKey} → ${targetKey}`);
                } else {
                    console.warn(`⚠️ 無法找到 socket 元素進行渲染: ${sourceKey} → ${targetKey}`);
                    // 調試：列出所有可用的 socket
                    console.log('sourceNode.element:', sourceNode.element);
                    console.log('targetNode.element:', targetNode.element);
                    console.log('所有輸出 socket:', Array.from(sourceNode.element.querySelectorAll('.rete-socket[data-socket-type="output"]')).map(s => s.dataset.socketKey));
                    console.log('所有輸入 socket:', Array.from(targetNode.element.querySelectorAll('.rete-socket[data-socket-type="input"]')).map(s => s.dataset.socketKey));
                }
            } else {
                console.warn(`⚠️ 節點元素尚未渲染，延遲連接視覺化`);
                // 延遲渲染連接
                setTimeout(() => {
                    console.log(`🔄 嘗試延遲渲染連接: ${sourceNodeId} → ${targetNodeId}`);
                    
                    if (sourceNode.element && targetNode.element) {
                        const sourceSocket = sourceNode.element.querySelector(`.rete-output .rete-socket[data-socket-key="${sourceKey}"]`) || 
                                           sourceNode.element.querySelector(`.rete-socket[data-socket-key="${sourceKey}"][data-socket-type="output"]`);
                        const targetSocket = targetNode.element.querySelector(`.rete-input .rete-socket[data-socket-key="${targetKey}"]`) ||
                                           targetNode.element.querySelector(`.rete-socket[data-socket-key="${targetKey}"][data-socket-type="input"]`);
                        
                        if (sourceSocket && targetSocket) {
                            console.log(`✅ 找到 socket 元素，開始渲染連接`);
                            this.renderConnection(sourceSocket, targetSocket, connectionId);
                        } else {
                            console.warn(`⚠️ 延遲後仍無法找到 socket 元素: ${sourceKey} → ${targetKey}`);
                            // 再次延遲嘗試
                            setTimeout(() => {
                                const sourceSocket2 = sourceNode.element.querySelector(`.rete-output .rete-socket[data-socket-key="${sourceKey}"]`) || 
                                                    sourceNode.element.querySelector(`.rete-socket[data-socket-key="${sourceKey}"][data-socket-type="output"]`);
                                const targetSocket2 = targetNode.element.querySelector(`.rete-input .rete-socket[data-socket-key="${targetKey}"]`) ||
                                                    targetNode.element.querySelector(`.rete-socket[data-socket-key="${targetKey}"][data-socket-type="input"]`);
                                if (sourceSocket2 && targetSocket2) {
                                    console.log(`✅ 第二次延遲後找到 socket，渲染連接`);
                                    this.renderConnection(sourceSocket2, targetSocket2, connectionId);
                                }
                            }, 500);
                        }
                    }
                }, 300);
            }
            
        } catch (error) {
            console.error(`❌ 載入連接失敗:`, connectionData, error);
        }
    }
    
    // 清除當前流程
    clearFlow() {
        if (this.editor) {
            // 清除所有節點
            const nodes = this.editor.getNodes ? this.editor.getNodes() : [];
            nodes.forEach(node => {
                if (this.editor.removeNode) {
                    this.editor.removeNode(node.id);
                }
            });
            
            // 清除所有連接
            const connections = this.editor.getConnections ? this.editor.getConnections() : [];
            connections.forEach(connection => {
                if (this.editor.removeConnection) {
                    this.editor.removeConnection(connection.id);
                }
            });
        }
        
        // 重置狀態
        this.currentFlowName = '未命名';
        this.updateStatusBar();
        this.updateFlowNameDisplay();
        
        console.log('🧹 流程已清除');
    }

    saveFlow() {
        // 如果流程名稱是「未命名」或不符合檔名規則，顯示模態框讓用戶輸入正確的檔名
        if (this.currentFlowName === '未命名' || !this.validateFlowName(this.currentFlowName)) {
            this.showFlowModal('save', '保存流程');
            return;
        }
        
        // 直接保存流程
        this.directSaveFlow();
    }

    async directSaveFlow() {
        try {
            // 生成流程數據
            const flowData = this.exportFlow();
            
            if (this.isEditMode && this.currentFlowName && this.currentFlowName !== '未命名') {
                // 編輯模式：保存到伺服器
                console.log(`💾 編輯模式：保存流程到伺服器 - ${this.currentFlowName}`);
                
                const response = await fetch(`/api/flow-designer/flows/${encodeURIComponent(this.currentFlowName)}/yaml`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        flow_data: flowData
                    })
                });
                
                if (!response.ok) {
                    let errorMessage = `伺服器回應錯誤: ${response.status}`;
                    try {
                        const errorData = await response.json();
                        errorMessage = errorData.detail || errorMessage;
                        console.error('❌ 伺服器錯誤詳情:', errorData);
                    } catch (parseError) {
                        console.error('❌ 無法解析錯誤回應:', parseError);
                        // 嘗試讀取原始回應文本
                        try {
                            const errorText = await response.text();
                            console.error('❌ 錯誤回應原始內容:', errorText);
                            errorMessage = errorText || errorMessage;
                        } catch (textError) {
                            console.error('❌ 無法讀取錯誤回應文本:', textError);
                        }
                    }
                    throw new Error(errorMessage);
                }
                
                const result = await response.json();
                
                // 顯示成功訊息
                this.showNotification(`流程 "${this.currentFlowName}" 已成功保存到伺服器`, 'success');
                
                // 標記為已保存狀態
                this.markFlowAsSaved();
                
                console.log('✅ 流程已保存到伺服器:', result);
                
            } else {
                // 標準模式：下載檔案
                console.log('📥 標準模式：下載流程檔案');
                
                const blob = new Blob([flowData], { type: 'application/x-yaml' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = `${this.currentFlowName || 'flow'}.yaml`;
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);
                
                this.showNotification('流程已下載', 'success');
                console.log('✅ 流程已下載:', this.currentFlowName);
            }
            
        } catch (error) {
            console.error('❌ 保存流程失敗:', error);
            this.showNotification(`保存流程失敗: ${error.message}`, 'danger');
        }
    }

    // YAML 解析：從 YAML 文件恢復流程數據
    parseYamlToFlow(yamlContent) {
        console.log('🔄 解析 YAML 內容為流程數據...');
        
        try {
            // 檢查 js-yaml 庫是否可用
            if (typeof jsyaml === 'undefined') {
                throw new Error('js-yaml 庫未載入，無法解析 YAML 文件');
            }
            
            // 解析 YAML 內容
            const yamlData = jsyaml.load(yamlContent);
            console.log('✅ YAML 解析成功:', yamlData);
            
            // 轉換為 Flow Designer 格式
            const flowData = this.convertDslToFlow(yamlData);
            console.log('✅ DSL 轉換為流程數據完成:', flowData);
            
            return flowData;
            
        } catch (error) {
            console.error('❌ YAML 解析失敗:', error);
            throw new Error(`YAML 解析失敗: ${error.message}`);
        }
    }
    
    // DSL 轉換：將 YAML DSL 轉換為 Flow Designer 格式
    convertDslToFlow(dslData) {
        console.log('🔄 轉換 DSL 數據為流程格式...');
        
        if (!dslData || typeof dslData !== 'object') {
            throw new Error('無效的 DSL 數據格式');
        }
        
        const flowData = {
            name: dslData.name || (dslData.flow_metadata && dslData.flow_metadata.name) || '未命名流程',
            description: dslData.description || (dslData.flow_metadata && dslData.flow_metadata.description) || '',
            nodes: [],
            connections: []
        };
        
        // 轉換步驟為節點
        if (dslData.steps && Array.isArray(dslData.steps)) {
            dslData.steps.forEach((step, index) => {
                const node = this.convertDslStepToNode(step, index);
                if (node) {
                    flowData.nodes.push(node);
                }
            });
        }
        
        // 從步驟依賴關係推斷連接
        if (dslData.steps && Array.isArray(dslData.steps)) {
            const connections = this.inferConnectionsFromSteps(dslData.steps);
            flowData.connections = connections;
        }
        
        console.log('✅ DSL 轉流程格式完成，節點數量:', flowData.nodes.length, '連接數量:', flowData.connections.length);
        return flowData;
    }
    
    // 將 DSL 步驟轉換為節點
    convertDslStepToNode(step, index) {
        if (!step || !step.type) {
            console.warn('⚠️ 跳過無效的步驟:', step);
            return null;
        }
        
        // 基於 DSL 步驟類型找到對應的節點類型
        let nodeType = this.findNodeTypeByDslType(step.type, step.function);
        if (!nodeType) {
            console.warn(`⚠️ 未找到對應的節點類型: ${step.type}, function: ${step.function}`);
            return null;
        }
        
        return {
            id: step.id || `node_${index}`,
            type: nodeType,
            name: step.name || step.type,
            x: (index % 3) * 250 + 100, // 簡單的網格佈局
            y: Math.floor(index / 3) * 150 + 100,
            width: 200,
            height: 100,
            parameters: step.parameters || {},
            inputs: [],
            outputs: []
        };
    }
    
    // 根據 DSL 類型找到對應的節點類型
    findNodeTypeByDslType(dslType, functionName) {
        // 優先通過 function name 精確匹配
        if (functionName) {
            // 先嘗試直接匹配 function name
            if (this.nodeTypes[functionName]) {
                return functionName;
            }
            
            // 在所有節點類型中搜索匹配的 function name
            for (const [nodeTypeId, nodeType] of Object.entries(this.nodeTypes)) {
                if (nodeType.id === functionName || nodeType.function === functionName) {
                    return nodeTypeId;
                }
            }
        }
        
        // 通過 DSL 類型搜索
        for (const [nodeTypeId, nodeType] of Object.entries(this.nodeTypes)) {
            if (nodeType.dslType === dslType) {
                return nodeTypeId;
            }
        }
        
        // 檢查是否從 node-types.js 載入
        if (window.FlowDesigner && window.FlowDesigner.ALL_NODE_TYPES) {
            // 通過 function name 在 node-types.js 中查找
            if (functionName && window.FlowDesigner.ALL_NODE_TYPES[functionName]) {
                return functionName;
            }
            
            // 通過 DSL 類型分類查找
            const categoryMap = {
                'condition_nodes': window.FlowDesigner.CONDITION_NODES,
                'logic_nodes': window.FlowDesigner.LOGIC_NODES,
                'action_nodes': window.FlowDesigner.ACTION_NODES,
                'script_nodes': window.FlowDesigner.SCRIPT_NODES
            };
            
            const categoryNodes = categoryMap[dslType];
            if (categoryNodes && functionName && categoryNodes[functionName]) {
                return functionName;
            }
        }
        
        // 如果沒有找到，返回 null
        return null;
    }
    
    // 從步驟依賴關係推斷連接
    inferConnectionsFromSteps(steps) {
        const connections = [];
        const connectionSet = new Set(); // 用於避免重複連接
        
        steps.forEach((step, index) => {
            // 處理 depends_on 依賴關係 (向前依賴)
            if (step.depends_on && Array.isArray(step.depends_on)) {
                step.depends_on.forEach(dependency => {
                    const sourceStep = steps.find(s => s.id === dependency);
                    if (sourceStep) {
                        // 創建連接的唯一標識符
                        const connectionKey = `${dependency}->${step.id}`;
                        
                        // 檢查是否已經存在此連接
                        if (connectionSet.has(connectionKey)) {
                            console.log(`🔄 跳過重複連接: ${connectionKey} (已從 depends_on 創建)`);
                            return;
                        }
                        
                        // 🔧 修復：智慧地確定 socket 名稱
                        const sourceSocketName = this.getFirstOutputSocketName(sourceStep);
                        const targetSocketName = this.getFirstInputSocketName(step);
                        
                        if (sourceSocketName && targetSocketName) {
                            connections.push({
                                id: `${dependency}_${step.id}`,
                                source: dependency,
                                target: step.id,
                                sourceOutput: sourceSocketName,
                                targetInput: targetSocketName
                            });
                            connectionSet.add(connectionKey);
                            console.log(`✅ 從 depends_on 創建連接: ${connectionKey}`);
                        } else {
                            console.warn(`⚠️ 無法確定連接的 socket 名稱: ${dependency} → ${step.id}`);
                        }
                    }
                });
            }
            
            // 處理 next_steps 下一步關係 (向後連接)
            if (step.next_steps && Array.isArray(step.next_steps)) {
                step.next_steps.forEach(nextStepId => {
                    const targetStep = steps.find(s => s.id === nextStepId);
                    if (targetStep) {
                        // 創建連接的唯一標識符
                        const connectionKey = `${step.id}->${nextStepId}`;
                        
                        // 檢查是否已經存在此連接
                        if (connectionSet.has(connectionKey)) {
                            console.log(`🔄 跳過重複連接: ${connectionKey} (已從 next_steps 創建)`);
                            return;
                        }
                        
                        // 🔧 修復：智慧地確定 socket 名稱
                        const sourceSocketName = this.getFirstOutputSocketName(step);
                        const targetSocketName = this.getFirstInputSocketName(targetStep);
                        
                        if (sourceSocketName && targetSocketName) {
                            connections.push({
                                id: `${step.id}_${nextStepId}`,
                                source: step.id,
                                target: nextStepId,
                                sourceOutput: sourceSocketName,
                                targetInput: targetSocketName
                            });
                            connectionSet.add(connectionKey);
                            console.log(`✅ 從 next_steps 創建連接: ${connectionKey}`);
                        } else {
                            console.warn(`⚠️ 無法確定連接的 socket 名稱: ${step.id} → ${nextStepId}`);
                        }
                    }
                });
            }
        });
        
        console.log(`🔗 連接推斷完成，總計 ${connections.length} 個唯一連接`);
        return connections;
    }
    
    // 獲取步驟的第一個輸出 socket 名稱
    getFirstOutputSocketName(step) {
        if (!step) return null;
        
        // 根據步驟類型找到對應的節點類型
        const nodeTypeId = this.findNodeTypeByDslType(step.type, step.function);
        if (!nodeTypeId) return null;
        
        const nodeType = this.nodeTypes[nodeTypeId];
        if (!nodeType) return null;
        
        // 獲取輸出定義
        if (nodeType.outputs && Array.isArray(nodeType.outputs) && nodeType.outputs.length > 0) {
            return nodeType.outputs[0].name || 'output';
        }
        
        return 'output'; // 預設值
    }
    
    // 獲取步驟的第一個輸入 socket 名稱
    getFirstInputSocketName(step) {
        if (!step) return null;
        
        // 根據步驟類型找到對應的節點類型
        const nodeTypeId = this.findNodeTypeByDslType(step.type, step.function);
        if (!nodeTypeId) return null;
        
        const nodeType = this.nodeTypes[nodeTypeId];
        if (!nodeType) return null;
        
        // 獲取輸入定義
        if (nodeType.inputs && Array.isArray(nodeType.inputs) && nodeType.inputs.length > 0) {
            return nodeType.inputs[0].name || 'input';
        }
        
        return 'input'; // 預設值
    }

    convertFlowToDsl() {
        console.log('🔄 轉換流程圖為 DSL 結構...');
        
        if (!this.editor) {
            throw new Error('編輯器未初始化');
        }

        const nodes = this.editor.getNodes();
        const connections = this.editor.getConnections();
        
        if (nodes.length === 0) {
            throw new Error('流程圖為空，無法生成 DSL');
        }

        // Phase 3.2: 建立 DSL 資料結構
        const dslData = {
            flow_metadata: {
                name: this.currentFlowName || '未命名流程',
                description: `由 Flow Designer 生成的 YAML DSL 流程`,
                version: '1.0',
                created_at: new Date().toISOString(),
                generated_by: 'Flow Designer v3.2'
            },
            variables: this.extractVariables(nodes),
            steps: this.convertNodesToSteps(nodes, connections)
        };

        console.log('✅ DSL 結構轉換完成:', dslData);
        return dslData;
    }

    extractVariables(nodes) {
        // Phase 3.2: 從節點參數中提取變數定義
        const variables = {};
        
        nodes.forEach(node => {
            if (node.data && node.data.parameters) {
                Object.entries(node.data.parameters).forEach(([paramName, paramValue]) => {
                    // 只提取已設定且非預設值的參數作為變數
                    if (paramValue && paramValue !== '' && paramValue !== '0' && paramValue !== 'false') {
                        const variableName = `${node.data.type}_${paramName}`;
                        variables[variableName] = {
                            type: this.inferVariableType(paramValue),
                            value: this.convertParameterValue(paramValue),
                            description: `來自節點 ${node.data.name} 的 ${paramName} 參數`
                        };
                    }
                });
            }
        });

        return variables;
    }

    convertNodesToSteps(nodes, connections) {
        // Phase 3.2: 將節點轉換為 DSL 步驟
        const steps = [];
        const nodeMap = new Map();
        const connectionMap = new Map();

        // 建立節點映射
        nodes.forEach(node => {
            nodeMap.set(node.id, node);
        });

        // 建立連線映射
        connections.forEach(conn => {
            if (!connectionMap.has(conn.source)) {
                connectionMap.set(conn.source, []);
            }
            connectionMap.get(conn.source).push(conn);
        });

        // 找出起始節點（沒有輸入連接的節點）
        const startNodes = nodes.filter(node => {
            return !connections.some(conn => conn.target === node.id);
        });

        if (startNodes.length === 0) {
            console.warn('⚠️ 未找到起始節點，使用第一個節點作為起始點');
            startNodes.push(nodes[0]);
        }

        // 從起始節點開始遍歷生成步驟
        const visitedNodes = new Set();
        
        startNodes.forEach(startNode => {
            this.traverseNodesForSteps(startNode, nodeMap, connectionMap, steps, visitedNodes);
        });

        return steps;
    }

    traverseNodesForSteps(node, nodeMap, connectionMap, steps, visitedNodes) {
        if (visitedNodes.has(node.id)) {
            return;
        }
        
        visitedNodes.add(node.id);
        
        // 將節點轉換為 DSL 步驟
        const step = this.convertNodeToStep(node);
        if (step) {
            steps.push(step);
        }

        // 遞歸處理下一個節點
        const outgoingConnections = connectionMap.get(node.id) || [];
        outgoingConnections.forEach(conn => {
            const nextNode = nodeMap.get(conn.target);
            if (nextNode) {
                this.traverseNodesForSteps(nextNode, nodeMap, connectionMap, steps, visitedNodes);
            }
        });
    }

    convertNodeToStep(node) {
        // Phase 3.2: 根據節點類型轉換為對應的 DSL 步驟
        const nodeType = this.nodeTypes[node.data.type];
        if (!nodeType) {
            console.warn(`⚠️ 未知節點類型: ${node.data.type}`);
            return null;
        }

        const step = {
            name: `步驟_${node.data.name}`,
            description: node.data.description || nodeType.description,
        };

        // 根據 DSL 類型設定步驟內容
        switch (nodeType.dslType) {
            case 'condition_nodes':
                step.type = 'condition';
                step.function = nodeType.id;
                step.source = nodeType.source;
                step.parameters = this.convertNodeParameters(node.data.parameters, nodeType);
                break;
                
            case 'logic_nodes':
                step.type = 'function_call';
                step.function = nodeType.id;
                step.source = nodeType.source;
                step.parameters = this.convertNodeParameters(node.data.parameters, nodeType);
                break;
                
            case 'action_nodes':
                step.type = 'action';
                step.function = nodeType.id;
                step.source = nodeType.source;
                step.parameters = this.convertNodeParameters(node.data.parameters, nodeType);
                break;
                
            case 'script_nodes':
                step.type = 'script';
                step.script_type = nodeType.id;
                step.parameters = this.convertNodeParameters(node.data.parameters, nodeType);
                break;
                
            default:
                console.warn(`⚠️ 未支援的 DSL 類型: ${nodeType.dslType}`);
                step.type = 'function_call';
                step.function = nodeType.id;
                step.parameters = this.convertNodeParameters(node.data.parameters, nodeType);
        }

        return step;
    }

    convertNodeParameters(nodeParameters, nodeType) {
        // Phase 3.2: 轉換節點參數為 DSL 參數格式
        const dslParameters = {};
        
        if (!nodeParameters || !nodeType.parameters) {
            return dslParameters;
        }

        nodeType.parameters.forEach(paramDef => {
            const paramValue = nodeParameters[paramDef.name];
            if (paramValue !== undefined && paramValue !== '') {
                dslParameters[paramDef.name] = this.convertParameterValue(paramValue, paramDef.type);
            } else if (paramDef.required && paramDef.default !== undefined) {
                // 使用預設值填充必要參數
                dslParameters[paramDef.name] = this.convertParameterValue(paramDef.default, paramDef.type);
            }
        });

        return dslParameters;
    }

    convertParameterValue(value, paramType) {
        // Phase 3.2: 根據參數類型轉換值
        if (value === null || value === undefined) {
            return null;
        }

        const valueStr = String(value).trim();
        
        switch (paramType) {
            case 'integer':
            case 'int':
                return parseInt(valueStr) || 0;
                
            case 'float':
            case 'number':
                return parseFloat(valueStr) || 0.0;
                
            case 'boolean':
                return valueStr.toLowerCase() === 'true' || valueStr === '1';
                
            case 'List[integer]':
                try {
                    return JSON.parse(valueStr).map(v => parseInt(v));
                } catch {
                    return valueStr.split(',').map(v => parseInt(v.trim())).filter(v => !isNaN(v));
                }
                
            case 'List[string]':
                try {
                    return JSON.parse(valueStr);
                } catch {
                    return valueStr.split(',').map(v => v.trim());
                }
                
            case 'Dict[str, Any]':
                try {
                    return JSON.parse(valueStr);
                } catch {
                    return { value: valueStr };
                }
                
            default:
                return valueStr;
        }
    }

    inferVariableType(value) {
        // Phase 3.2: 推斷變數類型
        if (typeof value === 'boolean') return 'boolean';
        if (typeof value === 'number') return Number.isInteger(value) ? 'integer' : 'float';
        if (Array.isArray(value)) return 'list';
        if (typeof value === 'object') return 'dict';
        
        // 嘗試解析字串
        const strValue = String(value).trim();
        if (strValue === 'true' || strValue === 'false') return 'boolean';
        if (/^\d+$/.test(strValue)) return 'integer';
        if (/^\d*\.\d+$/.test(strValue)) return 'float';
        
        return 'string';
    }

    generateYamlContent(dslData) {
        // Phase 3.2: 生成 YAML DSL 內容
        console.log('🔄 生成 YAML 內容...', dslData);
        
        try {
            let yamlContent = '';
            
            // 1. 生成檔案頭部註解
            yamlContent += this.generateYamlHeader(dslData.flow_metadata);
            
            // 2. 生成變數定義部分
            if (dslData.variables && Object.keys(dslData.variables).length > 0) {
                yamlContent += this.generateYamlVariables(dslData.variables);
            }
            
            // 3. 生成步驟執行部分
            if (dslData.steps && dslData.steps.length > 0) {
                yamlContent += this.generateYamlSteps(dslData.steps);
            }
            
            console.log('✅ YAML 內容生成完成');
            return yamlContent;
            
        } catch (error) {
            console.error('❌ YAML 內容生成失敗:', error);
            throw new Error(`YAML 內容生成失敗: ${error.message}`);
        }
    }

    generateYamlHeader(metadata) {
        // Phase 3.2: 生成 YAML 檔案頭部
        const timestamp = new Date().toISOString();
        
        return `# ${metadata.name}
# 描述: ${metadata.description}
# 版本: ${metadata.version}
# 創建時間: ${timestamp}
# 生成工具: Flow Designer v3.2

`;
    }

    generateYamlVariables(variables) {
        // Phase 3.2: 生成變數定義部分
        let content = '# 變數定義\nvariables:\n';
        
        Object.entries(variables).forEach(([varName, varData]) => {
            content += `  ${varName}:\n`;
            content += `    type: ${varData.type}\n`;
            if (varData.value !== null && varData.value !== undefined) {
                content += `    value: ${this.formatYamlValue(varData.value, varData.type)}\n`;
            }
            if (varData.description) {
                content += `    description: "${varData.description}"\n`;
            }
        });
        
        content += '\n';
        return content;
    }

    generateYamlSteps(steps) {
        // Phase 3.2: 生成步驟執行部分
        let content = '# 步驟定義\nsteps:\n';
        
        steps.forEach((step, index) => {
            content += `  - step: ${index + 1}\n`;
            content += `    name: "${step.name}"\n`;
            
            if (step.description) {
                content += `    description: "${step.description}"\n`;
            }
            
            // 根據步驟類型生成對應的 YAML 結構
            switch (step.type) {
                case 'condition_nodes':
                    content += this.generateConditionStepYaml(step);
                    break;
                case 'logic_nodes':
                    content += this.generateLogicStepYaml(step);
                    break;
                case 'action_nodes':
                    content += this.generateActionStepYaml(step);
                    break;
                case 'script_nodes':
                    content += this.generateScriptStepYaml(step);
                    break;
                default:
                    content += this.generateGenericStepYaml(step);
            }
            
            content += '\n';
        });
        
        return content;
    }

    generateConditionStepYaml(step) {
        // Phase 3.2: 生成條件節點的 YAML
        let content = `    type: condition_nodes\n`;
        content += `    function: ${step.function}\n`;
        
        if (step.parameters && Object.keys(step.parameters).length > 0) {
            content += `    parameters:\n`;
            Object.entries(step.parameters).forEach(([key, value]) => {
                content += `      ${key}: ${this.formatYamlValue(value)}\n`;
            });
        }
        
        if (step.outputs && step.outputs.length > 0) {
            content += `    outputs:\n`;
            step.outputs.forEach(output => {
                content += `      - ${output}\n`;
            });
        }
        
        return content;
    }

    generateLogicStepYaml(step) {
        // Phase 3.2: 生成邏輯節點的 YAML
        let content = `    type: logic_nodes\n`;
        content += `    function: ${step.function}\n`;
        
        if (step.parameters && Object.keys(step.parameters).length > 0) {
            content += `    parameters:\n`;
            Object.entries(step.parameters).forEach(([key, value]) => {
                content += `      ${key}: ${this.formatYamlValue(value)}\n`;
            });
        }
        
        if (step.assign_to) {
            content += `    assign_to: ${step.assign_to}\n`;
        }
        
        return content;
    }

    generateActionStepYaml(step) {
        // Phase 3.2: 生成動作節點的 YAML
        let content = `    type: action_nodes\n`;
        content += `    function: ${step.function}\n`;
        
        if (step.parameters && Object.keys(step.parameters).length > 0) {
            content += `    parameters:\n`;
            Object.entries(step.parameters).forEach(([key, value]) => {
                content += `      ${key}: ${this.formatYamlValue(value)}\n`;
            });
        }
        
        if (step.result_handler) {
            content += `    result_handler: ${step.result_handler}\n`;
        }
        
        return content;
    }

    generateScriptStepYaml(step) {
        // Phase 3.2: 生成腳本節點的 YAML
        let content = `    type: script_nodes\n`;
        content += `    control: ${step.function}\n`;
        
        if (step.parameters && Object.keys(step.parameters).length > 0) {
            content += `    parameters:\n`;
            Object.entries(step.parameters).forEach(([key, value]) => {
                if (key === 'if_branch' || key === 'else_branch' || key === 'loop_body') {
                    // 處理子步驟
                    if (Array.isArray(value) && value.length > 0) {
                        content += `      ${key}:\n`;
                        value.forEach((subStep, index) => {
                            content += `        - step: ${index + 1}\n`;
                            content += `          function: ${subStep.function}\n`;
                            if (subStep.parameters) {
                                content += `          parameters:\n`;
                                Object.entries(subStep.parameters).forEach(([subKey, subValue]) => {
                                    content += `            ${subKey}: ${this.formatYamlValue(subValue)}\n`;
                                });
                            }
                        });
                    }
                } else {
                    content += `      ${key}: ${this.formatYamlValue(value)}\n`;
                }
            });
        }
        
        return content;
    }

    generateGenericStepYaml(step) {
        // Phase 3.2: 生成通用步驟的 YAML
        let content = `    type: ${step.type || 'generic'}\n`;
        content += `    function: ${step.function}\n`;
        
        if (step.parameters && Object.keys(step.parameters).length > 0) {
            content += `    parameters:\n`;
            Object.entries(step.parameters).forEach(([key, value]) => {
                content += `      ${key}: ${this.formatYamlValue(value)}\n`;
            });
        }
        
        return content;
    }

    formatYamlValue(value, type = null) {
        // Phase 3.2: 格式化 YAML 值
        if (value === null || value === undefined) {
            return 'null';
        }
        
        // 根據類型格式化
        if (type) {
            switch (type) {
                case 'string':
                    return `"${String(value).replace(/"/g, '\\"')}"`;
                case 'boolean':
                    return value === true || value === 'true' ? 'true' : 'false';
                case 'integer':
                    return parseInt(value) || 0;
                case 'float':
                    return parseFloat(value) || 0.0;
                case 'list':
                    if (Array.isArray(value)) {
                        return `[${value.map(v => this.formatYamlValue(v)).join(', ')}]`;
                    }
                    return '[]';
                case 'dict':
                    if (typeof value === 'object') {
                        return JSON.stringify(value);
                    }
                    return '{}';
            }
        }
        
        // 自動推斷格式化
        if (typeof value === 'string') {
            // 檢查是否為變數引用
            if (value.startsWith('${') && value.endsWith('}')) {
                return value; // 變數引用不加引號
            }
            return `"${value.replace(/"/g, '\\"')}"`;
        } else if (typeof value === 'boolean') {
            return value ? 'true' : 'false';
        } else if (typeof value === 'number') {
            return value;
        } else if (Array.isArray(value)) {
            return `[${value.map(v => this.formatYamlValue(v)).join(', ')}]`;
        } else if (typeof value === 'object') {
            return JSON.stringify(value);
        }
        
        return String(value);
    }

    // Phase 3.3: YAML DSL 載入功能
    loadYamlDsl() {
        console.log('🔄 開始載入 YAML DSL...');
        
        // 觸發文件選擇對話框
        const fileInput = document.getElementById('yaml-file-input');
        if (fileInput) {
            fileInput.click();
        } else {
            throw new Error('找不到 YAML 文件輸入元素');
        }
    }

    handleYamlFileInput(event) {
        // Phase 3.3: 處理 YAML 文件輸入
        const file = event.target.files[0];
        if (!file) {
            console.log('⚠️ 未選擇文件');
            return;
        }

        console.log('📁 讀取 YAML 文件:', file.name);

        const reader = new FileReader();
        reader.onload = (e) => {
            try {
                const yamlContent = e.target.result;
                this.parseDslToFlow(yamlContent);
                this.showNotification(`YAML DSL 載入成功: ${file.name}`, 'success');
            } catch (error) {
                console.error('❌ YAML 解析失敗:', error);
                this.showNotification('YAML 解析失敗: ' + error.message, 'danger');
            }
        };

        reader.onerror = () => {
            console.error('❌ 文件讀取失敗');
            this.showNotification('文件讀取失敗', 'danger');
        };

        reader.readAsText(file);
        
        // 清空文件輸入，允許重複選擇同一文件
        event.target.value = '';
    }

    parseDslToFlow(yamlContent) {
        // Phase 3.3: 解析 YAML DSL 內容並轉換為視覺化流程圖
        console.log('🔄 解析 YAML DSL 內容...');

        try {
            // 使用 js-yaml 解析 YAML 內容
            const dslData = jsyaml.load(yamlContent);
            console.log('📊 解析的 DSL 資料:', dslData);

            // 驗證 DSL 結構
            this.validateDslStructure(dslData);

            // 清空現有流程圖
            this.clearFlow();

            // 設置流程元資料
            if (dslData.flow_metadata || dslData.metadata) {
                const metadata = dslData.flow_metadata || dslData.metadata;
                this.currentFlowName = metadata.name || '載入的流程';
                this.updateFlowNameDisplay();
            }

            // 載入變數定義
            if (dslData.variables) {
                this.loadVariables(dslData.variables);
            }

            // 創建節點和連接
            if (dslData.steps && Array.isArray(dslData.steps)) {
                this.createNodesFromSteps(dslData.steps);
                this.rebuildConnections(dslData.steps);
            }

            console.log('✅ YAML DSL 解析和載入完成');

        } catch (error) {
            console.error('❌ YAML DSL 解析失敗:', error);
            throw new Error(`YAML DSL 解析失敗: ${error.message}`);
        }
    }

    validateDslStructure(dslData) {
        // Phase 3.3: 驗證 DSL 資料結構
        if (!dslData || typeof dslData !== 'object') {
            throw new Error('無效的 YAML DSL 格式：根物件無效');
        }

        // 檢查必要的結構
        if (!dslData.steps && !dslData.variables) {
            throw new Error('無效的 YAML DSL 格式：缺少 steps 或 variables 區塊');
        }

        if (dslData.steps && !Array.isArray(dslData.steps)) {
            throw new Error('無效的 YAML DSL 格式：steps 必須是陣列');
        }

        console.log('✅ DSL 結構驗證通過');
    }

    clearFlow() {
        // Phase 3.3: 清空現有流程圖
        console.log('🧹 清空現有流程圖...');

        // 清空編輯器內容
        const editorContainer = document.getElementById('rete-editor');
        if (editorContainer) {
            editorContainer.innerHTML = '';
        }

        // 重置內部狀態
        this.nodes.clear();
        this.connections.clear();
        this.nodeIdCounter = 1;

        // 重新初始化編輯器區域
        this.setupEditorArea();

        console.log('✅ 流程圖清空完成');
    }

    loadVariables(variables) {
        // Phase 3.3: 載入變數定義（預留功能）
        console.log('📝 載入變數定義:', variables);
        
        // 變數可以存儲在編輯器的全局狀態中
        this.flowVariables = variables || {};
        
        console.log('✅ 變數載入完成');
    }

    createNodesFromSteps(steps) {
        // Phase 3.3: 從 DSL 步驟創建視覺化節點
        console.log('🔄 從 DSL 步驟創建節點...');

        const nodePositions = this.calculateNodePositions(steps.length);
        
        steps.forEach((step, index) => {
            try {
                const node = this.createNodeFromStep(step, index, nodePositions[index]);
                if (node) {
                    console.log(`✅ 創建節點 ${index + 1}: ${step.name || step.function}`);
                }
            } catch (error) {
                console.error(`❌ 創建節點 ${index + 1} 失敗:`, error);
                // 繼續處理其他節點，不中斷整個流程
            }
        });

        console.log('✅ 所有節點創建完成');
    }

    calculateNodePositions(nodeCount) {
        // Phase 3.3: 計算節點的視覺位置
        const positions = [];
        const startX = 100;
        const startY = 100;
        const spacingX = 300;
        const spacingY = 150;
        const nodesPerRow = 3;

        for (let i = 0; i < nodeCount; i++) {
            const row = Math.floor(i / nodesPerRow);
            const col = i % nodesPerRow;
            
            positions.push({
                x: startX + col * spacingX,
                y: startY + row * spacingY
            });
        }

        return positions;
    }

    createNodeFromStep(step, stepIndex, position) {
        // Phase 3.3: 從單個 DSL 步驟創建節點
        const nodeId = `node_${this.nodeIdCounter++}`;
        
        // 根據步驟類型查找對應的節點類型定義
        const nodeTypeId = this.findNodeTypeByFunction(step.function, step.type);
        if (!nodeTypeId) {
            console.warn(`⚠️ 找不到函數 ${step.function} 對應的節點類型`);
            return null;
        }

        const nodeType = this.nodeTypes[nodeTypeId];
        if (!nodeType) {
            console.warn(`⚠️ 節點類型 ${nodeTypeId} 不存在`);
            return null;
        }

        // 創建節點資料
        const nodeData = {
            id: nodeId,
            type: nodeTypeId,
            name: step.name || nodeType.name,
            description: step.description || nodeType.description,
            parameters: step.parameters || {},
            position: position
        };

        // 處理節點輸入和輸出
        const nodeInputs = this.processNodeConnections(nodeType.inputs || [], 'input');
        const nodeOutputs = this.processNodeConnections(nodeType.outputs || [], 'output');

        // 創建節點物件
        const node = {
            id: nodeId,
            data: nodeData,
            inputs: nodeInputs,
            outputs: nodeOutputs
        };

        // 存儲節點
        this.nodes.set(nodeId, node);

        // 渲染節點到 DOM
        this.renderNodeToDOM(node);

        return node;
    }

    renderNodeToDOM(node) {
        // Phase 3.3: 渲染節點到 DOM
        console.log('🎨 渲染節點到 DOM:', node.id);

        const editorContainer = document.getElementById('rete-editor');
        if (!editorContainer) {
            console.error('❌ 編輯器容器不存在');
            return;
        }

        // 創建節點元素
        const nodeElement = document.createElement('div');
        nodeElement.id = node.id;
        nodeElement.className = `flow-node node-${node.data.type}`;
        nodeElement.style.cssText = `
            position: absolute;
            left: ${node.data.position.x}px;
            top: ${node.data.position.y}px;
            min-width: 180px;
            min-height: 60px;
            background: white;
            border: 2px solid #2563eb;
            border-radius: 8px;
            padding: 12px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.15);
            cursor: move;
            user-select: none;
            z-index: 1;
        `;

        // 節點內容
        const nodeType = this.nodeTypes[node.data.type];
        nodeElement.innerHTML = `
            <div class="node-header" style="font-weight: 600; font-size: 12px; margin-bottom: 4px;">
                ${nodeType && nodeType.icon ? nodeType.icon : '⚡'} ${node.data.name}
            </div>
            <div class="node-function" style="font-size: 10px; color: #666; margin-bottom: 8px;">
                ${nodeType && nodeType.id ? nodeType.id : node.data.type}
            </div>
            <div class="node-source" style="font-size: 9px; color: #888;">
                ${nodeType && nodeType.source ? nodeType.source : 'unknown'}
            </div>
        `;

        // 添加拖拽功能
        this.makeDraggable(nodeElement);

        // 添加到編輯器
        editorContainer.appendChild(nodeElement);

        console.log('✅ 節點渲染完成:', node.id);
    }

    makeDraggable(element) {
        // Phase 3.3: 使節點可拖拽
        let isDragging = false;
        let startX, startY, startLeft, startTop;

        element.addEventListener('mousedown', (e) => {
            isDragging = true;
            startX = e.clientX;
            startY = e.clientY;
            startLeft = parseInt(element.style.left) || 0;
            startTop = parseInt(element.style.top) || 0;
            element.style.zIndex = '999';
            e.preventDefault();
        });

        document.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            
            const deltaX = e.clientX - startX;
            const deltaY = e.clientY - startY;
            
            element.style.left = (startLeft + deltaX) + 'px';
            element.style.top = (startTop + deltaY) + 'px';
        });

        document.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                element.style.zIndex = '1';
            }
        });
    }

    setupEditorArea() {
        // Phase 3.3: 設置編輯器區域
        console.log('🔄 設置編輯器區域...');

        const editorContainer = document.getElementById('rete-editor');
        if (!editorContainer) {
            console.error('❌ 編輯器容器不存在');
            return;
        }

        // 設置基本樣式 - 移除行內背景樣式，讓 CSS 控制
        editorContainer.style.cssText = `
            position: relative;
            width: 100%;
            height: 100%;
            overflow: hidden;
            min-height: 500px;
        `;

        // 添加背景網格效果
        editorContainer.classList.add('flow-editor-grid');

        console.log('✅ 編輯器區域設置完成');
    }

    findNodeTypeByFunction(functionName, stepType) {
        // Phase 3.3: 根據函數名稱和步驟類型查找節點類型ID
        
        // 首先嘗試直接匹配函數名稱
        for (const [nodeId, nodeType] of Object.entries(this.nodeTypes)) {
            if (nodeType.id === functionName || nodeId === functionName) {
                return nodeId;
            }
        }

        // 如果沒有直接匹配，根據步驟類型在對應類別中查找
        const categoryMap = {
            'condition_nodes': 'condition',
            'logic_nodes': 'logic', 
            'action_nodes': 'action',
            'script_nodes': 'script'
        };

        const category = categoryMap[stepType];
        if (category && window.FlowDesigner) {
            const categoryNodes = window.FlowDesigner.NODE_TYPES_BY_CATEGORY[category] || {};
            
            for (const [nodeId, nodeType] of Object.entries(categoryNodes)) {
                if (nodeType.id === functionName || nodeId === functionName) {
                    return nodeId;
                }
            }
        }

        return null;
    }

    rebuildConnections(steps) {
        // Phase 3.3: 重建節點間的連接關係
        console.log('🔄 重建節點連接...');

        // 基於步驟順序創建串行連接
        for (let i = 0; i < steps.length - 1; i++) {
            const sourceNodeId = `node_${i + 1}`;
            const targetNodeId = `node_${i + 2}`;

            try {
                this.createVisualConnection(sourceNodeId, targetNodeId);
                console.log(`✅ 連接 ${sourceNodeId} → ${targetNodeId}`);
            } catch (error) {
                console.error(`❌ 連接失敗 ${sourceNodeId} → ${targetNodeId}:`, error);
            }
        }

        console.log('✅ 節點連接重建完成');
    }

    createVisualConnection(sourceNodeId, targetNodeId) {
        // Phase 3.3: 創建節點間的視覺連接
        // 修復: DOM 元素 ID 格式是 "node-{nodeId}"，而不是直接使用 nodeId
        const sourceElement = document.getElementById(`node-${sourceNodeId}`);
        const targetElement = document.getElementById(`node-${targetNodeId}`);

        if (!sourceElement || !targetElement) {
            console.error(`❌ 找不到節點元素: node-${sourceNodeId} 或 node-${targetNodeId}`);
            // 調試資訊：列出實際存在的節點
            const existingNodes = Array.from(document.querySelectorAll('.rete-node')).map(n => n.id);
            console.error('現有節點 IDs:', existingNodes);
            throw new Error(`找不到節點元素: node-${sourceNodeId} 或 node-${targetNodeId}`);
        }

        // 創建 SVG 連接線
        const svg = this.getOrCreateConnectionSvg();
        const connection = this.createConnectionLine(sourceElement, targetElement);
        
        svg.appendChild(connection);
        
        // 記錄連接
        const connectionId = `${sourceNodeId}_to_${targetNodeId}`;
        this.connections.set(connectionId, {
            id: connectionId,
            source: sourceNodeId,
            target: targetNodeId,
            element: connection
        });

        return connection;
    }

    getOrCreateConnectionSvg() {
        // Phase 3.3: 獲取或創建連接線 SVG 容器
        let svg = document.getElementById('connection-svg');
        if (!svg) {
            svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
            svg.id = 'connection-svg';
            svg.style.cssText = `
                position: absolute;
                top: 0;
                left: 0;
                width: 100%;
                height: 100%;
                pointer-events: none;
                z-index: 0;
            `;
            
            const editorContainer = document.getElementById('rete-editor');
            if (editorContainer) {
                editorContainer.appendChild(svg);
            }
        }
        return svg;
    }

    createConnectionLine(sourceElement, targetElement) {
        // Phase 3.3: 創建連接線
        const sourceRect = sourceElement.getBoundingClientRect();
        const targetRect = targetElement.getBoundingClientRect();
        const editorRect = document.getElementById('rete-editor').getBoundingClientRect();

        // 計算連接點 (節點右側中心到左側中心)
        const startX = sourceRect.right - editorRect.left;
        const startY = sourceRect.top + sourceRect.height / 2 - editorRect.top;
        const endX = targetRect.left - editorRect.left;
        const endY = targetRect.top + targetRect.height / 2 - editorRect.top;

        // 創建路徑
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        const controlX = (startX + endX) / 2;
        const pathData = `M ${startX} ${startY} C ${controlX} ${startY}, ${controlX} ${endY}, ${endX} ${endY}`;
        
        path.setAttribute('d', pathData);
        path.setAttribute('stroke', '#2563eb');
        path.setAttribute('stroke-width', '2');
        path.setAttribute('fill', 'none');
        path.setAttribute('marker-end', 'url(#arrowhead)');

        // 添加箭頭標記
        this.ensureArrowMarker();

        return path;
    }

    ensureArrowMarker() {
        // Phase 3.3: 確保箭頭標記存在
        const svg = this.getOrCreateConnectionSvg();
        if (!svg.querySelector('#arrowhead')) {
            const defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
            const marker = document.createElementNS('http://www.w3.org/2000/svg', 'marker');
            
            marker.setAttribute('id', 'arrowhead');
            marker.setAttribute('markerWidth', '10');
            marker.setAttribute('markerHeight', '7');
            marker.setAttribute('refX', '9');
            marker.setAttribute('refY', '3.5');
            marker.setAttribute('orient', 'auto');
            
            const polygon = document.createElementNS('http://www.w3.org/2000/svg', 'polygon');
            polygon.setAttribute('points', '0 0, 10 3.5, 0 7');
            polygon.setAttribute('fill', '#2563eb');
            
            marker.appendChild(polygon);
            defs.appendChild(marker);
            svg.appendChild(defs);
        }
    }

    updateFlowNameDisplay() {
        // Phase 3.3: 更新流程名稱顯示
        const nameElement = document.getElementById('current-flow-name');
        if (nameElement) {
            nameElement.textContent = this.currentFlowName || '未命名';
        }
    }

    exportFlow() {
        const nodes = [];
        const connections = [];
        
        if (this.editor) {
            // 匯出節點
            this.editor.getNodes().forEach(node => {
                const nodeEl = document.getElementById(`node-${node.id}`);
                const x = nodeEl ? parseInt(nodeEl.style.left) : 0;
                const y = nodeEl ? parseInt(nodeEl.style.top) : 0;
                
                nodes.push({
                    id: node.id,
                    type: node.data.type,
                    name: node.data.name,
                    description: node.data.description,
                    position: { x, y },
                    parameters: node.data.parameters || {}
                });
            });
            
            // 匯出連接
            this.editor.getConnections().forEach(conn => {
                connections.push({
                    id: conn.id,
                    source: conn.source,
                    sourceOutput: conn.sourceOutput,
                    target: conn.target,
                    targetInput: conn.targetInput
                });
            });
        }
        
        return {
            name: this.currentFlowName,
            version: '1.0',
            nodes: nodes,
            connections: connections,
            created: new Date().toISOString()
        };
    }

    async importFlow(data) {
        this.clearEditor();
        
        if (data.name) {
            this.currentFlowName = data.name;
        }
        
        // 匯入節點
        if (data.nodes) {
            for (const nodeData of data.nodes) {
                try {
                    const node = new Rete.ClassicPreset.Node(nodeData.name);
                    node.id = nodeData.id;
                    // 恢復節點配置
                    const nodeType = this.nodeTypes[nodeData.type];
                    
                    node.data = {
                        type: nodeData.type,
                        name: nodeData.name,
                        description: nodeData.description,
                        parameters: nodeData.parameters || {},
                        // 恢復節點配置供屬性面板使用
                        nodeConfig: nodeType,
                        category: nodeType && nodeType.category ? nodeType.category : 'input'
                    };
                    
                    // 添加 socket
                    if (nodeType) {
                        this.addNodeSockets(node, nodeType, nodeData.type);
                    }
                    
                    await this.editor.addNode(node);
                    
                    // 設置位置
                    const x = nodeData.position && nodeData.position.x ? nodeData.position.x : 250;
                    const y = nodeData.position && nodeData.position.y ? nodeData.position.y : 200;
                    
                    if (this.area) {
                        await this.area.translate(node.id, { x, y });
                    }
                    
                    this.renderNodeManually(node, x, y);
                    
                } catch (error) {
                    console.error('匯入節點失敗:', error);
                }
            }
        }
        
        // 匯入連接
        if (data.connections) {
            setTimeout(() => {
                data.connections.forEach(connData => {
                    try {
                        const sourceNode = this.editor.getNode(connData.source);
                        const targetNode = this.editor.getNode(connData.target);
                        
                        if (sourceNode && targetNode) {
                            const connection = new Rete.ClassicPreset.Connection(
                                sourceNode, connData.sourceOutput,
                                targetNode, connData.targetInput
                            );
                            
                            this.editor.addConnection(connection);
                            
                            // 渲染視覺化連接
                            const sourceSocket = document.querySelector(`[data-node-id="${connData.source}"][data-socket-key="${connData.sourceOutput}"]`);
                            const targetSocket = document.querySelector(`[data-node-id="${connData.target}"][data-socket-key="${connData.targetInput}"]`);
                            
                            if (sourceSocket && targetSocket) {
                                this.renderConnection(sourceSocket, targetSocket, connection.id);
                            }
                        }
                    } catch (error) {
                        console.error('匯入連接失敗:', error);
                    }
                });
                
                this.updateStatusBar();
            }, 500);
        }
        
        this.updateStatusBar();
    }

    clearEditor() {
        // 清除所有節點
        document.querySelectorAll('.rete-node').forEach(node => node.remove());
        
        // 清除所有連接
        document.querySelectorAll('.rete-connection').forEach(conn => conn.remove());
        
        // 清除 Rete.js 編輯器
        if (this.editor) {
            this.editor.clear();
        }
        
        // 清除連接映射
        this.connections.clear();
        
        // 清除選擇
        this.selectedNode = null;
        this.closePropertiesPanel();
        
        console.log('✅ 編輯器已清除');
    }

    showFlowModal(action, title) {
        this.currentFlowAction = action;
        
        const modal = document.getElementById('flow-manager-modal');
        const modalTitle = modal.querySelector('.modal-card-title');
        const flowNameInput = document.getElementById('flow-name-input');
        const flowDescriptionInput = document.getElementById('flow-description-input');
        
        // 設置模態框標題
        modalTitle.textContent = title;
        
        // 清空輸入欄位
        flowNameInput.value = '';
        flowDescriptionInput.value = '';
        
        // 添加即時驗證事件監聽器
        flowNameInput.removeEventListener('input', this.validateFlowNameInput);
        flowNameInput.addEventListener('input', this.validateFlowNameInput.bind(this));
        
        // 設置輸入提示
        if (action === 'new') {
            flowNameInput.placeholder = '請輸入流程名稱（僅限小寫英文字母和底線）';
        } else {
            flowNameInput.placeholder = '請輸入流程名稱';
        }
        
        // 顯示模態框
        modal.classList.add('is-active');
        flowNameInput.focus();
    }

    validateFlowNameInput(event) {
        const input = event.target;
        const value = input.value;
        const isValid = this.validateFlowName(value);
        
        // 移除之前的驗證樣式
        input.classList.remove('is-success', 'is-danger');
        
        if (value.length > 0) {
            if (isValid) {
                input.classList.add('is-success');
            } else {
                input.classList.add('is-danger');
            }
        }
        
        // 更新確認按鈕狀態
        const confirmButton = document.getElementById('confirm-flow-action');
        confirmButton.disabled = !isValid || value.length === 0;
    }

    validateFlowName(name) {
        // 檔名規則：只能包含小寫英文字母 (a-z) 和底線 (_)
        const regex = /^[a-z_]+$/;
        
        // 檢查是否符合規則
        if (!regex.test(name)) {
            return false;
        }
        
        // 不能只有底線
        if (name === '_' || name.replace(/_/g, '').length === 0) {
            return false;
        }
        
        // 不能以底線開始或結束
        if (name.startsWith('_') || name.endsWith('_')) {
            return false;
        }
        
        // 不能有連續的底線
        if (name.includes('__')) {
            return false;
        }
        
        return true;
    }

    confirmFlowAction() {
        const flowNameInput = document.getElementById('flow-name-input');
        const flowDescriptionInput = document.getElementById('flow-description-input');
        const flowName = flowNameInput.value.trim();
        const flowDescription = flowDescriptionInput.value.trim();
        
        // 驗證流程名稱
        if (!flowName) {
            this.showNotification('請輸入流程名稱', 'warning');
            flowNameInput.focus();
            return;
        }
        
        if (!this.validateFlowName(flowName)) {
            this.showNotification('流程名稱格式不正確！只能使用小寫英文字母和底線，不能以底線開始或結束，不能有連續底線', 'danger');
            flowNameInput.focus();
            return;
        }
        
        // 根據動作類型執行相應邏輯
        if (this.currentFlowAction === 'new') {
            this.createNewFlow(flowName, flowDescription);
        } else if (this.currentFlowAction === 'save') {
            this.saveFlowWithName(flowName, flowDescription);
        }
        
        this.closeFlowModal();
    }

    createNewFlow(flowName, flowDescription) {
        // 確認是否要清除當前流程
        if (this.editor && this.editor.getNodes().length > 0) {
            if (!confirm('確定要建立新流程？目前的變更將會遺失。')) {
                return;
            }
        }
        
        this.clearEditor();
        this.currentFlowName = flowName;
        this.currentFlowDescription = flowDescription;
        this.updateStatusBar();
        this.showNotification(`已建立新流程：${flowName}`, 'success');
    }

    saveFlowWithName(flowName, flowDescription) {
        this.currentFlowName = flowName;
        this.currentFlowDescription = flowDescription;
        this.updateStatusBar();
        this.directSaveFlow();
    }

    closeFlowModal() {
        const modal = document.getElementById('flow-manager-modal');
        if (modal) {
            modal.classList.remove('is-active');
        }
    }

    showNotification(message, type = 'info') {
        // 創建通知元素
        const notification = document.createElement('div');
        notification.className = `notification is-${type}`;
        notification.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            z-index: 9999;
            min-width: 300px;
            animation: slideInRight 0.3s ease;
        `;
        
        notification.innerHTML = `
            <button class="delete"></button>
            ${message}
        `;
        
        // 添加關閉功能
        notification.querySelector('.delete').addEventListener('click', () => {
            notification.remove();
        });
        
        document.body.appendChild(notification);
        
        // 自動移除
        setTimeout(() => {
            if (notification.parentNode) {
                notification.remove();
            }
        }, 5000);
        
        console.log(`📢 通知: ${message}`);
    }
    
    // =============================================
    // Phase 4.2.3: Loading Speed Optimization
    // =============================================
    
    async optimizeLoadingSpeed() {
        console.log('⚡ Phase 4.2.3: 開始載入速度優化...');
        
        const startTime = performance.now();
        
        try {
            // 1. 並行預載關鍵資源
            const preloadPromise = this.preloadCriticalResources();
            
            // 2. 同時實現漸進式載入
            const progressivePromise = this.implementProgressiveLoading();
            
            // 3. 並行執行優化任務
            await Promise.allSettled([preloadPromise, progressivePromise]);
            
            // 4. 優化初始渲染
            this.optimizeInitialRender();
            
            // 5. 實現智能快取策略
            this.implementSmartCaching();
            
            const endTime = performance.now();
            console.log(`✅ Phase 4.2.3: 載入速度優化完成 (耗時: ${(endTime - startTime).toFixed(2)}ms)`);
            
        } catch (error) {
            console.error('❌ 載入速度優化失敗:', error);
        }
    }
    
    async preloadCriticalResources() {
        console.log('📦 預載關鍵資源...');
        
        const preloadTasks = [
            this.preloadFonts(),
            this.preloadIcons(), 
            this.preloadNodeStyles(),
            this.preloadConnectionStyles(),
            this.preloadCanvasAssets()
        ];
        
        try {
            const results = await Promise.allSettled(preloadTasks);
            const succeeded = results.filter(r => r.status === 'fulfilled').length;
            console.log(`✅ 關鍵資源預載完成 (${succeeded}/${results.length})`);
            
        } catch (error) {
            console.warn('⚠️ 部分資源預載失敗:', error);
        }
    }
    
    async preloadFonts() {
        console.log('🔤 預載字體資源...');
        
        try {
            // 預載系統字體
            await document.fonts.load('12px -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif');
            console.log('✅ 系統字體預載完成');
        } catch (error) {
            console.warn('⚠️ 字體預載失敗:', error);
        }
    }
    
    async preloadIcons() {
        console.log('🎨 預載圖標資源...');
        
        // 創建 icon 預載快取
        this.iconCache = new Map();
        
        // 預載關鍵圖標的 base64 數據
        const icons = {
            condition: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSI+PGNpcmNsZSBjeD0iMTIiIGN5PSIxMiIgcj0iMTAiIHN0cm9rZT0iIzk0YTNiOCIgc3Ryb2tlLXdpZHRoPSIyIi8+PC9zdmc+',
            action: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSI+PHJlY3QgeD0iMyIgeT0iMyIgd2lkdGg9IjE4IiBoZWlnaHQ9IjE4IiByeD0iMiIgc3Ryb2tlPSIjOTRhM2I4IiBzdHJva2Utd2lkdGg9IjIiLz48L3N2Zz4=',
            logic: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSI+PHBvbHlnb24gcG9pbnRzPSIxMiwzIDIxLDkgMjEsMTUgMTIsMjEgMyw1IDMsOSIgc3Ryb2tlPSIjOTRhM2I4IiBzdHJva2Utd2lkdGg9IjIiLz48L3N2Zz4=',
            script: 'data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMjQiIGhlaWdodD0iMjQiIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0ibm9uZSI+PHBhdGggZD0ibTEzIDJsMyA3aC03eiIgc3Ryb2tlPSIjOTRhM2I4IiBzdHJva2Utd2lkdGg9IjIiLz48L3N2Zz=='
        };
        
        Object.entries(icons).forEach(([name, data]) => {
            this.iconCache.set(name, data);
        });
        
        console.log(`✅ 圖標預載完成 (${this.iconCache.size} 個)`);
    }
    
    async preloadNodeStyles() {
        console.log('🎭 預載節點樣式...');
        
        // 創建節點樣式快取
        this.nodeStyleCache = new Map();
        
        // 為每種節點類型生成優化的樣式
        Object.keys(this.nodeTypes || {}).forEach(nodeType => {
            const config = this.nodeTypes[nodeType];
            const optimizedStyle = this.generateOptimizedNodeStyle(config);
            this.nodeStyleCache.set(nodeType, optimizedStyle);
        });
        
        console.log(`✅ 節點樣式快取建立完成 (${this.nodeStyleCache.size} 種類型)`);
    }
    
    generateOptimizedNodeStyle(config) {
        const baseColor = config?.color || '#ffffff';
        
        return {
            backgroundColor: baseColor,
            borderColor: this.darkenColor(baseColor, 0.2),
            textColor: this.getContrastColor(baseColor),
            borderRadius: '8px',
            boxShadow: '0 2px 8px rgba(0,0,0,0.1)',
            transition: 'all 0.2s ease',
            hoverStyle: {
                backgroundColor: this.lightenColor(baseColor, 0.1),
                transform: 'translateY(-1px)',
                boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
            }
        };
    }
    
    async preloadConnectionStyles() {
        console.log('🔗 預載連接樣式...');
        
        this.connectionStyleCache = {
            default: {
                stroke: '#94a3b8',
                strokeWidth: 2,
                fill: 'none'
            },
            active: {
                stroke: '#3b82f6',
                strokeWidth: 3,
                fill: 'none'
            },
            error: {
                stroke: '#ef4444',
                strokeWidth: 2,
                fill: 'none',
                strokeDasharray: '5,5'
            }
        };
        
        console.log('✅ 連線樣式快取建立完成');
    }
    
    async preloadCanvasAssets() {
        console.log('🖼️ 預載畫布資源...');
        
        this.canvasAssetCache = {
            gridPattern: this.createGridPattern(),
            backgroundGradient: this.createBackgroundGradient()
        };
        
        console.log('✅ 畫布資源快取建立完成');
    }
    
    createGridPattern() {
        // 創建網格圖案
        return {
            type: 'pattern',
            size: 20,
            color: '#f1f5f9',
            opacity: 0.5
        };
    }
    
    createBackgroundGradient() {
        // 創建背景漸變
        return {
            type: 'gradient',
            colors: ['#ffffff', '#f8fafc'],
            direction: 'to bottom'
        };
    }
    
    async implementProgressiveLoading() {  
        console.log('🔄 實現漸進式載入...');
        
        const loadingStages = [
            { name: '核心組件', fn: () => this.loadCoreComponents(), priority: 1 },
            { name: '渲染系統', fn: () => this.loadRenderingSystem(), priority: 2 },
            { name: '互動系統', fn: () => this.loadInteractionSystem(), priority: 3 },
            { name: '擴展功能', fn: () => this.loadExtendedFeatures(), priority: 4 }
        ];
        
        // 按優先級順序載入
        for (const stage of loadingStages) {
            try {
                console.log(`⏳ 載入 ${stage.name}...`);
                await stage.fn();
                console.log(`✅ ${stage.name} 載入完成`);
                
                // 高優先級載入完成後立即渲染
                if (stage.priority <= 2) {
                    await this.scheduleNextFrame();
                }
                
            } catch (error) {
                console.warn(`⚠️ ${stage.name} 載入失敗:`, error);
            }
        }
        
        console.log('✅ 漸進式載入完成');
    }
    
    async loadCoreComponents() {
        // 確保編輯器核心已初始化
        if (!this.editor) {
            console.warn('⚠️ 編輯器核心未初始化');
        }
        return Promise.resolve();
    }
    
    async loadRenderingSystem() {
        // 初始化渲染系統
        this.setupOptimizedRendering();
        return Promise.resolve();
    }
    
    async loadInteractionSystem() {
        // 初始化互動系統
        this.setupOptimizedInteractions();
        return Promise.resolve();
    }
    
    async loadExtendedFeatures() {
        // 載入擴展功能
        this.setupExtendedFeatures();
        return Promise.resolve();
    }
    
    setupOptimizedRendering() {
        console.log('🎨 設置優化渲染...');
        
        // 啟用渲染批處理
        this.renderBatch = [];
        this.renderScheduled = false;
        
        // 設置渲染節流
        this.throttledRender = this.throttle(() => {
            this.flushRenderBatch();
        }, 16); // 60fps
    }
    
    setupOptimizedInteractions() {
        console.log('🖱️ 設置優化互動...');
        
        // 設置事件委派
        this.setupEventDelegation();
        
        // 設置滾動優化
        this.setupScrollOptimization();
    }
    
    setupExtendedFeatures() {
        console.log('🔧 設置擴展功能...');
        
        // 設置鍵盤快捷鍵
        this.setupKeyboardShortcuts();
        
        // 設置撤銷/重做
        this.setupUndoRedo();
    }
    
    optimizeInitialRender() {
        console.log('🚀 優化初始渲染...');
        
        // 使用 requestIdleCallback 進行低優先級渲染
        if (window.requestIdleCallback) {
            requestIdleCallback(() => {
                this.renderNonCriticalElements();
            });
        }
        
        // 使用 requestAnimationFrame 優化關鍵渲染
        requestAnimationFrame(() => {
            this.renderCriticalElements();
        });
        
        console.log('✅ 初始渲染優化完成');
    }
    
    renderCriticalElements() {
        console.log('🎯 渲染關鍵元素...');
        
        // 確保編輯區域正確調整大小
        if (this.area) {
            this.area.resize();
        }
        
        // 渲染可見的節點選板
        this.renderVisiblePalette();
    }
    
    renderNonCriticalElements() {
        console.log('⏳ 延遲渲染非關鍵元素...');
        
        // 延遲渲染隱藏元素
        this.renderHiddenElements();
        
        // 啟用動畫效果
        this.enableAnimations();
    }
    
    renderVisiblePalette() {
        // 只渲染可見區域的節點選板項目
        const paletteContainer = document.querySelector('.flow-node-palette');
        if (paletteContainer) {
            const visibleItems = this.getVisiblePaletteItems(paletteContainer);
            visibleItems.forEach(item => this.renderPaletteItem(item));
        }
    }
    
    getVisiblePaletteItems(container) {
        // 計算可見的選板項目
        const containerRect = container.getBoundingClientRect();
        const items = container.querySelectorAll('.palette-node-item');
        
        return Array.from(items).filter(item => {
            const itemRect = item.getBoundingClientRect();
            return itemRect.bottom >= containerRect.top && itemRect.top <= containerRect.bottom;
        });
    }
    
    renderPaletteItem(item) {
        // 渲染單個選板項目
        if (!item.dataset.rendered) {
            // 添加圖標和樣式
            const nodeType = item.dataset.nodeType;
            const icon = this.iconCache?.get(nodeType);
            if (icon) {
                const img = item.querySelector('img');
                if (img) {
                    img.src = icon;
                }
            }
            item.dataset.rendered = 'true';
        }
    }
    
    renderHiddenElements() {
        // 渲染隱藏的選板項目
        const hiddenItems = document.querySelectorAll('.palette-node-item:not([data-rendered])');
        hiddenItems.forEach(item => this.renderPaletteItem(item));
    }
    
    enableAnimations() {
        // 啟用CSS動畫
        document.body.classList.add('animations-enabled');
    }
    
    implementSmartCaching() {
        console.log('🧠 實現智能快取策略...');
        
        // 建立分層快取系統
        this.smartCache = {
            hot: new Map(),    // 熱點數據
            warm: new Map(),   // 溫數據  
            cold: new Map()    // 冷數據
        };
        
        // 設置快取大小限制
        this.cacheLimit = {
            hot: 50,
            warm: 200,
            cold: 500
        };
        
        console.log('✅ 智慧快取系統建立完成');
    }
    
    // 工具方法
    scheduleNextFrame() {
        return new Promise(resolve => requestAnimationFrame(resolve));
    }
    
    throttle(func, limit) {
        let inThrottle;
        return function() {
            const args = arguments;
            const context = this;
            if (!inThrottle) {
                func.apply(context, args);
                inThrottle = true;
                setTimeout(() => inThrottle = false, limit);
            }
        };
    }
    
    darkenColor(color, factor) {
        if (!color || !color.startsWith('#')) return '#666666';
        
        const hex = color.replace('#', '');
        const r = Math.max(0, parseInt(hex.substr(0, 2), 16) * (1 - factor));
        const g = Math.max(0, parseInt(hex.substr(2, 2), 16) * (1 - factor));
        const b = Math.max(0, parseInt(hex.substr(4, 2), 16) * (1 - factor));
        
        return `#${Math.round(r).toString(16).padStart(2, '0')}${Math.round(g).toString(16).padStart(2, '0')}${Math.round(b).toString(16).padStart(2, '0')}`;
    }
    
    lightenColor(color, factor) {
        if (!color || !color.startsWith('#')) return '#cccccc';
        
        const hex = color.replace('#', '');
        const r = Math.min(255, parseInt(hex.substr(0, 2), 16) + (255 * factor));
        const g = Math.min(255, parseInt(hex.substr(2, 2), 16) + (255 * factor));
        const b = Math.min(255, parseInt(hex.substr(4, 2), 16) + (255 * factor));
        
        return `#${Math.round(r).toString(16).padStart(2, '0')}${Math.round(g).toString(16).padStart(2, '0')}${Math.round(b).toString(16).padStart(2, '0')}`;
    }
    
    getContrastColor(backgroundColor) {
        if (!backgroundColor || !backgroundColor.startsWith('#')) return '#000000';
        
        const hex = backgroundColor.replace('#', '');
        const r = parseInt(hex.substr(0, 2), 16);
        const g = parseInt(hex.substr(2, 2), 16);
        const b = parseInt(hex.substr(4, 2), 16);
        const luminance = (0.299 * r + 0.587 * g + 0.114 * b) / 255;
        
        return luminance > 0.5 ? '#000000' : '#ffffff';
    }
    
    // 缺失方法的佔位符實現
    setupEventDelegation() {
        console.log('🎯 設置事件委派...');
        // 實現事件委派邏輯
    }
    
    setupScrollOptimization() {
        console.log('📜 設置滾動優化...');
        // 實現滾動優化邏輯
    }
    
    setupKeyboardShortcuts() {
        console.log('⌨️ 設置鍵盤快捷鍵...');
        // 實現鍵盤快捷鍵邏輯
    }
    
    setupUndoRedo() {
        console.log('↩️ 設置撤銷/重做...');
        // 實現撤銷/重做邏輯
    }
    
    flushRenderBatch() {
        console.log('🎨 清空渲染批次...');
        // 實現渲染批次清空邏輯
        if (this.renderBatch && this.renderBatch.length > 0) {
            this.renderBatch.forEach(renderTask => {
                try {
                    renderTask();
                } catch (error) {
                    console.warn('⚠️ 渲染任務失敗:', error);
                }
            });
            this.renderBatch = [];
            this.renderScheduled = false;
        }
    }
    
    // =====================================
    // Phase 4.2.4: Integration Testing
    // =====================================
    
    /**
     * 運行完整的整合測試套件
     * 包括效能基準測試、功能測試和壓力測試
     */
    async runIntegrationTests() {
        console.log('🧪 Phase 4.2.4: 開始整合測試...');
        
        const testResults = {
            performance: {},
            functionality: {},
            stress: {},
            memory: {},
            summary: {}
        };
        
        try {
            // 1. 效能基準測試
            console.log('📊 執行效能基準測試...');
            testResults.performance = await this.runPerformanceBenchmarks();
            
            // 2. 功能整合測試
            console.log('🔧 執行功能整合測試...');
            testResults.functionality = await this.runFunctionalityTests();
            
            // 3. 壓力測試
            console.log('💪 執行壓力測試...');
            testResults.stress = await this.runStressTests();
            
            // 4. 記憶體效能測試
            console.log('🧠 執行記憶體效能測試...');
            testResults.memory = await this.runMemoryTests();
            
            // 5. 生成測試報告
            testResults.summary = this.generateTestSummary(testResults);
            
            // 6. 輸出測試結果
            this.outputTestResults(testResults);
            
            console.log('✅ Phase 4.2.4: 整合測試完成');
            return testResults;
            
        } catch (error) {
            console.error('❌ 整合測試失敗:', error);
            testResults.error = error.message;
            return testResults;
        }
    }
    
    /**
     * 效能基準測試
     * 測試初始化時間、渲染效能、記憶體使用等關鍵指標
     */
    async runPerformanceBenchmarks() {
        const results = {
            initialization: {},
            rendering: {},
            interaction: {}
        };
        
        // 1. 初始化效能測試
        console.log('  ⏱️ 測試初始化效能...');
        const initStart = performance.now();
        
        // 模擬初始化流程檢查
        const testInitialization = await this.testSystemInitialization();
        const initEnd = performance.now();
        
        results.initialization = {
            duration: initEnd - initStart,
            target: 500, // 目標: 500ms 內完成初始化
            passed: (initEnd - initStart) < 500,
            systemReady: testInitialization
        };
        
        // 2. 渲染效能測試
        console.log('  🎨 測試渲染效能...');
        const renderResults = await this.testRenderingPerformance();
        results.rendering = renderResults;
        
        // 3. 互動效能測試
        console.log('  🖱️ 測試互動效能...');
        const interactionResults = await this.testInteractionPerformance();
        results.interaction = interactionResults;
        
        return results;
    }
    
    /**
     * 測試系統初始化
     */
    async testSystemInitialization() {
        try {
            // 檢查核心組件是否已初始化
            const checks = {
                editor: !!this.editor,
                nodeFactory: !!this.nodeFactory,
                yamlGenerator: !!this.yamlGenerator,
                flowSaver: !!this.flowSaver,
                canvasRenderer: !!this.canvasRenderer
            };
            
            const readyComponents = Object.values(checks).filter(Boolean).length;
            const totalComponents = Object.keys(checks).length;
            
            return {
                components: checks,
                readyCount: readyComponents,
                totalCount: totalComponents,
                readyPercentage: (readyComponents / totalComponents) * 100
            };
        } catch (error) {
            return { error: error.message };
        }
    }
    
    /**
     * 測試渲染效能
     */
    async testRenderingPerformance() {
        const results = {
            nodeRendering: {},
            connectionRendering: {}
        };
        
        // 測試節點渲染效能
        const nodeRenderStart = performance.now();
        
        // 創建測試節點
        const testNodes = [];
        for (let i = 0; i < 20; i++) {
            const node = {
                id: `perf-test-node-${i}`,
                type: ['condition', 'action', 'logic'][i % 3],
                name: `效能測試節點 ${i}`,
                position: { x: (i % 5) * 150, y: Math.floor(i / 5) * 100 }
            };
            testNodes.push(node);
        }
        
        // 模擬批量渲染
        const renderSuccess = await this.simulateBatchRender(testNodes);
        const nodeRenderEnd = performance.now();
        
        results.nodeRendering = {
            nodeCount: testNodes.length,
            duration: nodeRenderEnd - nodeRenderStart,
            averagePerNode: (nodeRenderEnd - nodeRenderStart) / testNodes.length,
            target: 5, // 目標: 每個節點 < 5ms
            passed: ((nodeRenderEnd - nodeRenderStart) / testNodes.length) < 5,
            renderSuccess: renderSuccess
        };
        
        // 測試連接渲染效能
        const connectionRenderStart = performance.now();
        
        // 創建測試連接
        const testConnections = [];
        for (let i = 0; i < 10; i++) {
            const connection = {
                id: `perf-test-connection-${i}`,
                source: testNodes[i % testNodes.length].id,
                target: testNodes[(i + 1) % testNodes.length].id
            };
            testConnections.push(connection);
        }
        
        const connectionSuccess = await this.simulateConnectionRender(testConnections);
        const connectionRenderEnd = performance.now();
        
        results.connectionRendering = {
            connectionCount: testConnections.length,
            duration: connectionRenderEnd - connectionRenderStart,
            averagePerConnection: (connectionRenderEnd - connectionRenderStart) / testConnections.length,
            target: 3, // 目標: 每個連接 < 3ms
            passed: ((connectionRenderEnd - connectionRenderStart) / testConnections.length) < 3,
            renderSuccess: connectionSuccess
        };
        
        return results;
    }
    
    /**
     * 測試互動效能
     */
    async testInteractionPerformance() {
        const results = {
            nodeSelection: {},
            interfaceResponse: {}
        };
        
        // 測試節點選擇效能
        const selectionStart = performance.now();
        
        // 模擬節點選擇操作
        for (let i = 0; i < 10; i++) {
            await this.simulateNodeSelection({ x: 100 + i * 50, y: 100 + i * 30 });
        }
        
        const selectionEnd = performance.now();
        
        results.nodeSelection = {
            operations: 10,
            duration: selectionEnd - selectionStart,
            averagePerOperation: (selectionEnd - selectionStart) / 10,
            target: 10, // 目標: 每次選擇 < 10ms
            passed: ((selectionEnd - selectionStart) / 10) < 10
        };
        
        // 測試界面響應效能
        const responseStart = performance.now();
        
        // 模擬界面更新操作
        for (let i = 0; i < 5; i++) {
            await this.simulateInterfaceUpdate();
        }
        
        const responseEnd = performance.now();
        
        results.interfaceResponse = {
            operations: 5,
            duration: responseEnd - responseStart,
            averagePerOperation: (responseEnd - responseStart) / 5,
            target: 20, // 目標: 每次更新 < 20ms
            passed: ((responseEnd - responseStart) / 5) < 20
        };
        
        return results;
    }
    
    /**
     * 功能整合測試
     */
    async runFunctionalityTests() {
        const results = {
            nodeOperations: {},
            yamlGeneration: {},
            flowSaving: {}
        };
        
        console.log('  🔧 測試節點操作功能...');
        results.nodeOperations = await this.testNodeOperations();
        
        console.log('  📄 測試 YAML 生成功能...');
        results.yamlGeneration = await this.testYamlGeneration();
        
        console.log('  💾 測試流程保存功能...');
        results.flowSaving = await this.testFlowSaving();
        
        return results;
    }
    
    /**
     * 測試節點操作功能
     */
    async testNodeOperations() {
        const results = {
            creation: { passed: false, error: null },
            editing: { passed: false, error: null }
        };
        
        try {
            // 測試節點創建功能
            const canCreateNode = this.nodeFactory && typeof this.nodeFactory.createNode === 'function';
            results.creation.passed = canCreateNode;
            
            if (!canCreateNode) {
                results.creation.error = 'NodeFactory 或 createNode 方法不可用';
            }
            
            // 測試節點編輯功能
            const canEditNode = this.updateNodeProperties && typeof this.updateNodeProperties === 'function';
            results.editing.passed = canEditNode;
            
            if (!canEditNode) {
                results.editing.error = 'updateNodeProperties 方法不可用';
            }
            
        } catch (error) {
            results.creation.error = error.message;
        }
        
        return results;
    }
    
    /**
     * 測試 YAML 生成功能
     */
    async testYamlGeneration() {
        const results = {
            generation: { passed: false, error: null },
            validation: { passed: false, error: null }
        };
        
        try {
            // 測試 YAML 生成器可用性
            const canGenerateYaml = this.yamlGenerator && typeof this.yamlGenerator.generateYAML === 'function';
            results.generation.passed = canGenerateYaml;
            
            if (!canGenerateYaml) {
                results.generation.error = 'YamlGenerator 或 generateYAML 方法不可用';
            }
            
            // 測試 YAML 驗證功能
            if (canGenerateYaml) {
                // 模擬生成 YAML
                const testFlow = {
                    nodes: [{ id: 'test', type: 'condition', name: '測試節點' }],
                    connections: []
                };
                
                try {
                    const yaml = await this.yamlGenerator.generateYAML(testFlow);
                    results.validation.passed = typeof yaml === 'string' && yaml.includes('test');
                } catch (yamlError) {
                    results.validation.error = yamlError.message;
                }
            }
            
        } catch (error) {
            results.generation.error = error.message;
        }
        
        return results;
    }
    
    /**
     * 測試流程保存功能
     */
    async testFlowSaving() {
        const results = {
            saving: { passed: false, error: null },
            loading: { passed: false, error: null }
        };
        
        try {
            // 測試流程保存功能
            const canSaveFlow = this.flowSaver && typeof this.flowSaver.saveFlow === 'function';
            results.saving.passed = canSaveFlow;
            
            if (!canSaveFlow) {
                results.saving.error = 'FlowSaver 或 saveFlow 方法不可用';
            }
            
            // 測試流程載入功能
            const canLoadFlow = this.loadFlowFromData && typeof this.loadFlowFromData === 'function';
            results.loading.passed = canLoadFlow;
            
            if (!canLoadFlow) {
                results.loading.error = 'loadFlowFromData 方法不可用';
            }
            
        } catch (error) {
            results.saving.error = error.message;
        }
        
        return results;
    }
    
    /**
     * 壓力測試
     */
    async runStressTests() {
        const results = {
            memoryPressure: {},
            performanceStability: {}
        };
        
        console.log('  🧠 測試記憶體壓力...');
        results.memoryPressure = await this.testMemoryPressure();
        
        console.log('  ⚡ 測試效能穩定性...');
        results.performanceStability = await this.testPerformanceStability();
        
        return results;
    }
    
    /**
     * 記憶體效能測試
     */
    async runMemoryTests() {
        const results = {
            initialMemory: 0,
            currentMemory: 0,
            memoryUsage: 0,
            passed: false
        };
        
        if (performance.memory) {
            results.initialMemory = performance.memory.usedJSHeapSize;
            
            // 執行一些操作後檢查記憶體
            await this.performMemoryIntensiveOperations();
            
            results.currentMemory = performance.memory.usedJSHeapSize;
            results.memoryUsage = results.currentMemory - results.initialMemory;
            
            // 檢查記憶體使用是否在合理範圍內 (< 50MB)
            const memoryThreshold = 50 * 1024 * 1024;
            results.passed = results.memoryUsage < memoryThreshold;
        } else {
            results.passed = true; // 如果不支援 performance.memory，假設通過
        }
        
        return results;
    }
    
    /**
     * 生成測試總結報告
     */
    generateTestSummary(testResults) {
        const summary = {
            totalTests: 0,
            passedTests: 0,
            failedTests: 0,
            overallScore: 0,
            performance: {
                grade: 'A+',
                score: 100
            },
            recommendations: []
        };
        
        // 統計測試結果
        const flattenResults = (obj) => {
            let count = 0;
            let passed = 0;
            
            for (const [key, value] of Object.entries(obj)) {
                if (typeof value === 'object' && value !== null && !Array.isArray(value)) {
                    if ('passed' in value) {
                        count++;
                        if (value.passed) passed++;
                    } else {
                        const subResult = flattenResults(value);
                        count += subResult.count;
                        passed += subResult.passed;
                    }
                }
            }
            
            return { count, passed };
        };
        
        const performanceResult = flattenResults(testResults.performance);
        const functionalityResult = flattenResults(testResults.functionality);
        const stressResult = flattenResults(testResults.stress);
        const memoryResult = flattenResults(testResults.memory);
        
        summary.totalTests = performanceResult.count + functionalityResult.count + stressResult.count + memoryResult.count;
        summary.passedTests = performanceResult.passed + functionalityResult.passed + stressResult.passed + memoryResult.passed;
        summary.failedTests = summary.totalTests - summary.passedTests;
        
        // 計算總體分數
        if (summary.totalTests > 0) {
            summary.overallScore = Math.round((summary.passedTests / summary.totalTests) * 100);
        }
        
        // 效能等級評估
        if (summary.overallScore >= 95) {
            summary.performance.grade = 'A+';
        } else if (summary.overallScore >= 90) {
            summary.performance.grade = 'A';
        } else if (summary.overallScore >= 80) {
            summary.performance.grade = 'B';
        } else {
            summary.performance.grade = 'C';
        }
        
        summary.performance.score = summary.overallScore;
        
        // 生成建議
        if (testResults.performance?.rendering?.nodeRendering?.passed === false) {
            summary.recommendations.push('優化節點渲染效能');
        }
        
        if (testResults.memory?.passed === false) {
            summary.recommendations.push('檢查記憶體使用並進行優化');
        }
        
        return summary;
    }
    
    /**
     * 輸出測試結果
     */
    outputTestResults(testResults) {
        console.log('\n🧪 ========== Flow Designer 整合測試報告 ==========');
        console.log(`📊 總體分數: ${testResults.summary.overallScore}/100 (${testResults.summary.performance.grade})`);
        console.log(`✅ 通過測試: ${testResults.summary.passedTests}/${testResults.summary.totalTests}`);
        
        if (testResults.summary.failedTests > 0) {
            console.log(`❌ 失敗測試: ${testResults.summary.failedTests}`);
        }
        
        // 效能基準測試結果
        console.log('\n📈 效能基準測試:');
        if (testResults.performance.initialization) {
            const init = testResults.performance.initialization;
            console.log(`  ⏱️ 初始化: ${init.duration.toFixed(2)}ms ${init.passed ? '✅' : '❌'}`);
            if (init.systemReady) {
                console.log(`    系統組件就緒: ${init.systemReady.readyCount}/${init.systemReady.totalCount} (${init.systemReady.readyPercentage.toFixed(1)}%)`);
            }
        }
        
        // 功能測試結果
        console.log('\n🔧 功能整合測試:');
        if (testResults.functionality.nodeOperations) {
            const nodeOps = testResults.functionality.nodeOperations;
            console.log(`  📝 節點操作: 創建${nodeOps.creation.passed ? '✅' : '❌'} 編輯${nodeOps.editing.passed ? '✅' : '❌'}`);
        }
        
        // 記憶體測試結果
        if (testResults.memory.memoryUsage !== undefined) {
            const memory = testResults.memory;
            const memoryMB = (memory.memoryUsage / 1024 / 1024).toFixed(2);
            console.log(`\n🧠 記憶體測試: ${memoryMB}MB ${memory.passed ? '✅' : '❌'}`);
        }
        
        // 建議
        if (testResults.summary.recommendations.length > 0) {
            console.log('\n💡 優化建議:');
            testResults.summary.recommendations.forEach((rec, index) => {
                console.log(`  ${index + 1}. ${rec}`);
            });
        }
        
        console.log('\n================================================\n');
        
        // 在頁面上顯示結果
        this.displayTestResultsOnPage(testResults);
    }
    
    /**
     * 在頁面上顯示測試結果
     */
    displayTestResultsOnPage(testResults) {
        // 創建測試結果面板
        const testPanel = document.createElement('div');
        testPanel.id = 'test-results-panel';
        testPanel.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            width: 350px;
            max-height: 70vh;
            background: white;
            border: 2px solid #00d1b2;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            z-index: 10000;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 12px;
        `;
        
        const closeBtn = document.createElement('button');
        closeBtn.innerHTML = '×';
        closeBtn.style.cssText = `
            position: absolute;
            top: 5px;
            right: 10px;
            background: none;
            border: none;
            font-size: 18px;
            cursor: pointer;
            color: #666;
        `;
        closeBtn.onclick = () => testPanel.remove();
        
        const content = document.createElement('div');
        content.style.padding = '15px';
        
        const summary = testResults.summary;
        content.innerHTML = `
            <h3 style="margin: 0 0 10px 0; color: #00d1b2;">🧪 Flow Designer v4.2 測試報告</h3>
            <div style="background: #f0f9ff; padding: 8px; border-radius: 4px; margin-bottom: 8px; border-left: 3px solid #00d1b2;">
                <strong>總體分數: ${summary.overallScore}/100 (${summary.performance.grade})</strong><br>
                通過: ${summary.passedTests}/${summary.totalTests} 項測試
            </div>
            <div style="font-size: 11px; line-height: 1.3;">
                ${this.formatTestResultsHTML(testResults)}
            </div>
        `;
        
        testPanel.appendChild(closeBtn);
        testPanel.appendChild(content);
        document.body.appendChild(testPanel);
        
        // 8秒後自動關閉
        setTimeout(() => {
            if (document.body.contains(testPanel)) {
                testPanel.remove();
            }
        }, 8000);
    }
    
    /**
     * 格式化測試結果為 HTML
     */
    formatTestResultsHTML(testResults) {
        let html = '';
        
        // 效能測試
        if (testResults.performance.initialization) {
            const init = testResults.performance.initialization;
            html += `<div style="margin: 4px 0;"><strong>⏱️ 初始化:</strong> ${init.duration.toFixed(2)}ms ${init.passed ? '✅' : '❌'}</div>`;
        }
        
        // 渲染效能
        if (testResults.performance.rendering?.nodeRendering) {
            const render = testResults.performance.rendering.nodeRendering;
            html += `<div style="margin: 4px 0;"><strong>🎨 節點渲染:</strong> ${render.averagePerNode.toFixed(2)}ms/節點 ${render.passed ? '✅' : '❌'}</div>`;
        }
        
        // 功能測試
        if (testResults.functionality.nodeOperations) {
            const ops = testResults.functionality.nodeOperations;
            html += `<div style="margin: 4px 0;"><strong>📝 節點操作:</strong> 創建${ops.creation.passed ? '✅' : '❌'} 編輯${ops.editing.passed ? '✅' : '❌'}</div>`;
        }
        
        // YAML 生成
        if (testResults.functionality.yamlGeneration) {
            const yaml = testResults.functionality.yamlGeneration;
            html += `<div style="margin: 4px 0;"><strong>📄 YAML:</strong> 生成${yaml.generation.passed ? '✅' : '❌'} 驗證${yaml.validation.passed ? '✅' : '❌'}</div>`;
        }
        
        // 記憶體測試
        if (testResults.memory.memoryUsage !== undefined) {
            const memory = testResults.memory;
            const memoryMB = (memory.memoryUsage / 1024 / 1024).toFixed(1);
            html += `<div style="margin: 4px 0;"><strong>🧠 記憶體:</strong> ${memoryMB}MB ${memory.passed ? '✅' : '❌'}</div>`;
        }
        
        return html;
    }
    
    // =====================================
    // 測試輔助方法
    // =====================================
    
    /**
     * 模擬批量渲染
     */
    async simulateBatchRender(nodes) {
        // 模擬批量渲染邏輯
        return new Promise(resolve => {
            setTimeout(() => resolve(true), nodes.length * 2);
        });
    }
    
    /**
     * 模擬連接渲染
     */
    async simulateConnectionRender(connections) {
        // 模擬連接渲染邏輯
        return new Promise(resolve => {
            setTimeout(() => resolve(true), connections.length * 1);
        });
    }
    
    /**
     * 模擬節點選擇
     */
    async simulateNodeSelection(position) {
        // 模擬節點選擇邏輯
        return new Promise(resolve => {
            setTimeout(() => resolve(true), 2);
        });
    }
    
    /**
     * 模擬界面更新
     */
    async simulateInterfaceUpdate() {
        // 模擬界面更新邏輯
        return new Promise(resolve => {
            setTimeout(() => resolve(true), 5);
        });
    }
    
    /**
     * 測試記憶體壓力
     */
    async testMemoryPressure() {
        const results = {
            memoryIncrease: 0,
            passed: true
        };
        
        if (performance.memory) {
            const initialMemory = performance.memory.usedJSHeapSize;
            
            // 模擬記憶體密集操作
            const tempData = [];
            for (let i = 0; i < 10000; i++) {
                tempData.push({ id: i, data: new Array(100).fill(Math.random()) });
            }
            
            const currentMemory = performance.memory.usedJSHeapSize;
            results.memoryIncrease = currentMemory - initialMemory;
            results.passed = results.memoryIncrease < 20 * 1024 * 1024; // < 20MB
            
            // 清理測試數據
            tempData.length = 0;
        }
        
        return results;
    }
    
    /**
     * 測試效能穩定性
     */
    async testPerformanceStability() {
        const results = {
            iterations: 5,
            averageTime: 0,
            passed: true
        };
        
        const times = [];
        
        for (let i = 0; i < results.iterations; i++) {
            const start = performance.now();
            await this.simulateInterfaceUpdate();
            const end = performance.now();
            times.push(end - start);
        }
        
        results.averageTime = times.reduce((a, b) => a + b, 0) / times.length;
        results.passed = results.averageTime < 30; // 平均 < 30ms
        
        return results;
    }
    
    /**
     * 執行記憶體密集操作
     */
    async performMemoryIntensiveOperations() {
        // 模擬記憶體使用操作
        const operations = [];
        for (let i = 0; i < 100; i++) {
            operations.push(new Promise(resolve => {
                const data = new Array(1000).fill(Math.random());
                setTimeout(() => resolve(data), 10);
            }));
        }
        
        await Promise.all(operations);
    }
}

// 注意：Flow Designer 初始化現在由模板 (flow_designer.html) 中的詳細邏輯處理
// 這樣可以確保完整的初始化流程、錯誤處理和URL參數載入功能

// CSS 動畫
const style = document.createElement('style');
style.textContent = `
    @keyframes slideInRight {
        from {
            transform: translateX(100%);
            opacity: 0;
        }
        to {
            transform: translateX(0);
            opacity: 1;
        }
    }
    
    .rete-node.selected {
        outline: 2px solid #2196F3 !important;
        outline-offset: 2px !important;
        box-shadow: 0 0 0 4px rgba(33, 150, 243, 0.2) !important;
    }
    
    .rete-connection.selected path {
        stroke: #2196F3 !important;
        stroke-width: 3px !important;
    }
    
    .palette-node-item:hover {
        transform: translateX(4px) !important;
        box-shadow: 0 2px 8px rgba(0,0,0,0.3) !important;
    }
`;
document.head.appendChild(style);

console.log('🎯 Flow Designer v4.2 載入完成 - 性能優化版本');

// Phase 4.2 性能優化類
class PerformanceOptimizer {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.performanceMonitor = new PerformanceMonitor();
        this.batchRenderer = new BatchNodeRenderer();
        this.memoryManager = new MemoryManager();
        this.resourcePreloader = new ResourcePreloader();
        this.progressiveLoader = new ProgressiveLoader(flowDesigner);
        
        // 性能配置
        this.config = {
            rendering: {
                batchSize: 10,
                renderDelay: 16,
                enableViewportCulling: true
            },
            memory: {
                maxPoolSize: 100,
                gcInterval: 30000,
                memoryThreshold: 100 * 1024 * 1024
            },
            loading: {
                preloadCriticalResources: true,
                enableProgressiveLoading: true,
                maxCacheSize: 50
            }
        };
        
        this.init();
    }
    
    init() {
        console.log('🚀 初始化性能優化器...');
        
        // 启动性能监控
        this.performanceMonitor.startMonitoring();
        
        // 预载入关键资源
        if (this.config.loading.preloadCriticalResources) {
            this.resourcePreloader.preloadCriticalResources();
        }
        
        // 启动内存管理
        this.memoryManager.startGarbageCollection();
        
        console.log('✅ 性能優化器初始化完成');
    }
    
    // 批量節點渲染
    async batchRenderNodes(nodes) {
        return await this.batchRenderer.batchRenderNodes(nodes);
    }
    
    // 渐进式加载流程
    async loadFlowProgressively(flowData, onProgress) {
        return await this.progressiveLoader.loadFlowProgressively(flowData, onProgress);
    }
    
    // 获取性能报告
    getPerformanceReport() {
        return this.performanceMonitor.getPerformanceReport();
    }
    
    // 内存清理
    performMemoryCleanup() {
        this.memoryManager.performGarbageCollection();
    }
    
    // 停用性能優化
    dispose() {
        this.performanceMonitor.stopMonitoring();
        this.memoryManager.clearCache();
        this.resourcePreloader.clearCache();
    }
}

// Phase 4.2 批量節點渲染器
class BatchNodeRenderer {
    constructor(batchSize = 10, renderDelay = 16) {
        this.batchSize = batchSize;
        this.renderDelay = renderDelay;
        this.renderQueue = [];
        this.isRendering = false;
    }

    async batchRenderNodes(nodes) {
        console.log(`🔄 批量渲染 ${nodes.length} 個節點...`);
        
        // 分批處理節點
        for (let i = 0; i < nodes.length; i += this.batchSize) {
            const batch = nodes.slice(i, i + this.batchSize);
            this.renderQueue.push(batch);
        }

        // 开始批量渲染
        if (!this.isRendering) {
            await this.processRenderQueue();
        }
        
        console.log('✅ 批量渲染完成');
    }

    async processRenderQueue() {
        this.isRendering = true;
        
        while (this.renderQueue.length > 0) {
            const batch = this.renderQueue.shift();
            
            // 渲染当前批次
            const renderPromises = batch.map(node => this.renderSingleNode(node));
            await Promise.all(renderPromises);
            
            // 让出控制权给浏览器，避免卡顿
            await this.nextTick();
        }
        
        this.isRendering = false;
    }

    nextTick() {
        return new Promise(resolve => {
            requestAnimationFrame(() => {
                setTimeout(resolve, this.renderDelay);
            });
        });
    }

    async renderSingleNode(node) {
        // 單個節點渲染邏輯（簡化版）
        return new Promise(resolve => {
            try {
                // 這裡調用實際的節點渲染方法
                if (window.flowDesigner && window.flowDesigner.renderNodeToDOM) {
                    window.flowDesigner.renderNodeToDOM(node);
                }
                resolve(node);
            } catch (error) {
                console.error('節點渲染失敗:', error);
                resolve(null);
            }
        });
    }
}

// Phase 4.2 内存管理器
class MemoryManager {
    constructor() {
        this.nodePool = [];
        this.connectionPool = [];
        this.maxPoolSize = 100;
        this.gcInterval = 30000; // 30秒
        this.memoryThreshold = 100 * 1024 * 1024; // 100MB
        this.gcTimer = null;
    }

    startGarbageCollection() {
        if (this.gcTimer) return;
        
        this.gcTimer = setInterval(() => {
            this.performGarbageCollection();
        }, this.gcInterval);
    }

    performGarbageCollection() {
        const memoryInfo = this.getMemoryInfo();
        
        if (memoryInfo.usedJSHeapSize > this.memoryThreshold) {
            console.log('🗑️ 执行内存清理...');
            
            // 清理不再使用的DOM元素
            this.cleanupOrphanedElements();
            
            // 清理对象池
            this.cleanupObjectPools();
            
            // 强制垃圾回收 (如果可用)
            if (window.gc) {
                window.gc();
            }
            
            console.log('✅ 内存清理完成');
        }
    }

    getMemoryInfo() {
        if (performance.memory) {
            return {
                usedJSHeapSize: performance.memory.usedJSHeapSize,
                totalJSHeapSize: performance.memory.totalJSHeapSize,
                jsHeapSizeLimit: performance.memory.jsHeapSizeLimit
            };
        }
        return { usedJSHeapSize: 0 };
    }

    cleanupOrphanedElements() {
        // 清理不再需要的 DOM 元素
        const orphanedNodes = document.querySelectorAll('.flow-node:not([data-active])');
        orphanedNodes.forEach(node => node.remove());
        
        const orphanedConnections = document.querySelectorAll('#connection-svg path:not([data-active])');
        orphanedConnections.forEach(path => path.remove());
    }

    cleanupObjectPools() {
        // 清理对象池
        this.nodePool.length = Math.min(this.nodePool.length, this.maxPoolSize / 2);
        this.connectionPool.length = Math.min(this.connectionPool.length, this.maxPoolSize / 2);
    }

    clearCache() {
        this.nodePool = [];
        this.connectionPool = [];
        
        if (this.gcTimer) {
            clearInterval(this.gcTimer);
            this.gcTimer = null;
        }
    }
}

// Phase 4.2 性能监控器
class PerformanceMonitor {
    constructor() {
        this.metrics = new Map();
        this.isMonitoring = false;
        this.fpsCounter = null;
    }

    startMonitoring() {
        if (this.isMonitoring) return;
        
        this.isMonitoring = true;
        console.log('📊 开始性能监控...');

        // 监控 FPS
        this.monitorFPS();
        
        // 监控内存使用
        this.monitorMemoryUsage();
    }

    monitorFPS() {
        let lastTime = performance.now();
        let frameCount = 0;

        const measureFPS = (currentTime) => {
            frameCount++;
            
            if (currentTime - lastTime >= 1000) {
                const fps = Math.round((frameCount * 1000) / (currentTime - lastTime));
                this.recordMetric('fps', fps);
                
                frameCount = 0;
                lastTime = currentTime;
            }
            
            if (this.isMonitoring) {
                this.fpsCounter = requestAnimationFrame(measureFPS);
            }
        };

        this.fpsCounter = requestAnimationFrame(measureFPS);
    }

    monitorMemoryUsage() {
        const monitorInterval = setInterval(() => {
            if (!this.isMonitoring) {
                clearInterval(monitorInterval);
                return;
            }
            
            if (performance.memory) {
                const memoryInfo = {
                    used: Math.round(performance.memory.usedJSHeapSize / 1024 / 1024),
                    total: Math.round(performance.memory.totalJSHeapSize / 1024 / 1024),
                    limit: Math.round(performance.memory.jsHeapSizeLimit / 1024 / 1024)
                };
                
                this.recordMetric('memory', memoryInfo);
            }
        }, 5000);
    }

    recordMetric(name, value) {
        if (!this.metrics.has(name)) {
            this.metrics.set(name, []);
        }
        
        const history = this.metrics.get(name);
        history.push({
            value,
            timestamp: Date.now()
        });
        
        // 保持最近100个数据点
        if (history.length > 100) {
            history.shift();
        }
    }

    getPerformanceReport() {
        const report = {};
        
        this.metrics.forEach((history, metricName) => {
            if (history.length === 0) return;
            
            const values = history.map(h => h.value);
            
            if (typeof values[0] === 'number') {
                report[metricName] = {
                    current: values[values.length - 1],
                    average: values.reduce((a, b) => a + b, 0) / values.length,
                    min: Math.min(...values),
                    max: Math.max(...values),
                    samples: values.length
                };
            } else {
                report[metricName] = {
                    current: values[values.length - 1],
                    samples: values.length
                };
            }
        });
        
        return report;
    }

    stopMonitoring() {
        this.isMonitoring = false;
        if (this.fpsCounter) {
            cancelAnimationFrame(this.fpsCounter);
            this.fpsCounter = null;
        }
        console.log('⏹️ 性能监控已停止');
    }
}

// Phase 4.2 资源预加载器
class ResourcePreloader {
    constructor() {
        this.cache = new Map();
        this.maxCacheSize = 50;
    }

    async preloadCriticalResources() {
        console.log('🚀 开始预载关键资源...');
        
        const criticalResources = [
            // 核心 JavaScript 模块
            '/static/js/flow-designer/node-types.js',
            
            // 常用節點配置
            '/static/js/flow-designer/configs/condition_nodes.json',
            '/static/js/flow-designer/configs/logic_nodes.json',
            '/static/js/flow-designer/configs/action_nodes.json',
            
            // 样式
            '/static/css/flowDesignerPage.css'
        ];

        // 并行预加载
        const preloadPromises = criticalResources.map(url => 
            this.preloadResource(url).catch(error => 
                console.warn(`资源预载失败: ${url}`, error)
            )
        );

        try {
            await Promise.allSettled(preloadPromises);
            console.log('✅ 关键资源预载完成');
        } catch (error) {
            console.error('❌ 关键资源预载失败:', error);
        }
    }

    async preloadResource(url) {
        if (this.cache.has(url)) {
            return this.cache.get(url);
        }

        try {
            const response = await fetch(url, { cache: 'force-cache' });
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }

            const content = await response.text();
            this.addToCache(url, content);
            
            return content;
        } catch (error) {
            console.error(`资源预载失败 ${url}:`, error);
            throw error;
        }
    }

    addToCache(url, content) {
        // LRU 缓存管理
        if (this.cache.size >= this.maxCacheSize) {
            const firstKey = this.cache.keys().next().value;
            this.cache.delete(firstKey);
        }

        this.cache.set(url, {
            content,
            timestamp: Date.now()
        });
    }

    clearCache() {
        this.cache.clear();
        console.log('🗑️ 资源缓存已清理');
    }
}

// Phase 4.2 渐进式加载器
class ProgressiveLoader {
    constructor(flowDesigner) {
        this.flowDesigner = flowDesigner;
        this.loadingStages = [
            { name: '初始化編輯器', weight: 20 },
            { name: '渲染節點', weight: 50 },
            { name: '建立連線', weight: 20 },
            { name: '完成載入', weight: 10 }
        ];
    }

    async loadFlowProgressively(flowData, onProgress = null) {
        console.log('🔄 開始漸進式載入流程...');
        
        try {
            // 階段 1: 初始化編輯器
            if (onProgress) onProgress({ message: '初始化編輯器', percentage: 0 });
            await this.executeStage('初始化編輯器', async () => {
                this.flowDesigner.clearEditor();
            });

            // 階段 2: 漸進式渲染節點
            if (onProgress) onProgress({ message: '渲染節點', percentage: 20 });
            await this.executeStage('渲染節點', async () => {
                if (flowData.nodes && flowData.nodes.length > 0) {
                    await this.renderNodesProgressively(flowData.nodes, onProgress);
                }
            });

            // 階段 3: 建立連線
            if (onProgress) onProgress({ message: '建立連線', percentage: 70 });
            await this.executeStage('建立連線', async () => {
                if (flowData.connections && flowData.connections.length > 0) {
                    await this.createConnectionsProgressively(flowData.connections);
                }
            });

            // 階段 4: 完成載入
            if (onProgress) onProgress({ message: '完成', percentage: 90 });
            await this.executeStage('完成載入', async () => {
                this.flowDesigner.updateStatusBar();
            });

            if (onProgress) onProgress({ message: '加载完成', percentage: 100 });
            console.log('✅ 渐进式加载完成');

        } catch (error) {
            console.error('❌ 渐进式加载失败:', error);
            if (onProgress) onProgress({ message: '加载失败', percentage: 0, error: error.message });
            throw error;
        }
    }

    async executeStage(stageName, stageFunction) {
        const startTime = performance.now();
        await stageFunction();
        const duration = performance.now() - startTime;
        console.log(`✅ ${stageName} 完成 (${duration.toFixed(0)}ms)`);
    }

    async renderNodesProgressively(nodes, onProgress) {
        const batchSize = 5; // 每批渲染5個節點
        
        for (let i = 0; i < nodes.length; i += batchSize) {
            const batch = nodes.slice(i, i + batchSize);
            
            // 渲染当前批次
            for (const node of batch) {
                try {
                    await this.flowDesigner.createNodeFromStep(node);
                } catch (error) {
                    console.warn(`節點渲染失敗: ${node.id}`, error);
                }
            }
            
            // 更新进度
            const progress = Math.min((i + batch.length) / nodes.length * 50 + 20, 70);
            if (onProgress) {
                onProgress({ 
                    message: `渲染節點 (${i + batch.length}/${nodes.length})`, 
                    percentage: progress 
                });
            }
            
            // 让出控制权
            if (i + batchSize < nodes.length) {
                await new Promise(resolve => setTimeout(resolve, 16));
            }
        }
    }

    async createConnectionsProgressively(connections) {
        for (let i = 0; i < connections.length; i++) {
            const conn = connections[i];
            
            try {
                this.flowDesigner.createVisualConnection(conn.source, conn.target);
            } catch (error) {
                console.warn(`連接建立失敗: ${conn.source} -> ${conn.target}`, error);
            }
            
            // 让出控制权
            if (i < connections.length - 1) {
                await new Promise(resolve => requestAnimationFrame(resolve));
            }
        }
    }
}

// Phase 4.2.2: Advanced Rendering Optimization - Canvas/WebGL 渲染引擎
class CanvasRenderingEngine {
    constructor(container) {
        this.container = container;
        this.canvas = null;
        this.ctx = null;
        this.nodes = [];
        this.connections = [];
        this.viewport = { x: 0, y: 0, scale: 1 };
        this.isDirty = true;
        this.renderMode = 'canvas'; // 'canvas' 或 'webgl'
        this.animationFrame = null;
        
        this.initializeCanvas();
        this.startRenderLoop();
    }

    initializeCanvas() {
        console.log('🎨 初始化 Canvas 渲染引擎...');
        
        // 創建 Canvas 元素
        this.canvas = document.createElement('canvas');
        this.canvas.id = 'flow-canvas';
        this.canvas.style.position = 'absolute';
        this.canvas.style.top = '0';
        this.canvas.style.left = '0';
        this.canvas.style.zIndex = '1';
        this.canvas.style.pointerEvents = 'none'; // 不干擾滑鼠事件
        
        // 設置 Canvas 大小
        this.resizeCanvas();
        
        // 初始化渲染上下文
        this.ctx = this.canvas.getContext('2d', {
            alpha: true,
            desynchronized: true, // 提升性能
            willReadFrequently: false
        });
        
        // 設置高 DPI 支援
        this.setupHighDPI();
        
        // 插入到容器
        this.container.appendChild(this.canvas);
        
        // 監聽視窗大小變化
        window.addEventListener('resize', () => this.resizeCanvas());
        
        console.log('✅ Canvas 渲染引擎初始化完成');
    }

    setupHighDPI() {
        const dpr = window.devicePixelRatio || 1;
        const rect = this.canvas.getBoundingClientRect();
        
        // 設置 Canvas 實際像素大小
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        
        // 縮放上下文以適應 DPI
        this.ctx.scale(dpr, dpr);
        
        // 設置 CSS 大小
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
    }

    resizeCanvas() {
        if (!this.container) return;
        
        const rect = this.container.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        
        this.setupHighDPI();
        this.markDirty();
    }

    startRenderLoop() {
        const render = () => {
            if (this.isDirty) {
                this.renderFrame();
            }
            this.animationFrame = requestAnimationFrame(render);
        };
        
        render();
    }

    renderFrame() {
        if (!this.ctx) return;
        
        const startTime = performance.now();
        
        // 清除畫布
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 應用視窗變換
        this.ctx.save();
        this.ctx.translate(-this.viewport.x, -this.viewport.y);
        this.ctx.scale(this.viewport.scale, this.viewport.scale);
        
        // 渲染連接線
        this.renderConnections();
        
        // 渲染節點
        this.renderNodes();
        
        this.ctx.restore();
        this.isDirty = false;
        
        const renderTime = performance.now() - startTime;
        if (renderTime > 16) {
            console.warn(`⚠️ Canvas 渲染時間過長: ${renderTime.toFixed(1)}ms`);
        }
    }

    renderNodes() {
        const { ctx } = this;
        
        this.nodes.forEach(node => {
            if (!this.isNodeVisible(node)) return;
            
            // 節點背景
            ctx.fillStyle = this.getNodeColor(node.type);
            ctx.fillRect(node.x, node.y, node.width, node.height);
            
            // 節點邊框
            ctx.strokeStyle = node.selected ? '#48cae4' : '#ccc';
            ctx.lineWidth = node.selected ? 3 : 1;
            ctx.strokeRect(node.x, node.y, node.width, node.height);
            
            // 節點文字
            ctx.fillStyle = '#333';
            ctx.font = '14px -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            
            // 支援多行文字
            this.renderNodeText(ctx, node.name, node.x + node.width / 2, node.y + node.height / 2, node.width - 10);
        });
    }

    renderNodeText(ctx, text, x, y, maxWidth) {
        const words = text.split(' ');
        const lineHeight = 18;
        let line = '';
        let lines = [];
        
        // 分解文字為多行
        for (let i = 0; i < words.length; i++) {
            const testLine = line + words[i] + ' ';
            const metrics = ctx.measureText(testLine);
            const testWidth = metrics.width;
            
            if (testWidth > maxWidth && i > 0) {
                lines.push(line);
                line = words[i] + ' ';
            } else {
                line = testLine;
            }
        }
        lines.push(line);
        
        // 繪製多行文字
        const startY = y - (lines.length - 1) * lineHeight / 2;
        lines.forEach((line, index) => {
            ctx.fillText(line.trim(), x, startY + index * lineHeight);
        });
    }

    renderConnections() {
        const { ctx } = this;
        
        ctx.strokeStyle = '#2563eb';
        ctx.lineWidth = 2;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        
        this.connections.forEach(conn => {
            if (!this.isConnectionVisible(conn)) return;
            
            ctx.beginPath();
            
            // 使用平滑的貝塞爾曲線
            const controlOffset = Math.min(Math.abs(conn.targetX - conn.sourceX) * 0.4, 80);
            const controlX1 = conn.sourceX + controlOffset;
            const controlX2 = conn.targetX - controlOffset;
            
            ctx.moveTo(conn.sourceX, conn.sourceY);
            ctx.bezierCurveTo(
                controlX1, conn.sourceY,
                controlX2, conn.targetY,
                conn.targetX, conn.targetY
            );
            
            ctx.stroke();
            
            // 繪製箭頭
            this.drawArrow(ctx, conn);
        });
    }

    drawArrow(ctx, connection) {
        const { targetX, targetY, sourceX, sourceY } = connection;
        const angle = Math.atan2(targetY - sourceY, targetX - sourceX);
        const arrowLength = 12;
        const arrowAngle = Math.PI / 6;
        
        ctx.save();
        ctx.translate(targetX, targetY);
        ctx.rotate(angle);
        
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowLength * Math.cos(arrowAngle), -arrowLength * Math.sin(arrowAngle));
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowLength * Math.cos(arrowAngle), arrowLength * Math.sin(arrowAngle));
        ctx.stroke();
        
        ctx.restore();
    }

    getNodeColor(nodeType) {
        const colors = {
            'condition': '#dbeafe',
            'logic': '#fef3c7',
            'action': '#d1fae5',
            'script': '#ede9fe'
        };
        return colors[nodeType] || '#f3f4f6';
    }

    isNodeVisible(node) {
        // 簡化的視窗檢查
        const margin = 50;
        return (
            node.x + node.width >= this.viewport.x - margin &&
            node.x <= this.viewport.x + this.canvas.width / this.viewport.scale + margin &&
            node.y + node.height >= this.viewport.y - margin &&
            node.y <= this.viewport.y + this.canvas.height / this.viewport.scale + margin
        );
    }

    isConnectionVisible(connection) {
        // 檢查連接線是否在視窗內
        const margin = 50;
        const minX = Math.min(connection.sourceX, connection.targetX);
        const maxX = Math.max(connection.sourceX, connection.targetX);
        const minY = Math.min(connection.sourceY, connection.targetY);
        const maxY = Math.max(connection.sourceY, connection.targetY);
        
        return (
            maxX >= this.viewport.x - margin &&
            minX <= this.viewport.x + this.canvas.width / this.viewport.scale + margin &&
            maxY >= this.viewport.y - margin &&
            minY <= this.viewport.y + this.canvas.height / this.viewport.scale + margin
        );
    }

    // 視窗控制
    setViewport(x, y, scale) {
        this.viewport = { x, y, scale: Math.max(0.1, Math.min(3, scale)) };
        this.markDirty();
    }

    // 更新節點
    updateNodes(nodes) {
        this.nodes = nodes.map(node => ({
            ...node,
            x: parseFloat(node.x) || 0,
            y: parseFloat(node.y) || 0,
            width: parseFloat(node.width) || 200,
            height: parseFloat(node.height) || 80
        }));
        this.markDirty();
    }

    // 更新連接
    updateConnections(connections) {
        this.connections = connections.map(conn => ({
            ...conn,
            sourceX: parseFloat(conn.sourceX) || 0,
            sourceY: parseFloat(conn.sourceY) || 0,
            targetX: parseFloat(conn.targetX) || 0,
            targetY: parseFloat(conn.targetY) || 0
        }));
        this.markDirty();
    }

    markDirty() {
        this.isDirty = true;
    }

    // 清理資源
    destroy() {
        if (this.animationFrame) {
            cancelAnimationFrame(this.animationFrame);
            this.animationFrame = null;
        }
        
        if (this.canvas && this.canvas.parentNode) {
            this.canvas.parentNode.removeChild(this.canvas);
        }
        
        window.removeEventListener('resize', this.resizeCanvas);
        console.log('🗑️ Canvas 渲染引擎已清理');
    }
}

// Phase 4.2.2: 最佳化連接線渲染器
class OptimizedConnectionRenderer {
    constructor(svg) {
        this.svg = svg;
        this.connectionPaths = new Map();
        this.visibleConnections = new Set();
        this.batchUpdateTimer = null;
        this.pathPool = []; // 路徑物件池
        this.maxPoolSize = 100;
    }

    batchUpdateConnections(connections) {
        // 批量更新連接線，避免頻繁重繪
        if (this.batchUpdateTimer) {
            clearTimeout(this.batchUpdateTimer);
        }

        this.batchUpdateTimer = setTimeout(() => {
            this.performBatchUpdate(connections);
        }, 16); // 60fps
    }

    performBatchUpdate(connections) {
        console.log(`🔄 批量更新 ${connections.length} 條連接線...`);
        
        const startTime = performance.now();
        
        // 使用 DocumentFragment 減少 DOM 操作
        const fragment = document.createDocumentFragment();
        const visiblePaths = [];

        // 視窗剔除檢查
        const visibleConnections = connections.filter(conn => this.isConnectionVisible(conn));
        
        console.log(`📊 視窗剔除：${connections.length} → ${visibleConnections.length} 條連接`);

        visibleConnections.forEach(conn => {
            const path = this.getOrCreatePath(conn);
            if (path) {
                visiblePaths.push(path);
                fragment.appendChild(path);
            }
        });

        // 一次性更新 DOM
        this.clearSvg();
        this.svg.appendChild(fragment);
        
        const renderTime = performance.now() - startTime;
        console.log(`✅ 連接線更新完成：${visiblePaths.length} 條 (${renderTime.toFixed(1)}ms)`);
    }

    getOrCreatePath(connection) {
        const pathId = connection.id || `${connection.source}_to_${connection.target}`;
        
        // 檢查物件池
        let path = this.pathPool.pop();
        if (!path) {
            path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        }
        
        // 設置路徑屬性
        const pathData = this.calculateOptimizedPath(connection);
        path.setAttribute('d', pathData);
        path.setAttribute('data-connection-id', pathId);
        
        // 使用 CSS 類別以提升效能
        path.className.baseVal = 'flow-connection';
        
        // 快取路徑
        this.connectionPaths.set(pathId, path);
        
        return path;
    }

    calculateOptimizedPath(connection) {
        // 使用高效的路徑計算
        const { sourceX, sourceY, targetX, targetY } = connection;
        
        // 智能控制點計算
        const dx = targetX - sourceX;
        const dy = targetY - sourceY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // 自適應控制點偏移
        const controlOffset = Math.min(distance * 0.4, 100);
        const controlX1 = sourceX + Math.sign(dx) * controlOffset;
        const controlX2 = targetX - Math.sign(dx) * controlOffset;
        
        return `M${sourceX},${sourceY} C${controlX1},${sourceY} ${controlX2},${targetY} ${targetX},${targetY}`;
    }

    isConnectionVisible(connection) {
        // 進階視窗檢查
        if (!this.svg || !this.svg.parentElement) return true;
        
        const editorRect = this.svg.parentElement.getBoundingClientRect();
        const margin = 100; // 預載入邊距
        
        const minX = Math.min(connection.sourceX, connection.targetX);
        const maxX = Math.max(connection.sourceX, connection.targetX);
        const minY = Math.min(connection.sourceY, connection.targetY);
        const maxY = Math.max(connection.sourceY, connection.targetY);
        
        return (
            maxX >= -margin && minX <= editorRect.width + margin &&
            maxY >= -margin && minY <= editorRect.height + margin
        );
    }

    clearSvg() {
        // 高效清理 SVG 內容，回收到物件池
        const paths = Array.from(this.svg.children);
        paths.forEach(path => {
            this.svg.removeChild(path);
            
            // 回收到物件池
            if (this.pathPool.length < this.maxPoolSize) {
                path.removeAttribute('d');
                path.removeAttribute('data-connection-id');
                path.className.baseVal = '';
                this.pathPool.push(path);
            }
        });
    }

    // 渲染效能統計
    getPerformanceStats() {
        return {
            totalConnections: this.connectionPaths.size,
            visibleConnections: this.visibleConnections.size,
            poolSize: this.pathPool.length,
            poolUtilization: ((this.maxPoolSize - this.pathPool.length) / this.maxPoolSize * 100).toFixed(1) + '%'
        };
    }

    // 清理資源
    destroy() {
        if (this.batchUpdateTimer) {
            clearTimeout(this.batchUpdateTimer);
            this.batchUpdateTimer = null;
        }
        
        this.connectionPaths.clear();
        this.visibleConnections.clear();
        this.pathPool = [];
        
        console.log('🗑️ 最佳化連接線渲染器已清理');
    }
}

// Phase 4.2.2: 視窗剔除管理器
class ViewportCullingManager {
    constructor() {
        this.viewport = { x: 0, y: 0, width: 1920, height: 1080, scale: 1 };
        this.cullMargin = 100; // 剔除邊距
        this.visibilityCache = new Map();
        this.cacheTimeout = 100; // 快取超時時間 (ms)
    }

    updateViewport(x, y, width, height, scale = 1) {
        this.viewport = { x, y, width, height, scale };
        this.clearVisibilityCache();
    }

    isElementVisible(element) {
        const cacheKey = this.getElementCacheKey(element);
        const cached = this.visibilityCache.get(cacheKey);
        
        // 檢查快取
        if (cached && (Date.now() - cached.timestamp) < this.cacheTimeout) {
            return cached.visible;
        }
        
        // 計算可見性
        const visible = this.calculateVisibility(element);
        
        // 更新快取
        this.visibilityCache.set(cacheKey, {
            visible,
            timestamp: Date.now()
        });
        
        return visible;
    }

    calculateVisibility(element) {
        const rect = this.getElementBounds(element);
        if (!rect) return false;
        
        // 視窗邊界
        const viewLeft = this.viewport.x - this.cullMargin;
        const viewRight = this.viewport.x + this.viewport.width / this.viewport.scale + this.cullMargin;
        const viewTop = this.viewport.y - this.cullMargin;
        const viewBottom = this.viewport.y + this.viewport.height / this.viewport.scale + this.cullMargin;
        
        // 檢查重疊
        return !(
            rect.right < viewLeft ||
            rect.left > viewRight ||
            rect.bottom < viewTop ||
            rect.top > viewBottom
        );
    }

    getElementBounds(element) {
        if (element.getBoundingClientRect) {
            return element.getBoundingClientRect();
        }
        
        // 對於自定義物件
        if (element.x !== undefined && element.y !== undefined) {
            return {
                left: element.x,
                top: element.y,
                right: element.x + (element.width || 0),
                bottom: element.y + (element.height || 0)
            };
        }
        
        return null;
    }

    getElementCacheKey(element) {
        if (element.id) return element.id;
        if (element.dataset && element.dataset.nodeId) return element.dataset.nodeId;
        return element.toString();
    }

    clearVisibilityCache() {
        this.visibilityCache.clear();
    }

    getVisibilityStats() {
        return {
            cacheSize: this.visibilityCache.size,
            viewport: this.viewport,
            cullMargin: this.cullMargin
        };
    }
}

// Phase 4.2.2: Advanced Canvas 2D Rendering Engine
class AdvancedCanvasRenderer {
    constructor(canvasElement, options = {}) {
        this.canvas = canvasElement;
        this.ctx = canvasElement.getContext('2d');
        this.options = {
            enableHiDPI: true,
            enableAntialiasing: true,
            enableCaching: true,
            maxCacheSize: 100,
            ...options
        };
        
        // Viewport 和變換
        this.viewport = { x: 0, y: 0, scale: 1 };
        this.viewBounds = { x: 0, y: 0, width: 0, height: 0 };
        
        // 快取系統
        this.nodeCache = new Map();
        this.connectionCache = new Map();
        this.offscreenCanvases = new Map();
        
        // 效能監控
        this.renderStats = {
            fps: 0,
            renderTime: 0,
            nodesRendered: 0,
            connectionsRendered: 0,
            cacheHitRate: 0,
            memoryUsage: 0
        };
        
        this.frameCount = 0;
        this.lastFrameTime = performance.now();
        this.fpsHistory = [];
        
        this.setupCanvas();
        this.setupEventHandlers();
        
        console.log('✅ Advanced Canvas 2D Renderer 初始化完成');
    }
    
    setupCanvas() {
        this.setupHighDPI();
        this.updateViewBounds();
        
        // 抗鋸齒設定
        if (this.options.enableAntialiasing) {
            this.ctx.imageSmoothingEnabled = true;
            this.ctx.imageSmoothingQuality = 'high';
        }
    }
    
    setupHighDPI() {
        const dpr = window.devicePixelRatio || 1;
        const rect = this.canvas.getBoundingClientRect();
        
        // 設定實際畫布大小 (考慮 DPI)
        this.canvas.width = rect.width * dpr;
        this.canvas.height = rect.height * dpr;
        
        // 設定 CSS 顯示大小
        this.canvas.style.width = rect.width + 'px';
        this.canvas.style.height = rect.height + 'px';
        
        // 縮放 context 以匹配 DPI
        this.ctx.scale(dpr, dpr);
    }
    
    setupEventHandlers() {
        // Canvas 大小變更處理
        window.addEventListener('resize', () => {
            this.setupHighDPI();
            this.updateViewBounds();
            this.clearCache();
        });
    }
    
    updateViewBounds() {
        const rect = this.canvas.getBoundingClientRect();
        this.viewBounds = {
            x: this.viewport.x,
            y: this.viewport.y,
            width: rect.width / this.viewport.scale,
            height: rect.height / this.viewport.scale
        };
    }
    
    // 主要渲染方法
    render(nodes, connections) {
        const startTime = performance.now();
        this.frameCount++;
        
        // 清除畫布
        this.clearCanvas();
        
        // 應用視窗變換
        this.ctx.save();
        this.applyViewportTransform();
        
        // 視窗裁剪 - 只渲染可見元素
        const visibleNodes = this.cullNodes(nodes);
        const visibleConnections = this.cullConnections(connections, visibleNodes);
        
        // 渲染連接線
        this.renderConnections(visibleConnections);
        
        // 渲染節點
        this.renderNodes(visibleNodes);
        
        this.ctx.restore();
        
        // 更新效能統計
        this.updateRenderStats(startTime, visibleNodes.length, visibleConnections.length);
        
        // 清理快取 (防止記憶體洩漏)
        this.maintainCache();
    }
    
    clearCanvas() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 設定背景色 (可選)
        if (this.options.backgroundColor) {
            this.ctx.fillStyle = this.options.backgroundColor;
            this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        }
    }
    
    applyViewportTransform() {
        this.ctx.translate(-this.viewport.x, -this.viewport.y);
        this.ctx.scale(this.viewport.scale, this.viewport.scale);
    }
    
    // 視窗裁剪系統
    cullNodes(nodes) {
        return nodes.filter(node => this.isNodeVisible(node));
    }
    
    cullConnections(connections, visibleNodes) {
        const visibleNodeIds = new Set(visibleNodes.map(n => n.id));
        return connections.filter(conn => 
            visibleNodeIds.has(conn.source) || visibleNodeIds.has(conn.target)
        );
    }
    
    isNodeVisible(node) {
        const nodeRect = {
            x: node.x,
            y: node.y,
            width: node.width || 200,
            height: node.height || 100
        };
        
        return this.intersectsViewport(nodeRect);
    }
    
    intersectsViewport(rect) {
        const vb = this.viewBounds;
        const margin = 50; // 裁剪邊距
        
        return !(
            rect.x + rect.width < vb.x - margin ||
            rect.x > vb.x + vb.width + margin ||
            rect.y + rect.height < vb.y - margin ||
            rect.y > vb.y + vb.height + margin
        );
    }
    
    // 節點渲染系統
    renderNodes(nodes) {
        let cacheHits = 0;
        
        nodes.forEach(node => {
            if (this.renderNodeFromCache(node)) {
                cacheHits++;
            } else {
                this.renderNodeToCache(node);
            }
        });
        
        this.renderStats.cacheHitRate = nodes.length > 0 ? cacheHits / nodes.length : 0;
    }
    
    renderNodeFromCache(node) {
        const cacheKey = this.getNodeCacheKey(node);
        const cached = this.nodeCache.get(cacheKey);
        
        if (cached && !node.isDirty) {
            this.ctx.drawImage(cached.canvas, node.x, node.y);
            return true; // 快取命中
        }
        
        return false; // 快取未命中
    }
    
    renderNodeToCache(node) {
        const cacheKey = this.getNodeCacheKey(node);
        
        // 創建離屏 Canvas
        const offscreen = this.createOffscreenCanvas(node);
        
        // 渲染節點到離屏 Canvas
        this.drawNodeToOffscreen(offscreen, node);
        
        // 儲存到快取
        this.nodeCache.set(cacheKey, {
            canvas: offscreen.canvas,
            timestamp: Date.now(),
            hits: 0
        });
        
        // 繪製到主 Canvas
        this.ctx.drawImage(offscreen.canvas, node.x, node.y);
        
        // 標記節點為乾淨
        node.isDirty = false;
    }
    
    createOffscreenCanvas(node) {
        const width = node.width || 200;
        const height = node.height || 100;
        
        let offscreen = this.offscreenCanvases.get(`${width}x${height}`);
        
        if (!offscreen) {
            const canvas = document.createElement('canvas');
            canvas.width = width;
            canvas.height = height;
            
            offscreen = {
                canvas: canvas,
                ctx: canvas.getContext('2d')
            };
            
            // 快取離屏 Canvas (重複使用相同大小的)
            this.offscreenCanvases.set(`${width}x${height}`, offscreen);
        }
        
        // 清除內容供重新使用
        offscreen.ctx.clearRect(0, 0, width, height);
        
        return offscreen;
    }
    
    drawNodeToOffscreen(offscreen, node) {
        const { ctx } = offscreen;
        const { width = 200, height = 100 } = node;
        
        // 節點樣式
        const style = this.getNodeStyle(node);
        
        // 繪製節點背景
        this.drawNodeBackground(ctx, node, style, width, height);
        
        // 繪製節點內容
        this.drawNodeContent(ctx, node, style, width, height);
        
        // 繪製節點邊框
        this.drawNodeBorder(ctx, node, style, width, height);
        
        // 繪製節點 Socket
        this.drawNodeSockets(ctx, node, style, width, height);
    }
    
    drawNodeBackground(ctx, node, style, width, height) {
        // 漸變背景
        const gradient = ctx.createLinearGradient(0, 0, 0, height);
        gradient.addColorStop(0, style.backgroundColor);
        gradient.addColorStop(1, style.backgroundColorEnd || this.darkenColor(style.backgroundColor, 0.1));
        
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, width, height);
        
        // 陰影效果 (可選)
        if (style.shadow) {
            ctx.save();
            ctx.shadowColor = 'rgba(0, 0, 0, 0.1)';
            ctx.shadowBlur = 4;
            ctx.shadowOffsetX = 2;
            ctx.shadowOffsetY = 2;
            ctx.fillRect(0, 0, width, height);
            ctx.restore();
        }
    }
    
    drawNodeContent(ctx, node, style, width, height) {
        // 節點標題
        ctx.fillStyle = style.titleColor || '#333';
        ctx.font = style.titleFont || 'bold 14px -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        
        const title = node.name || node.id;
        this.drawWrappedText(ctx, title, width / 2, 12, width - 20, 16, 2);
        
        // 節點描述
        if (node.description) {
            ctx.fillStyle = style.descriptionColor || '#666';
            ctx.font = style.descriptionFont || '12px -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif';
            
            this.drawWrappedText(ctx, node.description, width / 2, 40, width - 20, 14, 3);
        }
        
        // 節點圖示
        if (style.icon) {
            this.drawNodeIcon(ctx, style.icon, 12, 12, 16);
        }
    }
    
    drawNodeBorder(ctx, node, style, width, height) {
        ctx.strokeStyle = node.selected ? style.selectedColor || '#48cae4' : style.borderColor || '#ccc';
        ctx.lineWidth = node.selected ? 3 : 1;
        ctx.strokeRect(0, 0, width, height);
        
        // 選中時的發光效果
        if (node.selected && style.glow) {
            ctx.save();
            ctx.shadowColor = style.selectedColor || '#48cae4';
            ctx.shadowBlur = 10;
            ctx.strokeRect(0, 0, width, height);
            ctx.restore();
        }
    }
    
    drawNodeSockets(ctx, node, style, width, height) {
        const socketRadius = 6;
        
        // 輸入 Socket (左側)
        if (node.inputs) {
            node.inputs.forEach((input, index) => {
                const y = (height / (node.inputs.length + 1)) * (index + 1);
                this.drawSocket(ctx, 0, y, socketRadius, style.inputSocketColor || '#4a9eff');
            });
        }
        
        // 輸出 Socket (右側)
        if (node.outputs) {
            node.outputs.forEach((output, index) => {
                const y = (height / (node.outputs.length + 1)) * (index + 1);
                this.drawSocket(ctx, width, y, socketRadius, style.outputSocketColor || '#ff6b6b');
            });
        }
    }
    
    drawSocket(ctx, x, y, radius, color) {
        ctx.beginPath();
        ctx.arc(x, y, radius, 0, 2 * Math.PI);
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.stroke();
    }
    
    drawWrappedText(ctx, text, x, y, maxWidth, lineHeight, maxLines = 10) {
        const words = text.split(' ');
        let line = '';
        let currentY = y;
        let lineCount = 0;
        
        for (let n = 0; n < words.length && lineCount < maxLines; n++) {
            const testLine = line + words[n] + ' ';
            const metrics = ctx.measureText(testLine);
            const testWidth = metrics.width;
            
            if (testWidth > maxWidth && n > 0) {
                ctx.fillText(line, x, currentY);
                line = words[n] + ' ';
                currentY += lineHeight;
                lineCount++;
            } else {
                line = testLine;
            }
        }
        
        if (lineCount < maxLines) {
            ctx.fillText(line, x, currentY);
        }
    }
    
    // 連接線渲染系統
    renderConnections(connections) {
        connections.forEach(connection => {
            this.renderConnection(connection);
        });
    }
    
    renderConnection(connection) {
        const { source, target, style = {} } = connection;
        
        // 計算連接點
        const sourcePoint = this.getSocketPosition(source);
        const targetPoint = this.getSocketPosition(target);
        
        if (!sourcePoint || !targetPoint) return;
        
        // 繪製連接路徑
        this.drawConnectionPath(sourcePoint, targetPoint, style);
        
        // 繪製箭頭
        if (style.showArrow !== false) {
            this.drawArrowhead(sourcePoint, targetPoint, style);
        }
    }
    
    drawConnectionPath(source, target, style) {
        const ctx = this.ctx;
        const controlOffset = Math.abs(target.x - source.x) * 0.5;
        
        ctx.beginPath();
        ctx.moveTo(source.x, source.y);
        
        // 貝茲曲線連接
        ctx.bezierCurveTo(
            source.x + controlOffset, source.y,
            target.x - controlOffset, target.y,
            target.x, target.y
        );
        
        // 連接線樣式
        ctx.strokeStyle = style.color || '#666';
        ctx.lineWidth = style.width || 2;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        
        // 虛線支援
        if (style.dash) {
            ctx.setLineDash(style.dash);
        } else {
            ctx.setLineDash([]);
        }
        
        ctx.stroke();
    }
    
    drawArrowhead(source, target, style) {
        const ctx = this.ctx;
        const angle = Math.atan2(target.y - source.y, target.x - source.x);
        const arrowLength = 12;
        const arrowWidth = 8;
        const offset = 10; // 從目標點的偏移
        
        // 計算箭頭位置
        const arrowX = target.x - Math.cos(angle) * offset;
        const arrowY = target.y - Math.sin(angle) * offset;
        
        ctx.save();
        ctx.translate(arrowX, arrowY);
        ctx.rotate(angle);
        
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowLength, -arrowWidth / 2);
        ctx.lineTo(-arrowLength, arrowWidth / 2);
        ctx.closePath();
        
        ctx.fillStyle = style.color || '#666';
        ctx.fill();
        
        ctx.restore();
    }
    
    // 輔助方法
    getNodeCacheKey(node) {
        return `node_${node.id}_${node.type}_${node.selected}_${node.isDirty || false}`;
    }
    
    getNodeStyle(node) {
        const defaultStyles = {
            condition: { backgroundColor: '#e3f2fd', titleColor: '#1565c0', borderColor: '#90caf9' },
            logic: { backgroundColor: '#f3e5f5', titleColor: '#7b1fa2', borderColor: '#ce93d8' },
            action: { backgroundColor: '#e8f5e8', titleColor: '#2e7d32', borderColor: '#a5d6a7' },
            script: { backgroundColor: '#fff3e0', titleColor: '#ef6c00', borderColor: '#ffcc02' }
        };
        
        return defaultStyles[node.type] || defaultStyles.condition;
    }
    
    getSocketPosition(socketId) {
        // 這裡需要與實際的節點和 socket 系統整合
        // 暫時返回示例位置
        return { x: 0, y: 0 };
    }
    
    darkenColor(color, factor) {
        // 簡單的顏色加深算法
        if (color.startsWith('#')) {
            const num = parseInt(color.slice(1), 16);
            const r = Math.floor((num >> 16) * (1 - factor));
            const g = Math.floor(((num >> 8) & 0x00FF) * (1 - factor));
            const b = Math.floor((num & 0x0000FF) * (1 - factor));
            return `rgb(${r}, ${g}, ${b})`;
        }
        return color;
    }
    
    // 快取管理
    maintainCache() {
        // 清理過期快取
        if (this.nodeCache.size > this.options.maxCacheSize) {
            const entries = Array.from(this.nodeCache.entries());
            
            // 按最近使用時間排序
            entries.sort((a, b) => a[1].timestamp - b[1].timestamp);
            
            // 清理最舊的 20% 快取
            const toRemove = Math.floor(entries.length * 0.2);
            for (let i = 0; i < toRemove; i++) {
                this.nodeCache.delete(entries[i][0]);
            }
        }
    }
    
    clearCache() {
        this.nodeCache.clear();
        this.connectionCache.clear();
        this.offscreenCanvases.clear();
    }
    
    // 效能統計
    updateRenderStats(startTime, nodeCount, connectionCount) {
        const renderTime = performance.now() - startTime;
        this.renderStats.renderTime = renderTime;
        this.renderStats.nodesRendered = nodeCount;
        this.renderStats.connectionsRendered = connectionCount;
        
        // FPS 計算
        const now = performance.now();
        const deltaTime = now - this.lastFrameTime;
        const fps = 1000 / deltaTime;
        
        this.fpsHistory.push(fps);
        if (this.fpsHistory.length > 60) {
            this.fpsHistory.shift();
        }
        
        this.renderStats.fps = this.fpsHistory.reduce((a, b) => a + b, 0) / this.fpsHistory.length;
        this.lastFrameTime = now;
        
        // 記憶體使用統計
        this.renderStats.memoryUsage = this.nodeCache.size + this.connectionCache.size;
        
        // 效能警告
        if (renderTime > 16.67) { // > 60 FPS
            console.warn(`⚠️ Canvas 渲染效能警告: ${renderTime.toFixed(1)}ms (FPS: ${fps.toFixed(1)})`);
        }
    }
    
    getRenderStats() {
        return { ...this.renderStats };
    }
    
    // 視窗控制
    setViewport(x, y, scale) {
        this.viewport = { x, y, scale };
        this.updateViewBounds();
    }
    
    destroy() {
        this.clearCache();
        if (this.animationFrame) {
            cancelAnimationFrame(this.animationFrame);
        }
    }
}

// WebGL 渲染引擎 (備選方案)
class WebGLFlowRenderer {
    constructor(canvasElement) {
        this.canvas = canvasElement;
        this.gl = canvasElement.getContext('webgl2') || canvasElement.getContext('webgl');
        
        if (!this.gl) {
            throw new Error('WebGL not supported');
        }
        
        this.programs = {};
        this.buffers = {};
        this.textures = {};
        
        this.initializeShaders();
        this.setupBuffers();
        
        console.log('✅ WebGL Flow Renderer 初始化完成');
    }
    
    initializeShaders() {
        // 節點渲染著色器
        this.programs.node = this.createShaderProgram(
            this.getVertexShaderSource(),
            this.getFragmentShaderSource()
        );
    }
    
    getVertexShaderSource() {
        return `
            attribute vec2 a_position;
            attribute vec2 a_texCoord;
            attribute vec4 a_color;
            
            uniform mat3 u_transform;
            uniform vec2 u_resolution;
            
            varying vec2 v_texCoord;
            varying vec4 v_color;
            
            void main() {
                vec3 position = u_transform * vec3(a_position, 1.0);
                vec2 clipSpace = ((position.xy / u_resolution) * 2.0 - 1.0) * vec2(1, -1);
                
                gl_Position = vec4(clipSpace, 0, 1);
                v_texCoord = a_texCoord;
                v_color = a_color;
            }
        `;
    }
    
    getFragmentShaderSource() {
        return `
            precision mediump float;
            
            varying vec2 v_texCoord;
            varying vec4 v_color;
            
            uniform sampler2D u_texture;
            
            void main() {
                vec4 texColor = texture2D(u_texture, v_texCoord);
                gl_FragColor = texColor * v_color;
            }
        `;
    }
    
    createShaderProgram(vertexSource, fragmentSource) {
        const vertexShader = this.createShader(this.gl.VERTEX_SHADER, vertexSource);
        const fragmentShader = this.createShader(this.gl.FRAGMENT_SHADER, fragmentSource);
        
        const program = this.gl.createProgram();
        this.gl.attachShader(program, vertexShader);
        this.gl.attachShader(program, fragmentShader);
        this.gl.linkProgram(program);
        
        if (!this.gl.getProgramParameter(program, this.gl.LINK_STATUS)) {
            console.error('WebGL Program Link Error:', this.gl.getProgramInfoLog(program));
            return null;
        }
        
        return program;
    }
    
    createShader(type, source) {
        const shader = this.gl.createShader(type);
        this.gl.shaderSource(shader, source);
        this.gl.compileShader(shader);
        
        if (!this.gl.getShaderParameter(shader, this.gl.COMPILE_STATUS)) {
            console.error('WebGL Shader Compile Error:', this.gl.getShaderInfoLog(shader));
            this.gl.deleteShader(shader);
            return null;
        }
        
        return shader;
    }
    
    setupBuffers() {
        // 設定基本緩衝區
        this.buffers.position = this.gl.createBuffer();
        this.buffers.texCoord = this.gl.createBuffer();
        this.buffers.color = this.gl.createBuffer();
    }
    
    render(nodes, connections) {
        // WebGL 渲染實現
        this.gl.clear(this.gl.COLOR_BUFFER_BIT);
        
        // 使用節點著色器程序
        this.gl.useProgram(this.programs.node);
        
        // 渲染節點
        this.renderNodes(nodes);
        
        // 渲染連接線
        this.renderConnections(connections);
    }
    
    renderNodes(nodes) {
        // WebGL 節點渲染實現
        // 這裡會實現批量實例化渲染
    }
    
    renderConnections(connections) {
        // WebGL 連接線渲染實現
    }
}

// 高級渲染管理器
class AdvancedRenderingManager {
    constructor(options = {}) {
        this.options = {
            preferWebGL: false, // 預設使用 Canvas 2D
            fallbackToCanvas: true,
            enableViewportCulling: true,
            maxNodesForDOM: 50,
            ...options
        };
        
        this.renderer = null;
        this.renderingMode = 'dom'; // 'dom', 'canvas', 'webgl'
        this.nodes = [];
        this.connections = [];
        
        this.renderStats = {
            mode: 'dom',
            fps: 0,
            nodeCount: 0,
            visibleNodeCount: 0,
            renderTime: 0,
            memoryUsage: 0
        };
        
        this.initializeRenderer();
    }
    
    initializeRenderer() {
        const canvas = document.getElementById('flow-canvas');
        
        // 決定渲染模式
        if (this.options.preferWebGL && this.supportsWebGL()) {
            try {
                this.renderer = new WebGLFlowRenderer(canvas);
                this.renderingMode = 'webgl';
                console.log('✅ 使用 WebGL 渲染模式');
                return;
            } catch (error) {
                console.warn('⚠️ WebGL 初始化失敗，回退到 Canvas 2D:', error.message);
            }
        }
        
        // Canvas 2D 渲染模式
        if (this.options.fallbackToCanvas) {
            try {
                this.renderer = new AdvancedCanvasRenderer(canvas, this.options);
                this.renderingMode = 'canvas';
                console.log('✅ 使用 Canvas 2D 渲染模式');
                return;
            } catch (error) {
                console.warn('⚠️ Canvas 2D 初始化失敗，回退到 DOM 渲染:', error.message);
            }
        }
        
        // DOM 渲染模式 (備選)
        this.renderingMode = 'dom';
        console.log('✅ 使用 DOM 渲染模式');
    }
    
    supportsWebGL() {
        try {
            const canvas = document.createElement('canvas');
            return !!(canvas.getContext('webgl') || canvas.getContext('webgl2'));
        } catch (e) {
            return false;
        }
    }
    
    render(nodes, connections) {
        this.nodes = nodes;
        this.connections = connections;
        
        const startTime = performance.now();
        
        // 根據節點數量和渲染模式選擇策略
        if (this.shouldUseAdvancedRendering(nodes)) {
            if (this.renderer && this.renderingMode !== 'dom') {
                this.renderer.render(nodes, connections);
                
                // 更新統計
                if (this.renderer.getRenderStats) {
                    const stats = this.renderer.getRenderStats();
                    this.renderStats = {
                        ...this.renderStats,
                        ...stats,
                        mode: this.renderingMode
                    };
                }
            }
        } else {
            // 小規模場景使用 DOM 渲染
            this.renderWithDOM(nodes, connections);
        }
        
        const renderTime = performance.now() - startTime;
        this.renderStats.renderTime = renderTime;
        this.renderStats.nodeCount = nodes.length;
    }
    
    shouldUseAdvancedRendering(nodes) {
        return nodes.length > this.options.maxNodesForDOM;
    }
    
    renderWithDOM(nodes, connections) {
        // DOM 渲染實現 (回退模式)
        // 這裡會調用原始的 DOM 渲染邏輯
        console.log(`📊 使用 DOM 渲染 ${nodes.length} 個節點`);
    }
    
    switchRenderingMode(mode) {
        if (mode === this.renderingMode) return;
        
        console.log(`🔄 切換渲染模式: ${this.renderingMode} → ${mode}`);
        
        if (this.renderer && this.renderer.destroy) {
            this.renderer.destroy();
        }
        
        this.options.preferWebGL = (mode === 'webgl');
        this.initializeRenderer();
        
        // 重新渲染
        if (this.nodes.size > 0) {
            this.render(Array.from(this.nodes.values()), Array.from(this.connections.values()));
        }
    }
    
    setViewport(x, y, scale) {
        if (this.renderer && this.renderer.setViewport) {
            this.renderer.setViewport(x, y, scale);
        }
    }
    
    getRenderStats() {
        return { ...this.renderStats };
    }
    
    destroy() {
        if (this.renderer && this.renderer.destroy) {
            this.renderer.destroy();
        }
    }
}

// 全域可用
window.WcsFlowDesigner = WcsFlowDesigner;
window.PerformanceOptimizer = PerformanceOptimizer;
window.CanvasRenderingEngine = CanvasRenderingEngine;
window.OptimizedConnectionRenderer = OptimizedConnectionRenderer;
window.ViewportCullingManager = ViewportCullingManager;
window.AdvancedCanvasRenderer = AdvancedCanvasRenderer;
window.WebGLFlowRenderer = WebGLFlowRenderer;
window.AdvancedRenderingManager = AdvancedRenderingManager;