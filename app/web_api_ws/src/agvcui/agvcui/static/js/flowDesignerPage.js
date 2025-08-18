// WCS Flow Designer JavaScript 模組 v2.0
// 完全基於參考實現，整合到 AGVCUI 架構
// 修復版本：節點選板分類修復 + Socket.IO 衝突解決

class WcsFlowDesigner {
    constructor() {
        this.editor = null;
        this.area = null;
        this.selectedNode = null;
        this.nodeCounter = 0;
        this.connections = new Map(); // Track visual connections
        this.connectionState = null; // Connection state for drag operations
        this.globalEventsAdded = false; // Track global event listeners
        this.currentFlowName = '未命名';
        this.functionsConfig = null;
        this.socket = null; // Rete socket
        
        // 連線繪製模式設置
        this.connectionMode = 'curved'; // 'curved' | 'orthogonal' | 'straight'
        
        // 節點類型定義 - 從配置文件動態加載
        this.nodeTypes = {};
        this.nodeConfigs = {
            condition_nodes: null,
            logic_nodes: null,
            action_nodes: null
        };
        
        // Load node configurations from YAML files
        this.loadNodeConfigurations().then(() => {
            this.init();
        }).catch(error => {
            console.error('❌ 節點配置載入失敗:', error);
            this.init(); // Continue with empty configurations
        });
    }

    async init() {
        try {
            // 修復 Socket.IO 連接問題
            this.initSocket();
            
            // 載入函數配置
            await this.loadFunctionsConfig();
            
            // 設置編輯器
            await this.setupEditor();
            
            // 設置事件監聽器
            this.setupEventListeners();
            
            // 創建節點選板
            this.createNodePalette();
            
            // 更新狀態列
            this.updateStatusBar();
            
            console.log('✅ WCS Flow Designer 初始化完成');
            
        } catch (error) {
            console.error('❌ Flow Designer 初始化失敗:', error);
            this.showNotification('Flow Designer 初始化失敗', 'danger');
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
            // Load all three node configuration files
            const configPromises = [
                this.fetchNodeConfig('/static/config/wcs/nodes/condition_nodes.yaml', 'condition_nodes'),
                this.fetchNodeConfig('/static/config/wcs/nodes/logic_nodes.yaml', 'logic_nodes'),
                this.fetchNodeConfig('/static/config/wcs/nodes/action_nodes.yaml', 'action_nodes')
            ];
            
            await Promise.all(configPromises);
            
            // Transform loaded configurations into nodeTypes
            this.populateNodeTypes();
            
            console.log('✅ 節點配置載入完成:', Object.keys(this.nodeTypes).length, '個節點類型');
            
        } catch (error) {
            console.error('❌ 節點配置載入失敗:', error);
            // Use fallback empty configurations
            this.nodeConfigs = {
                condition_nodes: { condition_nodes: {} },
                logic_nodes: { logic_nodes: {} },
                action_nodes: { action_nodes: {} }
            };
            this.populateNodeTypes();
        }
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
        
        // 統一顏色系統 - 按種類分色
        const categoryColors = {
            // 條件節點 (輸入) - 藍色系
            'input': '#3B82F6',      // 藍色 - 條件輸入
            
            // 邏輯節點 (控制) - 橙色系
            'control': '#F59E0B',    // 橙色 - 邏輯控制
            
            // 動作節點 (輸出) - 綠色系  
            'output': '#10B981',     // 綠色 - 動作輸出
            
            // 特殊節點類型
            'storage': '#8B5CF6',    // 紫色 - 存儲節點
            'process': '#EF4444'     // 紅色 - 處理節點
        };
        
        // Process condition nodes
        if (this.nodeConfigs.condition_nodes?.condition_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.condition_nodes.condition_nodes)) {
                this.nodeTypes[nodeId] = {
                    name: nodeConfig.name,
                    type: 'condition',
                    category: nodeConfig.category || 'input',
                    icon: nodeConfig.icon || '❓',
                    color: categoryColors[nodeConfig.category || 'input'],
                    inputs: nodeConfig.inputs || [],
                    outputs: ['output'],
                    parameters: this.processNodeParameters(nodeConfig.parameters || []),
                    conditions: nodeConfig.conditions || [],
                    description: nodeConfig.description || ''
                };
            }
        }
        
        // Process logic nodes
        if (this.nodeConfigs.logic_nodes?.logic_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.logic_nodes.logic_nodes)) {
                this.nodeTypes[nodeId] = {
                    name: nodeConfig.name,
                    type: 'logic',
                    category: nodeConfig.category || 'control',
                    icon: nodeConfig.icon || '⚙️',
                    color: categoryColors[nodeConfig.category || 'control'],
                    inputs: nodeConfig.inputs || ['input'],
                    outputs: nodeConfig.outputs || ['output'],
                    parameters: this.processNodeParameters(nodeConfig.parameters || []),
                    logic: nodeConfig.logic || {},
                    description: nodeConfig.description || ''
                };
            }
        }
        
        // Process action nodes
        if (this.nodeConfigs.action_nodes?.action_nodes) {
            for (const [nodeId, nodeConfig] of Object.entries(this.nodeConfigs.action_nodes.action_nodes)) {
                this.nodeTypes[nodeId] = {
                    name: nodeConfig.name,
                    type: 'action',
                    category: nodeConfig.category || 'output',
                    icon: nodeConfig.icon || '🎯',
                    color: categoryColors[nodeConfig.category || 'output'],
                    inputs: nodeConfig.inputs || ['trigger'],
                    outputs: [],
                    parameters: this.processNodeParameters(nodeConfig.parameters || []),
                    actions: nodeConfig.actions || [],
                    description: nodeConfig.description || ''
                };
            }
        }
        
        console.log('✅ 節點類型轉換完成:', Object.keys(this.nodeTypes).length, '個節點類型');
        console.log('🎨 顏色系統:', categoryColors);
    }

    processNodeParameters(parameters) {
        // 處理節點參數，確保每個參數都有完整的配置
        return parameters.map(param => ({
            name: param.name,
            type: param.type || 'string',
            required: param.required || false,
            default: param.default || '',
            description: param.description || '',
            options: param.options || null,
            min: param.min || null,
            max: param.max || null,
            // 添加當前值屬性，用於用戶設定
            value: param.default || ''
        }));
    }

    hexToRgba(hex, alpha) {
        // 將十六進制顏色轉換為 RGBA 格式
        const r = parseInt(hex.slice(1, 3), 16);
        const g = parseInt(hex.slice(3, 5), 16);
        const b = parseInt(hex.slice(5, 7), 16);
        return `rgba(${r}, ${g}, ${b}, ${alpha})`;
    }

    async setupEditor() {
        const container = document.getElementById('rete-editor');
        if (!container) {
            throw new Error('找不到 rete-editor 容器');
        }

        console.log('🎨 初始化 Rete.js 編輯器...');
        
        // 創建 Rete.js v2 編輯器
        this.editor = new Rete.NodeEditor();
        
        // 創建區域插件用於視覺化
        this.area = new ReteAreaPlugin.AreaPlugin(container);
        
        // 使用區域插件
        this.editor.use(this.area);
        
        // 創建通用的流程連接 socket
        this.socket = new Rete.ClassicPreset.Socket('flow');
        console.log('🔌 創建通用 socket:', this.socket);
        
        // 添加連接插件 (Rete.js v2 連接系統)
        try {
            if (window.ReteConnectionPlugin && window.ReteConnectionPlugin.ConnectionPlugin) {
                const connection = new ReteConnectionPlugin.ConnectionPlugin();
                connection.addPreset(ReteConnectionPlugin.Presets.classic.setup());
                this.area.use(connection);
                console.log('✅ Connection plugin 載入成功');
            } else {
                console.warn('⚠️ Connection plugin 不可用');
            }
        } catch (error) {
            console.error('❌ Connection plugin 載入錯誤:', error);
        }
        
        // 設置區域擴展功能
        ReteAreaPlugin.AreaExtensions.selectableNodes(this.area, 
            ReteAreaPlugin.AreaExtensions.selector(), {
            accumulating: ReteAreaPlugin.AreaExtensions.accumulateOnCtrl()
        });
        
        // 初始化連接狀態
        this.connectionState = {
            isConnecting: false,
            startSocket: null,
            tempLine: null
        };
        
        // 設置編輯器事件監聽
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
        
        // 設置畫布互動
        this.setupCanvasInteractions();
        
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
        document.getElementById('btn-new-flow')?.addEventListener('click', () => this.newFlow());
        document.getElementById('btn-load-flow')?.addEventListener('click', () => this.loadFlow());
        document.getElementById('btn-save-flow')?.addEventListener('click', () => this.saveFlow());
        document.getElementById('btn-export-flow')?.addEventListener('click', () => this.exportFlow());
        
        // 模態框事件
        document.getElementById('confirm-flow-action')?.addEventListener('click', () => this.confirmFlowAction());
        document.getElementById('cancel-flow-action')?.addEventListener('click', () => this.closeFlowModal());
        
        // 屬性面板關閉
        document.getElementById('close-properties')?.addEventListener('click', () => this.closePropertiesPanel());
    }

    createNodePalette() {
        // 清空現有節點
        document.getElementById('condition-nodes').innerHTML = '';
        document.getElementById('action-nodes').innerHTML = '';
        document.getElementById('logic-nodes').innerHTML = '';

        // 正確的節點分類映射
        Object.entries(this.nodeTypes).forEach(([nodeId, nodeType]) => {
            let containerId;
            
            // 根據節點類別決定容器
            switch (nodeType.category) {
                case 'input':
                    containerId = 'condition-nodes'; // 輸入節點 → 條件節點區域
                    break;
                case 'process':
                    containerId = 'action-nodes';     // 處理節點 → 動作節點區域
                    break;
                case 'control':
                    containerId = 'logic-nodes';      // 控制節點 → 邏輯節點區域
                    break;
                case 'storage':
                    containerId = 'logic-nodes';      // 儲存節點 → 邏輯節點區域
                    break;
                case 'output':
                    containerId = 'action-nodes';     // 輸出節點 → 動作節點區域
                    break;
                default:
                    containerId = 'condition-nodes';  // 預設放在條件節點區域
            }
            
            const container = document.getElementById(containerId);
            if (container) {
                const item = this.createNodePaletteItem(nodeId, nodeType);
                container.appendChild(item);
            }
        });
        
        console.log('✅ 節點選板創建完成');
    }

    createNodePaletteItem(nodeId, nodeType) {
        const item = document.createElement('div');
        item.className = 'palette-node-item';
        item.style.cssText = `
            display: flex;
            align-items: center;
            padding: 8px 12px;
            margin: 4px 0;
            background: ${nodeType.color};
            color: white;
            border-radius: 4px;
            cursor: pointer;
            font-size: 12px;
            font-weight: 500;
            transition: all 0.2s ease;
        `;
        
        item.innerHTML = `
            <div class="icon" style="margin-right: 8px; width: 16px; height: 16px; background: rgba(255,255,255,0.3); border-radius: 50%;"></div>
            <span>${nodeType.name}</span>
        `;
        
        // 懸停效果
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
        
        // 根據節點類型添加適當的輸入/輸出接口
        switch (nodeType.category) {
            case 'input':
                // 輸入節點只有輸出
                node.addOutput('output', new Rete.ClassicPreset.Output(this.socket, '輸出'));
                break;
            case 'output':
                // 輸出節點只有輸入
                node.addInput('input', new Rete.ClassicPreset.Input(this.socket, '輸入'));
                break;
            case 'control':
                // 控制節點有輸入和輸出
                node.addInput('input', new Rete.ClassicPreset.Input(this.socket, '輸入'));
                node.addOutput('output', new Rete.ClassicPreset.Output(this.socket, '輸出'));
                // 決策節點有額外的 Yes/No 輸出
                if (nodeId === 'decision') {
                    node.addOutput('yes', new Rete.ClassicPreset.Output(this.socket, 'Yes'));
                    node.addOutput('no', new Rete.ClassicPreset.Output(this.socket, 'No'));
                }
                break;
            default:
                // 處理和儲存節點有輸入和輸出
                node.addInput('input', new Rete.ClassicPreset.Input(this.socket, '輸入'));
                node.addOutput('output', new Rete.ClassicPreset.Output(this.socket, '輸出'));
                break;
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
                    
                    console.log('開始從輸出建立連接:', socket.dataset.nodeId);
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
        
        // 儲存連接資訊以供更新
        this.connections.set(connectionId, {
            startSocket: startSocket,
            endSocket: endSocket,
            startNodeId: startSocket.dataset.nodeId,
            endNodeId: endSocket.dataset.nodeId
        });
        
        // 創建 SVG 連接線
        const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svg.style.position = 'absolute';
        svg.style.pointerEvents = 'none';
        svg.style.zIndex = '1';
        svg.id = `connection-${connectionId}`;
        svg.classList.add('rete-connection');
        
        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('stroke', '#666');
        path.setAttribute('stroke-width', '2');
        path.setAttribute('fill', 'none');
        path.style.cursor = 'pointer';
        path.style.pointerEvents = 'auto';
        
        // 點擊選擇連接
        path.addEventListener('click', (e) => {
            e.stopPropagation();
            this.selectConnection(connectionId, svg, path);
        });
        
        svg.appendChild(path);
        container.appendChild(svg);
        
        // 更新連接路徑
        this.updateConnectionPath(connectionId);
        
        console.log(`🔗 連接 ${connectionId} 已視覺化渲染`);
    }

    updateConnectionPath(connectionId) {
        const connectionData = this.connections.get(connectionId);
        if (!connectionData) return;
        
        const { startSocket, endSocket } = connectionData;
        const svg = document.getElementById(`connection-${connectionId}`);
        const path = svg?.querySelector('path');
        
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
        let pathData = '';
        let pathPoints = [];
        
        switch (this.connectionMode) {
            case 'orthogonal':
                pathData = this.generateOrthogonalPath(startX, startY, endX, endY);
                pathPoints = this.getOrthogonalPathPoints(startX, startY, endX, endY);
                break;
            case 'curved':
                pathData = this.generateCurvedPath(startX, startY, endX, endY);
                pathPoints = this.getCurvedPathPoints(startX, startY, endX, endY);
                break;
            case 'straight':
                pathData = this.generateStraightPath(startX, startY, endX, endY);
                pathPoints = [{ x: startX, y: startY }, { x: endX, y: endY }];
                break;
            default:
                pathData = this.generateOrthogonalPath(startX, startY, endX, endY);
                pathPoints = this.getOrthogonalPathPoints(startX, startY, endX, endY);
        }
        
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
        for (const [connectionId, connectionData] of this.connections) {
            if (connectionData.startNodeId === nodeId || connectionData.endNodeId === nodeId) {
                this.updateConnectionPath(connectionId);
            }
        }
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
            panel.style.display = 'block';
            
            // 填充基本屬性
            const nameInput = document.getElementById('node-name');
            const descInput = document.getElementById('node-description');
            
            if (nameInput) nameInput.value = node.data.name || '';
            if (descInput) descInput.value = node.data.description || '';
            
            // 動態生成參數編輯界面
            this.updateParametersPanel(node);
        }
    }

    updateParametersPanel(node) {
        console.log('🔧 更新參數面板:', node.data.type, node.data.parameters);
        
        // 隱藏所有參數區域
        document.getElementById('condition-parameters').style.display = 'none';
        document.getElementById('action-parameters').style.display = 'none';
        
        // 獲取節點配置
        const nodeConfig = node.data.nodeConfig;
        if (!nodeConfig || !nodeConfig.parameters || nodeConfig.parameters.length === 0) {
            console.log('📝 節點無參數配置');
            return;
        }
        
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
                
                // 生成參數編輯界面
                nodeConfig.parameters.forEach(param => {
                    const paramElement = this.createParameterInput(param, node);
                    dynamicParamsContainer.appendChild(paramElement);
                });
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
    
    createParameterInput(param, node) {
        const field = document.createElement('div');
        field.className = 'field';
        
        const label = document.createElement('label');
        label.className = 'label';
        label.textContent = param.description || param.name;
        if (param.required) {
            label.innerHTML += ' <span style="color: red;">*</span>';
        }
        
        const control = document.createElement('div');
        control.className = 'control';
        
        let input;
        const currentValue = node.data.parameters[param.name] || param.value || param.default || '';
        
        // 根據參數類型創建不同的輸入控件
        switch (param.type) {
            case 'integer':
            case 'number':
                input = document.createElement('input');
                input.className = 'input';
                input.type = 'number';
                input.value = currentValue;
                if (param.min !== null) input.min = param.min;
                if (param.max !== null) input.max = param.max;
                break;
                
            case 'boolean':
                const checkboxContainer = document.createElement('label');
                checkboxContainer.className = 'checkbox';
                
                input = document.createElement('input');
                input.type = 'checkbox';
                input.checked = currentValue === true || currentValue === 'true';
                
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
                input.value = Array.isArray(currentValue) ? currentValue.join('\n') : currentValue;
                break;
                
            case 'object':
                input = document.createElement('textarea');
                input.className = 'textarea';
                input.rows = 4;
                input.placeholder = 'JSON 格式的物件';
                input.value = typeof currentValue === 'object' ? 
                             JSON.stringify(currentValue, null, 2) : currentValue;
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
                        if (option === currentValue) {
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
                    input.value = currentValue;
                    input.placeholder = param.description || param.name;
                }
                break;
        }
        
        // 添加輸入變更事件監聽器
        if (input && param.type !== 'boolean') {
            input.addEventListener('change', (e) => {
                this.updateNodeParameter(node, param.name, e.target.value, param.type);
            });
            control.appendChild(input);
        } else if (param.type === 'boolean') {
            input.addEventListener('change', (e) => {
                this.updateNodeParameter(node, param.name, e.target.checked, param.type);
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
        
        // 更新節點參數
        node.data.parameters[paramName] = convertedValue;
        
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
            panel.style.display = 'none';
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
        if (confirm('確定要建立新流程？目前的變更將會遺失。')) {
            this.clearEditor();
            this.currentFlowName = '未命名';
            this.updateStatusBar();
            this.showNotification('已建立新流程', 'success');
        }
    }

    loadFlow() {
        const input = document.createElement('input');
        input.type = 'file';
        input.accept = '.json';
        input.onchange = (e) => {
            const file = e.target.files[0];
            if (file) {
                const reader = new FileReader();
                reader.onload = (e) => {
                    try {
                        const data = JSON.parse(e.target.result);
                        this.importFlow(data);
                        this.showNotification('流程載入成功', 'success');
                    } catch (error) {
                        console.error('載入流程失敗:', error);
                        this.showNotification('載入流程失敗', 'danger');
                    }
                };
                reader.readAsText(file);
            }
        };
        input.click();
    }

    saveFlow() {
        const data = this.exportFlow();
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${this.currentFlowName}.json`;
        a.click();
        URL.revokeObjectURL(url);
        this.showNotification('流程已儲存', 'success');
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
                        category: nodeType?.category || 'input'
                    };
                    
                    // 添加 socket
                    if (nodeType) {
                        this.addNodeSockets(node, nodeType, nodeData.type);
                    }
                    
                    await this.editor.addNode(node);
                    
                    // 設置位置
                    const x = nodeData.position?.x || 250;
                    const y = nodeData.position?.y || 200;
                    
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

    confirmFlowAction() {
        // 實現流程動作確認邏輯
        this.closeFlowModal();
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
}

// 頁面載入完成後初始化
document.addEventListener('DOMContentLoaded', () => {
    console.log('🚀 正在初始化 WCS Flow Designer...');
    window.flowDesigner = new WcsFlowDesigner();
});

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