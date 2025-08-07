/**
 * WCS Flow Designer
 * 可折疊節點選板版本，改善組織和可用性
 */

class FlowDesigner {
    constructor() {
        this.nodes = [];
        this.connections = [];
        this.selectedNode = null;
        this.selectedConnection = null;
        this.currentFlowName = 'untitled';
        this.isDragging = false;
        this.dragNode = null;
        this.dragOffset = { x: 0, y: 0 };
        this.canvas = null;
        this.ctx = null;
        this.scale = 1;
        this.offset = { x: 0, y: 0 };
        
        // 連線模式相關
        this.isConnecting = false;
        this.connectionStart = null;
        this.tempConnection = null;
        
        // 平移模式相關
        this.isPanning = false;
        this.panStart = { x: 0, y: 0 };
        this.spacePressed = false;
        
        // 節點定義庫 - 動態載入，初始使用預設定義
        this.nodeDefinitions = {};
        
        // 預設節點定義（備用）
        this.defaultNodeDefinitions = {
            // 條件節點 - 判斷邏輯
            'check_rack_a_space': {
                type: 'condition',
                name: '檢查架台A面',
                description: '檢查架台A面是否有可用空位',
                icon: 'A',
                color: '#3498db',
                inputs: { 
                    rack_id: { type: 'string', description: '架台編號' }
                },
                outputs: { 
                    has_space: { type: 'boolean', description: '有空位' },
                    no_space: { type: 'boolean', description: '無空位' }
                }
            },
            'check_rack_b_space': {
                type: 'condition', 
                name: '檢查架台B面',
                description: '檢查架台B面是否有可用空位',
                icon: 'B',
                color: '#3498db',
                inputs: { 
                    rack_id: { type: 'string', description: '架台編號' }
                },
                outputs: {
                    has_space: { type: 'boolean', description: '有空位' },
                    no_space: { type: 'boolean', description: '無空位' }
                }
            },
            'check_rack_status': {
                type: 'condition',
                name: '架台狀態',
                description: '檢查架台的整體運行狀態',
                icon: '◉',
                color: '#3498db',
                inputs: { 
                    rack_id: { type: 'string', description: '架台編號' }
                },
                outputs: {
                    ready: { type: 'boolean', description: '就緒' },
                    busy: { type: 'boolean', description: '忙碌' },
                    error: { type: 'boolean', description: '故障' }
                }
            },
            'check_agv_status': {
                type: 'condition',
                name: 'AGV狀態',
                description: '檢查AGV車輛的當前狀態',
                icon: '▣',
                color: '#3498db',
                inputs: { 
                    agv_id: { type: 'string', description: 'AGV編號' }
                },
                outputs: {
                    idle: { type: 'boolean', description: '閒置' },
                    working: { type: 'boolean', description: '工作中' },
                    charging: { type: 'boolean', description: '充電中' }
                }
            },
            'check_material': {
                type: 'condition',
                name: '物料檢查',
                description: '檢查物料是否存在或符合條件',
                icon: '◐',
                color: '#3498db',
                inputs: { 
                    location: { type: 'string', description: '位置' },
                    material_type: { type: 'string', description: '物料類型' }
                },
                outputs: {
                    exists: { type: 'boolean', description: '存在' },
                    not_exists: { type: 'boolean', description: '不存在' }
                }
            },
            
            // 動作節點 - 執行操作
            'rotate_rack': {
                type: 'action',
                name: '旋轉架台',
                description: '將架台旋轉180度切換A/B面',
                icon: '↻',
                color: '#27ae60',
                inputs: { 
                    rack_id: { type: 'string', description: '架台編號' },
                    trigger: { type: 'any', description: '觸發信號' }
                },
                outputs: { 
                    success: { type: 'boolean', description: '成功' },
                    failed: { type: 'boolean', description: '失敗' }
                }
            },
            'create_agv_task': {
                type: 'action',
                name: '創建AGV任務',
                description: '創建並分配AGV運輸任務',
                icon: '▶',
                color: '#27ae60',
                inputs: {
                    agv_id: { type: 'string', description: 'AGV編號' },
                    task_type: { type: 'string', description: '任務類型' },
                    from: { type: 'string', description: '起點' },
                    to: { type: 'string', description: '終點' }
                },
                outputs: {
                    task_id: { type: 'string', description: '任務ID' },
                    error: { type: 'boolean', description: '錯誤' }
                }
            },
            'move_agv': {
                type: 'action',
                name: '移動AGV',
                description: '控制AGV移動到指定位置',
                icon: '→',
                color: '#27ae60',
                inputs: {
                    agv_id: { type: 'string', description: 'AGV編號' },
                    target: { type: 'string', description: '目標位置' }
                },
                outputs: {
                    arrived: { type: 'boolean', description: '已到達' },
                    failed: { type: 'boolean', description: '失敗' }
                }
            },
            'pick_material': {
                type: 'action',
                name: '取料操作',
                description: 'AGV從指定位置取出物料',
                icon: '↑',
                color: '#27ae60',
                inputs: {
                    agv_id: { type: 'string', description: 'AGV編號' },
                    location: { type: 'string', description: '取料位置' }
                },
                outputs: {
                    success: { type: 'boolean', description: '成功' },
                    failed: { type: 'boolean', description: '失敗' }
                }
            },
            'place_material': {
                type: 'action',
                name: '放料操作',
                description: 'AGV將物料放置到指定位置',
                icon: '↓',
                color: '#27ae60',
                inputs: {
                    agv_id: { type: 'string', description: 'AGV編號' },
                    location: { type: 'string', description: '放料位置' }
                },
                outputs: {
                    success: { type: 'boolean', description: '成功' },
                    failed: { type: 'boolean', description: '失敗' }
                }
            },
            'notify_system': {
                type: 'action',
                name: '系統通知',
                description: '發送系統通知或警報',
                icon: '!',
                color: '#27ae60',
                inputs: {
                    message: { type: 'string', description: '訊息內容' },
                    level: { type: 'string', description: '級別' }
                },
                outputs: {
                    sent: { type: 'boolean', description: '已發送' }
                }
            },
            
            // 邏輯節點 - 流程控制
            'wait_time': {
                type: 'logic',
                name: '等待時間',
                description: '等待指定的時間長度',
                icon: '⏱',
                color: '#e74c3c',
                inputs: {
                    duration: { type: 'number', description: '時間(秒)' },
                    trigger: { type: 'any', description: '觸發' }
                },
                outputs: {
                    done: { type: 'boolean', description: '完成' }
                }
            },
            'wait_condition': {
                type: 'logic',
                name: '等待條件',
                description: '等待特定條件滿足',
                icon: '⏸',
                color: '#e74c3c',
                inputs: {
                    condition: { type: 'any', description: '條件' },
                    timeout: { type: 'number', description: '超時(秒)' }
                },
                outputs: {
                    satisfied: { type: 'boolean', description: '滿足' },
                    timeout: { type: 'boolean', description: '超時' }
                }
            },
            'parallel_split': {
                type: 'logic',
                name: '並行分支',
                description: '將流程分成多個並行執行的分支',
                icon: '⋈',
                color: '#e74c3c',
                inputs: {
                    trigger: { type: 'any', description: '觸發' }
                },
                outputs: {
                    branch1: { type: 'any', description: '分支1' },
                    branch2: { type: 'any', description: '分支2' },
                    branch3: { type: 'any', description: '分支3' }
                }
            },
            'parallel_join': {
                type: 'logic',
                name: '並行合併',
                description: '等待所有並行分支完成後合併',
                icon: '⋀',
                color: '#e74c3c',
                inputs: {
                    input1: { type: 'any', description: '輸入1' },
                    input2: { type: 'any', description: '輸入2' },
                    input3: { type: 'any', description: '輸入3' }
                },
                outputs: {
                    merged: { type: 'any', description: '合併輸出' }
                }
            },
            'loop': {
                type: 'logic',
                name: '循環',
                description: '重複執行指定次數或直到條件滿足',
                icon: '◯',
                color: '#e74c3c',
                inputs: {
                    count: { type: 'number', description: '次數' },
                    condition: { type: 'any', description: '條件' }
                },
                outputs: {
                    iterate: { type: 'any', description: '迭代' },
                    complete: { type: 'boolean', description: '完成' }
                }
            },
            
            // 腳本節點 - 自定義邏輯
            'custom_script': {
                type: 'script',
                name: '自定義腳本',
                description: '執行自定義Python腳本',
                icon: '{ }',
                color: '#f39c12',
                inputs: {
                    params: { type: 'object', description: '參數' }
                },
                outputs: {
                    result: { type: 'any', description: '結果' },
                    error: { type: 'boolean', description: '錯誤' }
                }
            },
            'api_call': {
                type: 'script',
                name: 'API調用',
                description: '調用外部API接口',
                icon: '☁',
                color: '#f39c12',
                inputs: {
                    endpoint: { type: 'string', description: 'API端點' },
                    method: { type: 'string', description: '方法' },
                    data: { type: 'object', description: '數據' }
                },
                outputs: {
                    response: { type: 'object', description: '響應' },
                    error: { type: 'boolean', description: '錯誤' }
                }
            }
        };
    }

    async initialize() {
        try {
            console.log('🚀 初始化 Flow Designer V2 Collapsible...');
            
            // 獲取容器
            this.container = document.getElementById('rete-editor');
            if (!this.container) {
                throw new Error('找不到編輯器容器 #rete-editor');
            }
            
            // 創建或獲取畫布
            this.canvas = document.getElementById('flow-canvas');
            if (!this.canvas) {
                this.canvas = document.createElement('canvas');
                this.canvas.id = 'flow-canvas';
                this.canvas.className = 'flow-canvas-layer';
                this.container.appendChild(this.canvas);
            }
            
            this.ctx = this.canvas.getContext('2d');
            if (!this.ctx) {
                throw new Error('無法獲取 Canvas 2D 上下文');
            }
            
            // 調整畫布大小
            this.resizeCanvas();
            
            // 初始化組件
            this.setupEventListeners();
            
            // 載入節點定義（從 API 或使用預設）
            await this.loadNodeDefinitions();
            
            this.initializeNodePalette();
            this.initializeCollapsibleSections();
            this.initializeSaveButton();
            this.initializePropertiesPanel();
            
            // 初始流程資料載入現在由模板處理
            
            // 開始渲染循環
            this.startRenderLoop();
            
            console.log('✅ Flow Designer V2 初始化完成');
        } catch (error) {
            console.error('❌ Flow Designer 初始化錯誤:', error);
            throw error;
        }
    }

    async loadNodeDefinitions() {
        console.log('🔄 開始載入節點定義...');
        
        try {
            // 嘗試從 API 載入節點定義
            const response = await fetch('/api/nodes/definitions');
            
            if (response.ok) {
                const data = await response.json();
                // API 返回格式: {nodes: {...}, count: 25}
                if (data.nodes) {
                    console.log(`✅ 從 API 載入節點定義成功，共 ${data.count || Object.keys(data.nodes).length} 個節點`);
                    // 直接使用 API 返回的節點（已經是正確格式）
                    this.nodeDefinitions = data.nodes;
                    console.log('載入的節點:', Object.keys(this.nodeDefinitions));
                    return;
                }
            }
        } catch (error) {
            console.warn('⚠️ 無法從 API 載入節點定義:', error);
        }
        
        // 如果 API 載入失敗，嘗試從當前流程的 YAML 提取節點定義
        if (window.INITIAL_FLOW_DATA && window.INITIAL_FLOW_DATA.data) {
            try {
                const flowData = typeof window.INITIAL_FLOW_DATA.data === 'string' 
                    ? JSON.parse(window.INITIAL_FLOW_DATA.data)
                    : window.INITIAL_FLOW_DATA.data;
                    
                if (flowData.nodes) {
                    console.log('📂 從當前流程 YAML 提取節點定義');
                    this.extractNodeDefinitionsFromFlow(flowData.nodes);
                }
            } catch (error) {
                console.warn('⚠️ 無法從流程資料提取節點定義:', error);
            }
        }
        
        // 如果都失敗，使用預設定義
        if (Object.keys(this.nodeDefinitions).length === 0) {
            console.log('📋 使用預設節點定義');
            this.nodeDefinitions = this.defaultNodeDefinitions;
        }
        
        console.log(`✅ 載入了 ${Object.keys(this.nodeDefinitions).length} 個節點定義`);
    }
    
    processNodeDefinitions(apiNodes) {
        // 處理從 API 返回的節點定義格式
        const definitions = {};
        
        // 處理各類別節點
        ['action_nodes', 'condition_nodes', 'logic_nodes', 'script_nodes'].forEach(category => {
            if (apiNodes[category]) {
                apiNodes[category].forEach(node => {
                    // 使用原始的 key（function 名稱），而不是從 name 生成
                    const key = node.key || this.generateNodeKey(node.name);
                    definitions[key] = {
                        type: node.category,
                        name: node.name,
                        description: node.description,
                        icon: node.icon || this.getDefaultIcon(node.category),
                        color: node.color || this.getDefaultColor(node.category),
                        inputs: this.convertPortsToObject(node.inputs),
                        outputs: this.convertPortsToObject(node.outputs)
                    };
                });
            }
        });
        
        console.log(`✅ 處理了 ${Object.keys(definitions).length} 個節點定義`);
        console.log('節點列表:', Object.keys(definitions));
        
        return definitions;
    }
    
    extractNodeDefinitionsFromFlow(nodes) {
        // 從流程的節點中提取節點定義
        const definitions = {};
        
        nodes.forEach(node => {
            const key = node.function || node.id;
            if (!definitions[key]) {
                definitions[key] = {
                    type: node.type || 'action',
                    name: node.name || key,
                    description: node.description || '',
                    icon: this.getDefaultIcon(node.type),
                    color: this.getDefaultColor(node.type),
                    inputs: node.inputs || {},
                    outputs: node.outputs || {}
                };
            }
        });
        
        // 合併到現有定義中
        this.nodeDefinitions = { ...this.nodeDefinitions, ...definitions };
    }
    
    generateNodeKey(name) {
        // 將節點名稱轉換為鍵值（移除空格，轉小寫）
        return name.toLowerCase().replace(/\s+/g, '_').replace(/[^\w_]/g, '');
    }
    
    getDefaultIcon(type) {
        const icons = {
            'condition': '◇',  // 菱形表示判斷
            'action': '▷',     // 三角形表示執行
            'logic': '⊙',      // 圓圈表示邏輯
            'script': '{ }'    // 大括號表示腳本
        };
        return icons[type] || '■';  // 方塊作為預設
    }
    
    getDefaultColor(type) {
        const colors = {
            'condition': '#3498db',
            'action': '#2ecc71',
            'logic': '#e74c3c',
            'script': '#f39c12'
        };
        return colors[type] || '#95a5a6';
    }
    
    convertPortsToObject(ports) {
        // 將端口陣列轉換為物件格式
        if (!ports) return {};
        
        if (Array.isArray(ports)) {
            const obj = {};
            ports.forEach(port => {
                const key = port.key || port;
                obj[key] = {
                    type: port.type || 'any',
                    description: port.description || ''
                };
            });
            return obj;
        }
        
        return ports;
    }

    initializeCollapsibleSections() {
        // 設置所有可折疊區塊的點擊事件
        const headers = document.querySelectorAll('.palette-section-header');
        headers.forEach(header => {
            header.addEventListener('click', (e) => {
                e.preventDefault();
                header.classList.toggle('is-collapsed');
                
                // 旋轉圖標
                const icon = header.querySelector('.icon i');
                if (icon) {
                    if (header.classList.contains('is-collapsed')) {
                        icon.className = 'mdi mdi-chevron-right';
                    } else {
                        icon.className = 'mdi mdi-chevron-down';
                    }
                }
            });
        });
        
        console.log('✅ 折疊區塊初始化完成');
    }

    initializeNodePalette() {
        const categories = ['condition', 'action', 'logic', 'script'];
        
        categories.forEach(category => {
            const container = document.getElementById(`${category}-nodes`);
            if (!container) {
                console.warn(`找不到節點容器: ${category}-nodes`);
                return;
            }
            
            container.innerHTML = '';
            let nodeCount = 0;
            
            Object.entries(this.nodeDefinitions).forEach(([key, nodeData]) => {
                if (nodeData.type !== category) return;
                
                nodeCount++;
                const nodeElement = this.createPaletteNode(key, nodeData);
                container.appendChild(nodeElement);
            });
            
            // 更新節點數量徽章
            const section = document.querySelector(`.palette-section[data-category="${category}"]`);
            if (section) {
                const badge = section.querySelector('.badge');
                if (badge) {
                    badge.textContent = nodeCount;
                }
            }
        });
        
        console.log('✅ 節點選板初始化完成');
    }

    createPaletteNode(key, nodeData) {
        const nodeElement = document.createElement('div');
        nodeElement.className = 'palette-node';
        nodeElement.draggable = true;
        nodeElement.dataset.nodeType = key;
        nodeElement.dataset.nodeCategory = nodeData.type;
        
        // 截短描述
        const description = nodeData.description || '無描述';
        const shortDesc = description.length > 25 ? 
            description.substring(0, 25) + '...' : description;
        
        nodeElement.innerHTML = `
            <span class="icon" style="color: ${nodeData.color || '#34495e'};">
                ${nodeData.icon || '◆'}
            </span>
            <div class="node-info">
                <span class="name">${nodeData.name}</span>
                <span class="desc">${shortDesc}</span>
            </div>
        `;
        
        // 添加完整描述的提示
        nodeElement.title = `${nodeData.name}\n${description}`;
        
        // 拖拽事件
        nodeElement.addEventListener('dragstart', (e) => {
            e.dataTransfer.effectAllowed = 'copy';
            e.dataTransfer.setData('text/plain', key);
            e.dataTransfer.setData('nodeType', key);
            e.dataTransfer.setData('nodeCategory', nodeData.type);
            nodeElement.classList.add('is-dragging');
        });
        
        nodeElement.addEventListener('dragend', () => {
            nodeElement.classList.remove('is-dragging');
        });
        
        return nodeElement;
    }

    initializeSaveButton() {
        const saveButton = document.getElementById('btn-save-flow');
        if (saveButton) {
            saveButton.addEventListener('click', () => this.saveFlow());
        }
        
        // Ctrl+S 快捷鍵
        document.addEventListener('keydown', (e) => {
            if ((e.ctrlKey || e.metaKey) && e.key === 's') {
                e.preventDefault();
                this.saveFlow();
            }
        });
    }

    initializePropertiesPanel() {
        const closeBtn = document.getElementById('close-properties');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => {
                const panel = document.getElementById('properties-panel');
                if (panel) {
                    panel.classList.remove('is-active');
                }
            });
        }
    }

    resizeCanvas() {
        const rect = this.container.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;
    }

    setupEventListeners() {
        // Canvas 事件
        this.canvas.addEventListener('mousedown', this.handleMouseDown.bind(this));
        this.canvas.addEventListener('mousemove', this.handleMouseMove.bind(this));
        this.canvas.addEventListener('mouseup', this.handleMouseUp.bind(this));
        this.canvas.addEventListener('wheel', this.handleWheel.bind(this));
        this.canvas.addEventListener('dblclick', this.handleDoubleClick.bind(this));
        
        // 拖放事件
        this.canvas.addEventListener('dragover', (e) => {
            e.preventDefault();
            e.stopPropagation();
            e.dataTransfer.dropEffect = 'copy';
            this.container.classList.add('drop-zone-active');
        });
        
        this.canvas.addEventListener('dragleave', (e) => {
            e.preventDefault();
            this.container.classList.remove('drop-zone-active');
        });
        
        this.canvas.addEventListener('drop', (e) => {
            e.preventDefault();
            e.stopPropagation();
            this.container.classList.remove('drop-zone-active');
            
            const nodeType = e.dataTransfer.getData('nodeType') || 
                           e.dataTransfer.getData('text/plain') ||
                           e.dataTransfer.getData('text');
            
            if (nodeType && this.nodeDefinitions[nodeType]) {
                const rect = this.canvas.getBoundingClientRect();
                const x = (e.clientX - rect.left - this.offset.x) / this.scale;
                const y = (e.clientY - rect.top - this.offset.y) / this.scale;
                
                this.addNode(nodeType, x, y);
                console.log('✅ 節點已添加:', nodeType);
            }
        });
        
        // 鍵盤事件
        document.addEventListener('keydown', this.handleKeyDown.bind(this));
        document.addEventListener('keyup', this.handleKeyUp.bind(this));
        
        // 視窗調整
        window.addEventListener('resize', () => {
            this.resizeCanvas();
            this.render();
        });
    }

    startRenderLoop() {
        const render = () => {
            this.render();
            requestAnimationFrame(render);
        };
        requestAnimationFrame(render);
    }

    render() {
        if (!this.ctx) return;
        
        // 清空畫布
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 儲存狀態
        this.ctx.save();
        
        // 應用變換
        this.ctx.translate(this.offset.x, this.offset.y);
        this.ctx.scale(this.scale, this.scale);
        
        // 繪製網格
        this.drawGrid();
        
        // 繪製連接線
        this.connections.forEach(conn => {
            this.drawConnection(conn);
        });
        
        // 繪製臨時連接線
        if (this.isConnecting && this.tempConnection && this.tempConnection.start) {
            this.drawTempConnection();
        }
        
        // 繪製節點
        this.nodes.forEach(node => {
            this.drawNode(node);
        });
        
        // 恢復狀態
        this.ctx.restore();
        
        // 更新狀態欄
        this.updateStatusBar();
    }

    drawGrid() {
        const gridSize = 20;
        const width = this.canvas.width / this.scale;
        const height = this.canvas.height / this.scale;
        const offsetX = -this.offset.x / this.scale;
        const offsetY = -this.offset.y / this.scale;
        
        this.ctx.strokeStyle = 'rgba(220, 220, 220, 0.3)';
        this.ctx.lineWidth = 0.5;
        
        for (let x = -offsetX % gridSize; x < width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x + offsetX, offsetY);
            this.ctx.lineTo(x + offsetX, height + offsetY);
            this.ctx.stroke();
        }
        
        for (let y = -offsetY % gridSize; y < height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(offsetX, y + offsetY);
            this.ctx.lineTo(width + offsetX, y + offsetY);
            this.ctx.stroke();
        }
    }

    drawNode(node) {
        const { x, y, width } = node;
        const definition = this.nodeDefinitions[node.nodeDefId || node.type];
        
        // 計算高度（考慮資料類型標籤的空間）
        const inputCount = Object.keys(node.inputs || {}).length;
        const outputCount = Object.keys(node.outputs || {}).length;
        const portCount = Math.max(inputCount, outputCount);
        const minHeight = 60;
        const portHeight = 30;  // 增加端口高度以容納資料類型標籤
        const actualHeight = Math.max(minHeight, portCount * portHeight + 40);
        node.height = actualHeight;
        
        // 根據節點類型定義顏色
        const categoryColors = {
            'condition': '#3498db', // 藍色 - 條件節點
            'action': '#27ae60',    // 綠色 - 動作節點  
            'logic': '#e74c3c',     // 紅色 - 邏輯節點
            'script': '#f39c12'     // 橘色 - 腳本節點
        };
        const nodeColor = categoryColors[node.type] || definition?.color || '#667eea';
        
        // 繪製節點背景
        const gradient = this.ctx.createLinearGradient(x, y, x, y + actualHeight);
        if (this.selectedNode === node) {
            // 選中節點使用亮白色背景，不使用透明效果
            gradient.addColorStop(0, 'rgba(255, 255, 255, 0.98)');
            gradient.addColorStop(1, 'rgba(248, 250, 255, 0.95)');
        } else {
            gradient.addColorStop(0, 'rgba(255, 255, 255, 0.95)');
            gradient.addColorStop(1, 'rgba(250, 250, 250, 0.9)');
        }
        
        this.ctx.fillStyle = gradient;
        
        // 設置外框顏色
        let strokeColor = nodeColor;
        if (this.selectedNode === node) {
            // 選中時使用更粗的邊框
            this.ctx.strokeStyle = strokeColor;
            this.ctx.lineWidth = 3;
        } else {
            // 未選中時使用較淡的顏色
            this.ctx.strokeStyle = `${strokeColor}88`;  // 加透明度
            this.ctx.lineWidth = 1.5;
        }
        
        // 繪製圓角矩形
        this.roundRect(x, y, width, actualHeight, 8);
        this.ctx.fill();
        this.ctx.stroke();
        
        // 繪製節點頭部（使用節點類型顏色）
        const headerGradient = this.ctx.createLinearGradient(x, y, x, y + 30);
        headerGradient.addColorStop(0, `${nodeColor}33`);
        headerGradient.addColorStop(1, `${nodeColor}11`);
        
        this.ctx.fillStyle = headerGradient;
        this.ctx.beginPath();
        this.ctx.moveTo(x + 8, y);
        this.ctx.lineTo(x + width - 8, y);
        this.ctx.quadraticCurveTo(x + width, y, x + width, y + 8);
        this.ctx.lineTo(x + width, y + 30);
        this.ctx.lineTo(x, y + 30);
        this.ctx.lineTo(x, y + 8);
        this.ctx.quadraticCurveTo(x, y, x + 8, y);
        this.ctx.closePath();
        this.ctx.fill();
        
        // 繪製圖標和標題（使用節點類型顏色）
        this.ctx.fillStyle = nodeColor;
        this.ctx.font = 'bold 16px "Segoe UI", system-ui, sans-serif';
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';
        this.ctx.fillText(definition?.icon || '◆', x + 20, y + 15);
        
        this.ctx.fillStyle = '#2c3e50';
        this.ctx.font = 'bold 12px "Segoe UI", system-ui, sans-serif';
        this.ctx.textAlign = 'left';
        this.ctx.fillText(node.name || definition?.name || '未命名', x + 35, y + 15);
        
        // 繪製端口
        this.drawPorts(node, definition);
    }

    drawPorts(node, definition) {
        const { x, y, width } = node;
        const portRadius = 5;
        
        // 資料類型對應顏色（調整為更易區分的顏色）
        const typeColors = {
            'string': '#e74c3c',      // 紅色 - 字串
            'integer': '#2ecc71',     // 綠色 - 整數（原本藍色改為綠色）
            'boolean': '#f39c12',     // 橘色 - 布林（原本紫色改為橘色）
            'array': '#9b59b6',       // 紫色 - 陣列（原本橘色改為紫色）
            'object': '#3498db',      // 藍色 - 物件（原本青色改為藍色）
            'any': '#95a5a6'          // 灰色 - 任意類型
        };
        
        // 資料類型圖標
        const typeIcons = {
            'string': '"T"',
            'integer': '#',
            'boolean': '◈',
            'array': '[]',
            'object': '{}',
            'any': '◆'
        };
        
        // 輸入端口
        const inputs = Object.entries(node.inputs || {});
        inputs.forEach(([portName, portInfo], index) => {
            const portY = y + 45 + index * 30;  // 調整間距以配合新的端口高度
            const dataType = portInfo?.type || 'any';
            const portColor = typeColors[dataType] || typeColors['any'];
            
            // 繪製端口圓圈（使用類型顏色）
            this.ctx.fillStyle = portColor;
            this.ctx.strokeStyle = '#ffffff';
            this.ctx.lineWidth = 1.5;
            this.ctx.beginPath();
            this.ctx.arc(x - 2, portY, portRadius, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
            
            // 繪製端口名稱
            this.ctx.fillStyle = '#2c3e50';
            this.ctx.font = 'bold 10px "Segoe UI", system-ui, sans-serif';
            this.ctx.textAlign = 'left';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(portName, x + 8, portY);
            
            // 繪製資料類型標籤
            this.ctx.fillStyle = portColor;
            this.ctx.font = '9px "Segoe UI", system-ui, sans-serif';
            this.ctx.fillText(`[${dataType}]`, x + 8, portY + 10);
        });
        
        // 輸出端口
        const outputs = Object.entries(node.outputs || {});
        outputs.forEach(([portName, portInfo], index) => {
            const portY = y + 45 + index * 30;  // 調整間距以配合新的端口高度
            const dataType = portInfo?.type || 'any';
            const portColor = typeColors[dataType] || typeColors['any'];
            
            // 繪製端口圓圈（使用類型顏色）
            this.ctx.fillStyle = portColor;
            this.ctx.strokeStyle = '#ffffff';
            this.ctx.lineWidth = 1.5;
            this.ctx.beginPath();
            this.ctx.arc(x + width + 2, portY, portRadius, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
            
            // 繪製端口名稱
            this.ctx.fillStyle = '#2c3e50';
            this.ctx.font = 'bold 10px "Segoe UI", system-ui, sans-serif';
            this.ctx.textAlign = 'right';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(portName, x + width - 8, portY);
            
            // 繪製資料類型標籤
            this.ctx.fillStyle = portColor;
            this.ctx.font = '9px "Segoe UI", system-ui, sans-serif';
            this.ctx.fillText(`[${dataType}]`, x + width - 8, portY + 10);
        });
    }

    drawConnection(connection) {
        const fromNode = this.nodes.find(n => n.id === connection.from.nodeId);
        const toNode = this.nodes.find(n => n.id === connection.to.nodeId);
        
        if (!fromNode || !toNode) return;
        
        const fromPortIndex = Object.keys(fromNode.outputs || {}).indexOf(connection.from.port);
        const toPortIndex = Object.keys(toNode.inputs || {}).indexOf(connection.to.port);
        
        const startX = fromNode.x + fromNode.width + 2;
        const startY = fromNode.y + 45 + fromPortIndex * 30;  // 配合新的端口間距
        const endX = toNode.x - 2;
        const endY = toNode.y + 45 + toPortIndex * 30;  // 配合新的端口間距
        
        const controlOffset = Math.abs(endX - startX) * 0.5;
        
        this.ctx.strokeStyle = this.selectedConnection === connection ? 
            '#e74c3c' : '#95a5a6';
        this.ctx.lineWidth = this.selectedConnection === connection ? 3 : 2;
        this.ctx.lineCap = 'round';
        
        this.ctx.beginPath();
        this.ctx.moveTo(startX, startY);
        this.ctx.bezierCurveTo(
            startX + controlOffset, startY,
            endX - controlOffset, endY,
            endX, endY
        );
        this.ctx.stroke();
        
        // 繪製箭頭
        const angle = Math.atan2(endY - startY, endX - startX);
        this.ctx.save();
        this.ctx.translate(endX, endY);
        this.ctx.rotate(angle);
        this.ctx.beginPath();
        this.ctx.moveTo(-8, -4);
        this.ctx.lineTo(0, 0);
        this.ctx.lineTo(-8, 4);
        this.ctx.stroke();
        this.ctx.restore();
    }

    drawTempConnection() {
        if (!this.tempConnection || !this.tempConnection.start) return;
        
        const startX = this.tempConnection.start.x;
        const startY = this.tempConnection.start.y;
        const endX = this.tempConnection.end.x;
        const endY = this.tempConnection.end.y;
        
        const controlOffset = Math.abs(endX - startX) * 0.5;
        
        this.ctx.strokeStyle = 'rgba(102, 126, 234, 0.5)';
        this.ctx.lineWidth = 2;
        this.ctx.setLineDash([5, 5]);
        
        this.ctx.beginPath();
        this.ctx.moveTo(startX, startY);
        this.ctx.bezierCurveTo(
            startX + controlOffset, startY,
            endX - controlOffset, endY,
            endX, endY
        );
        this.ctx.stroke();
        
        this.ctx.setLineDash([]);
    }

    roundRect(x, y, width, height, radius) {
        this.ctx.beginPath();
        this.ctx.moveTo(x + radius, y);
        this.ctx.lineTo(x + width - radius, y);
        this.ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
        this.ctx.lineTo(x + width, y + height - radius);
        this.ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
        this.ctx.lineTo(x + radius, y + height);
        this.ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
        this.ctx.lineTo(x, y + radius);
        this.ctx.quadraticCurveTo(x, y, x + radius, y);
        this.ctx.closePath();
    }

    handleMouseDown(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        const port = this.getPortAtPosition(x, y);
        if (port) {
            if (port.type === 'output') {
                this.startConnection(port, x, y);
            }
            return;
        }
        
        const node = this.getNodeAtPosition(x, y);
        if (node) {
            this.selectNode(node);
            this.isDragging = true;
            this.dragNode = node;
            this.dragOffset = {
                x: x - node.x,
                y: y - node.y
            };
        } else if (this.spacePressed) {
            this.isPanning = true;
            this.panStart = { x: e.clientX, y: e.clientY };
            this.canvas.style.cursor = 'grabbing';
        } else {
            const connection = this.getConnectionAtPosition(x, y);
            if (connection) {
                this.selectConnection(connection);
            } else {
                this.selectedNode = null;
                this.selectedConnection = null;
                this.updatePropertiesPanel();
            }
        }
    }

    handleMouseMove(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        if (this.isDragging && this.dragNode) {
            this.dragNode.x = x - this.dragOffset.x;
            this.dragNode.y = y - this.dragOffset.y;
        } else if (this.isPanning) {
            const dx = e.clientX - this.panStart.x;
            const dy = e.clientY - this.panStart.y;
            this.offset.x += dx;
            this.offset.y += dy;
            this.panStart = { x: e.clientX, y: e.clientY };
        } else if (this.isConnecting && this.tempConnection) {
            this.tempConnection.end = { x, y };
        } else {
            const port = this.getPortAtPosition(x, y);
            const node = this.getNodeAtPosition(x, y);
            const connection = this.getConnectionAtPosition(x, y);
            
            if (port) {
                this.canvas.style.cursor = 'crosshair';
            } else if (node) {
                this.canvas.style.cursor = 'move';
            } else if (connection) {
                this.canvas.style.cursor = 'pointer';
            } else if (this.spacePressed) {
                this.canvas.style.cursor = 'grab';
            } else {
                this.canvas.style.cursor = 'default';
            }
        }
    }

    handleMouseUp(e) {
        if (this.isConnecting) {
            const rect = this.canvas.getBoundingClientRect();
            const x = (e.clientX - rect.left - this.offset.x) / this.scale;
            const y = (e.clientY - rect.top - this.offset.y) / this.scale;
            
            const port = this.getPortAtPosition(x, y);
            if (port && port.type === 'input') {
                this.completeConnection(port);
            } else {
                this.cancelConnection();
            }
        }
        
        this.isDragging = false;
        this.dragNode = null;
        this.isPanning = false;
        if (!this.spacePressed) {
            this.canvas.style.cursor = 'default';
        }
    }

    handleWheel(e) {
        e.preventDefault();
        
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        const newScale = Math.max(0.1, Math.min(3, this.scale * delta));
        
        const scaleChange = newScale - this.scale;
        this.offset.x -= (x - this.offset.x) * scaleChange / this.scale;
        this.offset.y -= (y - this.offset.y) * scaleChange / this.scale;
        
        this.scale = newScale;
    }

    handleDoubleClick(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        const node = this.getNodeAtPosition(x, y);
        if (node) {
            this.editNode(node);
        }
    }

    handleKeyDown(e) {
        if (e.code === 'Space' && !this.spacePressed) {
            e.preventDefault();
            this.spacePressed = true;
            this.canvas.style.cursor = 'grab';
        } else if (e.key === 'Delete' || e.key === 'Backspace') {
            if (this.selectedNode) {
                this.deleteNode(this.selectedNode);
            } else if (this.selectedConnection) {
                this.deleteConnection(this.selectedConnection);
            }
        }
    }

    handleKeyUp(e) {
        if (e.code === 'Space') {
            this.spacePressed = false;
            this.canvas.style.cursor = 'default';
        }
    }

    addNode(type, x, y) {
        const definition = this.nodeDefinitions[type];
        if (!definition) return;
        
        const node = {
            id: `node_${Date.now()}`,
            nodeDefId: type,  // 節點定義ID (如 'create_agv_task')
            type: definition.type,  // 節點類型 (如 'action')
            name: definition.name,
            x: x - 75,
            y: y - 30,
            width: 150,
            height: 100,
            inputs: definition.inputs || {},
            outputs: definition.outputs || {},
            parameters: {}
        };
        
        this.nodes.push(node);
        this.selectNode(node);
        
        console.log('節點已添加:', node);
    }

    selectNode(node) {
        this.selectedNode = node;
        this.selectedConnection = null;
        this.updatePropertiesPanel();
    }

    selectConnection(connection) {
        this.selectedConnection = connection;
        this.selectedNode = null;
        this.updatePropertiesPanel();
    }

    deleteNode(node) {
        this.connections = this.connections.filter(conn => 
            conn.from.nodeId !== node.id && conn.to.nodeId !== node.id
        );
        
        const index = this.nodes.indexOf(node);
        if (index > -1) {
            this.nodes.splice(index, 1);
        }
        
        this.selectedNode = null;
        this.updatePropertiesPanel();
    }

    deleteConnection(connection) {
        const index = this.connections.indexOf(connection);
        if (index > -1) {
            this.connections.splice(index, 1);
        }
        
        this.selectedConnection = null;
        this.updatePropertiesPanel();
    }

    editNode(node) {
        this.selectNode(node);
        const panel = document.getElementById('properties-panel');
        if (panel) {
            panel.classList.add('is-active');
        }
    }
    
    // 創建參數輸入欄位
    createParameterInput(paramName, paramType, currentValue) {
        const inputId = `param-${paramName}`;
        
        if (paramType === 'boolean') {
            return `
                <label class="checkbox">
                    <input type="checkbox" id="${inputId}" 
                           ${currentValue === 'true' || currentValue === true ? 'checked' : ''}>
                    ${paramName}
                </label>
            `;
        } else if (paramType === 'integer') {
            return `
                <input class="input" type="number" id="${inputId}" 
                       value="${currentValue}" placeholder="輸入整數">
            `;
        } else if (paramType === 'float') {
            return `
                <input class="input" type="number" step="0.01" id="${inputId}" 
                       value="${currentValue}" placeholder="輸入浮點數">
            `;
        } else {
            return `
                <input class="input" type="text" id="${inputId}" 
                       value="${currentValue}" placeholder="輸入 ${paramType}">
            `;
        }
    }
    
    // 創建輸入值欄位
    createInputValueField(portName, dataType, currentValue, description) {
        const inputId = `input-value-${portName}`;
        
        if (dataType === 'boolean') {
            return `
                <label class="checkbox">
                    <input type="checkbox" id="${inputId}" 
                           ${currentValue === 'true' || currentValue === true ? 'checked' : ''}>
                    ${portName}
                </label>
            `;
        } else if (dataType === 'integer') {
            return `
                <input class="input" type="number" id="${inputId}" 
                       value="${currentValue}" placeholder="輸入整數值">
            `;
        } else if (dataType === 'float') {
            return `
                <input class="input" type="number" step="0.01" id="${inputId}" 
                       value="${currentValue}" placeholder="輸入浮點數值 (例如: 3.14)">
            `;
        } else if (dataType === 'object' || dataType === 'array') {
            return `
                <textarea class="textarea" id="${inputId}" rows="3" 
                          placeholder="輸入 JSON 格式 (例如: ${dataType === 'array' ? '["item1", "item2"]' : '{"key": "value"}'})">${currentValue}</textarea>
            `;
        } else {
            // string, any, 或其他類型
            return `
                <input class="input" type="text" id="${inputId}" 
                       value="${currentValue}" placeholder="輸入 ${dataType} 值">
            `;
        }
    }
    
    // 取得參數的輸入類型
    getInputTypeForParameter(paramType) {
        const typeMap = {
            'integer': 'number',
            'float': 'number',
            'boolean': 'checkbox',
            'string': 'text',
            'object': 'textarea',
            'array': 'textarea'
        };
        return typeMap[paramType] || 'text';
    }
    
    // 綁定參數輸入事件
    bindParameterInputEvents() {
        if (!this.selectedNode) return;
        
        const nodeDefinition = this.nodeDefinitions[this.selectedNode.type] || {};
        const parameters = nodeDefinition.parameters || {};
        
        Object.entries(parameters).forEach(([paramName, paramType]) => {
            const inputId = `param-${paramName}`;
            const input = document.getElementById(inputId);
            
            if (input) {
                if (paramType === 'boolean') {
                    input.addEventListener('change', (e) => {
                        if (!this.selectedNode.parameters) {
                            this.selectedNode.parameters = {};
                        }
                        this.selectedNode.parameters[paramName] = e.target.checked;
                        this.draw();
                    });
                } else {
                    input.addEventListener('input', (e) => {
                        if (!this.selectedNode.parameters) {
                            this.selectedNode.parameters = {};
                        }
                        this.selectedNode.parameters[paramName] = e.target.value;
                        this.draw();
                    });
                }
            }
        });
    }
    
    // 綁定輸入值事件
    bindInputValueEvents() {
        if (!this.selectedNode) return;
        
        Object.entries(this.selectedNode.inputs || {}).forEach(([portName, portInfo]) => {
            const inputId = `input-value-${portName}`;
            const input = document.getElementById(inputId);
            
            if (input) {
                const dataType = portInfo?.type || portInfo || 'any';
                
                if (dataType === 'boolean') {
                    input.addEventListener('change', (e) => {
                        if (!this.selectedNode.inputValues) {
                            this.selectedNode.inputValues = {};
                        }
                        this.selectedNode.inputValues[portName] = e.target.checked;
                        this.draw();
                    });
                } else {
                    input.addEventListener('input', (e) => {
                        if (!this.selectedNode.inputValues) {
                            this.selectedNode.inputValues = {};
                        }
                        this.selectedNode.inputValues[portName] = e.target.value;
                        this.draw();
                    });
                }
            }
        });
    }

    startConnection(port, mouseX, mouseY) {
        this.isConnecting = true;
        this.connectionStart = port;
        this.tempConnection = {
            start: {
                x: port.node.x + port.node.width + 2,
                y: port.node.y + 40 + port.index * 25
            },
            end: { x: mouseX, y: mouseY }  // 使用當前鼠標位置而不是(0,0)
        };
    }

    completeConnection(endPort) {
        if (!this.connectionStart || !endPort) return;
        
        if (this.connectionStart.node.id === endPort.node.id) {
            this.cancelConnection();
            return;
        }
        
        const exists = this.connections.some(conn =>
            conn.from.nodeId === this.connectionStart.node.id &&
            conn.from.port === this.connectionStart.portName &&
            conn.to.nodeId === endPort.node.id &&
            conn.to.port === endPort.portName
        );
        
        if (!exists) {
            const connection = {
                id: `conn_${Date.now()}`,
                from: {
                    nodeId: this.connectionStart.node.id,
                    port: this.connectionStart.portName
                },
                to: {
                    nodeId: endPort.node.id,
                    port: endPort.portName
                }
            };
            
            this.connections.push(connection);
        }
        
        this.cancelConnection();
    }

    cancelConnection() {
        this.isConnecting = false;
        this.connectionStart = null;
        this.tempConnection = null;
    }

    getNodeAtPosition(x, y) {
        for (let i = this.nodes.length - 1; i >= 0; i--) {
            const node = this.nodes[i];
            if (x >= node.x && x <= node.x + node.width &&
                y >= node.y && y <= node.y + node.height) {
                return node;
            }
        }
        return null;
    }

    getPortAtPosition(x, y) {
        const threshold = 10;
        
        for (const node of this.nodes) {
            const inputs = Object.keys(node.inputs || {});
            for (let i = 0; i < inputs.length; i++) {
                const portX = node.x - 2;
                const portY = node.y + 40 + i * 25;
                
                if (Math.abs(x - portX) < threshold && Math.abs(y - portY) < threshold) {
                    return {
                        type: 'input',
                        node: node,
                        portName: inputs[i],
                        index: i
                    };
                }
            }
            
            const outputs = Object.keys(node.outputs || {});
            for (let i = 0; i < outputs.length; i++) {
                const portX = node.x + node.width + 2;
                const portY = node.y + 40 + i * 25;
                
                if (Math.abs(x - portX) < threshold && Math.abs(y - portY) < threshold) {
                    return {
                        type: 'output',
                        node: node,
                        portName: outputs[i],
                        index: i
                    };
                }
            }
        }
        
        return null;
    }

    getConnectionAtPosition(x, y) {
        for (const connection of this.connections) {
            if (this.isPointOnConnection(x, y, connection)) {
                return connection;
            }
        }
        return null;
    }

    isPointOnConnection(x, y, connection) {
        const fromNode = this.nodes.find(n => n.id === connection.from.nodeId);
        const toNode = this.nodes.find(n => n.id === connection.to.nodeId);
        
        if (!fromNode || !toNode) return false;
        
        const fromPortIndex = Object.keys(fromNode.outputs || {}).indexOf(connection.from.port);
        const toPortIndex = Object.keys(toNode.inputs || {}).indexOf(connection.to.port);
        
        const startX = fromNode.x + fromNode.width + 2;
        const startY = fromNode.y + 40 + fromPortIndex * 25;
        const endX = toNode.x - 2;
        const endY = toNode.y + 40 + toPortIndex * 25;
        
        const steps = 20;
        const threshold = 5;
        const controlOffset = Math.abs(endX - startX) * 0.5;
        
        for (let i = 0; i <= steps; i++) {
            const t = i / steps;
            const bx = Math.pow(1-t, 3) * startX + 
                      3 * Math.pow(1-t, 2) * t * (startX + controlOffset) +
                      3 * (1-t) * Math.pow(t, 2) * (endX - controlOffset) +
                      Math.pow(t, 3) * endX;
            const by = Math.pow(1-t, 3) * startY +
                      3 * Math.pow(1-t, 2) * t * startY +
                      3 * (1-t) * Math.pow(t, 2) * endY +
                      Math.pow(t, 3) * endY;
            
            if (Math.abs(x - bx) < threshold && Math.abs(y - by) < threshold) {
                return true;
            }
        }
        
        return false;
    }

    updatePropertiesPanel() {
        const panel = document.getElementById('properties-panel');
        const content = document.getElementById('properties-content');
        
        if (!panel || !content) return;
        
        if (this.selectedNode) {
            panel.classList.add('is-active');
            
            const definition = this.nodeDefinitions[this.selectedNode.type];
            
            content.innerHTML = `
                <div class="field">
                    <label class="label">節點類型</label>
                    <div class="control">
                        <span class="tag is-info">${this.selectedNode.type || '未知'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">節點名稱</label>
                    <div class="control">
                        <input class="input" type="text" id="node-name" 
                               value="${this.selectedNode.name || ''}" 
                               placeholder="請輸入節點名稱">
                    </div>
                </div>
                <div class="field">
                    <label class="label">描述</label>
                    <div class="control">
                        <p class="help">${this.selectedNode.description || definition?.description || '無描述'}</p>
                    </div>
                </div>
                
                <!-- 參數設定區域 -->
                <div class="field">
                    <label class="label">節點參數</label>
                    <div class="content">
                        ${(() => {
                            // 取得節點定義中的參數
                            const nodeDefinition = this.nodeDefinitions[this.selectedNode.type] || {};
                            const parameters = nodeDefinition.parameters || {};
                            
                            // 初始化節點的參數值（如果還沒有）
                            if (!this.selectedNode.parameters) {
                                this.selectedNode.parameters = {};
                            }
                            
                            // 如果沒有參數定義，顯示提示
                            if (Object.keys(parameters).length === 0) {
                                return '<p class="help">此節點類型沒有可設定的參數</p>';
                            }
                            
                            // 為每個參數生成輸入欄位
                            return Object.entries(parameters).map(([paramName, paramType]) => {
                                const currentValue = this.selectedNode.parameters[paramName] || '';
                                const inputType = this.getInputTypeForParameter(paramType);
                                
                                return `
                                    <div class="field">
                                        <label class="label is-small">${paramName}</label>
                                        <div class="control">
                                            ${this.createParameterInput(paramName, paramType, currentValue)}
                                        </div>
                                        <p class="help">類型: ${paramType}</p>
                                    </div>
                                `;
                            }).join('');
                        })()}
                    </div>
                </div>
                
                <!-- 輸入值設定區域（用於沒有連線的輸入端口） -->
                <div class="field">
                    <label class="label">輸入值設定</label>
                    <div class="content">
                        ${Object.entries(this.selectedNode.inputs || {}).map(([portName, portInfo]) => {
                            const dataType = portInfo?.type || portInfo || 'any';
                            const description = portInfo?.description || portName;
                            
                            // 檢查這個輸入端口是否已經有連線
                            const hasConnection = this.connections.some(conn => 
                                conn.to.nodeId === this.selectedNode.id && conn.to.port === portName
                            );
                            
                            // 如果有連線，顯示連線提示
                            if (hasConnection) {
                                return `
                                    <div class="box" style="padding: 0.5rem; margin-bottom: 0.5rem; background-color: #f0f0f0;">
                                        <div class="level is-mobile">
                                            <div class="level-left">
                                                <div class="level-item">
                                                    <strong>${portName}</strong>
                                                </div>
                                            </div>
                                            <div class="level-right">
                                                <div class="level-item">
                                                    <span class="tag is-success">已連接</span>
                                                </div>
                                            </div>
                                        </div>
                                        <p class="help">此輸入端口已連接，將使用連線傳入的值</p>
                                    </div>
                                `;
                            }
                            
                            // 初始化輸入值
                            if (!this.selectedNode.inputValues) {
                                this.selectedNode.inputValues = {};
                            }
                            
                            const currentValue = this.selectedNode.inputValues[portName] || '';
                            
                            return `
                                <div class="box" style="padding: 0.5rem; margin-bottom: 0.5rem;">
                                    <div class="field">
                                        <label class="label is-small">${portName}</label>
                                        <div class="control">
                                            ${this.createInputValueField(portName, dataType, currentValue, description)}
                                        </div>
                                        <p class="help">${description}</p>
                                    </div>
                                </div>
                            `;
                        }).join('')}
                    </div>
                </div>
                
                <div class="field">
                    <label class="label">輸入端口資訊</label>
                    <div class="content">
                        ${Object.entries(this.selectedNode.inputs || {}).map(([portName, portInfo]) => {
                            const dataType = portInfo?.type || 'any';
                            const description = portInfo?.description || portName;
                            const typeColor = {
                                'string': 'is-danger',     // 紅色
                                'integer': 'is-success',   // 綠色（更易區分）
                                'float': 'is-success',     // 綠色
                                'boolean': 'is-warning',   // 黃/橘色（更易區分）
                                'array': 'is-primary',     // 紫色
                                'object': 'is-info',       // 藍色
                                'any': 'is-light'          // 灰色
                            }[dataType] || 'is-light';
                            
                            return `
                                <div class="box" style="padding: 0.5rem; margin-bottom: 0.5rem;">
                                    <div class="level is-mobile">
                                        <div class="level-left">
                                            <div class="level-item">
                                                <strong>${portName}</strong>
                                            </div>
                                        </div>
                                        <div class="level-right">
                                            <div class="level-item">
                                                <span class="tag ${typeColor}">${dataType}</span>
                                            </div>
                                        </div>
                                    </div>
                                    <p class="help">${description}</p>
                                </div>
                            `;
                        }).join('')}
                    </div>
                </div>
                <div class="field">
                    <label class="label">輸出端口</label>
                    <div class="content">
                        ${Object.entries(this.selectedNode.outputs || {}).map(([portName, portInfo]) => {
                            const dataType = portInfo?.type || 'any';
                            const description = portInfo?.description || portName;
                            const typeColor = {
                                'string': 'is-danger',     // 紅色
                                'integer': 'is-success',   // 綠色（更易區分）
                                'boolean': 'is-warning',   // 黃/橘色（更易區分）
                                'array': 'is-primary',     // 紫色
                                'object': 'is-info',       // 藍色
                                'any': 'is-light'          // 灰色
                            }[dataType] || 'is-light';
                            
                            return `
                                <div class="box" style="padding: 0.5rem; margin-bottom: 0.5rem;">
                                    <div class="level is-mobile">
                                        <div class="level-left">
                                            <div class="level-item">
                                                <strong>${portName}</strong>
                                            </div>
                                        </div>
                                        <div class="level-right">
                                            <div class="level-item">
                                                <span class="tag ${typeColor}">${dataType}</span>
                                            </div>
                                        </div>
                                    </div>
                                    <p class="help">${description}</p>
                                </div>
                            `;
                        }).join('')}
                    </div>
                </div>
                <hr>
                <button class="button is-danger is-fullwidth" 
                        onclick="window.flowDesigner.deleteNode(window.flowDesigner.selectedNode)">
                    刪除節點
                </button>
            `;
            
            const nameInput = document.getElementById('node-name');
            if (nameInput) {
                nameInput.addEventListener('input', (e) => {
                    this.selectedNode.name = e.target.value;
                });
            }
            
            // 為參數輸入和輸入值綁定事件
            this.bindParameterInputEvents();
            this.bindInputValueEvents();
        } else if (this.selectedConnection) {
            panel.classList.add('is-active');
            
            const fromNode = this.nodes.find(n => n.id === this.selectedConnection.from.nodeId);
            const toNode = this.nodes.find(n => n.id === this.selectedConnection.to.nodeId);
            
            content.innerHTML = `
                <div class="field">
                    <label class="label">連接資訊</label>
                    <div class="control">
                        <p><strong>從:</strong> ${fromNode?.name} (${this.selectedConnection.from.port})</p>
                        <p><strong>到:</strong> ${toNode?.name} (${this.selectedConnection.to.port})</p>
                    </div>
                </div>
                <hr>
                <button class="button is-danger is-fullwidth" 
                        onclick="window.flowDesigner.deleteConnection(window.flowDesigner.selectedConnection)">
                    刪除連接
                </button>
            `;
        } else {
            content.innerHTML = `
                <div class="notification is-light">
                    選擇一個節點或連接以查看詳細資訊
                </div>
            `;
        }
    }

    updateStatusBar() {
        const nodeCount = document.getElementById('node-count');
        const connectionCount = document.getElementById('connection-count');
        const flowName = document.getElementById('current-flow-name');
        
        if (nodeCount) nodeCount.textContent = this.nodes.length;
        if (connectionCount) connectionCount.textContent = this.connections.length;
        if (flowName) flowName.textContent = this.currentFlowName;
    }

    async saveFlow() {
        console.log('🔄 保存流程...', this.currentFlowName);
        
        if (!this.currentFlowName || this.currentFlowName === '未命名') {
            this.showErrorNotification('請先載入一個流程進行編輯');
            return;
        }
        
        try {
            // 準備保存資料，轉換為與 YAML 格式相容的結構
            const saveData = {
                name: this.currentFlowName,
                data: {
                    // 保存節點資料，轉換為 flow_designer_data 格式
                    nodes: this.nodes.map(node => ({
                        id: node.id,
                        position: { x: node.x, y: node.y },
                        data: {
                            name: node.name,
                            type: node.type,
                            function: node.nodeDefId,
                            parameters: node.parameters || {},
                            inputValues: node.inputValues || {},  // 保存輸入值
                        },
                        inputs: Object.entries(node.inputs || {}).map(([key, value]) => ({ 
                            key, 
                            type: value.type, 
                            description: value.description 
                        })),
                        outputs: Object.entries(node.outputs || {}).map(([key, value]) => ({ 
                            key, 
                            type: value.type, 
                            description: value.description 
                        }))
                    })),
                    // 保存連接資料，轉換為 YAML 格式
                    connections: this.connections.map(conn => ({
                        source: conn.from.nodeId,
                        sourceOutput: conn.from.port,
                        target: conn.to.nodeId,
                        targetInput: conn.to.port
                    }))
                }
            };
            
            console.log('💾 準備保存資料:', saveData);
            
            // 調用後端 API 保存流程
            const response = await fetch('/api/flows/save', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(saveData)
            });
            
            const result = await response.json();
            console.log('📤 保存 API 回應:', result);
            
            if (response.ok && result.success) {
                this.showSuccessNotification(`流程已成功保存：${this.currentFlowName}`);
                console.log('✅ 流程保存成功:', this.currentFlowName);
            } else {
                throw new Error(result.error || result.message || '保存失敗');
            }
            
        } catch (error) {
            console.error('❌ 保存流程失敗:', error);
            this.showErrorNotification(`保存失敗: ${error.message}`);
        }
    }

    loadFlow(flowName) {
        console.log('載入流程:', flowName);
        // 舊版方法，僅用於向後相容
        this.currentFlowName = flowName;
        this.updateStatusBar();
    }
    
    findFullNodeDefinition(nodeId, fullFlowData) {
        /**
         * 從完整的 YAML 流程資料中查找節點的完整定義
         * 包含詳細的 inputs/outputs 資訊
         */
        try {
            // 檢查是否有傳入的完整流程資料
            if (!fullFlowData) {
                console.warn(`⚠️ 無法找到完整流程資料來查找節點 ${nodeId}`);
                return null;
            }
            
            // 查找主要 nodes 區段中的完整節點定義
            if (fullFlowData.nodes && Array.isArray(fullFlowData.nodes)) {
                const fullNodeDef = fullFlowData.nodes.find(node => node.id === nodeId);
                if (fullNodeDef) {
                    console.log(`🔍 找到節點 ${nodeId} 的完整定義:`, fullNodeDef);
                    console.log(`   - 原始 inputs:`, fullNodeDef.inputs);
                    console.log(`   - 原始 outputs:`, fullNodeDef.outputs);
                    
                    // 直接返回原始的 inputs 和 outputs（保持 YAML 中的結構）
                    const result = {
                        inputs: fullNodeDef.inputs || {},
                        outputs: fullNodeDef.outputs || {}
                    };
                    
                    console.log(`   - 返回的結構:`, result);
                    return result;
                }
            }
            
            console.warn(`⚠️ 未找到節點 ${nodeId} 的完整定義`);
            return null;
        } catch (error) {
            console.error(`❌ 查找節點完整定義時出錯 (${nodeId}):`, error);
            return null;
        }
    }
    
    loadFlowFromData(flowName, flowData) {
        console.log(`📥 開始載入流程資料: ${flowName}`, flowData);
        
        try {
            // 清空現有資料
            this.nodes = [];
            this.connections = [];
            this.selectedNode = null;
            this.selectedConnection = null;
            
            // 設置流程名稱
            this.currentFlowName = flowName;
            
            // 保存完整資料供查找類型定義
            this.fullFlowData = flowData;
            
            // 使用 flow_designer_data 區段來載入節點和連線（如果存在）
            const designerData = flowData.flow_designer_data || flowData;
            
            // 載入節點 - 需要從完整的流程資料中獲取詳細的 inputs/outputs 定義
            if (designerData.nodes && Array.isArray(designerData.nodes)) {
                console.log(`📋 載入 ${designerData.nodes.length} 個節點`);
                
                designerData.nodes.forEach((nodeData, index) => {
                    try {
                        // 尋找對應的完整節點定義（從 YAML 的主要 nodes 區段）
                        const fullNodeDef = this.findFullNodeDefinition(nodeData.id, this.fullFlowData);
                        
                        // 取得節點的 function 名稱，用於查找節點定義
                        const functionName = nodeData.data?.function || fullNodeDef?.function || nodeData.id;
                        const apiNodeDef = this.nodeDefinitions[functionName];
                        
                        console.log(`🔍 節點 ${nodeData.id} 查找定義:`, {
                            functionName: functionName,
                            hasFullNodeDef: !!fullNodeDef,
                            hasApiNodeDef: !!apiNodeDef,
                            fullNodeDefInputs: fullNodeDef?.inputs,
                            apiNodeDefInputs: apiNodeDef?.inputs
                        });
                        
                        // 處理 inputs - 優先順序：1. YAML完整定義 2. API節點定義 3. flow_designer_data
                        let inputs = {};
                        
                        // 先嘗試從完整定義取得
                        if (fullNodeDef?.inputs && typeof fullNodeDef.inputs === 'object' && Object.keys(fullNodeDef.inputs).length > 0) {
                            console.log(`✅ 節點 ${nodeData.id} 使用完整定義的 inputs:`, fullNodeDef.inputs);
                            // 特別檢查 check_pending_tasks 節點
                            if (nodeData.id === 'check_pending_tasks') {
                                console.log('🔍 特別檢查 check_pending_tasks 節點的 inputs 結構:', {
                                    fullNodeDef_inputs: fullNodeDef.inputs,
                                    typeof_inputs: typeof fullNodeDef.inputs,
                                    keys: Object.keys(fullNodeDef.inputs || {}),
                                    location_def: fullNodeDef.inputs?.location
                                });
                            }
                            inputs = fullNodeDef.inputs;
                        } else if (apiNodeDef?.inputs) {
                            // 從 API 載入的節點定義取得 inputs
                            console.log(`📚 節點 ${nodeData.id} 使用 API 節點定義的 inputs:`, apiNodeDef.inputs);
                            inputs = apiNodeDef.inputs;
                        } else if (nodeData.inputs && Array.isArray(nodeData.inputs)) {
                            // flow_designer_data 的陣列格式 - 嘗試從完整定義中查找類型
                            console.log(`⚠️ 節點 ${nodeData.id} 使用陣列格式，嘗試查找類型`);
                            nodeData.inputs.forEach(input => {
                                const key = input.key || input;
                                // 嘗試從完整定義中找到對應的類型
                                const fullInputDef = fullNodeDef?.inputs?.[key];
                                inputs[key] = fullInputDef || {
                                    type: input.type || 'any',
                                    description: input.description || key
                                };
                            });
                        } else if (nodeData.inputs) {
                            // 已經是物件格式
                            inputs = nodeData.inputs;
                        }
                        
                        // 處理 outputs - 優先順序：1. YAML完整定義 2. API節點定義 3. flow_designer_data
                        let outputs = {};
                        
                        // 先嘗試從完整定義取得
                        if (fullNodeDef?.outputs && typeof fullNodeDef.outputs === 'object' && Object.keys(fullNodeDef.outputs).length > 0) {
                            console.log(`✅ 節點 ${nodeData.id} 使用完整定義的 outputs:`, fullNodeDef.outputs);
                            outputs = fullNodeDef.outputs;
                        } else if (apiNodeDef?.outputs) {
                            // 從 API 載入的節點定義取得 outputs
                            console.log(`📚 節點 ${nodeData.id} 使用 API 節點定義的 outputs:`, apiNodeDef.outputs);
                            outputs = apiNodeDef.outputs;
                        } else if (nodeData.outputs && Array.isArray(nodeData.outputs)) {
                            // flow_designer_data 的陣列格式 - 嘗試從完整定義中查找類型
                            console.log(`⚠️ 節點 ${nodeData.id} 使用陣列格式，嘗試查找類型`);
                            nodeData.outputs.forEach(output => {
                                const key = output.key || output;
                                // 嘗試從完整定義中找到對應的類型
                                const fullOutputDef = fullNodeDef?.outputs?.[key];
                                outputs[key] = fullOutputDef || {
                                    type: output.type || 'any',
                                    description: output.description || key
                                };
                            });
                        } else if (nodeData.outputs) {
                            // 已經是物件格式
                            outputs = nodeData.outputs;
                        }
                        
                        console.log(`📊 節點 ${nodeData.id} 最終的 inputs/outputs:`, {
                            inputs: inputs,
                            outputs: outputs
                        });
                        
                        // 嘗試從完整節點定義中取得描述
                        let description = '';
                        if (fullNodeDef && this.fullFlowData?.nodes) {
                            const fullNode = this.fullFlowData.nodes.find(n => n.id === nodeData.id);
                            description = fullNode?.description || '';
                        }
                        
                        const node = {
                            id: nodeData.id || `node_${Date.now()}_${index}`,
                            nodeDefId: nodeData.data?.function || nodeData.id,  // 節點定義ID
                            type: nodeData.data?.type || 'condition',           // 節點類型
                            name: nodeData.data?.name || `節點 ${index + 1}`,
                            description: description || nodeData.data?.description || '',  // 節點描述
                            x: nodeData.position?.x || 100 + (index * 150),
                            y: nodeData.position?.y || 200,
                            width: 160,
                            height: 120,
                            inputs: inputs,
                            outputs: outputs,
                            parameters: nodeData.data?.parameters || {},
                            inputValues: nodeData.data?.inputValues || {}  // 恢復輸入值
                        };
                        
                        this.nodes.push(node);
                        console.log(`   ✅ 載入節點: ${node.name} (${node.id})`, {
                            inputs: Object.keys(node.inputs),
                            outputs: Object.keys(node.outputs)
                        });
                    } catch (nodeError) {
                        console.error(`   ❌ 載入節點失敗 (index: ${index}):`, nodeError, nodeData);
                    }
                });
            } else {
                console.warn('⚠️ 流程資料中沒有節點或節點格式不正確');
            }
            
            // 載入連線
            if (designerData.connections && Array.isArray(designerData.connections)) {
                console.log(`🔗 載入 ${designerData.connections.length} 條連線`);
                
                designerData.connections.forEach((connData, index) => {
                    try {
                        // 找到源節點和目標節點
                        const sourceNode = this.nodes.find(n => n.id === connData.source);
                        const targetNode = this.nodes.find(n => n.id === connData.target);
                        
                        if (sourceNode && targetNode) {
                            const connection = {
                                id: connData.id || `connection_${Date.now()}_${index}`,
                                from: {
                                    nodeId: connData.source,
                                    port: connData.sourceOutput
                                },
                                to: {
                                    nodeId: connData.target,
                                    port: connData.targetInput
                                },
                                // 保留原始資料供參考
                                _originalData: {
                                    source: connData.source,
                                    sourceOutput: connData.sourceOutput,
                                    target: connData.target,
                                    targetInput: connData.targetInput
                                }
                            };
                            
                            this.connections.push(connection);
                            console.log(`   ✅ 載入連線: ${sourceNode.name} → ${targetNode.name}`);
                        } else {
                            console.warn(`   ⚠️ 連線節點不存在: ${connData.source} → ${connData.target}`);
                        }
                    } catch (connError) {
                        console.error(`   ❌ 載入連線失敗 (index: ${index}):`, connError, connData);
                    }
                });
            } else {
                console.warn('⚠️ 流程資料中沒有連線或連線格式不正確');
            }
            
            // 更新狀態列
            this.updateStatusBar();
            
            // 更新畫面名稱顯示
            const currentFlowNameElement = document.getElementById('current-flow-name');
            if (currentFlowNameElement) {
                currentFlowNameElement.textContent = flowName || '未命名';
            }
            
            console.log(`✅ 流程載入完成: ${this.nodes.length} 個節點, ${this.connections.length} 條連線`);
            
            // 觸發重新渲染
            if (this.ctx) {
                this.render();
            }
            
        } catch (error) {
            console.error('❌ 載入流程資料失敗:', error);
            // 顯示錯誤通知給用戶
            this.showErrorNotification(`載入流程失敗: ${error.message}`);
        }
    }
    
    showErrorNotification(message) {
        // 創建錯誤通知元素
        const notification = document.createElement('div');
        notification.className = 'notification is-danger error-notification-fixed';
        notification.innerHTML = `
            <button class="delete"></button>
            <strong>錯誤</strong><br>
            ${message}
        `;
        
        // 添加到頁面
        document.body.appendChild(notification);
        
        // 設置自動消失和點擊關閉
        const closeBtn = notification.querySelector('.delete');
        const closeNotification = () => {
            if (notification.parentNode) {
                notification.parentNode.removeChild(notification);
            }
        };
        
        if (closeBtn) {
            closeBtn.addEventListener('click', closeNotification);
        }
        
        // 5秒後自動消失
        setTimeout(closeNotification, 5000);
    }
    
    showSuccessNotification(message) {
        // 創建成功通知元素
        const notification = document.createElement('div');
        notification.className = 'notification is-success';
        notification.style.cssText = 'position: fixed; top: 20px; right: 20px; z-index: 9999; animation: slideIn 0.3s ease;';
        notification.innerHTML = `
            <button class="delete"></button>
            <span class="icon"><i class="mdi mdi-check-circle"></i></span>
            ${message}
        `;
        
        document.body.appendChild(notification);
        
        // 設置自動消失和點擊關閉
        const closeBtn = notification.querySelector('.delete');
        const closeNotification = () => {
            notification.style.animation = 'slideOut 0.3s ease';
            setTimeout(() => {
                if (notification.parentNode) {
                    notification.parentNode.removeChild(notification);
                }
            }, 300);
        };
        
        if (closeBtn) {
            closeBtn.addEventListener('click', closeNotification);
        }
        
        // 3秒後自動消失
        setTimeout(closeNotification, 3000);
    }
}

// 確保全局可用
window.FlowDesignerV2 = FlowDesignerV2;