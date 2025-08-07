/**
 * WCS Flow Designer V2
 * 基於標準流程格式 (FLOW_FORMAT_STANDARD.yaml)
 * 支援節點可視化編輯和 YAML 格式導入導出
 */

class FlowDesignerV2 {
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
        this.hoveredPort = null;
        
        // 平移模式相關
        this.isPanning = false;
        this.panStart = { x: 0, y: 0 };
        this.spacePressed = false;
        
        // 節點類型定義
        this.nodeTypes = {
            condition: {
                color: '#3498db',
                icon: '?',
                functions: [
                    'get_locations_by_type',
                    'check_pending_rotation_tasks',
                    'check_rack_side_status',
                    'check_rack_side_carrier',
                    'check_rack_side_space'
                ]
            },
            action: {
                color: '#2ecc71',
                icon: '⚡',
                functions: [
                    'create_rack_rotation_task',
                    'create_task'
                ]
            },
            logic: {
                color: '#9b59b6',
                icon: '⚙',
                functions: [
                    'log_task_creation',
                    'log_skip_reason',
                    'process_data'
                ]
            }
        };
    }

    async initialize() {
        console.log('🚀 初始化 Flow Designer V2');
        
        // 獲取 canvas 元素
        this.canvas = document.getElementById('flow-canvas');
        this.container = document.getElementById('rete-editor');
        
        if (!this.canvas || !this.container) {
            throw new Error('找不到必要的 DOM 元素');
        }
        
        // 設置 canvas 大小
        this.resizeCanvas();
        this.ctx = this.canvas.getContext('2d');
        
        // 添加 roundRect polyfill（如果需要）
        this.addRoundRectPolyfill();
        
        // 設置事件監聽器
        this.setupEventListeners();
        
        // 初始化節點面板
        this.initializeNodePalette();
        
        // 初始化工具欄
        this.initializeToolbar();
        
        // 載入示例流程（如果有）
        const urlParams = new URLSearchParams(window.location.search);
        const flowName = urlParams.get('flow');
        if (flowName) {
            await this.loadFlow(flowName);
        }
        
        // 開始渲染循環
        this.render();
        
        console.log('✅ Flow Designer V2 初始化完成');
    }

    resizeCanvas() {
        const rect = this.container.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;
        // 確保 canvas 可以接收所有事件
        this.canvas.style.pointerEvents = 'auto';
        this.canvas.style.position = 'absolute';
        this.canvas.style.top = '0';
        this.canvas.style.left = '0';
        this.canvas.style.zIndex = '1';  // 確保在適當的層級
    }

    addRoundRectPolyfill() {
        // 為不支援 roundRect 的瀏覽器添加 polyfill
        if (!this.ctx.roundRect) {
            CanvasRenderingContext2D.prototype.roundRect = function(x, y, width, height, radius) {
                if (width < 2 * radius) radius = width / 2;
                if (height < 2 * radius) radius = height / 2;
                this.beginPath();
                this.moveTo(x + radius, y);
                this.arcTo(x + width, y, x + width, y + height, radius);
                this.arcTo(x + width, y + height, x, y + height, radius);
                this.arcTo(x, y + height, x, y, radius);
                this.arcTo(x, y, x + width, y, radius);
                this.closePath();
                return this;
            };
        }
    }

    setupEventListeners() {
        // Canvas 事件
        this.canvas.addEventListener('mousedown', this.handleMouseDown.bind(this));
        this.canvas.addEventListener('mousemove', this.handleMouseMove.bind(this));
        this.canvas.addEventListener('mouseup', this.handleMouseUp.bind(this));
        this.canvas.addEventListener('wheel', this.handleWheel.bind(this));
        this.canvas.addEventListener('dblclick', this.handleDoubleClick.bind(this));
        this.canvas.addEventListener('contextmenu', this.handleContextMenu.bind(this));
        
        // 鍵盤事件 (用於刪除和平移)
        document.addEventListener('keydown', this.handleKeyDown.bind(this));
        document.addEventListener('keyup', this.handleKeyUp.bind(this));
        
        // Window 事件
        window.addEventListener('resize', () => {
            this.resizeCanvas();
            this.render();
        });
    }

    initializeNodePalette() {
        // 初始化條件節點
        const conditionContainer = document.getElementById('condition-nodes');
        if (conditionContainer) {
            conditionContainer.innerHTML = '';
            this.nodeTypes.condition.functions.forEach(func => {
                const nodeEl = this.createPaletteNode('condition', func);
                conditionContainer.appendChild(nodeEl);
            });
        }
        
        // 初始化動作節點
        const actionContainer = document.getElementById('action-nodes');
        if (actionContainer) {
            actionContainer.innerHTML = '';
            this.nodeTypes.action.functions.forEach(func => {
                const nodeEl = this.createPaletteNode('action', func);
                actionContainer.appendChild(nodeEl);
            });
        }
        
        // 初始化邏輯節點
        const logicContainer = document.getElementById('logic-nodes');
        if (logicContainer) {
            logicContainer.innerHTML = '';
            this.nodeTypes.logic.functions.forEach(func => {
                const nodeEl = this.createPaletteNode('logic', func);
                logicContainer.appendChild(nodeEl);
            });
        }
    }
    
    // 確保拖放功能正常運作
    handleDragOver(e) {
        e.preventDefault();
        e.stopPropagation(); // 防止事件冒泡
        e.dataTransfer.dropEffect = 'copy';
    }
    
    handleDrop(e) {
        e.preventDefault();
        e.stopPropagation(); // 防止事件冒泡導致重複添加
        
        const type = e.dataTransfer.getData('nodeType');
        const func = e.dataTransfer.getData('nodeFunction');
        
        if (type && func) {
            const rect = this.canvas.getBoundingClientRect();
            const x = (e.clientX - rect.left - this.offset.x) / this.scale;
            const y = (e.clientY - rect.top - this.offset.y) / this.scale;
            
            // 只添加一次節點
            this.addNode(type, func, x, y);
            
            // 清空 dataTransfer 防止重複處理
            e.dataTransfer.clearData();
        }
    }

    createPaletteNode(type, func) {
        const div = document.createElement('div');
        div.className = 'palette-node';
        div.draggable = true;
        div.innerHTML = `
            <span class="icon">${this.nodeTypes[type].icon}</span>
            <span class="name">${func}</span>
        `;
        div.style.backgroundColor = this.nodeTypes[type].color;
        
        // 拖放事件
        div.addEventListener('dragstart', (e) => {
            e.dataTransfer.effectAllowed = 'copy';
            e.dataTransfer.setData('nodeType', type);
            e.dataTransfer.setData('nodeFunction', func);
            console.log(`開始拖動節點: type=${type}, func=${func}`);
        });
        
        div.addEventListener('dragend', (e) => {
            console.log(`拖動結束`);
        });
        
        return div;
    }

    initializeToolbar() {
        // 保存流程按鈕（覆蓋原檔案）
        const btnSave = document.getElementById('btn-save-flow');
        if (btnSave) {
            btnSave.addEventListener('click', () => this.saveFlow());
        }
        
        // 為整個編輯器容器添加拖放事件
        // 注意：container 是 rete-editor，不是 canvas
        if (this.container) {
            this.container.addEventListener('dragover', this.handleDragOver.bind(this));
            this.container.addEventListener('drop', this.handleDrop.bind(this));
        }
        
        // 為 canvas 元素也添加拖放事件，確保覆蓋整個區域
        if (this.canvas) {
            // 修改 canvas 的 pointer-events 讓它可以接收拖放事件
            this.canvas.style.pointerEvents = 'auto';
            this.canvas.addEventListener('dragover', this.handleDragOver.bind(this));
            this.canvas.addEventListener('drop', this.handleDrop.bind(this));
        }
    }

    // 節點操作
    addNode(type, func, x, y) {
        const node = {
            id: `node_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            type: type,
            name: func,
            function: func,
            parameters: {},
            inputs: {},
            outputs: {},
            position: { x, y },
            size: { width: 200, height: 100 }
        };
        
        // 根據函數設置默認輸入輸出
        this.setDefaultPorts(node);
        
        this.nodes.push(node);
        this.render();
        
        // 更新統計
        this.updateStatistics();
        
        return node;
    }

    setDefaultPorts(node) {
        // 根據節點類型和函數設置默認端口
        switch (node.function) {
            case 'get_locations_by_type':
                node.outputs = {
                    locations: { type: 'array', description: '位置列表' },
                    no_locations: { type: 'boolean', description: '無符合位置' }
                };
                break;
            case 'check_pending_rotation_tasks':
                node.inputs = {
                    location: { type: 'object', description: '位置資訊' }
                };
                node.outputs = {
                    has_pending: { type: 'boolean', description: '有進行中任務' },
                    no_pending: { type: 'boolean', description: '無進行中任務' },
                    location_data: { type: 'object', description: '位置資料' }
                };
                break;
            case 'check_rack_side_status':
                node.inputs = {
                    location_data: { type: 'object', description: '位置資料' }
                };
                node.outputs = {
                    a_side_done: { type: 'boolean', description: 'A面已完成' },
                    a_side_not_done: { type: 'boolean', description: 'A面未完成' },
                    rack_info: { type: 'object', description: 'Rack資訊' }
                };
                break;
            case 'create_rack_rotation_task':
                node.inputs = {
                    rack_info: { type: 'object', description: 'Rack資訊' },
                    location_data: { type: 'object', description: '位置資料' }
                };
                node.outputs = {
                    task_id: { type: 'string', description: '任務ID' },
                    success: { type: 'boolean', description: '創建成功' }
                };
                break;
            case 'log_task_creation':
            case 'log_skip_reason':
                node.inputs = {
                    task_id: { type: 'string', description: '任務ID' },
                    location_data: { type: 'object', description: '位置資料' }
                };
                break;
        }
    }

    removeNode(nodeId) {
        // 移除節點
        this.nodes = this.nodes.filter(n => n.id !== nodeId);
        
        // 移除相關連線
        this.connections = this.connections.filter(c => 
            !c.from.startsWith(nodeId) && !c.to.startsWith(nodeId)
        );
        
        this.render();
        this.updateStatistics();
    }

    // 連線操作
    addConnection(from, to) {
        const connection = {
            from: from,  // source_node_id.output_name
            to: to,      // target_node_id.input_name
            id: `conn_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
        };
        
        // 檢查連線是否有效
        if (this.isValidConnection(connection)) {
            this.connections.push(connection);
            this.render();
            this.updateStatistics();
            return connection;
        }
        
        return null;
    }

    isValidConnection(connection) {
        // 檢查連線的有效性
        const [fromNodeId, fromPort] = connection.from.split('.');
        const [toNodeId, toPort] = connection.to.split('.');
        
        const fromNode = this.nodes.find(n => n.id === fromNodeId);
        const toNode = this.nodes.find(n => n.id === toNodeId);
        
        if (!fromNode || !toNode) return false;
        if (!fromNode.outputs[fromPort]) return false;
        if (!toNode.inputs[toPort]) return false;
        
        // 檢查是否已存在相同連線
        const exists = this.connections.some(c => 
            c.from === connection.from && c.to === connection.to
        );
        
        return !exists;
    }

    // 渲染
    render() {
        if (!this.ctx) return;
        
        // 清空畫布
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 保存狀態
        this.ctx.save();
        
        // 應用變換
        this.ctx.translate(this.offset.x, this.offset.y);
        this.ctx.scale(this.scale, this.scale);
        
        // 繪製網格背景
        this.drawGrid();
        
        // 繪製連線
        this.connections.forEach(conn => this.drawConnection(conn));
        
        // 繪製臨時連線（正在拖動時）
        if (this.isConnecting && this.tempConnection) {
            this.drawTempConnection();
        }
        
        // 繪製節點
        this.nodes.forEach(node => this.drawNode(node));
        
        // 恢復狀態
        this.ctx.restore();
    }

    drawGrid() {
        const gridSize = 20;
        const width = this.canvas.width / this.scale;
        const height = this.canvas.height / this.scale;
        const offsetX = -this.offset.x / this.scale;
        const offsetY = -this.offset.y / this.scale;
        
        this.ctx.strokeStyle = '#e0e0e0';
        this.ctx.lineWidth = 0.5;
        
        // 垂直線
        for (let x = -offsetX % gridSize; x < width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x + offsetX, offsetY);
            this.ctx.lineTo(x + offsetX, offsetY + height);
            this.ctx.stroke();
        }
        
        // 水平線
        for (let y = -offsetY % gridSize; y < height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(offsetX, y + offsetY);
            this.ctx.lineTo(offsetX + width, y + offsetY);
            this.ctx.stroke();
        }
    }

    drawNode(node) {
        const { x, y } = node.position;
        const { width } = node.size;
        const type = this.nodeTypes[node.type];
        
        // 如果找不到對應的節點類型，使用預設值
        if (!type) {
            console.warn(`Unknown node type: ${node.type}`);
            return;
        }
        
        // 動態計算節點高度
        const inputCount = Object.keys(node.inputs || {}).length;
        const outputCount = Object.keys(node.outputs || {}).length;
        const maxPorts = Math.max(inputCount, outputCount, 1);
        const dynamicHeight = Math.max(80, 50 + maxPorts * 25); // 基礎高度 + 端口間距
        
        // 更新節點大小
        node.size.height = dynamicHeight;
        
        // 繪製節點背景
        this.ctx.fillStyle = node === this.selectedNode ? 
            this.lightenColor(type.color) : type.color;
        this.ctx.strokeStyle = node === this.selectedNode ? '#fff' : '#333';
        this.ctx.lineWidth = node === this.selectedNode ? 3 : 2;
        
        this.ctx.beginPath();
        this.ctx.roundRect(x, y, width, dynamicHeight, 10);
        this.ctx.fill();
        this.ctx.stroke();
        
        // 繪製節點標題
        this.ctx.fillStyle = '#fff';
        this.ctx.font = 'bold 14px Arial';
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';
        this.ctx.fillText(node.name, x + width / 2, y + 25);
        
        // 繪製節點函數名稱（更小的字）
        if (node.function && node.function !== node.name) {
            this.ctx.font = '11px Arial';
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
            this.ctx.fillText(node.function, x + width / 2, y + 40);
        }
        
        // 繪製輸入端口
        let inputY = y + 60;
        Object.keys(node.inputs || {}).forEach((key, i) => {
            this.drawPort(x - 5, inputY + i * 25, 'input', key);
        });
        
        // 繪製輸出端口
        let outputY = y + 60;
        Object.keys(node.outputs || {}).forEach((key, i) => {
            this.drawPort(x + width - 5, outputY + i * 25, 'output', key);
        });
    }

    drawPort(x, y, type, name) {
        // 端口外圈（更大的點擊區域）
        this.ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
        this.ctx.beginPath();
        this.ctx.arc(x + 5, y, 8, 0, Math.PI * 2);
        this.ctx.fill();
        
        // 端口內圈
        this.ctx.fillStyle = type === 'input' ? '#e74c3c' : '#27ae60';
        this.ctx.beginPath();
        this.ctx.arc(x + 5, y, 5, 0, Math.PI * 2);
        this.ctx.fill();
        
        // 端口邊框
        this.ctx.strokeStyle = '#fff';
        this.ctx.lineWidth = 1;
        this.ctx.beginPath();
        this.ctx.arc(x + 5, y, 5, 0, Math.PI * 2);
        this.ctx.stroke();
        
        // 繪製端口名稱背景（提高可讀性）
        const metrics = this.ctx.measureText(name);
        const textWidth = metrics.width;
        const textHeight = 14;
        const padding = 3;
        
        if (type === 'input') {
            // 輸入端口文字在右邊
            this.ctx.fillStyle = 'rgba(45, 45, 45, 0.9)';
            this.ctx.fillRect(x + 16, y - textHeight/2 - padding/2, textWidth + padding*2, textHeight + padding);
        } else {
            // 輸出端口文字在左邊
            this.ctx.fillStyle = 'rgba(45, 45, 45, 0.9)';
            this.ctx.fillRect(x - 10 - textWidth - padding*2, y - textHeight/2 - padding/2, textWidth + padding*2, textHeight + padding);
        }
        
        // 繪製端口名稱（更清晰的文字）
        this.ctx.fillStyle = '#fff';
        this.ctx.font = 'bold 12px Arial';
        this.ctx.textAlign = type === 'input' ? 'left' : 'right';
        this.ctx.textBaseline = 'middle';
        const textX = type === 'input' ? x + 18 : x - 8;
        this.ctx.fillText(name, textX, y);
    }

    drawConnection(connection) {
        const [fromNodeId, fromPort] = connection.from.split('.');
        const [toNodeId, toPort] = connection.to.split('.');
        
        const fromNode = this.nodes.find(n => n.id === fromNodeId);
        const toNode = this.nodes.find(n => n.id === toNodeId);
        
        if (!fromNode || !toNode) return;
        
        // 計算端口位置（使用新的間距）
        const fromOutputs = Object.keys(fromNode.outputs || {});
        const fromIndex = fromOutputs.indexOf(fromPort);
        const toInputs = Object.keys(toNode.inputs || {});
        const toIndex = toInputs.indexOf(toPort);
        
        if (fromIndex === -1 || toIndex === -1) return;
        
        const fromX = fromNode.position.x + fromNode.size.width;
        const fromY = fromNode.position.y + 60 + fromIndex * 25; // 更新為新的間距
        const toX = toNode.position.x;
        const toY = toNode.position.y + 60 + toIndex * 25; // 更新為新的間距
        
        // 判斷是否為選中的連線
        const isSelected = connection === this.selectedConnection;
        
        // 繪製連線陰影（提高可見度）
        this.ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
        this.ctx.shadowBlur = 4;
        this.ctx.shadowOffsetX = 2;
        this.ctx.shadowOffsetY = 2;
        
        // 繪製貝塞爾曲線
        this.ctx.strokeStyle = isSelected ? '#3498db' : '#2ecc71';
        this.ctx.lineWidth = isSelected ? 4 : 3;
        this.ctx.beginPath();
        this.ctx.moveTo(fromX, fromY);
        
        const cp1x = fromX + 80;
        const cp1y = fromY;
        const cp2x = toX - 80;
        const cp2y = toY;
        
        this.ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, toX, toY);
        this.ctx.stroke();
        
        // 重置陰影
        this.ctx.shadowColor = 'transparent';
        this.ctx.shadowBlur = 0;
        this.ctx.shadowOffsetX = 0;
        this.ctx.shadowOffsetY = 0;
        
        // 繪製箭頭
        this.drawArrow(cp2x, cp2y, toX, toY);
    }

    drawArrow(fromX, fromY, toX, toY) {
        const angle = Math.atan2(toY - fromY, toX - fromX);
        const arrowLength = 10;
        const arrowAngle = Math.PI / 6;
        
        this.ctx.beginPath();
        this.ctx.moveTo(toX, toY);
        this.ctx.lineTo(
            toX - arrowLength * Math.cos(angle - arrowAngle),
            toY - arrowLength * Math.sin(angle - arrowAngle)
        );
        this.ctx.moveTo(toX, toY);
        this.ctx.lineTo(
            toX - arrowLength * Math.cos(angle + arrowAngle),
            toY - arrowLength * Math.sin(angle + arrowAngle)
        );
        this.ctx.stroke();
    }
    
    drawTempConnection() {
        if (!this.tempConnection || !this.connectionStart) return;
        
        // 獲取起始端口的位置
        const startPort = this.connectionStart;
        let fromX = startPort.x;
        let fromY = startPort.y;
        
        // 如果起始端口有節點信息，使用更精確的位置
        if (startPort.node) {
            if (startPort.type === 'output') {
                fromX = startPort.node.position.x + startPort.node.size.width;
            } else {
                fromX = startPort.node.position.x;
            }
            fromY = startPort.y;
        }
        
        // 終點位置（鼠標位置或目標端口）
        let toX = this.tempConnection.to.x;
        let toY = this.tempConnection.to.y;
        
        // 如果懸停在有效端口上，吸附到端口位置
        if (this.hoveredPort && this.canConnect(this.connectionStart, this.hoveredPort)) {
            toX = this.hoveredPort.x;
            toY = this.hoveredPort.y;
            
            // 繪製高亮端口
            this.ctx.fillStyle = '#27ae60';
            this.ctx.beginPath();
            this.ctx.arc(toX, toY, 8, 0, Math.PI * 2);
            this.ctx.fill();
        }
        
        // 繪製臨時連線（虛線）
        this.ctx.strokeStyle = '#3498db';
        this.ctx.lineWidth = 2;
        this.ctx.setLineDash([5, 5]);
        this.ctx.beginPath();
        this.ctx.moveTo(fromX, fromY);
        
        // 使用貝塞爾曲線
        const cp1x = fromX + 50;
        const cp1y = fromY;
        const cp2x = toX - 50;
        const cp2y = toY;
        
        this.ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, toX, toY);
        this.ctx.stroke();
        
        // 重置虛線樣式
        this.ctx.setLineDash([]);
        
        // 繪製箭頭
        this.drawArrow(cp2x, cp2y, toX, toY);
    }

    // 事件處理
    handleMouseDown(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        // 檢查是否中鍵或空格鍵平移模式
        if (e.button === 1 || this.spacePressed) {
            e.preventDefault();
            this.isPanning = true;
            this.panStart = {
                x: e.clientX - this.offset.x,
                y: e.clientY - this.offset.y
            };
            this.canvas.style.cursor = 'grabbing';
            return;
        }
        
        // 首先檢查是否點擊了端口（優先級最高）
        const port = this.getPortAt(x, y);
        if (port) {
            e.preventDefault();
            e.stopPropagation();
            
            if (!this.isConnecting) {
                // 開始連線模式
                console.log('開始連線:', port);
                this.isConnecting = true;
                this.connectionStart = port;
                this.tempConnection = {
                    from: port,
                    to: { x, y }
                };
            } else {
                // 完成連線
                if (this.canConnect(this.connectionStart, port)) {
                    let fromStr, toStr;
                    
                    // 確保從輸出連到輸入
                    if (this.connectionStart.type === 'output') {
                        fromStr = `${this.connectionStart.nodeId}.${this.connectionStart.portName}`;
                        toStr = `${port.nodeId}.${port.portName}`;
                    } else {
                        fromStr = `${port.nodeId}.${port.portName}`;
                        toStr = `${this.connectionStart.nodeId}.${this.connectionStart.portName}`;
                    }
                    
                    console.log('創建連線:', fromStr, '->', toStr);
                    this.addConnection(fromStr, toStr);
                } else {
                    console.log('無效的連線');
                }
                
                // 結束連線模式
                this.isConnecting = false;
                this.connectionStart = null;
                this.tempConnection = null;
            }
            
            this.render();
            return; // 防止觸發節點拖動
        }
        
        // 檢查是否點擊了連線
        const connection = this.getConnectionAt(x, y);
        if (connection) {
            this.selectedConnection = connection;
            this.selectedNode = null;
            this.hideNodeProperties();
            this.render();
            return;
        }
        
        // 檢查是否點擊了節點
        const node = this.getNodeAt(x, y);
        if (node) {
            // 如果正在連線，取消連線
            if (this.isConnecting) {
                console.log('取消連線');
                this.isConnecting = false;
                this.connectionStart = null;
                this.tempConnection = null;
            } else {
                // 開始拖動節點
                this.selectedNode = node;
                this.selectedConnection = null;
                this.isDragging = true;
                this.dragNode = node;
                this.dragOffset = {
                    x: x - node.position.x,
                    y: y - node.position.y
                };
                this.showNodeProperties(node);
            }
        } else {
            // 點擊空白處
            this.selectedNode = null;
            this.selectedConnection = null;
            this.hideNodeProperties();
            
            // 取消連線模式
            if (this.isConnecting) {
                console.log('取消連線');
                this.isConnecting = false;
                this.connectionStart = null;
                this.tempConnection = null;
            }
        }
        
        this.render();
    }

    handleMouseMove(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        if (this.isPanning) {
            // 平移畫布
            this.offset.x = e.clientX - this.panStart.x;
            this.offset.y = e.clientY - this.panStart.y;
            this.render();
        } else if (this.isDragging && this.dragNode) {
            // 拖動節點
            this.dragNode.position.x = x - this.dragOffset.x;
            this.dragNode.position.y = y - this.dragOffset.y;
            this.render();
        } else if (this.isConnecting && this.tempConnection) {
            // 更新臨時連線的終點
            this.tempConnection.to = { x, y };
            
            // 檢查是否懸停在端口上
            const port = this.getPortAt(x, y);
            this.hoveredPort = port;
            
            this.render();
        }
        
        // 更新游標樣式
        if (this.isPanning) {
            this.canvas.style.cursor = 'grabbing';
        } else if (this.spacePressed) {
            this.canvas.style.cursor = 'grab';
        } else {
            const port = this.getPortAt(x, y);
            if (port) {
                this.canvas.style.cursor = 'crosshair';
            } else if (this.getNodeAt(x, y)) {
                this.canvas.style.cursor = 'move';
            } else {
                this.canvas.style.cursor = 'default';
            }
        }
    }

    handleMouseUp(e) {
        if (this.isPanning) {
            this.isPanning = false;
            if (!this.spacePressed) {
                this.canvas.style.cursor = 'default';
            } else {
                this.canvas.style.cursor = 'grab';
            }
        }
        this.isDragging = false;
        this.dragNode = null;
    }

    handleWheel(e) {
        e.preventDefault();
        
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        const newScale = this.scale * delta;
        
        if (newScale >= 0.1 && newScale <= 3) {
            this.scale = newScale;
            this.render();
        }
    }

    handleDoubleClick(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        const node = this.getNodeAt(x, y);
        if (node) {
            this.editNode(node);
        }
    }
    
    handleContextMenu(e) {
        e.preventDefault(); // 防止顯示瀏覽器的右鍵選單
    }
    
    handleKeyDown(e) {
        // 空格鍵：進入平移模式
        if (e.code === 'Space' && !this.spacePressed) {
            e.preventDefault();
            this.spacePressed = true;
            if (!this.isPanning) {
                this.canvas.style.cursor = 'grab';
            }
        }
        
        // Delete 鍵：刪除選中的節點或連線
        if (e.code === 'Delete' || e.code === 'Backspace') {
            e.preventDefault();
            if (this.selectedNode) {
                // 刪除節點
                this.removeNode(this.selectedNode.id);
                this.selectedNode = null;
                this.hideNodeProperties();
                this.showNotification('節點已刪除', 'info');
            } else if (this.selectedConnection) {
                // 刪除連線
                this.removeConnection(this.selectedConnection.id);
                this.selectedConnection = null;
                this.showNotification('連線已刪除', 'info');
            }
        }
        
        // Escape 鍵：取消當前操作
        if (e.code === 'Escape') {
            if (this.isConnecting) {
                this.isConnecting = false;
                this.connectionStart = null;
                this.tempConnection = null;
                this.render();
            }
            if (this.selectedNode) {
                this.selectedNode = null;
                this.hideNodeProperties();
                this.render();
            }
            if (this.selectedConnection) {
                this.selectedConnection = null;
                this.render();
            }
        }
    }
    
    handleKeyUp(e) {
        // 釋放空格鍵：退出平移模式
        if (e.code === 'Space') {
            this.spacePressed = false;
            if (!this.isPanning) {
                this.canvas.style.cursor = 'default';
            }
        }
    }

    getNodeAt(x, y) {
        for (let i = this.nodes.length - 1; i >= 0; i--) {
            const node = this.nodes[i];
            const { x: nx, y: ny } = node.position;
            const { width, height } = node.size;
            
            if (x >= nx && x <= nx + width && y >= ny && y <= ny + height) {
                return node;
            }
        }
        return null;
    }
    
    getPortAt(x, y) {
        const portRadius = 10; // 端口的點擊半徑（增大以便點擊）
        
        for (const node of this.nodes) {
            const { position, size } = node;
            
            // 檢查輸入端口（使用新的位置計算）
            const inputs = Object.keys(node.inputs || {});
            for (let index = 0; index < inputs.length; index++) {
                const portName = inputs[index];
                const portX = position.x - 5;
                const portY = position.y + 60 + index * 25; // 更新為新的間距
                
                const dist = Math.sqrt((x - portX) ** 2 + (y - portY) ** 2);
                if (dist <= portRadius) {
                    return {
                        nodeId: node.id,
                        portName: portName,
                        type: 'input',
                        x: portX,
                        y: portY,
                        node: node
                    };
                }
            }
            
            // 檢查輸出端口（使用新的位置計算）
            const outputs = Object.keys(node.outputs || {});
            for (let index = 0; index < outputs.length; index++) {
                const portName = outputs[index];
                const portX = position.x + size.width - 5;
                const portY = position.y + 60 + index * 25; // 更新為新的間距
                
                const dist = Math.sqrt((x - portX - 5) ** 2 + (y - portY) ** 2);
                if (dist <= portRadius) {
                    return {
                        nodeId: node.id,
                        portName: portName,
                        type: 'output',
                        x: portX + 5,
                        y: portY,
                        node: node
                    };
                }
            }
        }
        
        return null;
    }
    
    canConnect(from, to) {
        // 檢查連線的有效性
        if (!from || !to) return false;
        
        // 不能連接到自己
        if (from.nodeId === to.nodeId) return false;
        
        // 輸出只能連到輸入
        if (from.type === to.type) return false;
        
        // 確保從輸出連到輸入
        if (from.type !== 'output' || to.type !== 'input') {
            // 如果反向，交換
            if (from.type === 'input' && to.type === 'output') {
                return this.canConnect(to, from);
            }
            return false;
        }
        
        // 檢查是否已存在相同連線
        const connectionStr = `${from.nodeId}.${from.portName}`;
        const toStr = `${to.nodeId}.${to.portName}`;
        
        const exists = this.connections.some(c => 
            c.from === connectionStr && c.to === toStr
        );
        
        return !exists;
    }
    
    getConnectionAt(x, y) {
        // 檢查點擊位置是否在某條連線上
        const threshold = 10; // 點擊範圍閾值
        
        for (const connection of this.connections) {
            const [fromNodeId, fromPort] = connection.from.split('.');
            const [toNodeId, toPort] = connection.to.split('.');
            
            const fromNode = this.nodes.find(n => n.id === fromNodeId);
            const toNode = this.nodes.find(n => n.id === toNodeId);
            
            if (!fromNode || !toNode) continue;
            
            // 計算端口位置
            const fromOutputs = Object.keys(fromNode.outputs || {});
            const fromIndex = fromOutputs.indexOf(fromPort);
            const toInputs = Object.keys(toNode.inputs || {});
            const toIndex = toInputs.indexOf(toPort);
            
            if (fromIndex === -1 || toIndex === -1) continue;
            
            const fromX = fromNode.position.x + fromNode.size.width;
            const fromY = fromNode.position.y + 60 + fromIndex * 25;
            const toX = toNode.position.x;
            const toY = toNode.position.y + 60 + toIndex * 25;
            
            // 檢查點是否在貝塞爾曲線附近
            const cp1x = fromX + 80;
            const cp1y = fromY;
            const cp2x = toX - 80;
            const cp2y = toY;
            
            // 簡化檢測：將貝塞爾曲線分成多個線段檢查
            const segments = 20;
            for (let i = 0; i < segments; i++) {
                const t1 = i / segments;
                const t2 = (i + 1) / segments;
                
                // 計算貝塞爾曲線上的點
                const x1 = this.getBezierPoint(fromX, cp1x, cp2x, toX, t1);
                const y1 = this.getBezierPoint(fromY, cp1y, cp2y, toY, t1);
                const x2 = this.getBezierPoint(fromX, cp1x, cp2x, toX, t2);
                const y2 = this.getBezierPoint(fromY, cp1y, cp2y, toY, t2);
                
                // 檢查點到線段的距離
                const dist = this.pointToLineDistance(x, y, x1, y1, x2, y2);
                if (dist < threshold) {
                    return connection;
                }
            }
        }
        
        return null;
    }
    
    getBezierPoint(p0, p1, p2, p3, t) {
        // 計算貝塞爾曲線上的點
        const t1 = 1 - t;
        return t1 * t1 * t1 * p0 + 
               3 * t1 * t1 * t * p1 + 
               3 * t1 * t * t * p2 + 
               t * t * t * p3;
    }
    
    pointToLineDistance(px, py, x1, y1, x2, y2) {
        // 計算點到線段的距離
        const A = px - x1;
        const B = py - y1;
        const C = x2 - x1;
        const D = y2 - y1;
        
        const dot = A * C + B * D;
        const lenSq = C * C + D * D;
        
        let param = -1;
        if (lenSq !== 0) {
            param = dot / lenSq;
        }
        
        let xx, yy;
        
        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }
        
        const dx = px - xx;
        const dy = py - yy;
        
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    removeConnection(connectionId) {
        // 移除連線
        this.connections = this.connections.filter(c => c.id !== connectionId);
        this.render();
        this.updateStatistics();
    }

    // 節點屬性面板
    showNodeProperties(node) {
        const panel = document.getElementById('properties-panel');
        if (!panel) return;
        
        panel.classList.add('is-active');
        
        // 更新屬性內容
        const content = document.getElementById('properties-content');
        if (content) {
            content.innerHTML = `
                <div class="field">
                    <label class="label">節點 ID</label>
                    <div class="control">
                        <input class="input is-small" type="text" value="${node.id}" readonly>
                    </div>
                </div>
                <div class="field">
                    <label class="label">節點類型</label>
                    <div class="control">
                        <input class="input is-small" type="text" value="${node.type}" readonly>
                    </div>
                </div>
                <div class="field">
                    <label class="label">節點名稱</label>
                    <div class="control">
                        <input class="input" type="text" id="node-name" value="${node.name}" placeholder="請輸入節點名稱">
                    </div>
                </div>
                <div class="field">
                    <label class="label">函數</label>
                    <div class="control">
                        <input class="input is-small" type="text" value="${node.function}" readonly>
                    </div>
                </div>
                <div class="field">
                    <label class="label">描述</label>
                    <div class="control">
                        <textarea class="textarea" id="node-description" placeholder="請輸入節點描述">${node.description || ''}</textarea>
                    </div>
                </div>
                <div class="field">
                    <label class="label">輸入端口</label>
                    <div class="control">
                        <div class="tags">
                            ${Object.keys(node.inputs || {}).map(key => 
                                `<span class="tag is-info">${key}</span>`
                            ).join('')}
                            ${Object.keys(node.inputs || {}).length === 0 ? '<span class="tag is-light">無</span>' : ''}
                        </div>
                    </div>
                </div>
                <div class="field">
                    <label class="label">輸出端口</label>
                    <div class="control">
                        <div class="tags">
                            ${Object.keys(node.outputs || {}).map(key => 
                                `<span class="tag is-success">${key}</span>`
                            ).join('')}
                            ${Object.keys(node.outputs || {}).length === 0 ? '<span class="tag is-light">無</span>' : ''}
                        </div>
                    </div>
                </div>
                <div class="field">
                    <label class="label">位置</label>
                    <div class="control">
                        <div class="field is-grouped">
                            <div class="control">
                                <input class="input is-small" type="number" id="node-x" value="${Math.round(node.position.x)}" style="width: 80px;">
                            </div>
                            <div class="control">
                                <input class="input is-small" type="number" id="node-y" value="${Math.round(node.position.y)}" style="width: 80px;">
                            </div>
                        </div>
                    </div>
                </div>
            `;
            
            // 綁定事件處理器
            const nameInput = document.getElementById('node-name');
            if (nameInput) {
                nameInput.onchange = () => {
                    node.name = nameInput.value;
                    this.render();
                };
            }
            
            const descInput = document.getElementById('node-description');
            if (descInput) {
                descInput.onchange = () => {
                    node.description = descInput.value;
                };
            }
            
            const xInput = document.getElementById('node-x');
            const yInput = document.getElementById('node-y');
            if (xInput && yInput) {
                xInput.onchange = () => {
                    node.position.x = parseInt(xInput.value) || 0;
                    this.render();
                };
                yInput.onchange = () => {
                    node.position.y = parseInt(yInput.value) || 0;
                    this.render();
                };
            }
        }
    }

    hideNodeProperties() {
        const panel = document.getElementById('properties-panel');
        if (panel) {
            panel.classList.remove('is-active');
        }
    }
    
    editNode(node) {
        // 雙擊節點時編輯節點屬性
        this.selectedNode = node;
        this.showNodeProperties(node);
    }

    // 流程操作

    async loadFlow(flowName) {
        try {
            const response = await fetch(`/api/flow-designer/flows/${flowName}`);
            if (!response.ok) throw new Error('載入流程失敗');
            
            const flowData = await response.json();
            
            // 載入節點並轉換為正確的格式
            this.nodes = (flowData.flow_designer_data?.nodes || []).map(node => {
                // 從 flow_designer_data 轉換為內部格式
                const convertedNode = {
                    id: node.id,
                    type: node.data?.type || node.type || 'action',
                    name: node.data?.name || node.name || 'Unknown',
                    function: node.data?.function || node.function || '',
                    parameters: node.data?.parameters || node.parameters || {},
                    inputs: node.inputs || {},
                    outputs: node.outputs || {},
                    position: node.position || { x: 100, y: 100 },
                    size: node.size || { width: 200, height: 100 }
                };
                
                // 設定預設的輸入輸出
                this.setDefaultPorts(convertedNode);
                
                return convertedNode;
            });
            
            // 載入連線（轉換格式）
            this.connections = [];
            const connections = flowData.flow_designer_data?.connections || [];
            connections.forEach(conn => {
                this.connections.push({
                    id: conn.id,
                    from: `${conn.source}.${conn.sourceOutput}`,
                    to: `${conn.target}.${conn.targetInput}`
                });
            });
            
            this.currentFlowName = flowName;
            this.render();
            this.updateStatistics();
            this.updateFlowNameDisplay();
            
            this.showNotification(`流程 "${flowName}" 載入成功`, 'success');
        } catch (error) {
            console.error('載入流程失敗:', error);
            this.showNotification(`載入流程失敗: ${error.message}`, 'error');
        }
    }

    async saveFlow() {
        // 直接使用當前流程名稱，覆蓋原檔案
        if (!this.currentFlowName || this.currentFlowName === 'untitled') {
            this.showNotification('請先載入一個流程進行編輯', 'warning');
            return;
        }
        
        const flowName = this.currentFlowName;
        
        // 構建標準格式數據
        const flowData = {
            name: flowName,
            description: `Flow Designer 創建的流程 - ${flowName}`,
            version: '1.0',
            author: 'Flow Designer',
            enabled: true,
            priority: 100,
            work_id: '',
            nodes: this.nodes,
            connections: this.connections,
            flow_designer_data: {
                nodes: this.nodes.map(node => ({
                    id: node.id,
                    position: node.position,
                    data: {
                        name: node.name,
                        type: node.type,
                        function: node.function,
                        parameters: node.parameters
                    },
                    inputs: Object.keys(node.inputs || {}).map(key => ({ key })),
                    outputs: Object.keys(node.outputs || {}).map(key => ({ key }))
                })),
                connections: this.connections.map(conn => {
                    const [source, sourceOutput] = conn.from.split('.');
                    const [target, targetInput] = conn.to.split('.');
                    return {
                        id: conn.id,
                        source,
                        sourceOutput,
                        target,
                        targetInput
                    };
                })
            }
        };
        
        try {
            const response = await fetch(`/api/flow-designer/flows/${flowName}/yaml`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ flow_data: flowData })
            });
            
            if (!response.ok) throw new Error('保存流程失敗');
            
            this.updateFlowNameDisplay();
            this.showNotification(`流程 "${flowName}" 已保存`, 'success');
        } catch (error) {
            console.error('保存流程失敗:', error);
            this.showNotification(`保存流程失敗: ${error.message}`, 'error');
        }
    }

    // 工具函數
    updateStatistics() {
        const nodeCount = document.getElementById('node-count');
        const connCount = document.getElementById('connection-count');
        
        if (nodeCount) nodeCount.textContent = this.nodes.length;
        if (connCount) connCount.textContent = this.connections.length;
    }

    updateFlowNameDisplay() {
        const display = document.getElementById('current-flow-name');
        if (display) {
            display.textContent = this.currentFlowName;
        }
    }

    showNotification(message, type = 'info') {
        // 簡單的通知實現
        const notification = document.createElement('div');
        notification.className = `notification is-${type}`;
        notification.style.cssText = 'position: fixed; top: 20px; right: 20px; z-index: 9999;';
        notification.innerHTML = `
            <button class="delete"></button>
            ${message}
        `;
        
        document.body.appendChild(notification);
        
        notification.querySelector('.delete').onclick = () => {
            notification.remove();
        };
        
        setTimeout(() => {
            notification.remove();
        }, 5000);
    }

    lightenColor(color) {
        // 將顏色變亮用於選中效果
        const num = parseInt(color.slice(1), 16);
        const amt = 40;
        const R = (num >> 16) + amt;
        const G = (num >> 8 & 0x00FF) + amt;
        const B = (num & 0x0000FF) + amt;
        return '#' + (0x1000000 + (R < 255 ? R < 1 ? 0 : R : 255) * 0x10000 +
            (G < 255 ? G < 1 ? 0 : G : 255) * 0x100 +
            (B < 255 ? B < 1 ? 0 : B : 255))
            .toString(16).slice(1);
    }
}

// 導出類
window.FlowDesignerV2 = FlowDesignerV2;