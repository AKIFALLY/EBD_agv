/**
 * WCS Flow Designer V2 - Light Theme Edition
 * 基於標準流程格式 (FLOW_FORMAT_STANDARD.yaml)
 * 支援節點可視化編輯和 YAML 格式導入導出
 * 採用淺色主題和透明玻璃感設計
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
        
        // 節點定義庫 - 包含詳細描述
        this.nodeDefinitions = {
            // 條件節點
            'check_a_side_space': {
                type: 'condition',
                name: '檢查 A 面空位',
                description: '檢查架台 A 面是否有空位可供放置物料',
                icon: '🔍',
                inputs: { trigger: { type: 'any', description: '觸發信號' } },
                outputs: { 
                    has_space: { type: 'boolean', description: 'A面有空位' },
                    no_space: { type: 'boolean', description: 'A面無空位' }
                }
            },
            'check_b_side_space': {
                type: 'condition', 
                name: '檢查 B 面空位',
                description: '檢查架台 B 面是否有空位可供放置物料',
                icon: '🔍',
                inputs: { trigger: { type: 'any', description: '觸發信號' } },
                outputs: {
                    has_space: { type: 'boolean', description: 'B面有空位' },
                    no_space: { type: 'boolean', description: 'B面無空位' }
                }
            },
            'check_rack_status': {
                type: 'condition',
                name: '檢查架台狀態',
                description: '檢查架台當前的運行狀態和可用性',
                icon: '📊',
                inputs: { rack_id: { type: 'string', description: '架台編號' } },
                outputs: {
                    ready: { type: 'boolean', description: '架台就緒' },
                    busy: { type: 'boolean', description: '架台忙碌' },
                    error: { type: 'boolean', description: '架台故障' }
                }
            },
            'check_agv_status': {
                type: 'condition',
                name: '檢查 AGV 狀態',
                description: '檢查 AGV 當前的運行狀態和位置',
                icon: '🚗',
                inputs: { agv_id: { type: 'string', description: 'AGV編號' } },
                outputs: {
                    idle: { type: 'boolean', description: 'AGV閒置' },
                    working: { type: 'boolean', description: 'AGV工作中' },
                    charging: { type: 'boolean', description: 'AGV充電中' }
                }
            },
            
            // 動作節點
            'rotate_rack': {
                type: 'action',
                name: '旋轉架台',
                description: '將架台旋轉 180 度，切換 A/B 面',
                icon: '🔄',
                inputs: { 
                    rack_id: { type: 'string', description: '架台編號' },
                    trigger: { type: 'any', description: '觸發信號' }
                },
                outputs: { 
                    success: { type: 'boolean', description: '旋轉成功' },
                    failed: { type: 'boolean', description: '旋轉失敗' }
                }
            },
            'create_task': {
                type: 'action',
                name: '創建任務',
                description: '創建新的 AGV 搬運任務',
                icon: '📝',
                inputs: {
                    task_type: { type: 'string', description: '任務類型' },
                    priority: { type: 'number', description: '優先級' }
                },
                outputs: {
                    task_id: { type: 'string', description: '任務ID' },
                    created: { type: 'boolean', description: '創建成功' }
                }
            },
            'send_agv': {
                type: 'action',
                name: '派遣 AGV',
                description: '派遣 AGV 執行指定任務',
                icon: '🚛',
                inputs: {
                    agv_id: { type: 'string', description: 'AGV編號' },
                    destination: { type: 'string', description: '目的地' }
                },
                outputs: {
                    dispatched: { type: 'boolean', description: '派遣成功' },
                    rejected: { type: 'boolean', description: '派遣失敗' }
                }
            },
            
            // 邏輯節點
            'and_gate': {
                type: 'logic',
                name: 'AND 邏輯門',
                description: '當所有輸入都為真時輸出真',
                icon: '&',
                inputs: {
                    input1: { type: 'boolean', description: '輸入1' },
                    input2: { type: 'boolean', description: '輸入2' }
                },
                outputs: {
                    result: { type: 'boolean', description: '邏輯結果' }
                }
            },
            'or_gate': {
                type: 'logic',
                name: 'OR 邏輯門',
                description: '當任一輸入為真時輸出真',
                icon: '∨',
                inputs: {
                    input1: { type: 'boolean', description: '輸入1' },
                    input2: { type: 'boolean', description: '輸入2' }
                },
                outputs: {
                    result: { type: 'boolean', description: '邏輯結果' }
                }
            },
            'delay': {
                type: 'logic',
                name: '延遲',
                description: '延遲指定時間後繼續執行',
                icon: '⏱',
                inputs: {
                    trigger: { type: 'any', description: '觸發信號' },
                    duration: { type: 'number', description: '延遲時間(秒)' }
                },
                outputs: {
                    done: { type: 'any', description: '延遲完成' }
                }
            }
        };
    }

    async initialize() {
        console.log('🚀 初始化 Flow Designer V2 - Light Theme');
        
        // 獲取元素
        this.canvas = document.getElementById('flow-canvas');
        this.container = document.getElementById('rete-editor');
        
        if (!this.canvas || !this.container) {
            throw new Error('找不到必要的 DOM 元素');
        }
        
        // 設置 canvas
        this.resizeCanvas();
        this.ctx = this.canvas.getContext('2d');
        
        // 設置淺色主題樣式
        this.canvas.style.backgroundColor = '#f8f9fa';
        this.canvas.style.backgroundImage = `
            linear-gradient(90deg, rgba(200, 200, 200, 0.2) 1px, transparent 1px),
            linear-gradient(rgba(200, 200, 200, 0.2) 1px, transparent 1px)
        `;
        this.canvas.style.backgroundSize = '20px 20px';
        
        // 初始化
        this.setupEventListeners();
        this.initializeNodePalette();
        this.initializeToolbar();
        
        // 開始渲染
        this.render();
        
        console.log('✅ Flow Designer V2 初始化完成');
    }

    resizeCanvas() {
        const rect = this.container.getBoundingClientRect();
        this.canvas.width = rect.width;
        this.canvas.height = rect.height;
        this.canvas.style.pointerEvents = 'auto';
        this.canvas.style.position = 'absolute';
        this.canvas.style.top = '0';
        this.canvas.style.left = '0';
        this.canvas.style.zIndex = '1';
    }

    render() {
        if (!this.ctx) return;
        
        // 清空畫布
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        // 繪製網格背景
        this.drawGrid();
        
        // 繪製連接線
        this.connections.forEach(conn => {
            this.drawConnection(conn);
        });
        
        // 繪製節點
        this.nodes.forEach(node => {
            this.drawNode(node);
        });
        
        // 繪製臨時連接線
        if (this.isConnecting && this.tempConnection.start) {
            this.drawTempConnection();
        }
    }

    drawGrid() {
        const gridSize = 20 * this.scale;
        const offsetX = this.offset.x % gridSize;
        const offsetY = this.offset.y % gridSize;
        
        // 淺色主題網格
        this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.3)';
        this.ctx.lineWidth = 1;
        
        // 垂直線
        for (let x = offsetX; x < this.canvas.width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }
        
        // 水平線
        for (let y = offsetY; y < this.canvas.height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
    }

    drawNode(node) {
        const x = node.position.x * this.scale + this.offset.x;
        const y = node.position.y * this.scale + this.offset.y;
        const width = node.size.width * this.scale;
        const height = node.size.height * this.scale;
        
        // 透明玻璃感背景
        const gradient = this.ctx.createLinearGradient(x, y, x, y + height);
        if (node === this.selectedNode) {
            // 選中狀態 - 更明亮的玻璃感
            gradient.addColorStop(0, 'rgba(255, 255, 255, 0.95)');
            gradient.addColorStop(1, 'rgba(245, 245, 245, 0.85)');
        } else {
            // 正常狀態 - 透明玻璃感
            gradient.addColorStop(0, 'rgba(255, 255, 255, 0.8)');
            gradient.addColorStop(1, 'rgba(250, 250, 250, 0.7)');
        }
        
        // 添加陰影效果
        this.ctx.shadowColor = 'rgba(0, 0, 0, 0.1)';
        this.ctx.shadowBlur = 8;
        this.ctx.shadowOffsetX = 0;
        this.ctx.shadowOffsetY = 2;
        
        // 繪製節點背景
        this.ctx.fillStyle = gradient;
        this.ctx.strokeStyle = this.getNodeBorderColor(node.type);
        this.ctx.lineWidth = 2;
        
        this.roundRect(x, y, width, height, 8);
        this.ctx.fill();
        
        // 重置陰影
        this.ctx.shadowColor = 'transparent';
        this.ctx.shadowBlur = 0;
        
        // 繪製邊框
        this.ctx.stroke();
        
        // 繪製節點圖標和標題
        const nodeData = this.nodeDefinitions[node.function] || {};
        
        // 圖標
        if (nodeData.icon) {
            this.ctx.fillStyle = '#333';
            this.ctx.font = `${20 * this.scale}px Arial`;
            this.ctx.textAlign = 'center';
            this.ctx.fillText(nodeData.icon, x + 25 * this.scale, y + 25 * this.scale);
        }
        
        // 標題 - 深色文字
        this.ctx.fillStyle = '#2c3e50';
        this.ctx.font = `bold ${14 * this.scale}px Arial`;
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'top';
        const titleX = nodeData.icon ? x + width/2 + 10 * this.scale : x + width/2;
        this.ctx.fillText(node.name || nodeData.name, titleX, y + 10 * this.scale);
        
        // 繪製節點描述 - 更清晰的描述文字
        if (nodeData.description) {
            this.ctx.fillStyle = '#666';
            this.ctx.font = `${11 * this.scale}px Arial`;
            this.ctx.textAlign = 'center';
            
            // 文字換行處理
            const maxWidth = width - 20 * this.scale;
            const lines = this.wrapText(nodeData.description, maxWidth);
            let descY = y + 35 * this.scale;
            
            lines.forEach(line => {
                this.ctx.fillText(line, x + width/2, descY);
                descY += 14 * this.scale;
            });
        }
        
        // 繪製節點輸入輸出
        this.drawNodePorts(node, x, y, width, height);
    }

    drawNodePorts(node, x, y, width, height) {
        const nodeData = this.nodeDefinitions[node.function] || {};
        const inputs = nodeData.inputs || {};
        const outputs = nodeData.outputs || {};
        
        // 繪製輸入端口
        let inputY = y + 30 * this.scale;
        Object.keys(inputs).forEach(key => {
            this.drawPort(x, inputY, 'input', key);
            inputY += 25 * this.scale;
        });
        
        // 繪製輸出端口
        let outputY = y + 30 * this.scale;
        Object.keys(outputs).forEach(key => {
            this.drawPort(x + width, outputY, 'output', key);
            outputY += 25 * this.scale;
        });
    }

    drawPort(x, y, type, name) {
        // 繪製端口圓圈
        this.ctx.beginPath();
        this.ctx.arc(x, y, 6, 0, Math.PI * 2);
        this.ctx.fillStyle = type === 'input' ? '#3B82F6' : '#10B981';
        this.ctx.fill();
        this.ctx.strokeStyle = '#fff';
        this.ctx.lineWidth = 2;
        this.ctx.stroke();
        
        // 繪製端口名稱 - 淺色主題
        this.ctx.font = 'bold 11px Arial';
        const metrics = this.ctx.measureText(name);
        const textWidth = metrics.width;
        const textHeight = 14;
        const padding = 4;
        
        if (type === 'input') {
            // 輸入端口：文字在右側 - 半透明白色背景
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
            this.ctx.fillRect(x + 12, y - textHeight/2 - padding/2, textWidth + padding*2, textHeight + padding);
            
            // 添加細邊框
            this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.5)';
            this.ctx.lineWidth = 1;
            this.ctx.strokeRect(x + 12, y - textHeight/2 - padding/2, textWidth + padding*2, textHeight + padding);
            
            // 深色文字
            this.ctx.fillStyle = '#2c3e50';
            this.ctx.font = 'bold 11px Arial';
            this.ctx.textAlign = 'left';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(name, x + 12 + padding, y);
        } else {
            // 輸出端口：文字在左側 - 半透明白色背景
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
            this.ctx.fillRect(x - 12 - textWidth - padding*2, y - textHeight/2 - padding/2, textWidth + padding*2, textHeight + padding);
            
            // 添加細邊框
            this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.5)';
            this.ctx.lineWidth = 1;
            this.ctx.strokeRect(x - 12 - textWidth - padding*2, y - textHeight/2 - padding/2, textWidth + padding*2, textHeight + padding);
            
            // 深色文字
            this.ctx.fillStyle = '#2c3e50';
            this.ctx.font = 'bold 11px Arial';
            this.ctx.textAlign = 'right';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(name, x - 12 - padding, y);
        }
    }

    drawConnection(connection) {
        const fromNode = this.nodes.find(n => n.id === connection.from);
        const toNode = this.nodes.find(n => n.id === connection.to);
        
        if (!fromNode || !toNode) return;
        
        const fromPort = this.getPortPosition(fromNode, connection.fromPort, 'output');
        const toPort = this.getPortPosition(toNode, connection.toPort, 'input');
        
        if (!fromPort || !toPort) return;
        
        // 添加陰影效果
        this.ctx.shadowColor = 'rgba(0, 0, 0, 0.15)';
        this.ctx.shadowBlur = 3;
        this.ctx.shadowOffsetX = 0;
        this.ctx.shadowOffsetY = 1;
        
        // 設置連接線樣式 - 使用更鮮艷的顏色
        const isSelected = connection === this.selectedConnection;
        this.ctx.strokeStyle = isSelected ? '#e74c3c' : '#27ae60';
        this.ctx.lineWidth = isSelected ? 4 : 3;
        
        // 繪製貝塞爾曲線
        const cp1x = fromPort.x + 100;
        const cp1y = fromPort.y;
        const cp2x = toPort.x - 100;
        const cp2y = toPort.y;
        
        this.ctx.beginPath();
        this.ctx.moveTo(fromPort.x, fromPort.y);
        this.ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, toPort.x, toPort.y);
        this.ctx.stroke();
        
        // 重置陰影
        this.ctx.shadowColor = 'transparent';
        this.ctx.shadowBlur = 0;
        
        // 繪製箭頭
        const angle = Math.atan2(toPort.y - cp2y, toPort.x - cp2x);
        const arrowLength = 10;
        const arrowAngle = Math.PI / 6;
        
        this.ctx.beginPath();
        this.ctx.moveTo(toPort.x, toPort.y);
        this.ctx.lineTo(
            toPort.x - arrowLength * Math.cos(angle - arrowAngle),
            toPort.y - arrowLength * Math.sin(angle - arrowAngle)
        );
        this.ctx.moveTo(toPort.x, toPort.y);
        this.ctx.lineTo(
            toPort.x - arrowLength * Math.cos(angle + arrowAngle),
            toPort.y - arrowLength * Math.sin(angle + arrowAngle)
        );
        this.ctx.stroke();
    }

    drawTempConnection() {
        if (!this.tempConnection.start) return;
        
        this.ctx.strokeStyle = '#95a5a6';
        this.ctx.lineWidth = 2;
        this.ctx.setLineDash([5, 5]);
        
        const cp1x = this.tempConnection.start.x + 50;
        const cp1y = this.tempConnection.start.y;
        const cp2x = this.tempConnection.end.x - 50;
        const cp2y = this.tempConnection.end.y;
        
        this.ctx.beginPath();
        this.ctx.moveTo(this.tempConnection.start.x, this.tempConnection.start.y);
        this.ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, this.tempConnection.end.x, this.tempConnection.end.y);
        this.ctx.stroke();
        
        this.ctx.setLineDash([]);
    }

    getNodeBorderColor(type) {
        const colors = {
            condition: '#3498db',
            action: '#27ae60',
            logic: '#9b59b6',
            script: '#e67e22'
        };
        return colors[type] || '#95a5a6';
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

    // 輔助函數：文字換行 - 支援中文
    wrapText(text, maxWidth) {
        if (!text) return [];
        
        const lines = [];
        let currentLine = '';
        
        // 支援中文字符逐字換行
        for (let i = 0; i < text.length; i++) {
            const char = text[i];
            const testLine = currentLine + char;
            const width = this.ctx.measureText(testLine).width;
            
            if (width > maxWidth && currentLine !== '') {
                lines.push(currentLine);
                currentLine = char;
            } else {
                currentLine = testLine;
            }
        }
        
        if (currentLine) {
            lines.push(currentLine);
        }
        
        // 限制最多顯示 3 行
        return lines.slice(0, 3);
    }

    initializeNodePalette() {
        // 為每個節點類別創建調色板項目
        ['condition', 'action', 'logic'].forEach(category => {
            const container = document.getElementById(`${category}-nodes`);
            if (!container) return;
            
            container.innerHTML = '';
            
            Object.entries(this.nodeDefinitions).forEach(([key, nodeData]) => {
                if (nodeData.type !== category) return;
                
                const nodeElement = document.createElement('div');
                nodeElement.className = 'palette-node';
                nodeElement.draggable = true;
                nodeElement.dataset.nodeType = key;
                nodeElement.dataset.nodeCategory = category;
                
                // 添加描述到節點面板，讓用戶知道節點功能
                const description = nodeData.description || '無描述';
                const shortDesc = description.length > 30 ? description.substring(0, 30) + '...' : description;
                
                nodeElement.innerHTML = `
                    <span class="icon">${nodeData.icon || '📦'}</span>
                    <div class="node-info" style="flex: 1; display: flex; flex-direction: column;">
                        <span class="name" style="font-weight: bold; color: #2c3e50;">${nodeData.name}</span>
                        <span class="desc" style="font-size: 0.75rem; color: #7f8c8d; margin-top: 2px;">${shortDesc}</span>
                    </div>
                `;
                
                // 添加提示信息
                nodeElement.title = `${nodeData.name}\n${description}`;
                
                nodeElement.addEventListener('dragstart', (e) => {
                    e.dataTransfer.effectAllowed = 'copy';
                    e.dataTransfer.setData('nodeType', key);
                    e.dataTransfer.setData('nodeCategory', category);
                });
                
                container.appendChild(nodeElement);
            });
        });
    }

    setupEventListeners() {
        // Canvas 事件
        this.canvas.addEventListener('mousedown', this.handleMouseDown.bind(this));
        this.canvas.addEventListener('mousemove', this.handleMouseMove.bind(this));
        this.canvas.addEventListener('mouseup', this.handleMouseUp.bind(this));
        this.canvas.addEventListener('wheel', this.handleWheel.bind(this));
        this.canvas.addEventListener('dblclick', this.handleDoubleClick.bind(this));
        this.canvas.addEventListener('contextmenu', this.handleContextMenu.bind(this));
        
        // 拖放事件
        this.canvas.addEventListener('dragover', this.handleDragOver.bind(this));
        this.canvas.addEventListener('drop', this.handleDrop.bind(this));
        
        // 鍵盤事件
        document.addEventListener('keydown', this.handleKeyDown.bind(this));
        document.addEventListener('keyup', this.handleKeyUp.bind(this));
        
        // Window 事件
        window.addEventListener('resize', () => {
            this.resizeCanvas();
            this.render();
        });
    }

    handleDragOver(e) {
        e.preventDefault();
        e.stopPropagation();
        e.dataTransfer.dropEffect = 'copy';
    }
    
    handleDrop(e) {
        e.preventDefault();
        e.stopPropagation();
        
        const nodeType = e.dataTransfer.getData('nodeType');
        const nodeCategory = e.dataTransfer.getData('nodeCategory');
        
        if (nodeType && this.nodeDefinitions[nodeType]) {
            const rect = this.canvas.getBoundingClientRect();
            const x = (e.clientX - rect.left - this.offset.x) / this.scale;
            const y = (e.clientY - rect.top - this.offset.y) / this.scale;
            
            this.addNode(nodeType, x, y);
        }
    }

    addNode(functionType, x, y) {
        const nodeData = this.nodeDefinitions[functionType];
        if (!nodeData) return;
        
        const node = {
            id: `node_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
            type: nodeData.type,
            name: nodeData.name,
            function: functionType,
            description: nodeData.description,
            position: { x, y },
            size: { width: 200, height: 100 },
            inputs: nodeData.inputs || {},
            outputs: nodeData.outputs || {}
        };
        
        // 動態調整節點高度
        const inputCount = Object.keys(node.inputs).length;
        const outputCount = Object.keys(node.outputs).length;
        const maxPorts = Math.max(inputCount, outputCount, 1);
        const dynamicHeight = Math.max(80, 50 + maxPorts * 25);
        node.size.height = dynamicHeight;
        
        this.nodes.push(node);
        this.render();
        
        // 自動選中新節點
        this.selectedNode = node;
        this.showNodeProperties(node);
    }

    handleMouseDown(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        // 檢查是否點擊平移鍵或中鍵
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
        
        // 檢查是否點擊端口
        const port = this.getPortAtPosition(x, y);
        if (port) {
            if (port.type === 'output') {
                this.startConnection(port);
            }
            return;
        }
        
        // 檢查是否點擊節點
        const node = this.getNodeAtPosition(x, y);
        if (node) {
            this.selectedNode = node;
            this.selectedConnection = null;
            this.isDragging = true;
            this.dragNode = node;
            this.dragOffset = {
                x: (x - this.offset.x) / this.scale - node.position.x,
                y: (y - this.offset.y) / this.scale - node.position.y
            };
            this.showNodeProperties(node);
        } else {
            // 檢查是否點擊連接線
            const connection = this.getConnectionAtPosition(x, y);
            if (connection) {
                this.selectedConnection = connection;
                this.selectedNode = null;
                this.hideNodeProperties();
            } else {
                this.selectedNode = null;
                this.selectedConnection = null;
                this.hideNodeProperties();
            }
        }
        
        this.render();
    }

    handleMouseMove(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        // 處理平移
        if (this.isPanning) {
            this.offset.x = e.clientX - this.panStart.x;
            this.offset.y = e.clientY - this.panStart.y;
            this.render();
            return;
        }
        
        // 處理節點拖曳
        if (this.isDragging && this.dragNode) {
            this.dragNode.position.x = (x - this.offset.x) / this.scale - this.dragOffset.x;
            this.dragNode.position.y = (y - this.offset.y) / this.scale - this.dragOffset.y;
            this.render();
            return;
        }
        
        // 處理連線
        if (this.isConnecting) {
            this.tempConnection.end = { x, y };
            
            // 檢查是否懸停在端口上
            const port = this.getPortAtPosition(x, y);
            this.hoveredPort = (port && port.type === 'input') ? port : null;
            
            this.render();
        }
    }

    handleMouseUp(e) {
        // 結束平移
        if (this.isPanning) {
            this.isPanning = false;
            this.canvas.style.cursor = 'default';
        }
        
        // 結束拖曳
        if (this.isDragging) {
            this.isDragging = false;
            this.dragNode = null;
        }
        
        // 完成連線
        if (this.isConnecting) {
            if (this.hoveredPort) {
                this.completeConnection(this.hoveredPort);
            }
            this.isConnecting = false;
            this.tempConnection = { start: null, end: null };
            this.hoveredPort = null;
            this.render();
        }
    }

    handleWheel(e) {
        e.preventDefault();
        
        const delta = e.deltaY > 0 ? 0.9 : 1.1;
        const newScale = this.scale * delta;
        
        if (newScale < 0.1 || newScale > 5) return;
        
        // 以鼠標位置為中心縮放
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        this.offset.x = x - (x - this.offset.x) * delta;
        this.offset.y = y - (y - this.offset.y) * delta;
        this.scale = newScale;
        
        this.render();
    }

    handleDoubleClick(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        
        const node = this.getNodeAtPosition(x, y);
        if (node) {
            this.showNodeProperties(node);
        }
    }

    handleContextMenu(e) {
        e.preventDefault();
    }

    handleKeyDown(e) {
        // 空格鍵啟用平移模式
        if (e.code === 'Space' && !this.spacePressed) {
            e.preventDefault();
            this.spacePressed = true;
            this.canvas.style.cursor = 'grab';
        }
        
        // Delete 或 Backspace 刪除選中的節點或連線
        if (e.code === 'Delete' || e.code === 'Backspace') {
            e.preventDefault();
            if (this.selectedNode) {
                this.removeNode(this.selectedNode.id);
                this.selectedNode = null;
                this.hideNodeProperties();
                this.showNotification('節點已刪除', 'info');
            } else if (this.selectedConnection) {
                this.removeConnection(this.selectedConnection.id);
                this.selectedConnection = null;
                this.showNotification('連線已刪除', 'info');
            }
        }
        
        // Escape 取消當前操作
        if (e.code === 'Escape') {
            if (this.isConnecting) {
                this.isConnecting = false;
                this.tempConnection = { start: null, end: null };
                this.render();
            }
            if (this.isDragging) {
                this.isDragging = false;
                this.dragNode = null;
            }
            if (this.isPanning) {
                this.isPanning = false;
                this.canvas.style.cursor = 'default';
            }
        }
    }

    handleKeyUp(e) {
        // 釋放空格鍵停止平移模式
        if (e.code === 'Space') {
            e.preventDefault();
            this.spacePressed = false;
            if (!this.isPanning) {
                this.canvas.style.cursor = 'default';
            }
        }
    }

    getNodeAtPosition(x, y) {
        // 從後往前遍歷，優先選中上層節點
        for (let i = this.nodes.length - 1; i >= 0; i--) {
            const node = this.nodes[i];
            const nx = node.position.x * this.scale + this.offset.x;
            const ny = node.position.y * this.scale + this.offset.y;
            const nw = node.size.width * this.scale;
            const nh = node.size.height * this.scale;
            
            if (x >= nx && x <= nx + nw && y >= ny && y <= ny + nh) {
                return node;
            }
        }
        return null;
    }

    getPortAtPosition(x, y) {
        for (const node of this.nodes) {
            const nx = node.position.x * this.scale + this.offset.x;
            const ny = node.position.y * this.scale + this.offset.y;
            const nw = node.size.width * this.scale;
            
            // 檢查輸入端口
            let portY = ny + 30 * this.scale;
            for (const inputKey of Object.keys(node.inputs || {})) {
                if (Math.abs(x - nx) < 10 && Math.abs(y - portY) < 10) {
                    return { node, port: inputKey, type: 'input', x: nx, y: portY };
                }
                portY += 25 * this.scale;
            }
            
            // 檢查輸出端口
            portY = ny + 30 * this.scale;
            for (const outputKey of Object.keys(node.outputs || {})) {
                if (Math.abs(x - (nx + nw)) < 10 && Math.abs(y - portY) < 10) {
                    return { node, port: outputKey, type: 'output', x: nx + nw, y: portY };
                }
                portY += 25 * this.scale;
            }
        }
        return null;
    }

    getPortPosition(node, portName, portType) {
        const nx = node.position.x * this.scale + this.offset.x;
        const ny = node.position.y * this.scale + this.offset.y;
        const nw = node.size.width * this.scale;
        
        const ports = portType === 'input' ? node.inputs : node.outputs;
        const portKeys = Object.keys(ports || {});
        const portIndex = portKeys.indexOf(portName);
        
        if (portIndex === -1) return null;
        
        const portY = ny + 30 * this.scale + portIndex * 25 * this.scale;
        const portX = portType === 'input' ? nx : nx + nw;
        
        return { x: portX, y: portY };
    }

    getConnectionAtPosition(x, y) {
        // 檢查點擊位置是否在連接線上
        for (const connection of this.connections) {
            if (this.isPointOnConnection(x, y, connection)) {
                return connection;
            }
        }
        return null;
    }

    isPointOnConnection(x, y, connection) {
        const fromNode = this.nodes.find(n => n.id === connection.from);
        const toNode = this.nodes.find(n => n.id === connection.to);
        
        if (!fromNode || !toNode) return false;
        
        const fromPort = this.getPortPosition(fromNode, connection.fromPort, 'output');
        const toPort = this.getPortPosition(toNode, connection.toPort, 'input');
        
        if (!fromPort || !toPort) return false;
        
        // 使用簡化的點到曲線距離檢測
        const steps = 20;
        const threshold = 10;
        
        for (let i = 0; i <= steps; i++) {
            const t = i / steps;
            const cp1x = fromPort.x + 100;
            const cp1y = fromPort.y;
            const cp2x = toPort.x - 100;
            const cp2y = toPort.y;
            
            // 貝塞爾曲線公式
            const bx = Math.pow(1-t, 3) * fromPort.x + 
                      3 * Math.pow(1-t, 2) * t * cp1x +
                      3 * (1-t) * Math.pow(t, 2) * cp2x +
                      Math.pow(t, 3) * toPort.x;
                      
            const by = Math.pow(1-t, 3) * fromPort.y + 
                      3 * Math.pow(1-t, 2) * t * cp1y +
                      3 * (1-t) * Math.pow(t, 2) * cp2y +
                      Math.pow(t, 3) * toPort.y;
            
            const distance = Math.sqrt(Math.pow(x - bx, 2) + Math.pow(y - by, 2));
            if (distance < threshold) {
                return true;
            }
        }
        
        return false;
    }

    startConnection(port) {
        this.isConnecting = true;
        this.connectionStart = port;
        this.tempConnection = {
            start: { x: port.x, y: port.y },
            end: { x: port.x, y: port.y }
        };
    }

    completeConnection(endPort) {
        if (!this.connectionStart || !endPort) return;
        
        // 不能連接到同一個節點
        if (this.connectionStart.node === endPort.node) return;
        
        // 檢查是否已存在相同連接
        const exists = this.connections.some(conn => 
            conn.from === this.connectionStart.node.id &&
            conn.to === endPort.node.id &&
            conn.fromPort === this.connectionStart.port &&
            conn.toPort === endPort.port
        );
        
        if (!exists) {
            const connection = {
                id: `conn_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
                from: this.connectionStart.node.id,
                to: endPort.node.id,
                fromPort: this.connectionStart.port,
                toPort: endPort.port
            };
            
            this.connections.push(connection);
            this.showNotification('連線建立成功', 'success');
        }
        
        this.connectionStart = null;
    }

    removeNode(nodeId) {
        // 移除節點
        this.nodes = this.nodes.filter(n => n.id !== nodeId);
        
        // 移除相關連接
        this.connections = this.connections.filter(c => 
            c.from !== nodeId && c.to !== nodeId
        );
        
        this.render();
    }

    removeConnection(connectionId) {
        this.connections = this.connections.filter(c => c.id !== connectionId);
        this.render();
    }

    showNodeProperties(node) {
        const panel = document.getElementById('properties-panel');
        panel.classList.add('is-active');
        
        const content = document.getElementById('properties-content');
        const nodeData = this.nodeDefinitions[node.function] || {};
        
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
                    <input class="input is-small" type="text" value="${node.name}" id="node-name-input">
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
                    <textarea class="textarea is-small" readonly>${nodeData.description || '無描述'}</textarea>
                </div>
            </div>
            <div class="field">
                <label class="label">輸入</label>
                <div class="control">
                    <div class="tags">
                        ${Object.keys(node.inputs || {}).map(key => 
                            `<span class="tag is-info">${key}</span>`
                        ).join('')}
                    </div>
                </div>
            </div>
            <div class="field">
                <label class="label">輸出</label>
                <div class="control">
                    <div class="tags">
                        ${Object.keys(node.outputs || {}).map(key => 
                            `<span class="tag is-success">${key}</span>`
                        ).join('')}
                    </div>
                </div>
            </div>
            <div class="field">
                <label class="label">位置</label>
                <div class="control">
                    <input class="input is-small" type="text" value="X: ${Math.round(node.position.x)}, Y: ${Math.round(node.position.y)}" readonly>
                </div>
            </div>
        `;
        
        // 添加名稱編輯事件
        const nameInput = document.getElementById('node-name-input');
        if (nameInput) {
            nameInput.addEventListener('change', (e) => {
                node.name = e.target.value;
                this.render();
            });
        }
    }

    hideNodeProperties() {
        const panel = document.getElementById('properties-panel');
        panel.classList.remove('is-active');
    }

    initializeToolbar() {
        // 保存按鈕
        const saveBtn = document.getElementById('btn-save-flow');
        if (saveBtn) {
            saveBtn.addEventListener('click', () => this.saveFlow());
        }
    }

    async saveFlow() {
        const flowData = {
            name: this.currentFlowName,
            nodes: this.nodes,
            connections: this.connections,
            metadata: {
                created: new Date().toISOString(),
                version: '2.0'
            }
        };
        
        try {
            const response = await fetch(`/api/flow-designer/flows/${this.currentFlowName}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ flow_data: flowData })
            });
            
            if (response.ok) {
                this.showNotification('流程已保存', 'success');
            } else {
                this.showNotification('保存失敗', 'error');
            }
        } catch (error) {
            console.error('保存失敗:', error);
            this.showNotification('保存失敗', 'error');
        }
    }

    async loadFlow(flowName) {
        try {
            const response = await fetch(`/api/flow-designer/flows/${flowName}`);
            if (response.ok) {
                const flowData = await response.json();
                
                this.currentFlowName = flowName;
                this.nodes = flowData.flow_designer_data?.nodes || [];
                this.connections = flowData.flow_designer_data?.connections || [];
                
                // 更新節點大小
                this.nodes.forEach(node => {
                    const inputCount = Object.keys(node.inputs || {}).length;
                    const outputCount = Object.keys(node.outputs || {}).length;
                    const maxPorts = Math.max(inputCount, outputCount, 1);
                    node.size.height = Math.max(80, 50 + maxPorts * 25);
                });
                
                this.render();
                this.showNotification(`已載入流程: ${flowName}`, 'success');
            }
        } catch (error) {
            console.error('載入失敗:', error);
            this.showNotification('載入失敗', 'error');
        }
    }

    showNotification(message, type = 'info') {
        const notification = document.createElement('div');
        notification.className = `notification is-${type}`;
        notification.style.cssText = `
            position: fixed;
            top: 80px;
            right: 20px;
            z-index: 9999;
            min-width: 250px;
            animation: slideIn 0.3s ease;
        `;
        
        notification.innerHTML = `
            <button class="delete"></button>
            ${message}
        `;
        
        document.body.appendChild(notification);
        
        // 自動移除
        setTimeout(() => {
            notification.style.animation = 'slideOut 0.3s ease';
            setTimeout(() => notification.remove(), 300);
        }, 3000);
        
        // 手動關閉
        notification.querySelector('.delete').addEventListener('click', () => {
            notification.remove();
        });
    }
}

// 初始化
window.FlowDesignerV2 = FlowDesignerV2;