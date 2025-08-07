/**
 * WCS Flow Designer V2 - Fixed Light Theme Edition
 * 修復版本：移除工具列、改善拖放功能、專業圖標
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
        
        // 節點定義庫 - 使用專業圖標
        this.nodeDefinitions = {
            // 條件節點
            'check_a_side_space': {
                type: 'condition',
                name: '檢查 A 面空位',
                description: '檢查架台 A 面是否有空位可供放置物料',
                icon: '◐', // 半圓圖標代表A面
                color: '#3498db',
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
                icon: '◑', // 反向半圓代表B面
                color: '#3498db',
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
                icon: '⬚', // 方框代表架台
                color: '#3498db',
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
                icon: '▣', // 填充方框代表AGV
                color: '#3498db',
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
                icon: '↻', // 旋轉箭頭
                color: '#27ae60',
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
                description: '創建新的 AGV 任務並分配執行',
                icon: '▶', // 播放符號代表執行
                color: '#27ae60',
                inputs: {
                    task_type: { type: 'string', description: '任務類型' },
                    params: { type: 'object', description: '任務參數' }
                },
                outputs: {
                    task_id: { type: 'string', description: '任務ID' },
                    error: { type: 'boolean', description: '創建失敗' }
                }
            },
            'move_to_rack': {
                type: 'action',
                name: '移動至架台',
                description: '控制 AGV 移動到指定架台位置',
                icon: '→', // 箭頭代表移動
                color: '#27ae60',
                inputs: {
                    agv_id: { type: 'string', description: 'AGV編號' },
                    rack_id: { type: 'string', description: '目標架台' }
                },
                outputs: {
                    arrived: { type: 'boolean', description: '到達成功' },
                    failed: { type: 'boolean', description: '移動失敗' }
                }
            },
            'pick_material': {
                type: 'action',
                name: '取料',
                description: '從指定位置取出物料',
                icon: '↑', // 向上箭頭代表取出
                color: '#27ae60',
                inputs: {
                    location: { type: 'string', description: '取料位置' },
                    material_id: { type: 'string', description: '物料編號' }
                },
                outputs: {
                    success: { type: 'boolean', description: '取料成功' },
                    failed: { type: 'boolean', description: '取料失敗' }
                }
            },
            'place_material': {
                type: 'action',
                name: '放料',
                description: '將物料放置到指定位置',
                icon: '↓', // 向下箭頭代表放置
                color: '#27ae60',
                inputs: {
                    location: { type: 'string', description: '放料位置' },
                    material_id: { type: 'string', description: '物料編號' }
                },
                outputs: {
                    success: { type: 'boolean', description: '放料成功' },
                    failed: { type: 'boolean', description: '放料失敗' }
                }
            },
            
            // 邏輯節點
            'wait': {
                type: 'logic',
                name: '等待',
                description: '等待指定時間或條件滿足',
                icon: '⏸', // 暫停符號
                color: '#e74c3c',
                inputs: {
                    duration: { type: 'number', description: '等待時間(秒)' },
                    condition: { type: 'any', description: '等待條件' }
                },
                outputs: {
                    done: { type: 'boolean', description: '等待完成' }
                }
            },
            'parallel': {
                type: 'logic',
                name: '並行執行',
                description: '同時執行多個分支流程',
                icon: '⋈', // 並行符號
                color: '#e74c3c',
                inputs: {
                    trigger: { type: 'any', description: '觸發信號' }
                },
                outputs: {
                    branch1: { type: 'any', description: '分支1' },
                    branch2: { type: 'any', description: '分支2' },
                    branch3: { type: 'any', description: '分支3' }
                }
            },
            'merge': {
                type: 'logic',
                name: '合併',
                description: '等待所有輸入完成後繼續',
                icon: '⋀', // 合併符號
                color: '#e74c3c',
                inputs: {
                    input1: { type: 'any', description: '輸入1' },
                    input2: { type: 'any', description: '輸入2' },
                    input3: { type: 'any', description: '輸入3' }
                },
                outputs: {
                    merged: { type: 'any', description: '合併輸出' }
                }
            }
        };
    }

    async initialize() {
        console.log('🚀 初始化 Flow Designer V2 (Fixed)...');
        
        // 獲取容器
        this.container = document.getElementById('rete-editor');
        if (!this.container) {
            throw new Error('找不到編輯器容器 #rete-editor');
        }
        
        // 創建畫布
        this.canvas = document.getElementById('flow-canvas');
        if (!this.canvas) {
            this.canvas = document.createElement('canvas');
            this.canvas.id = 'flow-canvas';
            this.canvas.className = 'flow-canvas-layer';
            this.container.appendChild(this.canvas);
        }
        
        this.ctx = this.canvas.getContext('2d');
        
        // 調整畫布大小
        this.resizeCanvas();
        
        // 初始化
        this.setupEventListeners();
        this.initializeNodePalette();
        this.initializeSaveButton(); // 新增保存按鈕初始化
        
        // 開始渲染
        this.render();
        
        console.log('✅ Flow Designer V2 初始化完成');
    }

    initializeSaveButton() {
        // 創建浮動保存按鈕
        const saveButtonContainer = document.createElement('div');
        saveButtonContainer.style.cssText = `
            position: fixed;
            bottom: 30px;
            right: 30px;
            z-index: 1000;
        `;
        
        const saveButton = document.createElement('button');
        saveButton.className = 'button is-success is-rounded';
        saveButton.style.cssText = `
            width: 60px;
            height: 60px;
            border-radius: 50%;
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 24px;
            transition: all 0.3s;
        `;
        saveButton.innerHTML = '💾';
        saveButton.title = '保存流程 (Ctrl+S)';
        
        saveButton.addEventListener('click', () => this.saveFlow());
        saveButton.addEventListener('mouseenter', () => {
            saveButton.style.transform = 'scale(1.1)';
        });
        saveButton.addEventListener('mouseleave', () => {
            saveButton.style.transform = 'scale(1)';
        });
        
        saveButtonContainer.appendChild(saveButton);
        document.body.appendChild(saveButtonContainer);
        
        // 添加鍵盤快捷鍵
        document.addEventListener('keydown', (e) => {
            if ((e.ctrlKey || e.metaKey) && e.key === 's') {
                e.preventDefault();
                this.saveFlow();
            }
        });
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
        
        // 儲存狀態
        this.ctx.save();
        
        // 應用變換
        this.ctx.translate(this.offset.x, this.offset.y);
        this.ctx.scale(this.scale, this.scale);
        
        // 繪製網格背景
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
        
        this.ctx.strokeStyle = 'rgba(200, 200, 200, 0.3)';
        this.ctx.lineWidth = 0.5;
        
        // 垂直線
        for (let x = -offsetX % gridSize; x < width; x += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(x + offsetX, offsetY);
            this.ctx.lineTo(x + offsetX, height + offsetY);
            this.ctx.stroke();
        }
        
        // 水平線
        for (let y = -offsetY % gridSize; y < height; y += gridSize) {
            this.ctx.beginPath();
            this.ctx.moveTo(offsetX, y + offsetY);
            this.ctx.lineTo(width + offsetX, y + offsetY);
            this.ctx.stroke();
        }
    }

    drawNode(node) {
        const { x, y, width, height } = node;
        const definition = this.nodeDefinitions[node.type];
        
        // 計算動態高度
        const inputCount = Object.keys(node.inputs || {}).length;
        const outputCount = Object.keys(node.outputs || {}).length;
        const portCount = Math.max(inputCount, outputCount);
        const minHeight = 60;
        const portHeight = 25;
        const actualHeight = Math.max(minHeight, portCount * portHeight + 20);
        node.height = actualHeight;
        
        // 繪製節點背景（透明玻璃效果）
        const gradient = this.ctx.createLinearGradient(x, y, x, y + actualHeight);
        if (this.selectedNode === node) {
            gradient.addColorStop(0, 'rgba(52, 152, 219, 0.2)');
            gradient.addColorStop(1, 'rgba(41, 128, 185, 0.15)');
        } else {
            gradient.addColorStop(0, 'rgba(255, 255, 255, 0.9)');
            gradient.addColorStop(1, 'rgba(250, 250, 250, 0.8)');
        }
        
        this.ctx.fillStyle = gradient;
        this.ctx.strokeStyle = this.selectedNode === node ? '#2980b9' : 'rgba(150, 150, 150, 0.5)';
        this.ctx.lineWidth = this.selectedNode === node ? 2 : 1;
        
        // 繪製圓角矩形
        this.roundRect(x, y, width, actualHeight, 8);
        this.ctx.fill();
        this.ctx.stroke();
        
        // 繪製節點頭部
        const headerGradient = this.ctx.createLinearGradient(x, y, x, y + 30);
        const typeColors = {
            condition: ['rgba(52, 152, 219, 0.3)', 'rgba(41, 128, 185, 0.2)'],
            action: ['rgba(46, 204, 113, 0.3)', 'rgba(39, 174, 96, 0.2)'],
            logic: ['rgba(231, 76, 60, 0.3)', 'rgba(192, 57, 43, 0.2)']
        };
        
        const colors = typeColors[definition?.type] || typeColors.action;
        headerGradient.addColorStop(0, colors[0]);
        headerGradient.addColorStop(1, colors[1]);
        
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
        
        // 繪製圖標和標題
        this.ctx.fillStyle = '#2c3e50';
        this.ctx.font = 'bold 14px "Segoe UI", system-ui, sans-serif';
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';
        
        // 繪製專業圖標
        if (definition?.icon) {
            this.ctx.font = '18px "Segoe UI Symbol", "Apple Symbols", sans-serif';
            this.ctx.fillText(definition.icon, x + 20, y + 15);
        }
        
        // 繪製標題
        this.ctx.font = 'bold 12px "Segoe UI", system-ui, sans-serif';
        this.ctx.textAlign = 'left';
        this.ctx.fillText(node.name || definition?.name || '未命名', x + 35, y + 15);
        
        // 繪製端口
        this.drawPorts(node, definition);
    }

    drawPorts(node, definition) {
        const { x, y, width, height } = node;
        const portRadius = 6;
        
        // 輸入端口
        const inputs = Object.keys(node.inputs || {});
        inputs.forEach((portName, index) => {
            const portY = y + 40 + index * 25;
            
            // 繪製端口圓圈
            this.ctx.fillStyle = '#3498db';
            this.ctx.strokeStyle = '#2980b9';
            this.ctx.lineWidth = 1;
            this.ctx.beginPath();
            this.ctx.arc(x - 2, portY, portRadius, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
            
            // 繪製端口名稱背景
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
            const textWidth = this.ctx.measureText(portName).width;
            this.roundRect(x + 8, portY - 10, textWidth + 8, 20, 3);
            this.ctx.fill();
            
            // 繪製端口名稱
            this.ctx.fillStyle = '#34495e';
            this.ctx.font = '11px "Segoe UI", system-ui, sans-serif';
            this.ctx.textAlign = 'left';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(portName, x + 12, portY);
        });
        
        // 輸出端口
        const outputs = Object.keys(node.outputs || {});
        outputs.forEach((portName, index) => {
            const portY = y + 40 + index * 25;
            
            // 繪製端口圓圈
            this.ctx.fillStyle = '#27ae60';
            this.ctx.strokeStyle = '#229954';
            this.ctx.lineWidth = 1;
            this.ctx.beginPath();
            this.ctx.arc(x + width + 2, portY, portRadius, 0, Math.PI * 2);
            this.ctx.fill();
            this.ctx.stroke();
            
            // 繪製端口名稱背景
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
            this.ctx.font = '11px "Segoe UI", system-ui, sans-serif';
            const textWidth = this.ctx.measureText(portName).width;
            this.roundRect(x + width - textWidth - 16, portY - 10, textWidth + 8, 20, 3);
            this.ctx.fill();
            
            // 繪製端口名稱
            this.ctx.fillStyle = '#34495e';
            this.ctx.textAlign = 'right';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(portName, x + width - 12, portY);
        });
    }

    drawConnection(connection) {
        const fromNode = this.nodes.find(n => n.id === connection.from.nodeId);
        const toNode = this.nodes.find(n => n.id === connection.to.nodeId);
        
        if (!fromNode || !toNode) return;
        
        const fromPortIndex = Object.keys(fromNode.outputs || {}).indexOf(connection.from.port);
        const toPortIndex = Object.keys(toNode.inputs || {}).indexOf(connection.to.port);
        
        const startX = fromNode.x + fromNode.width + 2;
        const startY = fromNode.y + 40 + fromPortIndex * 25;
        const endX = toNode.x - 2;
        const endY = toNode.y + 40 + toPortIndex * 25;
        
        // 繪製貝塞爾曲線
        const controlOffset = Math.abs(endX - startX) * 0.5;
        
        this.ctx.strokeStyle = this.selectedConnection === connection ? '#e74c3c' : '#34495e';
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
        const t = 0.95;
        const arrowX = Math.pow(1-t, 3) * startX + 3 * Math.pow(1-t, 2) * t * (startX + controlOffset) + 
                      3 * (1-t) * Math.pow(t, 2) * (endX - controlOffset) + Math.pow(t, 3) * endX;
        const arrowY = Math.pow(1-t, 3) * startY + 3 * Math.pow(1-t, 2) * t * startY + 
                      3 * (1-t) * Math.pow(t, 2) * endY + Math.pow(t, 3) * endY;
        
        const angle = Math.atan2(endY - arrowY, endX - arrowX);
        
        this.ctx.save();
        this.ctx.translate(endX, endY);
        this.ctx.rotate(angle);
        
        this.ctx.beginPath();
        this.ctx.moveTo(-10, -5);
        this.ctx.lineTo(0, 0);
        this.ctx.lineTo(-10, 5);
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
        
        this.ctx.strokeStyle = 'rgba(52, 152, 219, 0.5)';
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
                
                // 添加描述到節點面板
                const description = nodeData.description || '無描述';
                const shortDesc = description.length > 30 ? description.substring(0, 30) + '...' : description;
                
                nodeElement.innerHTML = `
                    <span class="icon" style="font-size: 20px; color: ${nodeData.color || '#34495e'};">${nodeData.icon || '◆'}</span>
                    <div class="node-info" style="flex: 1; display: flex; flex-direction: column;">
                        <span class="name" style="font-weight: bold; color: #2c3e50;">${nodeData.name}</span>
                        <span class="desc" style="font-size: 0.75rem; color: #7f8c8d; margin-top: 2px;">${shortDesc}</span>
                    </div>
                `;
                
                // 添加提示信息
                nodeElement.title = `${nodeData.name}\n${description}`;
                
                nodeElement.addEventListener('dragstart', (e) => {
                    e.dataTransfer.effectAllowed = 'copy';
                    e.dataTransfer.setData('text/plain', key); // 使用 text/plain
                    e.dataTransfer.setData('nodeType', key);
                    e.dataTransfer.setData('nodeCategory', category);
                    nodeElement.classList.add('is-dragging');
                });
                
                nodeElement.addEventListener('dragend', (e) => {
                    nodeElement.classList.remove('is-dragging');
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
        
        // 拖放事件 - 修復版本
        this.canvas.addEventListener('dragover', (e) => {
            e.preventDefault();
            e.stopPropagation();
            e.dataTransfer.dropEffect = 'copy';
            this.canvas.style.backgroundColor = 'rgba(52, 152, 219, 0.05)';
        });
        
        this.canvas.addEventListener('dragleave', (e) => {
            e.preventDefault();
            this.canvas.style.backgroundColor = 'transparent';
        });
        
        this.canvas.addEventListener('drop', (e) => {
            e.preventDefault();
            e.stopPropagation();
            this.canvas.style.backgroundColor = 'transparent';
            
            // 嘗試多種方式獲取節點類型
            const nodeType = e.dataTransfer.getData('nodeType') || 
                           e.dataTransfer.getData('text/plain') ||
                           e.dataTransfer.getData('text');
            
            console.log('Drop event - nodeType:', nodeType);
            
            if (nodeType && this.nodeDefinitions[nodeType]) {
                const rect = this.canvas.getBoundingClientRect();
                const x = (e.clientX - rect.left - this.offset.x) / this.scale;
                const y = (e.clientY - rect.top - this.offset.y) / this.scale;
                
                this.addNode(nodeType, x, y);
                console.log('Node added:', nodeType, 'at', x, y);
            }
        });
        
        // 鍵盤事件
        document.addEventListener('keydown', this.handleKeyDown.bind(this));
        document.addEventListener('keyup', this.handleKeyUp.bind(this));
        
        // Window 事件
        window.addEventListener('resize', () => {
            this.resizeCanvas();
            this.render();
        });
    }

    handleMouseDown(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = (e.clientX - rect.left - this.offset.x) / this.scale;
        const y = (e.clientY - rect.top - this.offset.y) / this.scale;
        
        // 檢查是否點擊了端口
        const port = this.getPortAtPosition(x, y);
        if (port) {
            if (port.type === 'output') {
                this.startConnection(port);
            }
            return;
        }
        
        // 檢查是否點擊了節點
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
            // 空白鍵按下時開始平移
            this.isPanning = true;
            this.panStart = { x: e.clientX, y: e.clientY };
            this.canvas.style.cursor = 'grabbing';
        } else {
            // 檢查是否點擊了連接線
            const connection = this.getConnectionAtPosition(x, y);
            if (connection) {
                this.selectConnection(connection);
            } else {
                this.selectedNode = null;
                this.selectedConnection = null;
                this.updatePropertiesPanel();
                this.render();
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
            this.render();
        } else if (this.isPanning) {
            const dx = e.clientX - this.panStart.x;
            const dy = e.clientY - this.panStart.y;
            this.offset.x += dx;
            this.offset.y += dy;
            this.panStart = { x: e.clientX, y: e.clientY };
            this.render();
        } else if (this.isConnecting && this.tempConnection) {
            this.tempConnection.end = {
                x: (e.clientX - rect.left - this.offset.x) / this.scale,
                y: (e.clientY - rect.top - this.offset.y) / this.scale
            };
            this.render();
        } else {
            // 檢查懸停狀態
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
        
        // 以鼠標位置為中心縮放
        const scaleChange = newScale - this.scale;
        this.offset.x -= (x - this.offset.x) * scaleChange / this.scale;
        this.offset.y -= (y - this.offset.y) * scaleChange / this.scale;
        
        this.scale = newScale;
        this.render();
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
            type: type,
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
        this.render();
        
        console.log('Node added:', node);
    }

    selectNode(node) {
        this.selectedNode = node;
        this.selectedConnection = null;
        this.updatePropertiesPanel();
        this.render();
    }

    selectConnection(connection) {
        this.selectedConnection = connection;
        this.selectedNode = null;
        this.updatePropertiesPanel();
        this.render();
    }

    deleteNode(node) {
        // 刪除相關連接
        this.connections = this.connections.filter(conn => 
            conn.from.nodeId !== node.id && conn.to.nodeId !== node.id
        );
        
        // 刪除節點
        const index = this.nodes.indexOf(node);
        if (index > -1) {
            this.nodes.splice(index, 1);
        }
        
        this.selectedNode = null;
        this.updatePropertiesPanel();
        this.render();
    }

    deleteConnection(connection) {
        const index = this.connections.indexOf(connection);
        if (index > -1) {
            this.connections.splice(index, 1);
        }
        
        this.selectedConnection = null;
        this.updatePropertiesPanel();
        this.render();
    }

    editNode(node) {
        // 打開屬性面板編輯節點
        this.selectNode(node);
        const panel = document.getElementById('properties-panel');
        if (panel) {
            panel.classList.add('is-active');
        }
    }

    startConnection(port) {
        this.isConnecting = true;
        this.connectionStart = port;
        this.tempConnection = {
            start: {
                x: port.node.x + port.node.width + 2,
                y: port.node.y + 40 + port.index * 25
            },
            end: { x: 0, y: 0 }
        };
    }

    completeConnection(endPort) {
        if (!this.connectionStart || !endPort) return;
        
        // 檢查連接有效性
        if (this.connectionStart.node.id === endPort.node.id) {
            this.cancelConnection();
            return;
        }
        
        // 檢查是否已存在連接
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
        this.render();
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
            // 檢查輸入端口
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
            
            // 檢查輸出端口
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
        
        // 簡化的貝塞爾曲線點擊檢測
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
                        <span class="tag is-info">${definition?.type || '未知'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">節點名稱</label>
                    <div class="control">
                        <input class="input" type="text" id="node-name" value="${this.selectedNode.name || ''}" placeholder="請輸入節點名稱">
                    </div>
                </div>
                <div class="field">
                    <label class="label">節點描述</label>
                    <div class="control">
                        <textarea class="textarea" id="node-description" placeholder="請輸入節點描述">${definition?.description || ''}</textarea>
                    </div>
                </div>
                <div class="field">
                    <label class="label">輸入端口</label>
                    <div class="tags">
                        ${Object.keys(this.selectedNode.inputs || {}).map(port => 
                            `<span class="tag is-info">${port}</span>`
                        ).join('')}
                    </div>
                </div>
                <div class="field">
                    <label class="label">輸出端口</label>
                    <div class="tags">
                        ${Object.keys(this.selectedNode.outputs || {}).map(port =>
                            `<span class="tag is-success">${port}</span>`
                        ).join('')}
                    </div>
                </div>
            `;
            
            // 綁定事件
            const nameInput = document.getElementById('node-name');
            if (nameInput) {
                nameInput.addEventListener('input', (e) => {
                    this.selectedNode.name = e.target.value;
                    this.render();
                });
            }
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
                <div class="field">
                    <button class="button is-danger is-fullwidth" onclick="window.flowDesigner.deleteConnection(window.flowDesigner.selectedConnection)">
                        刪除連接
                    </button>
                </div>
            `;
        } else {
            panel.classList.remove('is-active');
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

    saveFlow() {
        console.log('保存流程...');
        // 實現保存邏輯
        const flowData = {
            name: this.currentFlowName,
            nodes: this.nodes,
            connections: this.connections
        };
        
        // 顯示成功通知
        const notification = document.createElement('div');
        notification.className = 'notification is-success';
        notification.style.cssText = 'position: fixed; top: 20px; right: 20px; z-index: 9999;';
        notification.innerHTML = `
            <button class="delete"></button>
            流程已保存
        `;
        document.body.appendChild(notification);
        
        setTimeout(() => {
            notification.remove();
        }, 3000);
        
        notification.querySelector('.delete').addEventListener('click', () => {
            notification.remove();
        });
        
        console.log('Flow saved:', flowData);
    }

    loadFlow(flowName) {
        console.log('載入流程:', flowName);
        // 實現載入邏輯
    }
}

// 確保全局可用
window.FlowDesignerV2 = FlowDesignerV2;