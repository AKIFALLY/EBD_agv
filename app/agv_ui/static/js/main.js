// Cargo AGV 監控系統前端 JavaScript

class AGVMonitor {
    constructor() {
        this.socket = null;
        this.isConnected = false;
        this.reconnectInterval = null;
        
        this.init();
    }
    
    init() {
        this.initWebSocket();
        this.bindEvents();
    }
    
    initWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;
        
        try {
            this.socket = new WebSocket(wsUrl);
            
            this.socket.onopen = () => {
                this.onConnected();
            };
            
            this.socket.onmessage = (event) => {
                this.handleMessage(JSON.parse(event.data));
            };
            
            this.socket.onclose = () => {
                this.onDisconnected();
            };
            
            this.socket.onerror = (error) => {
                console.error('WebSocket 錯誤:', error);
                this.onDisconnected();
            };
            
        } catch (error) {
            console.error('連接失敗:', error);
            this.onDisconnected();
        }
    }
    
    onConnected() {
        this.isConnected = true;
        this.updateConnectionStatus('connected');
        console.log('WebSocket 連接成功');
        
        // 清除重連定時器
        if (this.reconnectInterval) {
            clearInterval(this.reconnectInterval);
            this.reconnectInterval = null;
        }
        
        // 關閉離線 Modal
        const offlineModal = bootstrap.Modal.getInstance(document.getElementById('offlineModal'));
        if (offlineModal) {
            offlineModal.hide();
        }
    }
    
    onDisconnected() {
        this.isConnected = false;
        this.updateConnectionStatus('disconnected');
        console.warn('WebSocket 連接中斷');
        
        // 顯示離線 Modal
        const offlineModal = new bootstrap.Modal(document.getElementById('offlineModal'));
        offlineModal.show();
        
        // 開始重連
        this.startReconnect();
    }
    
    startReconnect() {
        if (this.reconnectInterval) return;
        
        console.log('開始自動重連...');
        this.reconnectInterval = setInterval(() => {
            if (!this.isConnected) {
                console.log('嘗試重新連接...');
                this.initWebSocket();
            }
        }, 5000);
    }
    
    updateConnectionStatus(status) {
        const statusElement = document.getElementById('connectionStatus');
        const statusMap = {
            connected: {
                class: 'bg-success',
                icon: 'fas fa-check-circle',
                text: '已連接'
            },
            disconnected: {
                class: 'bg-danger',
                icon: 'fas fa-times-circle',
                text: '已斷開'
            },
            connecting: {
                class: 'bg-warning',
                icon: 'fas fa-circle-notch fa-spin',
                text: '連接中...'
            }
        };
        
        const config = statusMap[status];
        statusElement.className = `badge ${config.class} me-3`;
        statusElement.innerHTML = `<i class="${config.icon} me-1"></i>${config.text}`;
    }
    
    handleMessage(data) {
        try {
            switch (data.type) {
                case 'full_data':
                    this.updateFullData(data.data);
                    if (data.summary) {
                        this.updateSummary(data.summary);
                    }
                    break;
                case 'summary':
                    this.updateSummary(data.data);
                    break;
                default:
                    console.warn('Unknown message type:', data.type);
            }
        } catch (error) {
            console.error('處理訊息錯誤:', error);
        }
    }
    
    updateFullData(data) {
        if (!data) return;
        
        // 更新最後更新時間
        const timestamp = data.metadata?.timestamp;
        if (timestamp) {
            const updateElement = document.getElementById('lastUpdate');
            const time = new Date(timestamp).toLocaleString('zh-TW');
            updateElement.textContent = `最後更新: ${time}`;
        }
        
        // 更新狀態機 (使用新的 contexts 結構)
        this.updateStateMachines(data.contexts);
        
        // 更新設備狀態 (如果存在)
        if (data.devices) {
            this.updateDevices(data.devices);
        }
        
        // 更新完整 AGV 狀態變數
        this.updateAGVStatusVariables(data.agv_status_complete);
        
        // 更新 AGV 基礎節點變數
        this.updateAGVBaseVariables(data.agv_base_variables);
        
        // 暫時清空其他區塊
        this.clearUnusedSections();
        
        // 更新 PLC 狀態 (使用完整狀態)
        this.updatePLCStatus(data.agv_status_complete);
        
        console.log('資料已更新');
    }
    
    updateSummary(summary) {
        if (!summary) return;
        
        // 更新系統概覽
        document.getElementById('agvId').textContent = summary.agv_status?.agv_id || '-';
        document.getElementById('powerVoltage').textContent = 
            summary.agv_status?.power ? `${Number(summary.agv_status.power).toFixed(2)}%` : '-';
        document.getElementById('historyCount').textContent = 
            summary.history_count || '0';
        
        // 更新運行模式
        const autoMode = summary.agv_status?.auto;
        const autoElement = document.getElementById('autoMode');
        const autoIcon = document.getElementById('autoModeIcon');
        
        if (autoMode !== undefined) {
            if (autoMode) {
                autoElement.className = 'badge bg-success';
                autoElement.textContent = '自動';
                autoIcon.className = 'display-6 text-success';
            } else {
                autoElement.className = 'badge bg-warning text-dark';
                autoElement.textContent = '手動';
                autoIcon.className = 'display-6 text-warning';
            }
        }
        
        // 更新移動狀態
        const moving = summary.agv_status?.moving;
        const movingElement = document.getElementById('movingStatus');
        const movingIcon = document.getElementById('movingIcon');
        
        if (moving !== undefined) {
            if (moving) {
                movingElement.className = 'badge bg-info';
                movingElement.textContent = '移動中';
                movingIcon.className = 'display-6 text-info';
            } else {
                movingElement.className = 'badge bg-secondary';
                movingElement.textContent = '靜止';
                movingIcon.className = 'display-6 text-secondary';
            }
        }
        
        // 更新警報狀態
        const alarm = summary.agv_status?.alarm;
        const alarmElement = document.getElementById('alarmStatus');
        const alarmIcon = document.getElementById('alarmIcon');
        
        if (alarm !== undefined) {
            if (alarm) {
                alarmElement.className = 'badge bg-danger';
                alarmElement.textContent = '警報';
                alarmIcon.className = 'display-6 text-danger pulse';
            } else {
                alarmElement.className = 'badge bg-success';
                alarmElement.textContent = '正常';
                alarmIcon.className = 'display-6 text-success';
            }
        }
    }
    
    updateStateMachines(contexts) {
        if (!contexts) return;
        
        const stateMap = {
            base_context: { element: 'baseState' },
            cargo_context: { element: 'cargoState' },
            robot_context: { element: 'robotState' }
        };
        
        Object.entries(stateMap).forEach(([key, config]) => {
            const contextData = contexts[key];
            if (contextData) {
                const stateElement = document.getElementById(config.element);
                
                const stateName = contextData.current_state || '-';
                stateElement.textContent = stateName;
            }
        });
    }
    
    updateDevices(devices) {
        if (!devices) return;
        
        const devicesList = document.getElementById('devicesList');
        devicesList.innerHTML = '';
        
        Object.entries(devices).forEach(([deviceKey, deviceData]) => {
            const deviceItem = document.createElement('div');
            deviceItem.className = 'device-item fade-in';
            
            const deviceName = deviceData.device_name || deviceKey;
            const deviceClass = deviceData.device_class || 'Unknown';
            const variablesCount = Object.keys(deviceData.device_variables || {}).length;
            
            deviceItem.innerHTML = `
                <div class="d-flex justify-content-between align-items-center">
                    <div>
                        <strong class="d-flex align-items-center">
                            <span class="status-indicator active me-2"></span>
                            ${deviceName}
                        </strong>
                        <small class="text-muted">${deviceClass}</small>
                    </div>
                    <div class="text-end">
                        <span class="badge bg-info">${variablesCount} 項變數</span>
                    </div>
                </div>
            `;
            
            devicesList.appendChild(deviceItem);
        });
    }
    
    updatePLCStatus(plcVariables) {
        if (!plcVariables) return;
        
        const plcGrid = document.getElementById('plcStatusGrid');
        plcGrid.innerHTML = '';
        
        // AGV Status 變數 - 與 status_json_recorder.py 中指定變數保持一致
        const importantVars = [
            // 基本狀態變數 (移除AGV_ID，由metadata提供)
            'POWER', 'AGV_X_SPEED', 'AGV_Y_SPEED', 'AGV_THETA_SPEED',
            'AGV_FPGV', 'AGV_BPGV', 'AGV_START_POINT', 'AGV_END_POINT', 'AGV_ACTION',
            'AGV_ZONE', 'AGV_SLAM_X', 'AGV_SLAM_Y', 'AGV_SLAM_THETA', 'AGV_LAYER',
            'AGV_ID1', 'MAGIC', 'AGV_Auto', 'AGV_MOVING', 'AGV_ALARM',
            // 路徑和任務狀態
            'AGV_PATH', 'AGV_PATH_REQ', 'AGV_IN_MISSION', 'AGV_LOCAL', 'AGV_LD_COMPLETE',
            'AGV_UD_COMPLETE', 'LOW_POWER', 'MISSION_CANCEL', 'TRAFFIC_STOP', 'TRAFFIC_ALLOW',
            'PS_RETRUN', 'AGV_MANUAL', 'AGV_2POSITION',
            // 輸入信號
            'IN_1', 'IN_2', 'IN_3', 'IN_4', 'IN_5', 'BARCODE_READER_FINISH', 'TAG_REQ', 'Req_TAGNo',
            // 門控系統 (完整的8個門)
            'DOOR_OPEN_1', 'DOOR_CLOSE_1', 'DOOR_OPEN_2', 'DOOR_CLOSE_2',
            'DOOR_OPEN_3', 'DOOR_CLOSE_3', 'DOOR_OPEN_4', 'DOOR_CLOSE_4',
            'DOOR_OPEN_5', 'DOOR_CLOSE_5', 'DOOR_OPEN_6', 'DOOR_CLOSE_6',
            'DOOR_OPEN_7', 'DOOR_CLOSE_7', 'DOOR_OPEN_8', 'DOOR_CLOSE_8'
        ];
        
        // 使用所有變數 (57個變數，調整為8x8=64個格子)
        const selectedVars = importantVars;
        
        selectedVars.forEach(varName => {
            if (plcVariables.hasOwnProperty(varName)) {
                const value = plcVariables[varName];
                const gridItem = document.createElement('div');
                gridItem.className = 'plc-status-item';
                
                let displayValue = value;
                let valueClass = 'text-info';
                
                // 特殊處理某些變數
                if (typeof value === 'boolean') {
                    displayValue = value ? '✓' : '✗';
                    valueClass = value ? 'text-success' : 'text-secondary';
                } else if (typeof value === 'number') {
                    if (varName.includes('SPEED')) {
                        displayValue = `${value} mm/s`;
                    } else if (varName === 'POWER') {
                        displayValue = `${Number(value).toFixed(2)}%`;
                        valueClass = value > 20 ? 'text-success' : 'text-warning';
                    } else if (varName.includes('SLAM_X') || varName.includes('SLAM_Y')) {
                        displayValue = `${value} mm`;
                    } else if (varName.includes('SLAM_THETA')) {
                        displayValue = `${value}°`;
                    } else if (varName.includes('POINT') || varName === 'AGV_ZONE' || varName === 'AGV_LAYER') {
                        displayValue = `${value}`;
                        valueClass = 'text-info';
                    } else if (varName === 'MAGIC' || varName === 'Req_TAGNo') {
                        displayValue = `${value}`;
                        valueClass = 'text-primary';
                    }
                } else if (value === null || value === undefined) {
                    displayValue = '-';
                    valueClass = 'text-muted';
                } else if (typeof value === 'string') {
                    displayValue = value.length > 8 ? value.substring(0, 8) + '...' : value;
                    valueClass = 'text-warning';
                }
                
                gridItem.innerHTML = `
                    <div class="status-content">
                        <div class="status-title-value">
                            <div class="status-label">${varName}</div>
                            <div class="status-value ${valueClass}">${displayValue}</div>
                        </div>
                    </div>
                `;
                
                plcGrid.appendChild(gridItem);
            } else {
                // 如果變數不存在，建立空白格子
                const gridItem = document.createElement('div');
                gridItem.className = 'plc-status-item';
                gridItem.innerHTML = `
                    <div class="status-content">
                        <div class="status-title-value">
                            <div class="status-label">${varName}</div>
                            <div class="status-value text-muted">-</div>
                        </div>
                    </div>
                `;
                plcGrid.appendChild(gridItem);
            }
        });
        
        // 如果實際變數少於64個，填充空白格子以維持網格結構  
        const currentCount = plcGrid.children.length;
        for (let i = currentCount; i < 64; i++) {
            const emptyItem = document.createElement('div');
            emptyItem.className = 'plc-status-item';
            emptyItem.innerHTML = `
                <div class="status-content">
                    <div class="status-title-value">
                        <div class="status-label">N/A</div>
                        <div class="status-value text-muted">-</div>
                    </div>
                </div>
            `;
            plcGrid.appendChild(emptyItem);
        }
    }
    
    updateAGVStatusVariables(agvStatusComplete) {
        if (!agvStatusComplete) return;
        
        const container = document.getElementById('agvStatusVariables');
        container.innerHTML = '';
        
        Object.entries(agvStatusComplete).forEach(([key, value]) => {
            const colDiv = document.createElement('div');
            colDiv.className = 'col-md-2 col-sm-4 col-6 mb-2';
            
            let displayValue = value;
            let valueClass = 'text-info';
            
            // 特殊處理不同類型的值
            if (typeof value === 'boolean') {
                displayValue = value ? '✓' : '✗';
                valueClass = value ? 'text-success' : 'text-secondary';
            } else if (typeof value === 'number') {
                if (key.includes('SPEED')) {
                    displayValue = `${value} mm/s`;
                } else if (key === 'POWER') {
                    displayValue = `${Number(value).toFixed(2)}%`;
                    valueClass = value > 20 ? 'text-success' : 'text-warning';
                } else if (key.includes('SLAM')) {
                    displayValue = `${value} mm`;
                }
            } else if (value === null || value === undefined) {
                displayValue = '-';
                valueClass = 'text-muted';
            }
            
            colDiv.innerHTML = `
                <div class="agv-status-item">
                    <div class="status-label small">${key}</div>
                    <div class="status-value ${valueClass} fw-bold">${displayValue}</div>
                </div>
            `;
            
            container.appendChild(colDiv);
        });
    }
    
    updateAGVBaseVariables(agvBaseVariables) {
        if (!agvBaseVariables) return;
        
        const container = document.getElementById('agvBaseVariables');
        container.innerHTML = '';
        
        Object.entries(agvBaseVariables).forEach(([key, value]) => {
            const colDiv = document.createElement('div');
            colDiv.className = 'col-md-3 col-sm-6 col-12 mb-3';
            
            let displayValue = value;
            let valueClass = 'text-info';
            let displayType = '';
            
            // 根據變數名稱和類型特殊處理
            if (value === null || value === undefined) {
                displayValue = '-';
                valueClass = 'text-muted';
            } else if (typeof value === 'string') {
                displayValue = value || '-';
                valueClass = 'text-warning';
                displayType = 'string';
            } else if (typeof value === 'number') {
                displayValue = value.toString();
                valueClass = 'text-success';
                displayType = 'number';
            } else if (typeof value === 'object' && value !== null) {
                // 處理複雜物件
                if (value._class) {
                    displayValue = `${value._class}`;
                    displayType = 'object';
                } else {
                    displayValue = JSON.stringify(value);
                    if (displayValue.length > 50) {
                        displayValue = displayValue.substring(0, 50) + '...';
                    }
                }
                valueClass = 'text-primary';
                displayType = 'object';
            }
            
            colDiv.innerHTML = `
                <div class="agv-status-item">
                    <div class="status-label small">${key}</div>
                    <div class="status-value ${valueClass} fw-bold">${displayValue}</div>
                    ${displayType ? `<div class="text-muted small">${displayType}</div>` : ''}
                </div>
            `;
            
            container.appendChild(colDiv);
        });
    }
    
    clearUnusedSections() {
        // 清空 AGV Core Node 變數區塊 (保留舊版區塊)
        const agvCoreNodeVariables = document.getElementById('agvCoreNodeVariables');
        if (agvCoreNodeVariables) {
            agvCoreNodeVariables.innerHTML = '<p class="text-muted">已改為顯示 AGV 基礎節點變數</p>';
        }
        
        // 清空 AGV Base Node 變數區塊 (保留舊版區塊)
        const agvBaseNodeVariables = document.getElementById('agvBaseNodeVariables');
        if (agvBaseNodeVariables) {
            agvBaseNodeVariables.innerHTML = '<p class="text-muted">已改為顯示 AGV 基礎節點變數</p>';
        }
        
        // 清空 Context 詳細資訊區塊
        const baseContextDetails = document.getElementById('baseContextDetails');
        if (baseContextDetails) {
            baseContextDetails.innerHTML = '<p class="text-muted">暫時停用，僅顯示基本狀態</p>';
        }
        
        const cargoContextDetails = document.getElementById('cargoContextDetails');
        if (cargoContextDetails) {
            cargoContextDetails.innerHTML = '<p class="text-muted">暫時停用，僅顯示基本狀態</p>';
        }
        
        const robotContextDetails = document.getElementById('robotContextDetails');
        if (robotContextDetails) {
            robotContextDetails.innerHTML = '<p class="text-muted">暫時停用，僅顯示基本狀態</p>';
        }
    }
    
    updateAGVCoreNodeVariables(coreNodeVars) {
        if (!coreNodeVars) return;
        
        const container = document.getElementById('agvCoreNodeVariables');
        container.innerHTML = '';
        
        Object.entries(coreNodeVars).forEach(([key, value]) => {
            const itemDiv = document.createElement('div');
            itemDiv.className = 'variable-item mb-2 p-2 bg-dark rounded';
            
            let displayValue = value;
            if (typeof value === 'object' && value !== null) {
                displayValue = JSON.stringify(value, null, 2);
            } else if (value === null || value === undefined) {
                displayValue = '-';
            }
            
            itemDiv.innerHTML = `
                <div class="d-flex justify-content-between">
                    <strong class="text-warning">${key}</strong>
                    <code class="text-info small">${displayValue}</code>
                </div>
            `;
            
            container.appendChild(itemDiv);
        });
    }
    
    updateAGVBaseNodeVariables(baseNodeVars) {
        if (!baseNodeVars) return;
        
        const container = document.getElementById('agvBaseNodeVariables');
        container.innerHTML = '';
        
        Object.entries(baseNodeVars).forEach(([key, value]) => {
            const itemDiv = document.createElement('div');
            itemDiv.className = 'variable-item mb-2 p-2 bg-dark rounded';
            
            let displayValue = value;
            if (typeof value === 'object' && value !== null) {
                displayValue = JSON.stringify(value, null, 2);
            } else if (value === null || value === undefined) {
                displayValue = '-';
            }
            
            itemDiv.innerHTML = `
                <div class="d-flex justify-content-between">
                    <strong class="text-success">${key}</strong>
                    <code class="text-info small">${displayValue}</code>
                </div>
            `;
            
            container.appendChild(itemDiv);
        });
    }
    
    updateContextDetails(contexts) {
        if (!contexts) return;
        
        const contextMap = {
            base_context: 'baseContextDetails',
            cargo_context: 'cargoContextDetails', 
            robot_context: 'robotContextDetails'
        };
        
        Object.entries(contextMap).forEach(([contextKey, containerId]) => {
            const contextData = contexts[contextKey];
            const container = document.getElementById(containerId);
            
            if (contextData && container) {
                container.innerHTML = `
                    <div class="context-info">
                        <div class="mb-3">
                            <h6 class="text-warning">當前狀態</h6>
                            <div class="badge bg-primary">${contextData.current_state || 'Unknown'}</div>
                        </div>
                        <div class="mb-3">
                            <h6 class="text-warning">狀態模組</h6>
                            <small class="text-muted">${contextData.state_module || 'Unknown'}</small>
                        </div>
                        <div class="context-variables">
                            <h6 class="text-warning">Context 變數</h6>
                            <div class="variables-list">
                                ${this.renderContextVariables(contextData.context_variables || {})}
                            </div>
                        </div>
                    </div>
                `;
            }
        });
    }
    
    renderContextVariables(variables) {
        if (!variables || Object.keys(variables).length === 0) {
            return '<small class="text-muted">無變數</small>';
        }
        
        return Object.entries(variables).map(([key, value]) => {
            let displayValue = value;
            if (typeof value === 'object' && value !== null) {
                displayValue = JSON.stringify(value);
            } else if (value === null || value === undefined) {
                displayValue = '-';
            }
            
            return `
                <div class="d-flex justify-content-between mb-1">
                    <span class="text-light small">${key}:</span>
                    <code class="text-info small">${displayValue}</code>
                </div>
            `;
        }).join('');
    }
    
    
    bindEvents() {
        // 頁面可見性變化事件
        document.addEventListener('visibilitychange', () => {
            if (document.visibilityState === 'visible' && !this.isConnected) {
                console.log('頁面重新顯示，檢查連接狀態');
                this.initWebSocket();
            }
        });
        
        // 網路狀態變化事件
        window.addEventListener('online', () => {
            console.log('網路連接已恢復');
            if (!this.isConnected) {
                this.initWebSocket();
            }
        });
        
        window.addEventListener('offline', () => {
            console.warn('網路連接已中斷');
        });
    }
}

// 全域函數 (保留空白以備未來使用)

// 頁面載入完成後初始化
document.addEventListener('DOMContentLoaded', () => {
    window.agvMonitor = new AGVMonitor();
});

// 錯誤處理
window.addEventListener('error', (event) => {
    console.error('JavaScript 錯誤:', event.error);
});

window.addEventListener('unhandledrejection', (event) => {
    console.error('未處理的 Promise 錯誤:', event.reason);
});