/**
 * 地圖操作審計記錄器
 * 記錄用戶在地圖上的所有操作，提供審計追蹤功能
 */

import { mapPermissions } from './mapPermissions.js';

export const mapAuditLogger = (() => {
    let operationHistory = [];
    let sessionId = null;
    let isLoggingEnabled = true;

    // 操作類型定義
    const OPERATION_TYPES = {
        VIEW: 'view',
        CREATE: 'create',
        UPDATE: 'update',
        DELETE: 'delete',
        MOVE: 'move',
        ASSIGN: 'assign',
        CONTROL: 'control',
        NAVIGATE: 'navigate'
    };

    // 資源類型定義
    const RESOURCE_TYPES = {
        TASK: 'task',
        RACK: 'rack',
        CARRIER: 'carrier',
        AGV: 'agv',
        EQUIPMENT: 'equipment',
        NODE: 'node',
        MAP: 'map'
    };

    // 初始化
    function init() {
        sessionId = generateSessionId();
        setupEventListeners();
        loadStoredHistory();

        // 記錄會話開始
        logOperation(OPERATION_TYPES.NAVIGATE, RESOURCE_TYPES.MAP, null, {
            action: 'session_start',
            sessionId: sessionId
        });

        console.log('Audit logger initialized with session:', sessionId);
    }

    // 生成會話 ID
    function generateSessionId() {
        return 'map_session_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
    }

    // 設置事件監聽器
    function setupEventListeners() {
        // 監聽頁面卸載，記錄會話結束
        window.addEventListener('beforeunload', () => {
            logOperation(OPERATION_TYPES.NAVIGATE, RESOURCE_TYPES.MAP, null, {
                action: 'session_end',
                sessionId: sessionId,
                duration: Date.now() - getSessionStartTime()
            });

            // 保存歷史記錄
            saveHistoryToStorage();
        });

        // 監聽頁面可見性變化
        document.addEventListener('visibilitychange', () => {
            const action = document.hidden ? 'page_hidden' : 'page_visible';
            logOperation(OPERATION_TYPES.NAVIGATE, RESOURCE_TYPES.MAP, null, {
                action: action,
                timestamp: new Date().toISOString()
            });
        });
    }

    // 記錄操作
    function logOperation(operationType, resourceType, resourceId, details = {}) {
        if (!isLoggingEnabled) return;

        const userInfo = mapPermissions.getUserInfo();
        const operation = {
            id: generateOperationId(),
            sessionId: sessionId,
            timestamp: new Date().toISOString(),
            operationType: operationType,
            resourceType: resourceType,
            resourceId: resourceId,
            user: {
                username: userInfo.username,
                role: userInfo.role,
                isLoggedIn: userInfo.isLoggedIn
            },
            details: details,
            userAgent: navigator.userAgent,
            url: window.location.href
        };

        // 添加到歷史記錄
        operationHistory.push(operation);

        // 限制歷史記錄大小
        if (operationHistory.length > 1000) {
            operationHistory = operationHistory.slice(-500); // 保留最新的500條
        }

        // 發送到後端（如果需要）
        sendToBackend(operation);

        // 觸發事件
        dispatchAuditEvent(operation);

        console.log('Operation logged:', operation);
    }

    // 生成操作 ID
    function generateOperationId() {
        return 'op_' + Date.now() + '_' + Math.random().toString(36).substr(2, 6);
    }

    // 記錄查看操作
    function logView(resourceType, resourceId, details = {}) {
        logOperation(OPERATION_TYPES.VIEW, resourceType, resourceId, {
            ...details,
            action: 'view_details'
        });
    }

    // 記錄創建操作
    function logCreate(resourceType, resourceId, details = {}) {
        logOperation(OPERATION_TYPES.CREATE, resourceType, resourceId, {
            ...details,
            action: 'create_new'
        });
    }

    // 記錄更新操作
    function logUpdate(resourceType, resourceId, changes = {}, details = {}) {
        logOperation(OPERATION_TYPES.UPDATE, resourceType, resourceId, {
            ...details,
            action: 'update_data',
            changes: changes
        });
    }

    // 記錄刪除操作
    function logDelete(resourceType, resourceId, details = {}) {
        logOperation(OPERATION_TYPES.DELETE, resourceType, resourceId, {
            ...details,
            action: 'delete_resource'
        });
    }

    // 記錄移動操作
    function logMove(resourceType, resourceId, fromLocation, toLocation, details = {}) {
        logOperation(OPERATION_TYPES.MOVE, resourceType, resourceId, {
            ...details,
            action: 'move_resource',
            fromLocation: fromLocation,
            toLocation: toLocation
        });
    }

    // 記錄分配操作
    function logAssign(resourceType, resourceId, assignTo, details = {}) {
        logOperation(OPERATION_TYPES.ASSIGN, resourceType, resourceId, {
            ...details,
            action: 'assign_resource',
            assignTo: assignTo
        });
    }

    // 記錄控制操作
    function logControl(resourceType, resourceId, controlAction, details = {}) {
        logOperation(OPERATION_TYPES.CONTROL, resourceType, resourceId, {
            ...details,
            action: 'control_equipment',
            controlAction: controlAction
        });
    }

    // 記錄導航操作
    function logNavigation(action, details = {}) {
        logOperation(OPERATION_TYPES.NAVIGATE, RESOURCE_TYPES.MAP, null, {
            ...details,
            action: action
        });
    }

    // 發送到後端
    async function sendToBackend(operation) {
        try {
            // 只發送重要操作到後端
            const importantOperations = [
                OPERATION_TYPES.VIEW,
                OPERATION_TYPES.CREATE,
                OPERATION_TYPES.UPDATE,
                OPERATION_TYPES.DELETE,
                OPERATION_TYPES.ASSIGN,
                OPERATION_TYPES.CONTROL
            ];

            if (importantOperations.includes(operation.operationType)) {
                // 轉換欄位名稱以匹配後端期望的格式
                const backendData = {
                    operation_id: operation.id,
                    session_id: operation.sessionId,
                    timestamp: operation.timestamp,
                    operation_type: operation.operationType,
                    resource_type: operation.resourceType,
                    resource_id: operation.resourceId ? String(operation.resourceId) : null,
                    user: operation.user,
                    details: operation.details,
                    user_agent: operation.userAgent,
                    url: operation.url
                };

                const response = await fetch('/api/audit_logs', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify(backendData)
                });

                if (!response.ok) {
                    console.warn('Failed to send audit log to backend:', response.status);
                    // 顯示詳細錯誤資訊
                    const errorText = await response.text();
                    console.error('Backend error response:', errorText);
                }
            }
        } catch (error) {
            console.error('Error sending audit log to backend:', error);
        }
    }

    // 觸發審計事件
    function dispatchAuditEvent(operation) {
        const event = new CustomEvent('mapAuditLog', {
            detail: operation
        });
        window.dispatchEvent(event);
    }

    // 獲取操作歷史
    function getHistory(filters = {}) {
        let filteredHistory = [...operationHistory];

        // 按時間範圍過濾
        if (filters.startTime) {
            filteredHistory = filteredHistory.filter(op =>
                new Date(op.timestamp) >= new Date(filters.startTime)
            );
        }

        if (filters.endTime) {
            filteredHistory = filteredHistory.filter(op =>
                new Date(op.timestamp) <= new Date(filters.endTime)
            );
        }

        // 按操作類型過濾
        if (filters.operationType) {
            filteredHistory = filteredHistory.filter(op =>
                op.operationType === filters.operationType
            );
        }

        // 按資源類型過濾
        if (filters.resourceType) {
            filteredHistory = filteredHistory.filter(op =>
                op.resourceType === filters.resourceType
            );
        }

        // 按用戶過濾
        if (filters.username) {
            filteredHistory = filteredHistory.filter(op =>
                op.user.username === filters.username
            );
        }

        // 按資源 ID 過濾
        if (filters.resourceId) {
            filteredHistory = filteredHistory.filter(op =>
                op.resourceId === filters.resourceId
            );
        }

        return filteredHistory.sort((a, b) =>
            new Date(b.timestamp) - new Date(a.timestamp)
        );
    }

    // 獲取統計資訊
    function getStatistics(timeRange = '24h') {
        const now = new Date();
        const startTime = new Date(now.getTime() - getTimeRangeMs(timeRange));

        const recentHistory = getHistory({ startTime: startTime.toISOString() });

        const stats = {
            totalOperations: recentHistory.length,
            operationsByType: {},
            operationsByResource: {},
            operationsByUser: {},
            timeRange: timeRange,
            startTime: startTime.toISOString(),
            endTime: now.toISOString()
        };

        recentHistory.forEach(op => {
            // 按操作類型統計
            stats.operationsByType[op.operationType] =
                (stats.operationsByType[op.operationType] || 0) + 1;

            // 按資源類型統計
            stats.operationsByResource[op.resourceType] =
                (stats.operationsByResource[op.resourceType] || 0) + 1;

            // 按用戶統計
            const username = op.user.username || 'anonymous';
            stats.operationsByUser[username] =
                (stats.operationsByUser[username] || 0) + 1;
        });

        return stats;
    }

    // 獲取時間範圍毫秒數
    function getTimeRangeMs(timeRange) {
        const ranges = {
            '1h': 60 * 60 * 1000,
            '24h': 24 * 60 * 60 * 1000,
            '7d': 7 * 24 * 60 * 60 * 1000,
            '30d': 30 * 24 * 60 * 60 * 1000
        };
        return ranges[timeRange] || ranges['24h'];
    }

    // 保存歷史記錄到本地儲存
    function saveHistoryToStorage() {
        try {
            const data = {
                sessionId: sessionId,
                history: operationHistory.slice(-100), // 只保存最新100條
                timestamp: new Date().toISOString()
            };
            localStorage.setItem('map_audit_history', JSON.stringify(data));
        } catch (error) {
            console.error('Failed to save audit history to storage:', error);
        }
    }

    // 從本地儲存載入歷史記錄
    function loadStoredHistory() {
        try {
            const stored = localStorage.getItem('map_audit_history');
            if (stored) {
                const data = JSON.parse(stored);
                // 只載入最近的記錄
                const recentTime = new Date(Date.now() - 24 * 60 * 60 * 1000);
                operationHistory = data.history.filter(op =>
                    new Date(op.timestamp) > recentTime
                );
            }
        } catch (error) {
            console.error('Failed to load stored audit history:', error);
        }
    }

    // 獲取會話開始時間
    function getSessionStartTime() {
        const sessionStart = operationHistory.find(op =>
            op.details?.action === 'session_start' && op.sessionId === sessionId
        );
        return sessionStart ? new Date(sessionStart.timestamp).getTime() : Date.now();
    }

    // 清除歷史記錄
    function clearHistory() {
        operationHistory = [];
        localStorage.removeItem('map_audit_history');
        console.log('Audit history cleared');
    }

    // 啟用/禁用記錄
    function setLoggingEnabled(enabled) {
        isLoggingEnabled = enabled;
        console.log('Audit logging', enabled ? 'enabled' : 'disabled');
    }

    // 公開方法
    return {
        init,
        logView,
        logCreate,
        logUpdate,
        logDelete,
        logMove,
        logAssign,
        logControl,
        logNavigation,
        getHistory,
        getStatistics,
        clearHistory,
        setLoggingEnabled,

        // 常量
        OPERATION_TYPES,
        RESOURCE_TYPES
    };
})();

// 全域暴露
window.mapAuditLogger = mapAuditLogger;
