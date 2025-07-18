// OPUI 分離式狀態管理系統 - 參考 AGVCUI 架構
import { createStore } from './lib/miniStore.js';

/**
 * 獲取儲存的客戶端 ID（如果有的話）
 */
function getStoredClientId() {
    const clientId = localStorage.getItem('opui_client_id');
    return clientId;
}

/**
 * 儲存客戶端 ID 到 localStorage
 */
function storeClientId(clientId) {
    localStorage.setItem('opui_client_id', clientId);
}

// ===== 使用者和連線狀態管理 =====
const userStore = createStore('opuiUserState', {
    clientId: getStoredClientId(),
    machineId: 1,
    isConnected: false,
    userAgent: ''
});

// ===== 操作狀態管理 - 左右兩側的產品配置 =====
const operationStore = createStore('opuiOperationState', {
    left: {
        productSelected: 0,
        products: [
            { name: 'ABC12345', size: 'S', id: 1, count: 32, room: 2, rackId: null },
            { name: 'DEF67890', size: 'L', id: 2, count: 16, room: 2, rackId: null }
        ]
    },
    right: {
        productSelected: 0,
        products: [
            { name: 'ABC54321', size: 'S', id: 3, count: 32, room: 2, rackId: null },
            { name: 'DEF09876', size: 'L', id: 4, count: 16, room: 2, rackId: null }
        ]
    }
});

// ===== 基礎資料管理 =====
const dataStore = createStore('opuiDataState', {
    products: [
        { id: 1, name: 'ABC12345', size: 'S' },
        { id: 2, name: 'DEF67890', size: 'L' },
        { id: 3, name: 'ABC54321', size: 'S' },
        { id: 4, name: 'DEF09876', size: 'L' }
    ],
    machines: [
        { id: 1, enable: 1 },
        { id: 2, enable: 1 },
        { id: 3, enable: 0 },
        { id: 4, enable: 0 }
    ],
    rooms: [
        { id: 1, enable: 1 },
        { id: 2, enable: 1 },
        { id: 3, enable: 0 },
        { id: 4, enable: 0 },
        { id: 5, enable: 0 }
    ],
    parking: {
        left: [
            { id: 1, name: '001' },
            { id: 2, name: '002' }
        ],
        right: [
            { id: 3, name: '003' },
            { id: 4, name: '004' }
        ]
    }
});

// ===== 任務狀態管理 =====
const tasksStore = createStore('opuiTasksState', {
    active: {
        left: null,   // { taskId: 123, type: 'call_empty', status: 'pending' }
        right: null   // { taskId: 124, type: 'dispatch_full', status: 'pending' }
    }
});

// ===== UI 狀態管理 =====
const uiStore = createStore('opuiUIState', {
    loading: false,
    currentPage: 'home' // 'home' | 'setting'
    // 移除 notifications 陣列 - 只使用即時通知，不儲存歷史
});

/**
 * 狀態操作輔助函數 - 適配分離式 store 架構
 * 提供簡化的狀態更新方法
 */
const stateHelpers = {
    // 使用者相關
    setUser(userData) {
        const currentState = userStore.getState();
        const newUserState = { ...currentState, ...userData };
        userStore.setState(newUserState);

        // 儲存客戶端 ID
        if (userData.clientId) {
            storeClientId(userData.clientId);
        }
    },

    // 更新產品選擇
    updateProductSelection(side, productIndex) {
        const currentState = operationStore.getState();
        operationStore.setState({
            ...currentState,
            [side]: {
                ...currentState[side],
                productSelected: productIndex
            }
        });
    },

    // 更新產品資料
    updateProduct(side, productIndex, productData) {
        const currentState = operationStore.getState();
        const products = [...currentState[side].products];
        products[productIndex] = { ...products[productIndex], ...productData };

        operationStore.setState({
            ...currentState,
            [side]: {
                ...currentState[side],
                products
            }
        });
    },

    // 更新基礎資料
    updateData(dataType, newData) {
        const currentState = dataStore.getState();
        dataStore.setState({
            ...currentState,
            [dataType]: newData
        });

        // 如果是產品資料更新，同步更新操作狀態中的產品資訊
        if (dataType === 'products') {
            this.syncProductsToOperation(newData);
        }
    },

    // 同步產品資料到操作狀態
    syncProductsToOperation(products) {
        //console.log('🔄 同步產品資料到操作狀態:', products);
        const currentState = operationStore.getState();
        //console.log('🔄 當前操作狀態:', currentState);
        const updatedOperation = { ...currentState };

        ['left', 'right'].forEach(side => {
            if (updatedOperation[side] && updatedOperation[side].products) {
                updatedOperation[side].products = updatedOperation[side].products.map(opProduct => {
                    //console.log(`🔄 ${side} 側處理產品:`, opProduct);
                    if (opProduct.id) {
                        const fullProduct = products.find(p => p.id === opProduct.id);
                        //console.log(`🔄 ${side} 側找到匹配產品:`, fullProduct);
                        if (fullProduct) {
                            // 保留原本的操作狀態屬性（如 count, room, rackId），只更新基本產品資訊
                            const updatedProduct = {
                                ...opProduct,
                                name: fullProduct.name,
                                size: fullProduct.size
                            };
                            //console.log(`🔄 ${side} 側更新後產品:`, updatedProduct);
                            return updatedProduct;
                        }
                    }
                    return opProduct;
                });
            }
        });

        //console.log('🔄 更新後操作狀態:', updatedOperation);
        operationStore.setState(updatedOperation);
    },

    // 更新任務狀態
    updateTask(side, taskData) {
        const currentState = tasksStore.getState();
        tasksStore.setState({
            ...currentState,
            active: {
                ...currentState.active,
                [side]: taskData
            }
        });
    },

    // 同步活躍任務狀態（前端重新載入時）
    syncActiveTasks(activeTasks) {
        console.log('🔄 同步活躍任務狀態:', activeTasks);

        const newActiveTasks = {};

        // 添加活躍任務
        Object.keys(activeTasks).forEach(side => {
            const taskInfo = activeTasks[side];
            newActiveTasks[side] = {
                type: taskInfo.task_type,
                status: taskInfo.status || 'pending',  // 🔧 使用後端提供的詳細狀態
                task_id: taskInfo.task_id,
                node_id: taskInfo.node_id,
                createdAt: taskInfo.createdAt ? new Date(taskInfo.createdAt * 1000).toISOString() : new Date().toISOString()
            };
        });

        const currentState = tasksStore.getState();
        tasksStore.setState({
            ...currentState,
            active: newActiveTasks
        });

        console.log('✅ 活躍任務狀態同步完成:', newActiveTasks);
    },

    // UI 相關操作
    setLoading(loading) {
        const currentState = uiStore.getState();
        uiStore.setState({
            ...currentState,
            loading
        });
    },

    setCurrentPage(page) {
        const currentState = uiStore.getState();
        uiStore.setState({
            ...currentState,
            currentPage: page
        });
    },

    // 通知歷史功能已移除 - 只使用即時通知 (notify.js)
    // 如需顯示通知，請使用 notify.showSuccess(), notify.showError() 等方法
};

// ===== 匯出分離式 stores =====
export {
    userStore,
    operationStore,
    dataStore,
    tasksStore,
    uiStore,
    stateHelpers
};

// ===== 為了向後相容，提供統一的 getter 函數 =====
export const getClientState = () => ({
    ...userStore.getState(),
    op: operationStore.getState(),
    machineId: userStore.getState().machineId,
    clientId: userStore.getState().clientId
});

export const getDataState = (dataType) => dataStore.getState()[dataType];
export const getTaskState = () => tasksStore.getState();
export const getUIState = () => uiStore.getState();

// ===== 導出 clientId 管理函數 =====
export { storeClientId };


