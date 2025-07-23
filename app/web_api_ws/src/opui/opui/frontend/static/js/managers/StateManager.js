/**
 * 簡化狀態管理器
 * 採用單向資料流：用戶操作 → 後端處理 → Socket 事件 → store 更新 → UI 更新
 * 移除複雜的同步控制邏輯，讓系統更簡單直接
 */
import { socketAPI } from '../api.js';
import { notify } from '../notify.js';

export class StateManager {
    constructor() {
        // 簡化架構：移除複雜的同步控制邏輯
        // 不再需要 isSyncing、suppressServerUpdates、lastSyncedState 等標記
        // console.log('🔄 初始化簡化狀態管理器 - 採用單向資料流架構');
    }

    /**
     * 設定狀態變更監聽（簡化版）
     * 只負責 UI 更新，不處理複雜的同步邏輯
     */
    setupStateListeners(appStore) {
        appStore.on('change', (newState) => {
            // console.debug('📊 應用程式狀態變更:', newState);

            // 簡化：只負責 UI 更新
            // 所有同步都由用戶操作直接觸發 socketAPI.updateClient()
            // 後端透過 Socket 事件直接更新 store 狀態
            if (window.opuiApp && window.opuiApp.uiManager) {
                window.opuiApp.uiManager.updateUI(newState);
            }
        });
    }

    /**
     * 初始化狀態同步（簡化版）
     * 只在初始化時觸發一次同步，載入伺服器資料
     */
    initializeSync(appStore) {
        // console.log('🔄 初始化簡化狀態同步');

        // 設定狀態監聽器
        this.setupStateListeners(appStore);

        // 初始化時觸發一次同步，載入伺服器資料
        setTimeout(() => {
            const currentState = appStore.getState();
            // console.log('🔄 初始化同步，載入伺服器資料');

            socketAPI.updateClient(currentState)
                .then(() => {
                    // console.log('✅ 初始化同步完成');
                })
                .catch((error) => {
                    console.error('❌ 初始化同步失敗:', error);
                    notify.showErrorMessage('初始化同步失敗，請重新整理頁面');
                });
        }, 1000);

        // console.log('✅ 簡化狀態同步已初始化 - 採用單向資料流');
    }

    /**
     * 執行恢復原廠操作（簡化版）
     */
    executeFactoryRestore() {
        // console.log('🏭 執行恢復原廠操作');

        // 清除本機儲存
        localStorage.clear();

        // 重置應用程式狀態到初始值（保留伺服器資料）
        const currentState = window.appStore.getState();
        const initialState = {
            user: {
                clientId: null,
                machineId: 1,
                isConnected: currentState.user.isConnected, // 保留連線狀態
                userAgent: currentState.user.userAgent
            },
            operation: {
                left: {
                    productSelected: 0,
                    products: [
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null },
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null }
                    ]
                },
                right: {
                    productSelected: 0,
                    products: [
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null },
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null }
                    ]
                }
            },
            data: {
                // 保留從伺服器載入的資料，不要重置為固定值
                products: currentState.data.products || [{ id: 1, name: 'NODATA', size: 'S' }],
                machines: currentState.data.machines || [],
                rooms: currentState.data.rooms || [],
                parking: currentState.data.parking || { left: [], right: [] }
            },
            tasks: { left: null, right: null },
            notifications: []
        };

        window.appStore.setState(initialState);

        // 簡化：直接同步到伺服器，不需要複雜的同步控制
        socketAPI.updateClient(window.appStore.getState())
            .then(() => {
                // console.log('✅ 恢復原廠設定同步完成');
            })
            .catch((error) => {
                console.error('❌ 恢復原廠設定同步失敗:', error);
                notify.showErrorMessage('恢復原廠設定失敗');
            });

        // 如果在設定頁面，重新綁定機台按鈕
        if (window.opuiApp && window.opuiApp.pageManager) {
            const currentPage = window.opuiApp.pageManager.getCurrentPage();
            if (currentPage === 'settings') {
                // 延遲一點重新綁定，確保狀態已更新
                setTimeout(() => {
                    if (window.opuiApp.eventManager) {
                        window.opuiApp.eventManager.bindMachineButtons();
                    }
                    if (window.opuiApp.uiManager) {
                        const newState = window.appStore.getState();
                        window.opuiApp.uiManager.updateProductInputs(newState.operation);
                        window.opuiApp.uiManager.updateProductValidation(newState.data.products);
                    }
                }, 100);
            }
        }

        // 顯示成功訊息
        notify.showNotifyMessage("已恢復原廠設定");

        // console.log('✅ 恢復原廠操作完成');
    }

    /**
     * 強制同步狀態（簡化版）
     */
    forceSyncToServer(appStore) {
        const currentState = appStore.getState();
        // console.log('🔄 強制同步狀態到伺服器');

        // 簡化：直接同步，不需要複雜的狀態比較
        socketAPI.updateClient(currentState)
            .then(() => {
                // console.log('✅ 強制同步完成');
            })
            .catch((error) => {
                console.error('❌ 強制同步失敗:', error);
                notify.showErrorMessage('同步失敗，請重試');
            });
    }

    /**
     * 檢查狀態一致性（簡化版）
     */
    checkStateConsistency(appStore) {
        const currentState = appStore.getState();

        // 簡化：只檢查基本的必要欄位
        const requiredFields = [
            'user.machineId',
            'user.isConnected',
            'operation.left',
            'operation.right'
        ];

        const missingFields = [];

        requiredFields.forEach(field => {
            const fieldPath = field.split('.');
            let value = currentState;

            for (const key of fieldPath) {
                if (value && typeof value === 'object' && key in value) {
                    value = value[key];
                } else {
                    missingFields.push(field);
                    break;
                }
            }
        });

        if (missingFields.length > 0) {
            // console.warn('⚠️ 狀態一致性檢查失敗，缺少欄位:', missingFields);
            return false;
        }

        // console.debug('✅ 狀態一致性檢查通過');
        return true;
    }

    /**
     * 修復狀態（簡化版）
     */
    repairState(appStore) {
        // console.log('🔧 嘗試修復狀態');

        const currentState = appStore.getState();
        const repairedState = { ...currentState };

        // 修復 user 狀態
        if (!repairedState.user) {
            repairedState.user = {
                clientId: null,
                machineId: 1,
                isConnected: false,
                userAgent: navigator.userAgent
            };
        }

        // 修復 operation 狀態
        if (!repairedState.operation) {
            repairedState.operation = {
                left: {
                    productSelected: 0,
                    products: [
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null },
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null }
                    ]
                },
                right: {
                    productSelected: 0,
                    products: [
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null },
                        { name: '', size: 'S', id: null, count: 32, room: 2, rackId: null }
                    ]
                }
            };
        }

        // 修復 data 狀態
        if (!repairedState.data) {
            repairedState.data = {
                products: [{ id: 1, name: 'NODATA', size: 'S' }],
                machines: [],
                rooms: [],
                parking: { left: [], right: [] }
            };
        }

        // 修復 tasks 狀態
        if (!repairedState.tasks) {
            repairedState.tasks = { left: null, right: null };
        }

        // 修復 notifications 狀態
        if (!repairedState.notifications) {
            repairedState.notifications = [];
        }

        // 應用修復後的狀態
        appStore.setState(repairedState);

        // console.log('✅ 狀態修復完成');
        return repairedState;
    }
}
