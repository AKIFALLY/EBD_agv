// 簡化的 API 通訊層
import { userStore, operationStore, dataStore, tasksStore, uiStore, stateHelpers, storeClientId } from './store.js';
import { notify } from './notify.js';

/**
 * Socket.IO 通訊管理
 * 統一處理前後端通訊，簡化 Socket 事件處理
 */
class SocketAPI {
    constructor() {
        this.socket = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.initRetryCount = 0;
        this.maxInitRetries = 10;
        this.pendingRequests = new Map(); // 追蹤待處理的請求
        this.eventListeners = new Map(); // 追蹤事件監聽器
        this.connectedCallbacks = []; // 連線成功回調函數
    }

    /**
     * 清理 Socket 連線和事件監聽器
     */
    cleanup() {
        if (this.socket) {
            // 清理所有事件監聽器
            this.eventListeners.forEach((listeners, eventName) => {
                listeners.forEach(listener => {
                    this.socket.off(eventName, listener);
                });
            });
            this.eventListeners.clear();

            // 清理待處理的請求
            this.pendingRequests.clear();

            // 斷開連接
            this.socket.disconnect();
            this.socket = null;
        }

        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.initRetryCount = 0;

        // //console.log('🧹 Socket 連線和事件監聽器已清理');
    }

    /**
     * 獲取 Socket.IO 連線 URL（修復版）
     */
    getSocketUrl() {
        const hostname = window.location.hostname;
        const protocol = window.location.protocol === 'https:' ? 'https' : 'http';
        const currentPort = window.location.port;

        //console.log(`🔍 當前位置: ${protocol}://${hostname}:${currentPort}`);

        // 如果當前頁面的 hostname 是 op.ui，則連接到對應的後端
        if (hostname === 'op.ui') {
            const socketUrl = `${protocol}://op.ui:8002`;
            //console.log(`🎯 op.ui 環境，Socket URL: ${socketUrl}`);
            return socketUrl;
        }

        // 如果是 localhost 或其他情況，使用當前域名
        if (hostname === 'localhost' || hostname === '127.0.0.1') {
            const socketUrl = `${protocol}://${hostname}:8002`;
            //console.log(`🏠 本地環境，Socket URL: ${socketUrl}`);
            return socketUrl;
        }

        // 如果當前頁面已經在 8002 埠，使用相同的 host 和 port
        if (currentPort === '8002') {
            const socketUrl = `${protocol}://${hostname}:8002`;
            //console.log(`✅ 已在 8002 埠，Socket URL: ${socketUrl}`);
            return socketUrl;
        }

        // 預設情況：嘗試連接到 8002 埠
        const defaultUrl = `${protocol}://${hostname}:8002`;
        //console.log(`🔧 預設情況，Socket URL: ${defaultUrl}`);
        return defaultUrl;
    }

    /**
     * 初始化 Socket 連線（改善版 - 加強重試機制和錯誤處理）
     */
    init() {
        if (this.socket) return;

        if (typeof io === "undefined") {
            this.initRetryCount++;

            if (this.initRetryCount > this.maxInitRetries) {
                //console.error("❌ Socket.IO 載入失敗，已達最大重試次數");
                notify.showErrorMessage("Socket.IO 載入失敗，請重新整理頁面");
                return;
            }

            // //console.warn(`Socket.IO 尚未載入，重試中 (${this.initRetryCount}/${this.maxInitRetries})`);
            // 使用指數退避重試間隔
            const retryDelay = Math.min(50 * Math.pow(2, this.initRetryCount - 1), 5000);
            setTimeout(() => this.init(), retryDelay);
            return;
        }

        // 重置重試計數器
        this.initRetryCount = 0;

        try {
            // 立即建立連線，不等待
            // 檢查是否在開發環境或需要指定後端 URL
            const socketUrl = this.getSocketUrl();
            //console.log("🔌 Socket.IO 連線 URL:", socketUrl);

            // 檢查 URL 有效性
            if (!socketUrl) {
                throw new Error("無法確定 Socket.IO 連線 URL");
            }

            this.socket = io(socketUrl, {
                transports: ['websocket', 'polling'],
                upgrade: true,
                rememberUpgrade: true,
                timeout: 10000, // 連接超時 10 秒
                forceNew: false, // 重用現有連接
                autoConnect: true, // 自動連接
                reconnection: true, // 啟用重連
                reconnectionAttempts: 5, // 最大重連次數
                reconnectionDelay: 1000, // 重連延遲
                reconnectionDelayMax: 5000, // 最大重連延遲
                maxReconnectionAttempts: 5
            });

            this.setupEventHandlers();
            //console.log("✅ Socket 連線已啟動（改善版）");

        } catch (error) {
            //console.error("❌ Socket 初始化失敗:", error);
            notify.showErrorMessage("網路連線初始化失敗，請重新整理頁面");

            // 清理狀態
            this.socket = null;
            this.isConnected = false;
        }
    }

    /**
     * 異步初始化（等待連線完成）
     */
    initAsync() {
        return new Promise((resolve, reject) => {
            // 如果已經連線，立即返回
            if (this.socket && this.socket.connected) {
                resolve();
                return;
            }

            // 設置連線成功監聽器
            const connectHandler = () => {
                this.socket.off('connect', connectHandler);
                this.socket.off('connect_error', errorHandler);
                resolve();
            };

            // 設置連線錯誤監聽器
            const errorHandler = (error) => {
                this.socket.off('connect', connectHandler);
                this.socket.off('connect_error', errorHandler);
                reject(new Error(`Socket 連線失敗: ${error.message || error}`));
            };

            // 開始初始化
            this.init();

            // 如果初始化後立即連線成功
            if (this.socket && this.socket.connected) {
                resolve();
                return;
            }

            // 等待連線事件
            if (this.socket) {
                this.socket.on('connect', connectHandler);
                this.socket.on('connect_error', errorHandler);

                // 設置超時機制（10秒）
                setTimeout(() => {
                    this.socket.off('connect', connectHandler);
                    this.socket.off('connect_error', errorHandler);
                    reject(new Error('Socket 連線超時'));
                }, 10000);
            } else {
                reject(new Error('Socket.IO 初始化失敗'));
            }
        });
    }

    /**
     * 註冊事件監聽器（帶追蹤功能）
     */
    registerEventListener(eventName, handler) {
        if (!this.eventListeners.has(eventName)) {
            this.eventListeners.set(eventName, new Set());
        }
        this.eventListeners.get(eventName).add(handler);
        this.socket.on(eventName, handler);
    }

    /**
     * 設定事件處理器（改善版 - 加強錯誤處理和追蹤）
     */
    setupEventHandlers() {
        // 連線事件
        const connectHandler = () => {
            try {
                // //console.log("✅ Socket.IO 連線成功");
                this.isConnected = true;
                this.reconnectAttempts = 0;

                stateHelpers.setUser({
                    isConnected: true,
                    userAgent: navigator.userAgent
                });

                // 執行連線成功回調
                this.connectedCallbacks.forEach(callback => {
                    try {
                        callback();
                    } catch (error) {
                        console.error('❌ 連線回調執行失敗:', error);
                    }
                });

                // 自動登入（加強錯誤處理）
                this.login().catch(error => {
                    //console.error('❌ 自動登入失敗:', error);
                });

                notify.showErrorMessage("✅ 系統已連線", "is-success");
            } catch (error) {
                //console.error('❌ 連線處理器錯誤:', error);
            }
        };

        const disconnectHandler = () => {
            try {
                // //console.warn("❌ Socket 連線中斷");
                this.isConnected = false;
                stateHelpers.setUser({ isConnected: false });
                notify.showErrorMessage("連線中斷");

                // 清理待處理的請求
                this.pendingRequests.clear();
            } catch (error) {
                //console.error('❌ 斷線處理器錯誤:', error);
            }
        };

        // 連線錯誤處理
        const connectErrorHandler = (error) => {
            //console.error('❌ Socket 連線錯誤:', error);
            notify.showErrorMessage("網路連線失敗，請檢查網路狀態");
        };

        const reconnectAttemptHandler = (attempt) => {
            this.reconnectAttempts = attempt;
            // 只在前幾次重連時顯示通知，避免幹擾用戶
            if (attempt <= 3) {
                notify.showErrorMessage(`重連中（第 ${attempt} 次）`);
            }
        };

        const reconnectHandler = () => {
            notify.showErrorMessage(`✅ 重新連線成功`, "is-success");
            this.reconnectAttempts = 0;
        };

        const reconnectFailedHandler = () => {
            //console.error('❌ Socket 重連失敗');
            notify.showErrorMessage("重連失敗，請重新整理頁面");
        };

        // 註冊事件監聽器
        this.registerEventListener("connect", connectHandler);
        this.registerEventListener("disconnect", disconnectHandler);
        this.registerEventListener("connect_error", connectErrorHandler);

        // 重連事件（改善版 - 加強錯誤處理）
        this.socket.io.on("reconnect_attempt", reconnectAttemptHandler);
        this.socket.io.on("reconnect", reconnectHandler);
        this.socket.io.on("reconnect_failed", reconnectFailedHandler);

        // 資料更新事件（優化版 - 被動更新模式）
        this.socket.on("product_list", (data) => {
            //console.log('📥 收到產品列表資料:', data);
            // 被動更新，不觸發立即 UI 重新渲染
            stateHelpers.updateData('products', data.products || []);
        });

        this.socket.on("machine_list", (data) => {
            //console.log('📥 收到機台列表資料:', data);
            // 只更新 Store，UI 更新由 Store change 事件處理
            stateHelpers.updateData('machines', data.machines || []);
        });

        this.socket.on("room_list", (data) => {
            // 被動更新，不觸發立即 UI 重新渲染
            stateHelpers.updateData('rooms', data.rooms || []);
        });

        this.socket.on("parking_list", (data) => {
            // 舊架構中的資料結構是 parkingList，直接使用

            // 在更新 parking 資料前，先檢查並清除無效的料架選擇
            this._validateAndCleanRackSelections(data || {});

            stateHelpers.updateData('parking', data || {});


            // UI 更新由 Store change 事件處理，不需要直接調用
        });

        // 通知事件 - 只使用 notify.js 的通知系統
        this.socket.on("notify_message", (data) => {
            // 移除 stateHelpers.addNotification 調用，只使用 notify.js
            notify.showNotifyMessage(data.message || data);
        });

        this.socket.on("error_message", (data) => {
            // 移除 stateHelpers.addNotification 調用，只使用 notify.js
            notify.showErrorMessage(data.message || data);
        });

        // 活躍任務狀態同步事件
        this.socket.on("active_tasks", (data) => {

            stateHelpers.syncActiveTasks(data);
        });

        // 🔧 新增：任務狀態變更事件
        this.socket.on("task_status_update", (data) => {
            console.log('📥 收到任務狀態變更:', data);

            // 更新任務狀態
            if (data.side && data.type && data.status) {
                stateHelpers.updateTask(data.side, {
                    type: data.type,
                    status: data.status,
                    updatedAt: data.updatedAt || new Date().toISOString()
                });

                // 顯示狀態變更通知
                if (data.status === 'delivered') {
                    notify.showNotifyMessage(`${data.side} 側任務已送達，請確認送達`);
                } else if (data.status === 'confirmed') {
                    notify.showNotifyMessage(`${data.side} 側任務已確認完成`);
                    // 清除任務狀態
                    stateHelpers.updateTask(data.side, null);
                } else if (data.status === 'completed') {
                    // 派車任務完成（自動完成，不需要確認）
                    if (data.type === 'dispatch_full') {
                        notify.showNotifyMessage(`${data.side} 側派車任務已完成`);
                    } else {
                        notify.showNotifyMessage(`${data.side} 側任務已完成`);
                    }
                    // 清除任務狀態
                    stateHelpers.updateTask(data.side, null);
                }
            }
        });

        // 客戶端資料恢復事件
        this.socket.on("request_client_id", () => {
            // 後端請求clientId，發送localStorage中的clientId
            const clientId = userStore.getState().clientId;
            if (clientId) {
                //console.log('📤 發送clientId進行資料恢復:', clientId);
                this.socket.emit("restore_client_by_id", { clientId });
            }
        });

        this.socket.on("client_data_restored", (data) => {
            if (data.success && data.client) {
                //console.log('📥 收到恢復的客戶端資料:', data.client);
                this._restoreClientData(data.client);
            }
        });
    }

    /**
     * 驗證並清除無效的料架選擇
     */
    _validateAndCleanRackSelections(newParkingData) {
        try {
            const operation = operationStore.getState();
            let hasInvalidSelections = false;
            let needsSync = false;

            // //console.log('🔍 檢查料架選擇狀態同步...');
            // //console.log('🔍 新的停車格資料:', newParkingData);
            // //console.log('🔍 當前操作狀態:', operation);

            ['left', 'right'].forEach(side => {
                const sideData = operation[side];
                if (!sideData || !sideData.products) {
                    // //console.log(`🔍 ${side} 側沒有產品資料，跳過檢查`);
                    return;
                }

                const availableRacks = newParkingData[side] || [];
                const availableRackIds = availableRacks.map(rack => rack.id);

                // //console.log(`🔍 ${side} 側可用料架:`, availableRackIds);
                // //console.log(`🔍 ${side} 側產品數量:`, sideData.products.length);

                sideData.products.forEach((product, productIndex) => {
                    // //console.log(`🔍 檢查 ${side} 產品 ${productIndex}:`, product);

                    if (product.rackId && !availableRackIds.includes(product.rackId)) {
                        // //console.warn(`⚠️ 檢測到無效料架選擇: ${side} 產品 ${productIndex} 的料架 ${product.rackId} 已不存在於停車格`);
                        // //console.warn(`⚠️ 可用料架 IDs:`, availableRackIds);

                        // 清除無效的料架選擇
                        if (window.stateHelpers && window.stateHelpers.updateProduct) {
                            window.stateHelpers.updateProduct(side, productIndex, { rackId: null });
                            hasInvalidSelections = true;
                            needsSync = true;
                            // //console.log(`✅ 已清除 ${side} 產品 ${productIndex} 的無效料架選擇`);
                        } else {
                            // //console.error(`❌ stateHelpers 不可用，無法清除料架選擇`);
                            // //console.log(`🔍 window.stateHelpers:`, window.stateHelpers);

                            // 備用方案：直接更新 store
                            try {
                                const currentOperation = operationStore.getState();
                                const newOperation = { ...currentOperation };
                                newOperation[side].products[productIndex].rackId = null;
                                operationStore.setState(newOperation);
                                hasInvalidSelections = true;
                                needsSync = true;
                                // //console.log(`✅ 使用備用方案清除 ${side} 產品 ${productIndex} 的無效料架選擇`);
                            } catch (error) {
                                //console.error(`❌ 備用方案也失敗:`, error);
                            }
                        }

                        // 顯示通知給用戶
                        notify.showNotifyMessage(`料架 ${product.rackId} 已搬離，已自動清除選擇`, 'warning');
                    } else if (product.rackId) {
                        // //console.log(`✅ ${side} 產品 ${productIndex} 的料架 ${product.rackId} 仍然有效`);
                    } else {
                        // //console.log(`🔍 ${side} 產品 ${productIndex} 沒有選擇料架`);
                    }
                });
            });

            if (hasInvalidSelections) {
                // //console.log('🔄 已清除無效的料架選擇，準備同步到後端');

                // 即時同步更新到後端
                if (needsSync) {
                    const updatedState = operationStore.getState();
                    this.updateClient(updatedState)
                        .then(() => {
                            // //console.log('✅ 料架選擇狀態同步完成');
                        })
                        .catch((error) => {
                            //console.error('❌ 料架選擇狀態同步失敗:', error);
                        });
                }
            } else {
                // //console.log('✅ 所有料架選擇都有效，無需清除');
            }

        } catch (error) {
            //console.error('❌ 驗證料架選擇狀態時發生錯誤:', error);
            //console.error('錯誤詳情:', error.stack);
        }
    }

    /**
     * 檢查連線狀態（改善版）
     */
    checkConnection() {
        if (!this.socket) {
            //console.warn("⚠️ Socket 實例不存在");
            return false;
        }

        if (!this.socket.connected) {
            //console.warn("⚠️ Socket 未連線，當前狀態:", this.socket.readyState);
            return false;
        }

        return true;
    }

    /**
     * 註冊連線成功回調
     */
    onConnected(callback) {
        if (typeof callback === 'function') {
            this.connectedCallbacks.push(callback);
        }
    }

    /**
     * 使用者登入（統一扁平化格式版）
     */
    login() {
        if (!this.checkConnection()) return Promise.reject(new Error("Socket 未連線"));

        // 直接使用扁平化格式
        const userState = userStore.getState();

        // 從 URL 獲取 deviceId 作為 clientId
        const clientId = this._getClientIdFromURL();

        // 更新 userStore 中的 clientId，確保與 deviceId 一致
        userStore.setState({
            ...userState,
            clientId: clientId
        });

        const loginData = {
            clientId: clientId,
            machineId: userState.machineId,
            userAgent: userState.userAgent,
            isConnected: userState.isConnected
        };

        console.log('🔐 登入資料:', loginData);

        return new Promise((resolve, reject) => {
            // 設定超時機制
            const timeoutId = setTimeout(() => {
                reject(new Error("登入請求超時"));
                notify.showErrorMessage("登入請求超時，請重試");
            }, 10000); // 10秒超時

            // 使用回調處理登入回應
            this.socket.emit("login", loginData, (response) => {
                clearTimeout(timeoutId); // 清除超時計時器

                if (response && response.success) {
                    // //console.log('✅ 登入成功:', response);

                    // 儲存後端返回的 clientId
                    if (response.clientId) {
                        storeClientId(response.clientId);

                        // 更新 store 中的 clientId
                        stateHelpers.setUser({ clientId: response.clientId });

                        // //console.log('🆔 已更新並儲存 clientId:', response.clientId);
                    }

                    notify.showNotifyMessage("登入成功");
                    resolve(response);
                } else {
                    const errorMessage = response?.message || "登入失敗";
                    //console.error('❌ 登入失敗:', response);
                    notify.showErrorMessage(errorMessage);
                    reject(new Error(errorMessage));
                }
            });
        });
    }

    /**
     * 更新客戶端設定（優化版 - 只傳送必要資料）
     */
    updateClient(operationData = null) {
        if (!this.checkConnection()) {
            return Promise.reject(new Error("Socket 未連線"));
        }

        try {
            // 只傳送必要的欄位：clientId, machineId, op
            const userState = userStore.getState();

            // 🔧 修復：正確處理傳入的資料格式
            let operationState;
            if (operationData && operationData.operation) {
                // 如果傳入的是完整的狀態物件（包含user, operation, data等）
                operationState = operationData.operation;
            } else if (operationData && (operationData.left || operationData.right)) {
                // 如果傳入的直接是operation資料
                operationState = operationData;
            } else {
                // 預設情況：從store獲取
                operationState = operationStore.getState();
            }

            // 從 URL 獲取 deviceId 作為 clientId
            const clientId = this._getClientIdFromURL();

            // 更新 userStore 中的 clientId，確保與 deviceId 一致
            userStore.setState({
                ...userState,
                clientId: clientId
            });

            const minimalData = {
                clientId: clientId,
                machineId: userState.machineId,
                op: operationState
            };

            // 詳細日誌記錄（可選擇性啟用）
            if (this._isDebugMode()) {
                console.log('🔄 同步客戶端操作資料到後端:');
                console.log('  最小化資料:', this._sanitizeLogData(minimalData));
            }

            // 檢查是否有相同的請求正在處理
            if (this.pendingRequests.has('client_update')) {
                return this.pendingRequests.get('client_update');
            }

            // 返回 Promise 以支援 .then() 和 .catch() 調用
            const promise = new Promise((resolve, reject) => {
                // 設定超時機制
                const timeoutId = setTimeout(() => {
                    this.pendingRequests.delete('client_update');
                    const error = new Error("客戶端更新請求超時");
                    console.error('❌ updateClient 超時:', error);
                    reject(error);
                }, 15000); // 15秒超時

                this.socket.emit("client_update", minimalData, (response) => {
                    clearTimeout(timeoutId);
                    this.pendingRequests.delete('client_update');

                    if (this._isDebugMode()) {
                        console.log('📥 收到 client_update 回應:', response);
                    }

                    if (response && response.success) {
                        if (this._isDebugMode()) {
                            console.log('✅ 客戶端資料同步成功');
                        }
                        resolve(response);
                    } else {
                        const errorMessage = response?.message || "客戶端設定更新失敗";
                        const error = new Error(errorMessage);
                        console.warn("❌ 客戶端設定更新失敗:", errorMessage, response);
                        reject(error);
                    }
                });
            });

            // 儲存待處理的請求
            this.pendingRequests.set('client_update', promise);

            return promise;

        } catch (error) {
            console.error('❌ updateClient 資料處理錯誤:', error);
            return Promise.reject(new Error(`資料處理失敗: ${error.message}`));
        }
    }



    /**
     * 驗證操作資料格式
     */
    _validateOperationData(opData) {
        if (!opData || typeof opData !== 'object') {
            throw new Error('操作資料必須是物件');
        }

        if (!opData.left || !opData.right) {
            throw new Error('操作資料缺少 left 或 right 欄位');
        }

        return true;
    }

    /**
     * 檢查是否啟用除錯模式
     */
    _isDebugMode() {
        return window.location.search.includes('debug=true') ||
               localStorage.getItem('opui_debug') === 'true' ||
               false; // 預設關閉除錯模式
    }

    /**
     * 清理敏感資料用於日誌記錄
     */
    _sanitizeLogData(data) {
        if (!data || typeof data !== 'object') return data;

        const sanitized = JSON.parse(JSON.stringify(data));

        // 移除或遮蔽敏感資料
        if (sanitized.user?.userAgent) {
            sanitized.user.userAgent = sanitized.user.userAgent.substring(0, 50) + '...';
        }

        return sanitized;
    }

    /**
     * 驗證扁平化格式資料的完整性
     */
    _validateFlatFormat(data) {
        const requiredFields = ['clientId', 'machineId', 'op'];
        const missingFields = requiredFields.filter(field => !(field in data));

        if (missingFields.length > 0) {
            throw new Error(`扁平化格式資料缺少必要欄位: ${missingFields.join(', ')}`);
        }

        // 驗證 op 欄位結構
        if (!data.op.left || !data.op.right) {
            throw new Error('op 欄位缺少 left 或 right 資料');
        }

        return true;
    }



    /**
     * 叫空車
     */
    callEmpty(data) {
        if (!this.checkConnection()) return;

        // //console.log("發送叫空車請求:", data);
        this.socket.emit("call_empty", data, (response) => {
            // //console.log("🔄 收到叫空車回應:", response);

            if (response?.success) {
                // //console.log("✅ 叫空車成功");
                notify.showNotifyMessage("叫空車請求已送出");
                // 更新任務狀態
                stateHelpers.updateTask(data.side, {
                    type: 'call_empty',
                    status: 'pending',
                    createdAt: new Date().toISOString()
                });
            } else {
                // //console.log("❌ 叫空車失敗:", response?.message);
                notify.showErrorMessage(response?.message || "叫空車失敗");
            }
        });
    }

    /**
     * 派滿車
     */
    dispatchFull(data) {
        if (!this.checkConnection()) return;

        // //console.log("發送派滿車請求:", data);
        this.socket.emit("dispatch_full", data, (response) => {
            if (response?.success) {
                notify.showNotifyMessage("派滿車請求已送出");
                // 更新任務狀態
                stateHelpers.updateTask(data.side, {
                    type: 'dispatch_full',
                    status: 'pending',
                    createdAt: new Date().toISOString()
                });
            } else {
                notify.showErrorMessage(response?.message || "派滿車失敗");
            }
        });
    }

    /**
     * 取消任務
     */
    cancelTask(data) {
        if (!this.checkConnection()) {
            return Promise.reject(new Error("Socket 未連線"));
        }

        return new Promise((resolve, reject) => {
            // 設定超時機制
            const timeoutId = setTimeout(() => {
                reject(new Error("取消任務請求超時"));
            }, 10000); // 10秒超時

            // //console.log("發送取消任務請求:", data);
            this.socket.emit("cancel_task", data, (response) => {
                clearTimeout(timeoutId);
                // //console.log("🔄 收到取消任務回應:", response);

                if (response?.success) {
                    // //console.log("✅ 取消任務成功");
                    notify.showNotifyMessage("任務已取消");
                    // 清除任務狀態
                    stateHelpers.updateTask(data.side, null);
                    resolve(response);
                } else {
                    const errorMsg = response?.message || "取消任務失敗";
                    // //console.log("❌ 取消任務失敗:", errorMsg);
                    notify.showErrorMessage(errorMsg);
                    reject(new Error(errorMsg));
                }
            });
        });
    }

    /**
     * 確認送達
     */
    confirmDelivery(data) {
        if (!this.checkConnection()) return;

        // //console.log("發送確認送達請求:", data);
        this.socket.emit("confirm_delivery", data, (response) => {
            // //console.log("🔄 收到確認送達回應:", response);

            if (response?.success) {
                // //console.log("✅ 確認送達成功");
                notify.showNotifyMessage("已確認送達");
            } else {
                // //console.log("❌ 確認送達失敗:", response?.message);
                notify.showErrorMessage(response?.message || "確認送達失敗");
            }
        });
    }

    /**
     * 新增料架
     */
    addRack(side, rackName) {
        if (!this.checkConnection()) {
            return Promise.reject(new Error("Socket 未連線"));
        }

        const data = { side, rack: rackName };  // 注意：後端期望的是 "rack" 而不是 "rackName"
        // //console.debug('🏷️ 發送料架新增請求:', data);

        return new Promise((resolve, reject) => {
            this.socket.emit("add_rack", data, (response) => {
                if (response?.success) {
                    // //console.debug('✅ 料架新增成功:', response);
                    resolve(response);
                } else {
                    //console.error('❌ 料架新增失敗:', response?.message);
                    reject(new Error(response?.message || "新增料架失敗"));
                }
            });
        });
    }

    /**
     * 刪除料架
     */
    deleteRack(rackId) {
        if (!this.checkConnection()) {
            return Promise.reject(new Error("Socket 未連線"));
        }

        // //console.debug('🗑️ 發送料架刪除請求:', rackId);

        return new Promise((resolve, reject) => {
            this.socket.emit("del_rack", { rackId }, (response) => {
                if (response?.success) {
                    // //console.debug('✅ 料架刪除成功:', response);
                    resolve(response);
                } else {
                    //console.error('❌ 料架刪除失敗:', response?.message);
                    reject(new Error(response?.message || "刪除料架失敗"));
                }
            });
        });
    }

    /**
     * 恢復客戶端資料到前端store
     */
    _restoreClientData(clientData) {
        try {
            //console.log('🔄 開始恢復客戶端資料到store');

            // 恢復使用者狀態
            if (clientData.clientId || clientData.machineId) {
                stateHelpers.setUser({
                    clientId: clientData.clientId,
                    machineId: clientData.machineId || 1,
                    userAgent: clientData.userAgent || navigator.userAgent
                });
                //console.log('✅ 使用者狀態已恢復');
            }

            // 恢復操作狀態（OP2產品代碼）
            if (clientData.op) {
                //console.log('🔄 恢復OP資料:', clientData.op);

                // 轉換後端格式到前端格式
                const restoredOperation = this._convertBackendOpToFrontend(clientData.op);

                // 修復產品資料：確保ID和尺寸資訊正確
                const fixedOperation = this._fixProductData(restoredOperation);

                operationStore.setState(fixedOperation);
                //console.log('✅ OP2產品代碼已恢復並修復:', fixedOperation);

                // 立即更新UI以反映正確的產品資訊
                if (typeof updateProductButtons === 'function') {
                    updateProductButtons(fixedOperation);
                }
                if (typeof updateNumberButtons === 'function') {
                    updateNumberButtons(fixedOperation);
                }
            }

        } catch (error) {
            //console.error('❌ 恢復客戶端資料失敗:', error);
        }
    }

    /**
     * 將後端的op格式轉換為前端operationStore格式
     */
    _convertBackendOpToFrontend(backendOp) {
        const defaultProducts = {
            left: [
                { name: 'ABC12345', size: 'S', id: 1, count: 32, room: 2, rackId: null },
                { name: 'DEF67890', size: 'L', id: 2, count: 16, room: 2, rackId: null }
            ],
            right: [
                { name: 'ABC54321', size: 'S', id: 3, count: 32, room: 2, rackId: null },
                { name: 'DEF09876', size: 'L', id: 4, count: 16, room: 2, rackId: null }
            ]
        };

        return {
            left: {
                productSelected: backendOp.left?.productSelected || 0,
                products: (backendOp.left?.products && backendOp.left.products.length > 0)
                    ? backendOp.left.products
                    : defaultProducts.left  // 🔧 修復：檢查陣列是否為空
            },
            right: {
                productSelected: backendOp.right?.productSelected || 0,
                products: (backendOp.right?.products && backendOp.right.products.length > 0)
                    ? backendOp.right.products
                    : defaultProducts.right  // 🔧 修復：檢查陣列是否為空
            }
        };
    }

    /**
     * 修復產品資料：確保ID和尺寸資訊正確
     */
    _fixProductData(operationData) {
        try {
            //console.log('🔧 開始修復產品資料');

            // 獲取dataStore中的產品列表作為參考
            const dataState = dataStore.getState();
            const availableProducts = dataState.products || [];

            const fixedData = JSON.parse(JSON.stringify(operationData)); // 深拷貝
            let hasChanges = false;

            ['left', 'right'].forEach(side => {
                if (fixedData[side] && fixedData[side].products) {
                    fixedData[side].products.forEach((product, index) => {
                        let needsFix = false;
                        let fixedProduct = { ...product };

                        // 檢查是否需要修復ID
                        if (product.id === null || product.id === undefined) {
                            const fullProduct = availableProducts.find(p => p.name === product.name);
                            if (fullProduct) {
                                fixedProduct.id = fullProduct.id;
                                fixedProduct.size = fullProduct.size;
                                fixedProduct.count = fullProduct.size === 'S' ? 32 : 16;
                                needsFix = true;
                                //console.log(`🔄 修復 ${side} 側產品 ${index}: ${product.name} -> ID:${fullProduct.id}, Size:${fullProduct.size}`);
                            }
                        }

                        // 檢查尺寸資訊是否正確
                        if (product.id && !needsFix) {
                            const fullProduct = availableProducts.find(p => p.id === product.id);
                            if (fullProduct && product.size !== fullProduct.size) {
                                fixedProduct.size = fullProduct.size;
                                fixedProduct.count = fullProduct.size === 'S' ? 32 : 16;
                                needsFix = true;
                                //console.log(`🔄 修復 ${side} 側產品 ${index} 尺寸: ${product.size} -> ${fullProduct.size}`);
                            }
                        }

                        if (needsFix) {
                            fixedData[side].products[index] = fixedProduct;
                            hasChanges = true;
                        }
                    });
                }
            });

            if (hasChanges) {
                //console.log('✅ 產品資料修復完成');
            } else {
                //console.log('ℹ️ 產品資料無需修復');
            }

            return fixedData;

        } catch (error) {
            //console.error('❌ 修復產品資料失敗:', error);
            return operationData; // 返回原始資料
        }
    }

    /**
     * 從 URL 參數獲取 deviceId 作為 clientId
     */
    _getClientIdFromURL() {
        const urlParams = new URLSearchParams(window.location.search);
        const deviceId = urlParams.get('deviceId');

        if (deviceId) {
            console.log(`🆔 從 URL 參數獲取 clientId: ${deviceId}`);
            return deviceId;
        } else {
            const defaultClientId = "device_undefined";
            console.log(`🆔 URL 中沒有 deviceId 參數，使用預設值: ${defaultClientId}`);
            return defaultClientId;
        }
    }

    /**
     * 獲取所有 store 的當前狀態
     */
    getAllStates() {
        return {
            user: userStore.getState(),
            operation: operationStore.getState(),
            data: dataStore.getState(),
            tasks: tasksStore.getState(),
            ui: uiStore.getState()
        };
    }
}

// 建立 API 實例
const socketAPI = new SocketAPI();

// 匯出 API 介面
export { socketAPI };

// 為了向後相容，也匯出舊的 API 格式
export default {
    setup: () => socketAPI.init(),
    api: {
        login: () => socketAPI.login(),
        clientUpdate: (data) => socketAPI.updateClient(data),
        callEmpty: (data) => socketAPI.callEmpty(data),
        dispatchFull: (data) => socketAPI.dispatchFull(data),
        cancelTask: (data) => socketAPI.cancelTask(data),
        addRack: (data) => socketAPI.addRack(data),
        deleteRack: (data) => socketAPI.deleteRack(data)
    }
};
