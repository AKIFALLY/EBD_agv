// worksStore.js - Work 資料狀態管理
// 整合到 miniStore 系統，提供即時資料同步

import { createStore } from '../store/miniStore.js';

/**
 * Work Store - 管理工作類型資料狀態
 */
export const worksStore = (() => {
    // 使用 miniStore 創建基礎 store
    const baseStore = createStore('worksStore', {
        works: [],
        loading: false,
        error: null,
        lastUpdated: null
    });

    // 事件監聽器
    const listeners = new Map();

    /**
     * 添加事件監聽器
     * @param {string} event - 事件名稱
     * @param {Function} callback - 回調函數
     */
    function on(event, callback) {
        if (!listeners.has(event)) {
            listeners.set(event, new Set());
        }
        listeners.get(event).add(callback);
    }

    /**
     * 移除事件監聽器
     * @param {string} event - 事件名稱
     * @param {Function} callback - 回調函數
     */
    function off(event, callback) {
        if (listeners.has(event)) {
            listeners.get(event).delete(callback);
        }
    }

    /**
     * 觸發事件
     * @param {string} event - 事件名稱
     * @param {any} data - 事件資料
     */
    function emit(event, data) {
        if (listeners.has(event)) {
            listeners.get(event).forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    console.error(`Work Store 事件處理錯誤 (${event}):`, error);
                }
            });
        }
    }

    /**
     * 獲取當前狀態
     * @returns {Object} 當前狀態
     */
    function getState() {
        return baseStore.getState();
    }

    /**
     * 更新狀態
     * @param {Object} newState - 新狀態
     */
    function setState(newState) {
        const oldState = getState();
        const updatedState = { ...oldState, ...newState, lastUpdated: new Date() };
        baseStore.setState(updatedState);

        console.debug('Work Store 狀態更新:', {
            works: updatedState.works.length,
            loading: updatedState.loading,
            error: updatedState.error
        });

        // 觸發變化事件
        emit('change', updatedState);

        // 如果工作類型資料有變化，觸發特定事件
        if (oldState.works !== updatedState.works) {
            emit('works-updated', updatedState.works);
        }
    }

    /**
     * 設置載入狀態
     * @param {boolean} loading - 是否載入中
     */
    function setLoading(loading) {
        setState({ loading });
    }

    /**
     * 設置錯誤狀態
     * @param {string|null} error - 錯誤訊息
     */
    function setError(error) {
        setState({ error });
    }

    /**
     * 更新工作類型列表
     * @param {Array} works - 工作類型陣列
     */
    function updateWorks(works) {
        if (!Array.isArray(works)) {
            console.error('Work Store: updateWorks 需要陣列參數');
            return;
        }

        setState({
            works: works,
            error: null
        });

        console.debug(`Work Store: 更新了 ${works.length} 個工作類型`);
    }

    /**
     * 添加單個工作類型
     * @param {Object} work - 工作類型對象
     */
    function addWork(work) {
        if (!work || typeof work !== 'object') {
            console.error('Work Store: addWork 需要有效的工作類型對象');
            return;
        }

        const currentState = getState();
        const newWorks = [...currentState.works, work];
        setState({ works: newWorks });

        console.debug(`Work Store: 添加工作類型 ${work.name} (ID: ${work.id})`);
    }

    /**
     * 更新單個工作類型
     * @param {number} workId - 工作類型 ID
     * @param {Object} updatedWork - 更新的工作類型資料
     */
    function updateWork(workId, updatedWork) {
        const currentState = getState();
        const workIndex = currentState.works.findIndex(work => work.id === workId);

        if (workIndex === -1) {
            console.warn(`Work Store: 找不到 ID 為 ${workId} 的工作類型`);
            return;
        }

        const newWorks = [...currentState.works];
        newWorks[workIndex] = { ...newWorks[workIndex], ...updatedWork };

        setState({ works: newWorks });

        console.debug(`Work Store: 更新工作類型 ${updatedWork.name || workId} (ID: ${workId})`);
    }

    /**
     * 移除工作類型
     * @param {number} workId - 工作類型 ID
     */
    function removeWork(workId) {
        const currentState = getState();
        const newWorks = currentState.works.filter(work => work.id !== workId);

        if (newWorks.length === currentState.works.length) {
            console.warn(`Work Store: 找不到 ID 為 ${workId} 的工作類型`);
            return;
        }

        setState({ works: newWorks });

        console.debug(`Work Store: 移除工作類型 ID: ${workId}`);
    }

    /**
     * 根據 ID 獲取工作類型
     * @param {number} workId - 工作類型 ID
     * @returns {Object|null} 工作類型對象或 null
     */
    function getWorkById(workId) {
        const currentState = getState();
        return currentState.works.find(work => work.id === workId) || null;
    }

    /**
     * 搜尋工作類型
     * @param {string} query - 搜尋關鍵字
     * @returns {Array} 符合條件的工作類型陣列
     */
    function searchWorks(query) {
        const currentState = getState();
        if (!query || typeof query !== 'string') {
            return currentState.works;
        }

        const lowerQuery = query.toLowerCase();
        return currentState.works.filter(work =>
            work.name.toLowerCase().includes(lowerQuery) ||
            (work.description && work.description.toLowerCase().includes(lowerQuery))
        );
    }

    /**
     * 清空所有資料
     */
    function clear() {
        setState({
            works: [],
            loading: false,
            error: null
        });

        console.debug('Work Store: 已清空所有資料');
    }

    /**
     * 從 API 載入工作類型資料
     * @param {Object} options - 載入選項
     */
    async function loadWorks(options = {}) {
        const { page = 1, search = null, limit = 20 } = options;

        setLoading(true);
        setError(null);

        try {
            const params = new URLSearchParams({
                page: page.toString(),
                limit: limit.toString()
            });

            if (search) {
                params.append('search', search);
            }

            const response = await fetch(`/api/works?${params}`);

            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const data = await response.json();

            if (data.works) {
                updateWorks(data.works);
            }

            console.debug(`Work Store: 從 API 載入了 ${data.works?.length || 0} 個工作類型`);

        } catch (error) {
            console.error('Work Store: 載入工作類型失敗:', error);
            setError(error.message);
        } finally {
            setLoading(false);
        }
    }

    /**
     * 初始化 Store
     */
    function init() {
        console.log('🔧 初始化 Work Store');

        // 整合到 miniStore 系統
        if (window.miniStore) {
            window.miniStore.registerStore('works', {
                getState,
                setState: updateWorks,
                clear
            });
            console.debug('Work Store: 已註冊到 miniStore 系統');
        }

        // 可以在這裡添加其他初始化邏輯
        console.log('✅ Work Store 初始化完成');
    }

    /**
     * 清理資源
     */
    function destroy() {
        listeners.clear();
        clear();
        console.log('🧹 Work Store 資源已清理');
    }

    // 自動初始化
    init();

    // 返回公開的 API
    return {
        // 狀態管理
        getState,
        setState,
        setLoading,
        setError,

        // 事件系統
        on,
        off,
        emit,

        // 工作類型操作
        updateWorks,
        addWork,
        updateWork,
        removeWork,
        getWorkById,
        searchWorks,

        // 資料載入
        loadWorks,

        // 工具方法
        clear,
        destroy
    };
})();

// 將 worksStore 掛載到全域，方便其他模組使用
if (typeof window !== 'undefined') {
    window.worksStore = worksStore;
}
