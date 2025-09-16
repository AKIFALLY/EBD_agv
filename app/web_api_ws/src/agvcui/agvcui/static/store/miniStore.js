export function createStore(key, initialState = {}) {
    const saved = localStorage.getItem(key);
    let state;

    if (saved === null) {
        state = initialState;
        localStorage.setItem(key, JSON.stringify(initialState));
        console.debug(`[store] 沒有 localStorage 儲存，使用初始值`, state);
    } else {
        try {
            state = JSON.parse(saved);
            console.debug(`[store] 從 localStorage 載入`, state);
        } catch (e) {
            console.warn(`[store] JSON 解析失敗，重設為初始值`, e);
            state = initialState;
            localStorage.setItem(key, JSON.stringify(initialState));
        }
    }

    const listenersMap = new Map();

    function getState() {
        return { ...state };
    }

    function setState(newState) {
        console.debug(`[miniStore] setState called with:`, JSON.stringify(newState));
        console.debug(`[miniStore] Current state before merge:`, JSON.stringify(state));
        state = { ...state, ...newState };
        console.debug(`[miniStore] New state after merge:`, JSON.stringify(state));
        
        try {
            const stateStr = JSON.stringify(state);
            console.debug(`[miniStore] Saving to localStorage, key=${key}, size=${stateStr.length} bytes`);
            localStorage.setItem(key, stateStr);
            console.debug(`[store] setState 被呼叫，已保存到 localStorage:`, key, state);
            
            // Verify it was saved
            const saved = localStorage.getItem(key);
            if (saved) {
                const savedState = JSON.parse(saved);
                console.debug(`[miniStore] Verification - localStorage now has:`, savedState.currentFlow?.flow?.length, 'cards');
            } else {
                console.error(`[miniStore] ERROR: localStorage.getItem returned null after setItem!`);
            }
        } catch (e) {
            console.error(`[miniStore] ERROR saving to localStorage:`, e);
        }

        const listeners = listenersMap.get('change');
        if (listeners) {
            listeners.forEach((listener) => listener(getState()));
        }
    }

    function on(eventName, listener) {
        if (!listenersMap.has(eventName)) {
            listenersMap.set(eventName, new Set());
        }
        listenersMap.get(eventName).add(listener);
    }

    function off(eventName, listener) {
        const listeners = listenersMap.get(eventName);
        if (listeners) {
            listeners.delete(listener);
            if (listeners.size === 0) {
                listenersMap.delete(eventName);
            }
        }
    }

    function clear() {
        console.debug(`[store] clear 被呼叫 for key: ${key}`);
        console.debug(`[store] initialState:`, JSON.stringify(initialState));
        state = { ...initialState }; // 重置為初始狀態
        localStorage.removeItem(key);
        console.debug(`[store] localStorage removed for key: ${key}`);
        console.debug(`[store] state after clear:`, JSON.stringify(state));

        // 也觸發 change 通知
        const listeners = listenersMap.get('change');
        if (listeners) {
            console.debug(`[store] triggering ${listeners.size} change listeners`);
            listeners.forEach((listener) => listener(getState()));
        }
    }

    return {
        getState,
        setState,
        on,
        off,
        clear, // ✅ 新增清除功能
    };
}
