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
        state = { ...state, ...newState };
        localStorage.setItem(key, JSON.stringify(state));

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
        state = { ...initialState }; // 重置為初始狀態
        localStorage.removeItem(key);
        console.debug(`[store] clear 被呼叫，狀態已重置:`, state);

        // 也觸發 change 通知
        const listeners = listenersMap.get('change');
        if (listeners) {
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
