/**
 * 地圖頁面單門控制模態窗口管理器
 * 負責單個自動門的開關控制
 *
 * 功能：
 * - 顯示/隱藏單門控制模態窗口
 * - 調用 Web API 的門控制接口
 * - 處理開門/關門按鈕點擊事件
 * - 即時顯示門狀態（透過 signalsStore）
 */

export const mapDoorControlModal = (() => {
    // API 基礎 URL（使用 Web API Gateway Port 8000）
    const DOOR_API_BASE = 'http://agvc.webapi/door';

    // 門 ID 到 Signal ID 的映射
    // Signal value: "0" = 關閉, "1" = 開啟
    const DOOR_SIGNAL_MAP = {
        1: 99901,  // 門1 → Door_1_Status (DM5000)
        2: 99902,  // 門2 → Door_2_Status (DM5001)
        3: 99903,  // 門3 → Door_3_Status (DM5002)
        4: 99904   // 門4 → Door_4_Status (DM5003)
    };

    // 當前控制的門 ID
    let currentDoorId = null;

    // Modal 元素引用
    let modal = null;
    let statusTag = null;
    let doorNameSpan = null;
    let openBtn = null;
    let closeBtn = null;

    /**
     * 更新門狀態顯示
     * @param {string} signalValue - Signal 值（"0"=關閉, "1"=開啟）
     */
    function updateDoorStatus(signalValue) {
        if (!statusTag) return;

        // Signal value: "0" = 關閉, "1" = 開啟
        const isOpen = signalValue === "1";

        statusTag.textContent = isOpen ? "開啟" : "關閉";
        statusTag.className = `tag ${isOpen ? 'is-success' : 'is-danger'}`;
    }

    /**
     * 從 signalsStore 更新當前門的狀態
     */
    function updateCurrentDoorStatus() {
        if (!currentDoorId) return;

        // 檢查 signalsStore 是否存在
        if (!window.signalsStore) {
            console.warn('signalsStore 不存在，無法更新門狀態');
            return;
        }

        const signals = window.signalsStore.getState().signals || [];
        const signalId = DOOR_SIGNAL_MAP[currentDoorId];

        if (!signalId) {
            console.warn(`找不到門 ${currentDoorId} 的 Signal ID 映射`);
            return;
        }

        const signal = signals.find(s => s.id === signalId);
        if (signal) {
            updateDoorStatus(signal.value);
        } else {
            // 如果找不到對應的 signal，顯示未知狀態
            if (statusTag) {
                statusTag.textContent = "未知";
                statusTag.className = "tag is-light";
            }
        }
    }

    /**
     * 打開門控制 Modal
     * @param {number} doorId - 門的 ID（1-4）
     * @param {string} doorName - 門的名稱
     */
    function openModal(doorId, doorName) {
        if (!modal) {
            console.error('Modal 元素未初始化');
            return;
        }

        currentDoorId = doorId;

        // 更新 Modal 標題和門名稱
        if (doorNameSpan) {
            doorNameSpan.textContent = doorName || `門 ${doorId}`;
        }

        // 更新狀態顯示
        updateCurrentDoorStatus();

        // 顯示 Modal
        modal.classList.add('is-active');
    }

    /**
     * 關閉門控制 Modal
     */
    function closeModal() {
        if (!modal) return;

        modal.classList.remove('is-active');
        currentDoorId = null;
    }

    /**
     * 調用門控制 API
     * @param {string} doorId - 門的 ID（1-4）
     * @param {boolean} isOpen - true=開門, false=關門
     * @returns {Promise<Object>} API 響應數據
     */
    async function controlDoor(doorId, isOpen) {
        const response = await fetch(`${DOOR_API_BASE}/control`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                doorId: String(doorId),  // 确保传递字符串类型给 API
                isOpen: isOpen
            })
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        return data;
    }

    /**
     * 處理開門按鈕點擊
     */
    async function handleOpenDoor() {
        if (!currentDoorId) return;

        openBtn.disabled = true;
        openBtn.classList.add('is-loading');

        try {
            await controlDoor(currentDoorId, true);
            console.log(`門 ${currentDoorId} 開門指令已發送`);
        } catch (error) {
            console.error(`門 ${currentDoorId} 開門失敗:`, error);
        } finally {
            setTimeout(() => {
                openBtn.disabled = false;
                openBtn.classList.remove('is-loading');
            }, 500);
        }
    }

    /**
     * 處理關門按鈕點擊
     */
    async function handleCloseDoor() {
        if (!currentDoorId) return;

        closeBtn.disabled = true;
        closeBtn.classList.add('is-loading');

        try {
            await controlDoor(currentDoorId, false);
            console.log(`門 ${currentDoorId} 關門指令已發送`);
        } catch (error) {
            console.error(`門 ${currentDoorId} 關門失敗:`, error);
        } finally {
            setTimeout(() => {
                closeBtn.disabled = false;
                closeBtn.classList.remove('is-loading');
            }, 500);
        }
    }

    /**
     * 初始化門控制模態窗口
     */
    function setup() {
        // 獲取 Modal 元素
        modal = document.getElementById('mapDoorControlModal');
        if (!modal) {
            console.error('找不到 mapDoorControlModal 元素');
            return;
        }

        // 獲取其他元素
        statusTag = document.getElementById('mapDoorModalStatus');
        doorNameSpan = document.getElementById('mapDoorModalDoorName');
        openBtn = document.getElementById('mapDoorModalOpenBtn');
        closeBtn = document.getElementById('mapDoorModalCloseBtn');
        const closeModalBtn = document.getElementById('mapDoorModalClose');

        // 綁定關閉按鈕事件
        if (closeModalBtn) {
            closeModalBtn.addEventListener('click', closeModal);
        }

        // 點擊背景關閉
        const modalBackground = modal.querySelector('.modal-background');
        if (modalBackground) {
            modalBackground.addEventListener('click', closeModal);
        }

        // ESC 鍵關閉
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && modal.classList.contains('is-active')) {
                closeModal();
            }
        });

        // 綁定開門/關門按鈕
        if (openBtn) {
            openBtn.addEventListener('click', handleOpenDoor);
        }

        if (closeBtn) {
            closeBtn.addEventListener('click', handleCloseDoor);
        }

        // 訂閱 signalsStore 變化以即時更新門狀態
        if (window.signalsStore) {
            window.signalsStore.on('change', (newState) => {
                // 只在模態窗口開啟且有當前門 ID 時更新狀態
                if (modal.classList.contains('is-active') && currentDoorId) {
                    const signals = newState.signals || [];
                    const signalId = DOOR_SIGNAL_MAP[currentDoorId];
                    const signal = signals.find(s => s.id === signalId);

                    if (signal) {
                        updateDoorStatus(signal.value);
                    }
                }
            });

            console.log('地圖門控制 Modal 已訂閱 signalsStore 變化');
        } else {
            console.warn('signalsStore 未初始化，門狀態將無法即時更新');
        }
    }

    // 導出公共接口
    return {
        setup,
        openModal,
        closeModal
    };
})();
