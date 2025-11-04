/**
 * 門控制模態窗口管理器
 * 負責自動門的開關控制
 *
 * 功能：
 * - 顯示/隱藏門控制模態窗口
 * - 調用 Web API 的門控制接口
 * - 處理開門/關門按鈕點擊事件
 * - 即時顯示門狀態（透過 signalsStore）
 */

export const doorControlModal = (() => {
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

    /**
     * 更新門狀態顯示
     * @param {string|number} doorId - 門的 ID（1-4）
     * @param {string} signalValue - Signal 值（"0"=關閉, "1"=開啟）
     */
    function updateDoorStatus(doorId, signalValue) {
        const statusTag = document.getElementById(`door-${doorId}-status`);
        if (!statusTag) {
            console.warn(`找不到門 ${doorId} 的狀態標籤`);
            return;
        }

        // Signal value: "0" = 關閉, "1" = 開啟
        const isOpen = signalValue === "1";

        statusTag.textContent = isOpen ? "開啟" : "關閉";
        statusTag.className = `tag ${isOpen ? 'is-success' : 'is-danger'}`;
    }

    /**
     * 從 signalsStore 更新所有門的狀態
     */
    function updateAllDoorStatus() {
        // 檢查 signalsStore 是否存在
        if (!window.signalsStore) {
            console.warn('signalsStore 不存在，無法更新門狀態');
            return;
        }

        const signals = window.signalsStore.getState().signals || [];

        // 遍歷所有門，更新狀態
        Object.entries(DOOR_SIGNAL_MAP).forEach(([doorId, signalId]) => {
            const signal = signals.find(s => s.id === signalId);
            if (signal) {
                updateDoorStatus(doorId, signal.value);
            } else {
                // 如果找不到對應的 signal，顯示未知狀態
                const statusTag = document.getElementById(`door-${doorId}-status`);
                if (statusTag) {
                    statusTag.textContent = "未知";
                    statusTag.className = "tag is-light";
                }
            }
        });
    }

    /**
     * 初始化門控制模態窗口
     */
    function setup() {
        const modal = document.getElementById('doorControlModal');
        const openBtn = document.getElementById('doorControlBtn');
        const closeBtn = document.getElementById('closeDoorModal');

        // 如果元素不存在，直接返回（可能是權限不足）
        if (!modal || !openBtn) {
            return;
        }

        // 打開模態窗口
        openBtn.addEventListener('click', () => {
            modal.classList.add('is-active');
            // 立即更新門狀態顯示
            updateAllDoorStatus();
        });

        // 關閉模態窗口的函數
        const closeModal = () => {
            modal.classList.remove('is-active');
        };

        // 關閉按鈕
        if (closeBtn) {
            closeBtn.addEventListener('click', closeModal);
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

        // 刪除按鈕 X 關閉
        const deleteButton = modal.querySelector('.delete');
        if (deleteButton) {
            deleteButton.addEventListener('click', closeModal);
        }

        // 綁定門控制按鈕（使用事件委托）
        bindDoorControlButtons(modal);

        // 訂閱 signalsStore 變化以即時更新門狀態
        if (window.signalsStore) {
            window.signalsStore.on('change', (newState) => {
                // 只在模態窗口開啟時更新狀態（性能優化）
                if (modal.classList.contains('is-active')) {
                    const signals = newState.signals || [];

                    // 更新所有門的狀態
                    Object.entries(DOOR_SIGNAL_MAP).forEach(([doorId, signalId]) => {
                        const signal = signals.find(s => s.id === signalId);
                        if (signal) {
                            updateDoorStatus(doorId, signal.value);
                        }
                    });
                }
            });

            console.log('門控制模態窗口已訂閱 signalsStore 變化');
        } else {
            console.warn('signalsStore 未初始化，門狀態將無法即時更新');
        }
    }

    /**
     * 綁定門控制按鈕事件（使用事件委托模式）
     * @param {HTMLElement} modal - 模態窗口元素
     */
    function bindDoorControlButtons(modal) {
        modal.addEventListener('click', async (e) => {
            // 查找最近的門控制按鈕
            const button = e.target.closest('[data-door-action]');
            if (!button) return;

            const doorId = button.dataset.doorId;
            const action = button.dataset.doorAction;
            const isOpen = action === 'open';

            // 禁用按鈕防止重複點擊
            button.disabled = true;
            button.classList.add('is-loading');

            try {
                // 調用門控制 API
                await controlDoor(doorId, isOpen);

                // 無需反饋，靜默執行成功
                console.log(`門 ${doorId} ${isOpen ? '開門' : '關門'} 指令已發送`);

            } catch (error) {
                // 發生錯誤時在控制台記錄
                console.error(`門 ${doorId} 控制失敗:`, error);

                // 可選：如果需要錯誤提示，可以取消下面的註釋
                // alert(`門 ${doorId} 控制失敗，請稍後再試`);

            } finally {
                // 恢復按鈕狀態（延遲 500ms 以顯示執行效果）
                setTimeout(() => {
                    button.disabled = false;
                    button.classList.remove('is-loading');
                }, 500);
            }
        });
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
                doorId: doorId,
                isOpen: isOpen
            })
        });

        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        return data;
    }

    // 導出公共接口
    return {
        setup
    };
})();
