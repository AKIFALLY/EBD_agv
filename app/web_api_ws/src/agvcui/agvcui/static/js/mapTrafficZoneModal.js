/**
 * 地圖頁面交管區控制模態窗口管理器
 * 負責交管區的資訊顯示和強制解除控制
 *
 * 功能：
 * - 顯示/隱藏交管區控制模態窗口
 * - 即時顯示交管區狀態（透過 trafficZonesStore）
 * - 處理強制解除交管按鈕點擊事件（需要登入）
 * - 二次確認對話框
 */

export const mapTrafficZoneModal = (() => {
    // API 基礎 URL（使用 Web API Gateway Port 8000）
    const TRAFFIC_API_BASE = 'http://agvc.webapi/traffic';

    // 當前控制的交管區 ID
    let currentZoneId = null;

    // Modal 元素引用
    let modal = null;
    let statusTag = null;
    let zoneNameSpan = null;
    let statusTextSpan = null;
    let ownerSpan = null;
    let notLoggedInDiv = null;
    let forceReleaseSection = null;
    let forceReleaseBtn = null;

    /**
     * 狀態標籤樣式映射
     */
    const STATUS_STYLES = {
        'free': { text: '空閒', class: 'is-success' },
        'controlled': { text: '占用中', class: 'is-warning' },
        'occupied': { text: '占用中', class: 'is-warning' },
        'disabled': { text: '已禁用', class: 'is-dark' }
    };

    /**
     * 更新交管區狀態顯示
     * @param {Object} zone - 交管區數據
     */
    function updateTrafficZoneStatus(zone) {
        if (!zone) return;

        // 更新狀態標籤
        const statusStyle = STATUS_STYLES[zone.status] || { text: '未知', class: 'is-light' };
        if (statusTag) {
            statusTag.textContent = statusStyle.text;
            statusTag.className = `tag ${statusStyle.class}`;
        }

        // 更新狀態文字
        if (statusTextSpan) {
            statusTextSpan.textContent = statusStyle.text;
        }

        // 更新占用者
        if (ownerSpan) {
            if (zone.owner_agv_name) {
                ownerSpan.textContent = zone.owner_agv_name;
                ownerSpan.className = 'has-text-weight-bold has-text-danger';
            } else {
                ownerSpan.textContent = '無';
                ownerSpan.className = '';
            }
        }
    }

    /**
     * 從 trafficZonesStore 更新當前交管區的狀態
     */
    function updateCurrentZoneStatus() {
        if (!currentZoneId) return;

        // 檢查 trafficZonesStore 是否存在
        if (!window.trafficZonesStore) {
            console.warn('trafficZonesStore 不存在，無法更新交管區狀態');
            return;
        }

        const zones = window.trafficZonesStore.getState().trafficZones || [];
        const zone = zones.find(z => z.id === currentZoneId);

        if (zone) {
            updateTrafficZoneStatus(zone);
        } else {
            // 如果找不到對應的交管區，顯示未知狀態
            if (statusTag) {
                statusTag.textContent = "未知";
                statusTag.className = "tag is-light";
            }
        }
    }

    /**
     * 更新登入狀態顯示
     */
    function updateLoginStatus() {
        // 使用 userStore 檢查登入狀態
        const isLoggedIn = window.userStore && window.userStore.getState().isLoggedIn;

        if (notLoggedInDiv) {
            notLoggedInDiv.style.display = isLoggedIn ? 'none' : 'block';
        }

        if (forceReleaseSection) {
            forceReleaseSection.style.display = isLoggedIn ? 'block' : 'none';
        }
    }

    /**
     * 打開交管區控制 Modal
     * @param {number} zoneId - 交管區的 ID
     * @param {string} zoneName - 交管區的名稱
     */
    function openModal(zoneId, zoneName) {
        if (!modal) {
            console.error('Modal 元素未初始化');
            return;
        }

        currentZoneId = zoneId;

        // 更新 Modal 標題和交管區名稱
        if (zoneNameSpan) {
            zoneNameSpan.textContent = zoneName || `交管區 ${zoneId}`;
        }

        // 更新狀態顯示
        updateCurrentZoneStatus();

        // 更新登入狀態
        updateLoginStatus();

        // 顯示 Modal
        modal.classList.add('is-active');
    }

    /**
     * 關閉交管區控制 Modal
     */
    function closeModal() {
        if (!modal) return;

        modal.classList.remove('is-active');
        currentZoneId = null;
    }

    /**
     * 調用強制釋放 API
     * @param {number} zoneId - 交管區的 ID
     * @returns {Promise<Object>} API 響應數據
     */
    async function forceReleaseTrafficZone(zoneId) {
        const response = await fetch(`${TRAFFIC_API_BASE}/force_release`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                trafficId: String(zoneId)
            })
        });

        if (!response.ok) {
            const errorData = await response.json().catch(() => ({ detail: '未知錯誤' }));
            throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
        }

        const data = await response.json();
        return data;
    }

    /**
     * 處理強制解除交管按鈕點擊
     */
    async function handleForceRelease() {
        if (!currentZoneId) return;

        // 檢查登入狀態（使用 userStore）
        const isLoggedIn = window.userStore && window.userStore.getState().isLoggedIn;
        if (!isLoggedIn) {
            alert('請先登入才能強制解除交管');
            return;
        }

        // 二次確認對話框
        const zoneName = zoneNameSpan ? zoneNameSpan.textContent : `交管區 ${currentZoneId}`;
        const confirmed = confirm(
            `確定要強制解除 ${zoneName} 的交管嗎？\n\n` +
            `⚠️ 警告：請確認交管區內沒有車輛！\n\n` +
            `此操作將立即釋放交管區，無論當前是否有 AGV 佔用。`
        );

        if (!confirmed) {
            return;
        }

        // 禁用按鈕並顯示載入狀態
        forceReleaseBtn.disabled = true;
        forceReleaseBtn.classList.add('is-loading');

        try {
            const result = await forceReleaseTrafficZone(currentZoneId);
            console.log(`交管區 ${currentZoneId} 強制解除成功:`, result);

            // 顯示成功訊息
            alert(`成功強制解除交管！\n\n交管區：${zoneName}\n原占用者：${result.previous_owner || '無'}`);

            // 更新顯示（等待 Socket.IO 推送更新）
            setTimeout(() => {
                updateCurrentZoneStatus();
            }, 500);

        } catch (error) {
            console.error(`交管區 ${currentZoneId} 強制解除失敗:`, error);
            alert(`強制解除交管失敗：${error.message}`);
        } finally {
            // 恢復按鈕狀態
            setTimeout(() => {
                forceReleaseBtn.disabled = false;
                forceReleaseBtn.classList.remove('is-loading');
            }, 500);
        }
    }

    /**
     * 初始化交管區控制模態窗口
     */
    function setup() {
        // 獲取 Modal 元素
        modal = document.getElementById('mapTrafficZoneModal');
        if (!modal) {
            console.error('找不到 mapTrafficZoneModal 元素');
            return;
        }

        // 獲取其他元素
        statusTag = document.getElementById('trafficZoneModalStatus');
        zoneNameSpan = document.getElementById('trafficZoneModalName');
        statusTextSpan = document.getElementById('trafficZoneModalStatusText');
        ownerSpan = document.getElementById('trafficZoneModalOwner');
        notLoggedInDiv = document.getElementById('trafficZoneNotLoggedIn');
        forceReleaseSection = document.getElementById('trafficZoneForceReleaseSection');
        forceReleaseBtn = document.getElementById('trafficZoneForceReleaseBtn');

        // 綁定關閉按鈕事件
        const closeModalBtn = document.getElementById('trafficZoneModalClose');
        const closeFooterBtn = document.getElementById('trafficZoneModalCloseBtn');

        if (closeModalBtn) {
            closeModalBtn.addEventListener('click', closeModal);
        }

        if (closeFooterBtn) {
            closeFooterBtn.addEventListener('click', closeModal);
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

        // 綁定強制解除按鈕
        if (forceReleaseBtn) {
            forceReleaseBtn.addEventListener('click', handleForceRelease);
        }

        // 訂閱 trafficZonesStore 變化以即時更新交管區狀態
        if (window.trafficZonesStore) {
            window.trafficZonesStore.on('change', (newState) => {
                // 只在模態窗口開啟且有當前交管區 ID 時更新狀態
                if (modal.classList.contains('is-active') && currentZoneId) {
                    const zones = newState.trafficZones || [];
                    const zone = zones.find(z => z.id === currentZoneId);

                    if (zone) {
                        updateTrafficZoneStatus(zone);
                    }
                }
            });

            console.log('地圖交管區控制 Modal 已訂閱 trafficZonesStore 變化');
        } else {
            console.warn('trafficZonesStore 未初始化，交管區狀態將無法即時更新');
        }

        // 訂閱用戶狀態變化（如果有 userStore）
        if (window.userStore) {
            window.userStore.on('change', () => {
                if (modal.classList.contains('is-active')) {
                    updateLoginStatus();
                }
            });
        }
    }

    // 導出公共接口
    return {
        setup,
        openModal,
        closeModal
    };
})();
