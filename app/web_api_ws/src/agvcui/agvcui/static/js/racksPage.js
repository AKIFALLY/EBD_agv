import { racksStore } from '../store/index.js';
import { notify } from './notify.js';

export const racksPage = (() => {
    let currentRacks = []; // 當前顯示的貨架列表

    /**
     * 處理 racksStore 變化事件
     * @param {Object} newState - 新的貨架狀態
     */
    function handleRacksChange(newState) {
        if (!newState?.racks) return;

        const allRacks = newState.racks || [];
        console.debug('收到貨架更新:', allRacks.length, '個貨架');

        // 更新貨架顯示
        updateRacksDisplay(allRacks);
    }

    /**
     * 更新貨架顯示
     * @param {Array} allRacks - 所有貨架數據
     */
    function updateRacksDisplay(allRacks) {
        currentRacks = allRacks;

        // 更新頁面上的貨架值
        updateRackValues(allRacks);

        // 更新統計信息
        updateRackStats(allRacks);
    }

    /**
     * 更新頁面上的貨架值顯示（優化版本 - 使用唯一 ID 和精確變化檢測）
     * @param {Array} racks - 要顯示的貨架列表
     */
    function updateRackValues(racks) {
        racks.forEach(rack => {
            updateRackRowOptimized(rack.id, rack);
        });
    }

    /**
     * 優化的貨架行更新函數（只更新變化的欄位，不重建 DOM 結構）
     * @param {number} rackId - 貨架 ID
     * @param {Object} newRack - 新的貨架資料
     */
    function updateRackRowOptimized(rackId, newRack) {
        let hasChanges = false;

        // 更新貨架狀態（帶變化檢測）
        const statusElement = document.getElementById(`rack-status-${rackId}`);
        if (statusElement) {
            const oldStatus = extractStatusFromTag(statusElement);
            const newStatus = newRack.status_id;

            if (hasChanged(oldStatus, newStatus)) {
                updateRackStatusTag(statusElement, newRack.status_id);
                // 統一動畫目標：應用到 td 元素
                const statusTdElement = statusElement.closest('td');
                addUpdateAnimation(statusTdElement);
                hasChanges = true;
                console.debug(`貨架 ${newRack.name || rackId} 狀態更新: ${oldStatus} → ${newStatus}`);
            }
        }

        // 更新位置 ID（帶變化檢測和詳細 debug）
        const locationElement = document.getElementById(`rack-location-${rackId}`);
        if (locationElement) {
            const oldLocation = extractLocationFromTag(locationElement);
            const newLocation = newRack.location_id;

            console.debug(`貨架 ${rackId} 位置檢測: 舊值="${oldLocation}" (${typeof oldLocation}), 新值="${newLocation}" (${typeof newLocation})`);

            if (hasChanged(oldLocation, newLocation)) {
                updateRackLocationTag(locationElement, newRack.location_id);
                // 統一動畫目標：應用到 td 元素
                const locationTdElement = locationElement.closest('td');
                addUpdateAnimation(locationTdElement);
                hasChanges = true;
                console.debug(`貨架 ${newRack.name || rackId} 位置更新: "${oldLocation}" → "${newLocation}"`);
            } else {
                console.debug(`貨架 ${rackId} 位置無變化，跳過動畫`);
            }
        }

        // 更新產品 ID（帶變化檢測和詳細 debug）
        const productElement = document.getElementById(`rack-product-${rackId}`);
        if (productElement) {
            const oldProduct = extractProductFromTag(productElement);
            const newProduct = newRack.product_id;

            console.debug(`貨架 ${rackId} 產品檢測: 舊值="${oldProduct}" (${typeof oldProduct}), 新值="${newProduct}" (${typeof newProduct})`);

            if (hasChanged(oldProduct, newProduct)) {
                updateRackProductTag(productElement, newRack.product_id);
                // 統一動畫目標：應用到 td 元素
                const productTdElement = productElement.closest('td');
                addUpdateAnimation(productTdElement);
                hasChanges = true;
                console.debug(`貨架 ${newRack.name || rackId} 產品更新: "${oldProduct}" → "${newProduct}"`);
            } else {
                console.debug(`貨架 ${rackId} 產品無變化，跳過動畫`);
            }
        }

        // 更新方向（帶變化檢測）
        const directionElement = document.getElementById(`rack-direction-${rackId}`);
        if (directionElement) {
            const oldDirection = parseFloat(directionElement.textContent);
            const newDirection = newRack.direction;

            if (hasChanged(oldDirection, newDirection, 0.1)) { // 0.1度精度
                directionElement.textContent = `${newDirection.toFixed(1)}°`;
                // 統一動畫目標：應用到 td 元素
                const directionTdElement = directionElement.closest('td');
                addUpdateAnimation(directionTdElement);
                hasChanges = true;
                console.debug(`貨架 ${newRack.name || rackId} 方向更新: ${oldDirection}° → ${newDirection}°`);
            }
        }

        // 記錄變化但不添加整行動畫
        if (hasChanges) {
            console.debug(`貨架 ${newRack.name || rackId} 資料已更新`);
        }
    }

    /**
     * 精確的變化檢測函數（增強版本，處理數據類型一致性）
     * @param {any} oldValue - 舊值
     * @param {any} newValue - 新值
     * @param {number} precision - 數值比較精度（可選）
     * @returns {boolean} 是否有變化
     */
    function hasChanged(oldValue, newValue, precision = null) {
        // 處理 null/undefined 的情況
        if (oldValue === null && newValue === null) return false;
        if (oldValue === undefined && newValue === undefined) return false;
        if (oldValue === null && newValue === undefined) return false;
        if (oldValue === undefined && newValue === null) return false;

        // 處理數值比較（帶精度）
        if (precision !== null && typeof oldValue === 'number' && typeof newValue === 'number') {
            const result = Math.abs(oldValue - newValue) > precision;
            console.debug(`數值比較 (精度=${precision}): ${oldValue} vs ${newValue} = ${result}`);
            return result;
        }

        // 處理字符串比較（確保類型一致）
        if (typeof oldValue === 'string' || typeof newValue === 'string') {
            const oldStr = oldValue === null || oldValue === undefined ? null : String(oldValue);
            const newStr = newValue === null || newValue === undefined ? null : String(newValue);
            const result = oldStr !== newStr;
            console.debug(`字符串比較: "${oldStr}" vs "${newStr}" = ${result}`);
            return result;
        }

        // 默認比較
        const result = oldValue !== newValue;
        console.debug(`默認比較: ${oldValue} vs ${newValue} = ${result}`);
        return result;
    }

    /**
     * 添加更新動畫效果（帶防重疊機制）
     * @param {Element} element - 要添加動畫的元素
     */
    function addUpdateAnimation(element) {
        if (!element) return;

        // 檢查是否已經在播放動畫
        if (element.classList.contains('rack-updated')) {
            console.debug('Rack 動畫進行中，跳過重複添加');
            return;
        }

        element.classList.add('rack-updated');
        setTimeout(() => {
            element.classList.remove('rack-updated');
        }, 1000); // 與 CSS 動畫持續時間一致
    }

    /**
     * 從狀態標籤中提取狀態 ID
     * @param {Element} statusElement - 狀態標籤元素
     * @returns {number} 狀態 ID
     */
    function extractStatusFromTag(statusElement) {
        const text = statusElement.textContent.trim();
        if (text === '空閒') return 1;
        if (text === '使用中') return 2;
        if (text === '維護中') return 3;
        if (text === '故障') return 4;
        return null;
    }

    /**
     * 從位置標籤中提取位置 ID（增強版本，處理數據類型一致性）
     * @param {Element} locationElement - 位置標籤元素
     * @returns {string|null} 位置 ID
     */
    function extractLocationFromTag(locationElement) {
        const text = locationElement.textContent.trim();
        console.debug(`提取位置標籤文字: "${text}"`);

        if (text === '未設置' || text === '' || text === '-') {
            return null;
        }

        // 確保返回字符串類型，與後端數據保持一致
        return String(text);
    }

    /**
     * 從產品標籤中提取產品 ID（增強版本，處理數據類型一致性）
     * @param {Element} productElement - 產品標籤元素
     * @returns {string|null} 產品 ID
     */
    function extractProductFromTag(productElement) {
        const text = productElement.textContent.trim();
        console.debug(`提取產品標籤文字: "${text}"`);

        if (text === '無產品' || text === '' || text === '-') {
            return null;
        }

        // 確保返回字符串類型，與後端數據保持一致
        return String(text);
    }

    /**
     * 更新貨架狀態標籤
     * @param {Element} statusElement - 狀態標籤元素
     * @param {number} statusId - 狀態 ID
     */
    function updateRackStatusTag(statusElement, statusId) {
        if (statusId === 1) {
            statusElement.className = 'tag is-success';
            statusElement.textContent = '空閒';
        } else if (statusId === 2) {
            statusElement.className = 'tag is-info';
            statusElement.textContent = '使用中';
        } else if (statusId === 3) {
            statusElement.className = 'tag is-warning';
            statusElement.textContent = '維護中';
        } else if (statusId === 4) {
            statusElement.className = 'tag is-danger';
            statusElement.textContent = '故障';
        } else {
            statusElement.className = 'tag is-light';
            statusElement.textContent = statusId || '未知';
        }
    }

    /**
     * 更新貨架位置標籤
     * @param {Element} locationElement - 位置標籤元素
     * @param {string} locationId - 位置 ID
     */
    function updateRackLocationTag(locationElement, locationId) {
        if (locationId) {
            locationElement.className = 'tag is-info';
            locationElement.textContent = locationId;
        } else {
            locationElement.className = 'tag is-light';
            locationElement.textContent = '未設置';
        }
    }

    /**
     * 更新貨架產品標籤
     * @param {Element} productElement - 產品標籤元素
     * @param {string} productId - 產品 ID
     */
    function updateRackProductTag(productElement, productId) {
        if (productId) {
            productElement.className = 'tag is-primary';
            productElement.textContent = productId;
        } else {
            productElement.className = 'tag is-light';
            productElement.textContent = '無產品';
        }
    }

    /**
     * 更新統計信息
     * @param {Array} allRacks - 所有貨架
     */
    function updateRackStats(allRacks) {
        // 更新標題中的貨架數量
        const titleTag = document.querySelector('.level-item .tag');
        if (titleTag) {
            const count = allRacks.length;
            titleTag.innerHTML = `
                <span class="icon">
                    <i class="mdi mdi-view-grid"></i>
                </span>
                <span>貨架管理 (${count} 個貨架)</span>
            `;
        }
    }



    /**
     * 初始化頁面
     */
    function setup() {
        console.log('🔧 初始化 Racks 頁面');

        // 監聽 racksStore 變化
        racksStore.on('change', handleRacksChange);

        // 初始化時獲取當前狀態並顯示
        const currentState = racksStore.getState();
        if (currentState.racks) {
            handleRacksChange(currentState);
        }

        console.log('✅ Racks 頁面初始化完成');
    }

    /**
     * 清理資源
     */
    function cleanup() {
        racksStore.off('change', handleRacksChange);
        console.log('🧹 Racks 頁面資源已清理');
    }

    // 返回公開的方法
    return {
        setup,
        cleanup
    };
})();
