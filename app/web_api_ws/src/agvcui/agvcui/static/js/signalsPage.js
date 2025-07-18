import { signalsStore } from '../store/index.js';
import { notify } from './notify.js';

export const signalsPage = (() => {
    let currentEqpId = null; // 當前選中的設備ID
    let currentSignals = []; // 當前顯示的信號列表

    /**
     * 處理 signalsStore 變化事件
     * @param {Object} newState - 新的信號狀態
     */
    function handleSignalsChange(newState) {
        if (!newState?.signals) return;

        const allSignals = newState.signals || [];
        console.debug('收到信號更新:', allSignals.length, '個信號');

        // 根據當前篩選條件更新信號列表
        updateSignalsDisplay(allSignals);
    }

    /**
     * 更新信號顯示
     * @param {Array} allSignals - 所有信號數據
     */
    function updateSignalsDisplay(allSignals) {
        // 根據當前設備ID篩選信號
        let filteredSignals = allSignals;
        if (currentEqpId) {
            filteredSignals = allSignals.filter(signal => signal.eqp_id === currentEqpId);
        }

        currentSignals = filteredSignals;

        // 更新頁面上的信號值
        updateSignalValues(filteredSignals);

        // 更新統計信息
        updateSignalStats(filteredSignals, allSignals);
    }

    /**
     * 更新頁面上的信號值顯示（優化版本 - 使用唯一 ID 和精確變化檢測）
     * @param {Array} signals - 要顯示的信號列表
     */
    function updateSignalValues(signals) {
        signals.forEach(signal => {
            updateSignalRowOptimized(signal.id, signal);
        });
    }

    /**
     * 優化的信號行更新函數（只更新變化的欄位，不重建 DOM 結構）
     * @param {number} signalId - 信號 ID
     * @param {Object} newSignal - 新的信號資料
     */
    function updateSignalRowOptimized(signalId, newSignal) {
        let hasChanges = false;

        // 更新信號值（優化版本 - 只更新標籤內容，不重建整個 HTML）
        const valueTagElement = document.getElementById(`signal-tag-${signalId}`);
        if (valueTagElement) {
            const oldValue = extractValueFromTag(valueTagElement);
            const newValue = newSignal.value || '';

            console.debug(`信號 ${signalId} 值檢測: 舊值="${oldValue}" (${typeof oldValue}), 新值="${newValue}" (${typeof newValue})`);

            if (hasChanged(oldValue, newValue)) {
                // 只更新標籤的內容和樣式，不重建整個結構
                updateSignalValueTag(valueTagElement, newSignal);

                // 統一動畫目標：應用到 td 元素
                const valueCell = document.getElementById(`signal-value-${signalId}`);
                if (valueCell) {
                    addUpdateAnimation(valueCell);
                }

                hasChanges = true;
                console.debug(`信號 ${newSignal.name || signalId} 值更新: "${oldValue}" → "${newValue}"`);
            } else {
                console.debug(`信號 ${signalId} 值無變化，跳過動畫`);
            }
        }

        // 更新時間戳（帶變化檢測）
        const timestampElement = document.getElementById(`signal-timestamp-${signalId}`);
        if (timestampElement && newSignal.updated_at) {
            const oldTimestamp = timestampElement.textContent;
            const newTimestamp = new Date(newSignal.updated_at).toLocaleTimeString();

            if (hasChanged(oldTimestamp, newTimestamp)) {
                timestampElement.textContent = newTimestamp;
                // 統一動畫目標：應用到 td 元素
                const timestampTdElement = timestampElement.closest('td');
                addUpdateAnimation(timestampTdElement);
                hasChanges = true;
            }
        }

        // 記錄變化但不添加整行動畫
        if (hasChanges) {
            console.debug(`信號 ${newSignal.name || signalId} 資料已更新`);
        }
    }

    /**
     * 從 tag 元素中提取信號值
     * @param {Element} tagElement - tag 元素
     * @returns {string} 提取的值
     */
    function extractValueFromTag(tagElement) {
        const text = tagElement.textContent.trim();

        // 處理布爾值格式 "TRUE (1)" 或 "FALSE (0)"
        if (text.includes('TRUE') || text.includes('FALSE')) {
            return text.includes('TRUE') ? '1' : '0';
        }

        // 處理其他格式，直接返回文本
        return text === '無值' ? '' : text;
    }

    /**
     * 精確的變化檢測函數
     * @param {any} oldValue - 舊值
     * @param {any} newValue - 新值
     * @param {number} precision - 數值比較精度（可選）
     * @returns {boolean} 是否有變化
     */
    function hasChanged(oldValue, newValue, precision = null) {
        if (precision !== null && typeof oldValue === 'number' && typeof newValue === 'number') {
            return Math.abs(oldValue - newValue) > precision;
        }
        return oldValue !== newValue;
    }

    /**
     * 添加更新動畫效果（帶防重疊機制）
     * @param {Element} element - 要添加動畫的元素
     */
    function addUpdateAnimation(element) {
        if (!element) return;

        // 檢查是否已經在播放動畫
        if (element.classList.contains('signal-updated')) {
            console.debug('Signal 動畫進行中，跳過重複添加');
            return;
        }

        element.classList.add('signal-updated');
        setTimeout(() => {
            element.classList.remove('signal-updated');
        }, 1000); // 與 CSS 動畫持續時間一致
    }

    /**
     * 優化的信號值標籤更新（只更新內容和樣式，不重建結構）
     * @param {Element} tagElement - 標籤元素
     * @param {Object} signal - 信號對象
     */
    function updateSignalValueTag(tagElement, signal) {
        if (!signal.value && signal.value !== 0) {
            tagElement.className = 'tag is-light';
            tagElement.innerHTML = '無值';
            return;
        }

        const value = signal.value;
        const type = signal.type_of_value?.toLowerCase();

        if (type === 'bool' || type === 'boolean') {
            if (value == 1 || value === '1' || value === true || value === 'true') {
                tagElement.className = 'tag is-success';
                tagElement.innerHTML = `
                    <span class="icon"><i class="mdi mdi-check-circle"></i></span>
                    <span>TRUE (1)</span>
                `;
            } else if (value == 0 || value === '0' || value === false || value === 'false') {
                tagElement.className = 'tag is-danger';
                tagElement.innerHTML = `
                    <span class="icon"><i class="mdi mdi-close-circle"></i></span>
                    <span>FALSE (0)</span>
                `;
            } else {
                tagElement.className = 'tag is-warning';
                tagElement.textContent = value;
            }
        } else if (type === 'int' || type === 'integer' || type === 'number') {
            const numValue = parseInt(value);
            if (numValue > 0) {
                tagElement.className = 'tag is-info';
            } else if (numValue === 0) {
                tagElement.className = 'tag is-light';
            } else {
                tagElement.className = 'tag is-warning';
            }
            tagElement.textContent = value;
        } else if (type === 'float' || type === 'double' || type === 'decimal') {
            const numValue = parseFloat(value);
            if (numValue > 0) {
                tagElement.className = 'tag is-info';
            } else if (numValue === 0) {
                tagElement.className = 'tag is-light';
            } else {
                tagElement.className = 'tag is-warning';
            }
            tagElement.textContent = value;
        } else {
            tagElement.className = 'tag is-success';
            tagElement.textContent = value;
        }
    }

    /**
     * 根據信號生成值的 HTML
     * @param {Object} signal - 信號對象
     * @returns {string} 生成的 HTML
     */
    function generateValueHTML(signal) {
        if (!signal.value && signal.value !== 0) {
            return '<span class="tag is-light">無值</span>';
        }

        const value = signal.value;
        const type = signal.type_of_value?.toLowerCase();

        if (type === 'bool' || type === 'boolean') {
            if (value == 1 || value === '1' || value === true || value === 'true') {
                return `
                    <span class="tag is-success">
                        <span class="icon"><i class="mdi mdi-check-circle"></i></span>
                        <span>TRUE (1)</span>
                    </span>
                `;
            } else if (value == 0 || value === '0' || value === false || value === 'false') {
                return `
                    <span class="tag is-danger">
                        <span class="icon"><i class="mdi mdi-close-circle"></i></span>
                        <span>FALSE (0)</span>
                    </span>
                `;
            } else {
                return `<span class="tag is-warning">${value}</span>`;
            }
        } else if (type === 'int' || type === 'integer' || type === 'number') {
            const numValue = parseInt(value);
            if (numValue > 0) {
                return `<span class="tag is-info">${value}</span>`;
            } else if (numValue === 0) {
                return `<span class="tag is-light">${value}</span>`;
            } else {
                return `<span class="tag is-warning">${value}</span>`;
            }
        } else if (type === 'float' || type === 'double' || type === 'decimal') {
            const numValue = parseFloat(value);
            if (numValue > 0) {
                return `<span class="tag is-info">${value}</span>`;
            } else if (numValue === 0) {
                return `<span class="tag is-light">${value}</span>`;
            } else {
                return `<span class="tag is-warning">${value}</span>`;
            }
        } else {
            return `<span class="tag is-success">${value}</span>`;
        }
    }

    /**
     * 更新統計信息
     * @param {Array} filteredSignals - 當前篩選的信號
     * @param {Array} allSignals - 所有信號
     */
    function updateSignalStats(filteredSignals, allSignals) {
        // 更新標題中的信號數量
        const titleTag = document.querySelector('.level-item .tag');
        if (titleTag) {
            const count = filteredSignals.length;
            const deviceName = currentEqpId ? getDeviceName(currentEqpId) : '所有信號';
            titleTag.innerHTML = `
                <span class="icon">
                    <i class="mdi ${currentEqpId ? 'mdi-devices' : 'mdi-view-list'}"></i>
                </span>
                <span>${deviceName} (${count} 個信號)</span>
            `;
        }

        // 更新設備選擇器中的信號數量（如果需要的話）
        updateDeviceSelector(allSignals);
    }

    /**
     * 獲取設備名稱
     * @param {number} eqpId - 設備ID
     * @returns {string} 設備名稱
     */
    function getDeviceName(eqpId) {
        const selector = document.getElementById('eqpSelector');
        if (selector) {
            const option = selector.querySelector(`option[value="${eqpId}"]`);
            if (option) {
                return option.textContent.split(' (')[0]; // 提取設備名稱部分
            }
        }
        return `設備 ${eqpId}`;
    }

    /**
     * 更新設備選擇器中的信號數量
     * @param {Array} allSignals - 所有信號
     */
    function updateDeviceSelector(allSignals) {
        const selector = document.getElementById('eqpSelector');
        if (!selector) return;

        // 統計每個設備的信號數量
        const deviceSignalCounts = {};
        allSignals.forEach(signal => {
            if (signal.eqp_id) {
                deviceSignalCounts[signal.eqp_id] = (deviceSignalCounts[signal.eqp_id] || 0) + 1;
            }
        });

        // 更新選項文本（如果格式允許的話）
        selector.querySelectorAll('option[value]').forEach(option => {
            const eqpId = parseInt(option.value);
            if (eqpId && deviceSignalCounts[eqpId] !== undefined) {
                const deviceName = option.textContent.split(' (')[0];
                option.textContent = `${deviceName} (${deviceSignalCounts[eqpId]} 個信號)`;
            }
        });
    }

    /**
     * 初始化頁面
     */
    function setup() {
        console.log('🔧 初始化 Signals 頁面');

        // 獲取當前選中的設備ID（從URL參數或選擇器）
        const urlParams = new URLSearchParams(window.location.search);
        currentEqpId = urlParams.get('eqp_id') ? parseInt(urlParams.get('eqp_id')) : null;

        // 監聽 signalsStore 變化
        signalsStore.on('change', handleSignalsChange);

        // 初始化時獲取當前狀態並顯示
        const currentState = signalsStore.getState();
        if (currentState.signals) {
            handleSignalsChange(currentState);
        }

        // 綁定設備選擇器事件（如果還沒綁定的話）
        setupDeviceSelector();

        console.log('✅ Signals 頁面初始化完成');
    }

    /**
     * 設置設備選擇器
     */
    function setupDeviceSelector() {
        const selector = document.getElementById('eqpSelector');
        if (selector && !selector.hasAttribute('data-signals-page-bound')) {
            selector.setAttribute('data-signals-page-bound', 'true');

            selector.addEventListener('change', function () {
                const selectedEqpId = this.value;

                // 構建新的 URL
                let newUrl = '/signals';
                if (selectedEqpId) {
                    newUrl += `?eqp_id=${selectedEqpId}`;
                }

                // 跳轉到新的 URL，讓後端重新查詢
                window.location.href = newUrl;
            });
        }
    }

    /**
     * 清理資源
     */
    function cleanup() {
        signalsStore.off('change', handleSignalsChange);
        console.log('🧹 Signals 頁面資源已清理');
    }

    // 返回公開的方法
    return {
        setup,
        cleanup
    };
})();
