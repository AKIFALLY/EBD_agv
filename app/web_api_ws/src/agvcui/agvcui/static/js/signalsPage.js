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
     * 更新頁面上的信號值顯示
     * @param {Array} signals - 要顯示的信號列表
     */
    function updateSignalValues(signals) {
        signals.forEach(signal => {
            // 查找對應的信號行
            const signalRow = document.querySelector(`tr[data-signal-id="${signal.id}"]`);
            if (!signalRow) return;

            // 更新信號值
            const valueCell = signalRow.querySelector('.signal-value');
            if (valueCell) {
                // 獲取當前顯示的值（從 tag 中提取）
                const currentTag = valueCell.querySelector('.tag');
                const oldValue = currentTag ? extractValueFromTag(currentTag) : '';
                const newValue = signal.value || '';

                if (oldValue !== newValue) {
                    // 值有變化，重新生成 HTML 並添加動畫效果
                    valueCell.innerHTML = generateValueHTML(signal);

                    // 添加更新動畫
                    valueCell.classList.add('signal-updated');
                    setTimeout(() => {
                        valueCell.classList.remove('signal-updated');
                    }, 1000);

                    console.debug(`信號 ${signal.name} 值更新: ${oldValue} → ${newValue}`);
                }
            }

            // 更新時間戳（如果有的話）
            const timestampCell = signalRow.querySelector('.signal-timestamp');
            if (timestampCell && signal.updated_at) {
                const updateTime = new Date(signal.updated_at).toLocaleTimeString();
                timestampCell.textContent = updateTime;
            }
        });
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
