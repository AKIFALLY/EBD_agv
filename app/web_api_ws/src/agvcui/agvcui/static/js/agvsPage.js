// AGV 頁面 JavaScript
import { getStatusInfo } from './agvStatus.js';

export const agvsPage = (() => {

    // 設置頁面
    function setup() {
        console.log('AGV 頁面初始化');

        // 初始化表格排序
        initTableSorting();

        // 初始化搜尋功能
        initSearch();

        // 初始化篩選功能
        initFilters();

        // 監聽 agvsStore 變化
        if (window.agvsStore) {
            window.agvsStore.on('change', handleAgvsChange);
            console.log('agvsPage: 已訂閱 agvsStore 變化');
        }
    }

    // 處理 AGV 資料變化
    function handleAgvsChange(newState) {
        if (!newState?.agvs) return;

        const agvs = newState.agvs || [];
        console.log(`agvsPage: 收到 AGV 更新，共 ${agvs.length} 個 AGV`);

        // 更新頁面顯示
        updateAgvsList(agvs);
    }

    // 更新 AGV 列表（只更新值，不重建結構）
    function updateAgvsList(agvs) {
        const tbody = document.querySelector('.table tbody');
        if (!tbody) return;

        // 如果沒有現有的行，則進行初始化
        const existingRows = tbody.querySelectorAll('tr[data-agv-id]');
        if (existingRows.length === 0) {
            initializeAgvsList(agvs);
            return;
        }

        // 按 ID 排序
        const sortedAgvs = agvs.sort((a, b) => a.id - b.id);

        // 更新現有行的資料
        sortedAgvs.forEach(agv => {
            updateAgvRow(agv);
        });

        // 更新總數
        updateTotalCount(agvs.length);
    }

    // 初始化 AGV 列表（僅在首次載入時使用）
    function initializeAgvsList(agvs) {
        const tbody = document.querySelector('.table tbody');
        if (!tbody) return;

        // 清空現有內容
        tbody.innerHTML = '';

        if (agvs.length === 0) {
            // 顯示空狀態（優化版本 - 使用 DOM 創建而非 innerHTML）
            const emptyRow = document.createElement('tr');
            const emptyCell = document.createElement('td');
            emptyCell.setAttribute('colspan', '11');
            emptyCell.className = 'has-text-centered py-6';

            const iconSpan = document.createElement('span');
            iconSpan.className = 'icon is-large has-text-grey-light';
            const icon = document.createElement('i');
            icon.className = 'mdi mdi-robot mdi-48px';
            iconSpan.appendChild(icon);

            const title = document.createElement('p');
            title.className = 'title is-5 has-text-grey';
            title.textContent = '目前沒有 AGV';

            emptyCell.appendChild(iconSpan);
            emptyCell.appendChild(title);
            emptyRow.appendChild(emptyCell);
            tbody.appendChild(emptyRow);
        } else {
            // 按 ID 排序
            const sortedAgvs = agvs.sort((a, b) => a.id - b.id);

            // 生成表格行
            sortedAgvs.forEach(agv => {
                const row = createAgvRow(agv);
                tbody.appendChild(row);
            });
        }

        // 更新總數
        updateTotalCount(agvs.length);
    }

    // 創建 AGV 表格行
    function createAgvRow(agv) {
        const row = document.createElement('tr');
        row.setAttribute('data-agv-id', agv.id);

        // 獲取狀態樣式
        const statusInfo = getStatusInfo(agv.status_id);
        const enableInfo = getEnableInfo(agv.enable);
        const batteryInfo = getBatteryInfo(agv.battery, agv.id);

        row.innerHTML = `
            <td>
                <span class="tag is-info">${agv.id}</span>
            </td>
            <td>
                <strong>${agv.name}</strong>
                ${agv.description ? `<br><small class="has-text-grey">${agv.description}</small>` : ''}
            </td>
            <td>
                <span class="tag is-light">${agv.model}</span>
            </td>
            <td data-field="position">
                <span class="has-text-grey-dark" id="position-${agv.id}">
                    (${agv.x.toFixed(2)}, ${agv.y.toFixed(2)})
                </span>
            </td>
            <td data-field="heading">
                <span class="has-text-grey-dark" id="heading-${agv.id}">${agv.heading.toFixed(1)}°</span>
            </td>
            <td data-field="battery">
                ${batteryInfo}
            </td>
            <td data-field="status">
                <span class="tag ${statusInfo.color}" id="status-${agv.id}">${statusInfo.name}</span>
            </td>
            <td data-field="enable">
                <span class="tag ${enableInfo.class}" id="enable-${agv.id}">
                    <span class="icon">
                        <i class="mdi ${enableInfo.icon}"></i>
                    </span>
                    <span>${enableInfo.text}</span>
                </span>
            </td>
            <td data-field="last_node">
                <span id="last-node-${agv.id}">
                    ${agv.last_node_id ? `<span class="tag is-light" id="last-node-tag-${agv.id}">節點 ${agv.last_node_id}</span>` : '<span class="has-text-grey" id="last-node-tag-${agv.id}">-</span>'}
                </span>
            </td>
            <td>
                <div id="agv-status-json-${agv.id}">
                    ${agv.agv_status_json ? `
                    <div class="dropdown is-hoverable agv-status-json-dropdown">
                        <div class="dropdown-trigger">
                            <button class="button is-small is-info" aria-haspopup="true"
                                aria-controls="dropdown-menu-${agv.id}">
                                <span class="icon">
                                    <i class="mdi mdi-code-json"></i>
                                </span>
                                <span>JSON</span>
                                <span class="icon is-small">
                                    <i class="mdi mdi-chevron-down"></i>
                                </span>
                            </button>
                        </div>
                        <div class="dropdown-menu" id="dropdown-menu-${agv.id}" role="menu">
                            <div class="dropdown-content">
                                <div class="dropdown-item">
                                    <pre>${JSON.stringify(agv.agv_status_json, null, 2)}</pre>
                                </div>
                            </div>
                        </div>
                    </div>
                    ` : '<span class="has-text-grey-light">無資料</span>'}
                </div>
            </td>
            <td>
                <div class="buttons are-small">
                    <a class="button is-info is-small" href="/agvs/${agv.id}/edit">
                        <span class="icon">
                            <i class="mdi mdi-pencil"></i>
                        </span>
                        <span>編輯</span>
                    </a>
                </div>
            </td>
        `;

        return row;
    }

    // 更新特定 AGV 行的資料（只更新值，不重建結構）
    function updateAgvRow(agv) {
        const row = document.querySelector(`tr[data-agv-id="${agv.id}"]`);
        if (!row) return;

        let hasChanges = false;

        // 獲取狀態樣式
        const statusInfo = getStatusInfo(agv.status_id);
        const enableInfo = getEnableInfo(agv.enable);

        // 更新位置（帶變化檢測和詳細 debug）
        const positionElement = document.getElementById(`position-${agv.id}`);
        if (positionElement) {
            const oldPosition = positionElement.textContent.trim();
            const newPosition = `(${agv.x.toFixed(2)}, ${agv.y.toFixed(2)})`;

            console.debug(`AGV ${agv.id} 位置檢測: 舊值="${oldPosition}", 新值="${newPosition}"`);

            if (oldPosition !== newPosition) {
                positionElement.textContent = newPosition;
                // 統一動畫目標：應用到 td 元素
                const tdElement = positionElement.closest('td[data-field="position"]');
                addUpdateAnimation(tdElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 位置更新: "${oldPosition}" → "${newPosition}"`);
            } else {
                console.debug(`AGV ${agv.id} 位置無變化，跳過動畫`);
            }
        }

        // 更新方向（帶變化檢測和詳細 debug）
        const headingElement = document.getElementById(`heading-${agv.id}`);
        if (headingElement) {
            const oldHeading = headingElement.textContent.trim();
            const newHeading = `${agv.heading.toFixed(1)}°`;

            console.debug(`AGV ${agv.id} 方向檢測: 舊值="${oldHeading}", 新值="${newHeading}"`);

            if (oldHeading !== newHeading) {
                headingElement.textContent = newHeading;
                // 統一動畫目標：應用到 td 元素
                const tdElement = headingElement.closest('td[data-field="heading"]');
                addUpdateAnimation(tdElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 方向更新: "${oldHeading}" → "${newHeading}"`);
            } else {
                console.debug(`AGV ${agv.id} 方向無變化，跳過動畫`);
            }
        }

        // 更新電量（優化版本 - 只更新數值，不重建結構）
        const batteryProgressElement = document.getElementById(`battery-progress-${agv.id}`);
        const batteryTextElement = document.getElementById(`battery-text-${agv.id}`);

        if (batteryProgressElement && batteryTextElement) {
            const oldBatteryValue = parseFloat(batteryProgressElement.getAttribute('value'));
            const newBatteryValue = agv.battery !== null && agv.battery !== undefined ? agv.battery : null;

            if (newBatteryValue !== null && oldBatteryValue !== newBatteryValue) {
                // 更新進度條數值和樣式
                batteryProgressElement.setAttribute('value', newBatteryValue);

                // 更新進度條顏色樣式
                let progressClass = 'is-success';
                if (newBatteryValue < 50) {
                    progressClass = 'is-danger';
                } else if (newBatteryValue < 80) {
                    progressClass = 'is-warning';
                }

                // 移除舊的顏色類別並添加新的
                batteryProgressElement.className = batteryProgressElement.className.replace(/is-(success|warning|danger)/g, '');
                batteryProgressElement.classList.add(progressClass);

                // 更新文字顯示
                batteryTextElement.textContent = `${newBatteryValue.toFixed(1)}%`;

                // 統一動畫目標：應用到 td 元素
                const batteryTdElement = batteryProgressElement.closest('td[data-field="battery"]');
                addUpdateAnimation(batteryTdElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 電量更新: ${oldBatteryValue}% → ${newBatteryValue}%`);
            }
        }

        // 更新狀態（帶變化檢測）
        const statusElement = document.getElementById(`status-${agv.id}`);
        if (statusElement) {
            const oldStatus = statusElement.textContent;
            const newStatus = statusInfo.name;
            if (oldStatus !== newStatus) {
                statusElement.className = `tag ${statusInfo.color}`;
                statusElement.textContent = statusInfo.name;
                // 統一動畫目標：應用到 td 元素
                const statusTdElement = statusElement.closest('td[data-field="status"]');
                addUpdateAnimation(statusTdElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 狀態更新: ${oldStatus} → ${newStatus}`);
            }
        }

        // 更新啟用狀態（帶變化檢測）
        const enableElement = document.getElementById(`enable-${agv.id}`);
        if (enableElement) {
            const enableText = enableElement.querySelector('span:last-child');
            if (enableText) {
                const oldEnable = enableText.textContent;
                const newEnable = enableInfo.text;
                if (oldEnable !== newEnable) {
                    const enableIcon = enableElement.querySelector('i');
                    enableElement.className = `tag ${enableInfo.class}`;
                    enableIcon.className = `mdi ${enableInfo.icon}`;
                    enableText.textContent = enableInfo.text;
                    // 統一動畫目標：應用到 td 元素
                    const enableTdElement = enableElement.closest('td[data-field="enable"]');
                    addUpdateAnimation(enableTdElement);
                    hasChanges = true;
                    console.debug(`AGV ${agv.name} 啟用狀態更新: ${oldEnable} → ${newEnable}`);
                }
            }
        }

        // 更新最後節點（優化版本 - 只更新標籤內容，不重建結構）
        const lastNodeTagElement = document.getElementById(`last-node-tag-${agv.id}`);
        if (lastNodeTagElement) {
            const oldNode = extractNodeValue(lastNodeTagElement.parentElement);
            const newNode = agv.last_node_id;
            if (oldNode !== newNode) {
                updateLastNodeTag(lastNodeTagElement, agv.last_node_id);
                // 統一動畫目標：應用到 td 元素
                const nodeTdElement = lastNodeTagElement.closest('td[data-field="last_node"]');
                addUpdateAnimation(nodeTdElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 節點更新: ${oldNode} → ${newNode}`);
            }
        }

        // 更新 Status JSON（優化版本 - 只更新內容不重建 DOM）
        const statusJsonElement = document.getElementById(`agv-status-json-${agv.id}`);
        if (statusJsonElement) {
            updateAgvStatusJsonTag(statusJsonElement, agv.agv_status_json);
        }

        // 記錄變化但不添加整行動畫
        if (hasChanges) {
            console.debug(`AGV ${agv.name} 資料已更新`);
        }
    }

    // 添加更新動畫效果（帶防重疊機制）
    function addUpdateAnimation(element) {
        if (!element) return;

        // 檢查是否已經在播放動畫
        if (element.classList.contains('agv-field-updated')) {
            console.debug('AGV 動畫進行中，跳過重複添加');
            return;
        }

        element.classList.add('agv-field-updated');
        setTimeout(() => {
            element.classList.remove('agv-field-updated');
        }, 1000); // 與 CSS 動畫持續時間一致
    }



    // 從節點欄位提取節點 ID
    function extractNodeValue(nodeCell) {
        const tagElement = nodeCell.querySelector('.tag');
        if (tagElement) {
            const text = tagElement.textContent;
            const match = text.match(/節點 (\d+)/);
            return match ? parseInt(match[1]) : null;
        }
        return null;
    }

    /**
     * 優化的最後節點標籤更新（只更新內容和樣式，不重建結構）
     * @param {Element} tagElement - 節點標籤元素
     * @param {number} nodeId - 節點 ID
     */
    function updateLastNodeTag(tagElement, nodeId) {
        if (nodeId) {
            tagElement.className = 'tag is-light';
            tagElement.textContent = `節點 ${nodeId}`;
        } else {
            tagElement.className = 'has-text-grey';
            tagElement.textContent = '-';
        }
    }

    /**
     * 更新 AGV Status JSON 顯示（優化版本 - 只更新內容不重建 DOM）
     * @param {Element} element - 容器元素
     * @param {Object} statusJson - JSON 資料
     */
    function updateAgvStatusJsonTag(element, statusJson) {
        if (statusJson) {
            // 檢查 dropdown 是否已經存在
            const existingPre = element.querySelector('pre');

            if (existingPre) {
                // 只更新 <pre> 元素的文本內容，不重建 DOM
                existingPre.textContent = JSON.stringify(statusJson, null, 2);
            } else {
                // 首次創建：生成完整的 dropdown 結構
                const agvId = element.id.replace('agv-status-json-', '');
                element.innerHTML = `
                    <div class="dropdown is-hoverable agv-status-json-dropdown">
                        <div class="dropdown-trigger">
                            <button class="button is-small is-info" aria-haspopup="true"
                                    aria-controls="dropdown-menu-${agvId}">
                                <span class="icon">
                                    <i class="mdi mdi-code-json"></i>
                                </span>
                                <span>JSON</span>
                                <span class="icon is-small">
                                    <i class="mdi mdi-chevron-down"></i>
                                </span>
                            </button>
                        </div>
                        <div class="dropdown-menu" id="dropdown-menu-${agvId}" role="menu">
                            <div class="dropdown-content">
                                <div class="dropdown-item">
                                    <pre>${JSON.stringify(statusJson, null, 2)}</pre>
                                </div>
                            </div>
                        </div>
                    </div>
                `;
            }
        } else {
            // 無資料：只在結構不同時才更新
            const existingSpan = element.querySelector('span.has-text-grey-light');
            if (!existingSpan) {
                element.innerHTML = '<span class="has-text-grey-light">無資料</span>';
            }
        }
    }



    // 獲取啟用狀態資訊
    function getEnableInfo(enable) {
        if (enable === 1) {
            return { class: 'is-success', icon: 'mdi-check', text: '啟用' };
        } else {
            return { class: 'is-danger', icon: 'mdi-close', text: '停用' };
        }
    }

    // 獲取電量資訊（優化版本 - 添加唯一 ID）
    function getBatteryInfo(battery, agvId) {
        if (battery === null || battery === undefined) {
            return '<span class="has-text-grey">未知</span>';
        }

        let progressClass = 'is-success';
        if (battery < 50) {
            progressClass = 'is-danger';
        } else if (battery < 80) {
            progressClass = 'is-warning';
        }

        return `
            <div class="field">
                <div class="control">
                    <progress class="progress is-small ${progressClass}"
                              id="battery-progress-${agvId}"
                              value="${battery}" max="100">${battery}%</progress>
                </div>
                <p class="is-size-7 has-text-centered" id="battery-text-${agvId}">${battery.toFixed(1)}%</p>
            </div>
        `;
    }

    // 更新總數顯示（優化版本 - 只更新文字內容）
    function updateTotalCount(count) {
        const countElement = document.querySelector('.subtitle .level-item p');
        if (countElement) {
            // 檢查是否需要更新
            const currentText = countElement.textContent.trim();
            const newText = `總計 ${count} 個 AGV`;

            if (!currentText.includes(newText)) {
                // 重新構建內容，保持圖標結構
                countElement.innerHTML = `
                    <span class="icon">
                        <i class="mdi mdi-information"></i>
                    </span>
                    ${newText}
                `;
            }
        }
    }

    // 初始化表格排序
    function initTableSorting() {
        // 這裡可以加入表格排序功能
        console.log('表格排序功能初始化');
    }

    // 初始化搜尋功能
    function initSearch() {
        // 這裡可以加入搜尋功能
        console.log('搜尋功能初始化');
    }

    // 初始化篩選功能
    function initFilters() {
        // 這裡可以加入篩選功能
        console.log('篩選功能初始化');
    }

    // 公開方法
    return {
        setup
    };
})();
