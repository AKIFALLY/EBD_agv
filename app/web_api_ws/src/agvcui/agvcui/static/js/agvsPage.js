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
            // 顯示空狀態
            const emptyRow = document.createElement('tr');
            emptyRow.innerHTML = `
                <td colspan="10" class="has-text-centered py-6">
                    <span class="icon is-large has-text-grey-light">
                        <i class="mdi mdi-robot mdi-48px"></i>
                    </span>
                    <p class="title is-5 has-text-grey">目前沒有 AGV</p>
                </td>
            `;
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
        const batteryInfo = getBatteryInfo(agv.battery);

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
                <span class="has-text-grey-dark">
                    (${agv.x.toFixed(2)}, ${agv.y.toFixed(2)})
                </span>
            </td>
            <td data-field="heading">
                <span class="has-text-grey-dark">${agv.heading.toFixed(1)}°</span>
            </td>
            <td data-field="battery">
                ${batteryInfo}
            </td>
            <td data-field="status">
                <span class="tag ${statusInfo.color}">${statusInfo.name}</span>
            </td>
            <td data-field="enable">
                <span class="tag ${enableInfo.class}">
                    <span class="icon">
                        <i class="mdi ${enableInfo.icon}"></i>
                    </span>
                    <span>${enableInfo.text}</span>
                </span>
            </td>
            <td data-field="last_node">
                ${agv.last_node_id ? `<span class="tag is-light">節點 ${agv.last_node_id}</span>` : '<span class="has-text-grey">-</span>'}
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
        const batteryInfo = getBatteryInfo(agv.battery);

        // 更新位置（帶變化檢測）
        const positionCell = row.querySelector('[data-field="position"] span');
        if (positionCell) {
            const oldPosition = positionCell.textContent;
            const newPosition = `(${agv.x.toFixed(2)}, ${agv.y.toFixed(2)})`;
            if (oldPosition !== newPosition) {
                positionCell.textContent = newPosition;
                addUpdateAnimation(positionCell.parentElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 位置更新: ${oldPosition} → ${newPosition}`);
            }
        }

        // 更新方向（帶變化檢測）
        const headingCell = row.querySelector('[data-field="heading"] span');
        if (headingCell) {
            const oldHeading = headingCell.textContent;
            const newHeading = `${agv.heading.toFixed(1)}°`;
            if (oldHeading !== newHeading) {
                headingCell.textContent = newHeading;
                addUpdateAnimation(headingCell.parentElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 方向更新: ${oldHeading} → ${newHeading}`);
            }
        }

        // 更新電量（帶變化檢測）
        const batteryCell = row.querySelector('[data-field="battery"]');
        if (batteryCell) {
            const oldBatteryText = extractBatteryValue(batteryCell);
            const newBatteryText = agv.battery !== null && agv.battery !== undefined ?
                agv.battery.toFixed(1) : null;

            if (oldBatteryText !== newBatteryText) {
                batteryCell.innerHTML = batteryInfo;
                addUpdateAnimation(batteryCell);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 電量更新: ${oldBatteryText}% → ${newBatteryText}%`);
            }
        }

        // 更新狀態（帶變化檢測）
        const statusCell = row.querySelector('[data-field="status"] span');
        if (statusCell) {
            const oldStatus = statusCell.textContent;
            const newStatus = statusInfo.name;
            if (oldStatus !== newStatus) {
                statusCell.className = `tag ${statusInfo.color}`;
                statusCell.textContent = statusInfo.name;
                addUpdateAnimation(statusCell.parentElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 狀態更新: ${oldStatus} → ${newStatus}`);
            }
        }

        // 更新啟用狀態（帶變化檢測）
        const enableCell = row.querySelector('[data-field="enable"] span.tag');
        const enableText = row.querySelector('[data-field="enable"] span.tag span:last-child');
        if (enableCell && enableText) {
            const oldEnable = enableText.textContent;
            const newEnable = enableInfo.text;
            if (oldEnable !== newEnable) {
                const enableIcon = row.querySelector('[data-field="enable"] i');
                enableCell.className = `tag ${enableInfo.class}`;
                enableIcon.className = `mdi ${enableInfo.icon}`;
                enableText.textContent = enableInfo.text;
                addUpdateAnimation(enableCell.parentElement);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 啟用狀態更新: ${oldEnable} → ${newEnable}`);
            }
        }

        // 更新最後節點（帶變化檢測）
        const lastNodeCell = row.querySelector('[data-field="last_node"]');
        if (lastNodeCell) {
            const oldNode = extractNodeValue(lastNodeCell);
            const newNode = agv.last_node_id;
            if (oldNode !== newNode) {
                lastNodeCell.innerHTML = agv.last_node_id ?
                    `<span class="tag is-light">節點 ${agv.last_node_id}</span>` :
                    '<span class="has-text-grey">-</span>';
                addUpdateAnimation(lastNodeCell);
                hasChanges = true;
                console.debug(`AGV ${agv.name} 節點更新: ${oldNode} → ${newNode}`);
            }
        }

        // 記錄變化但不添加整行動畫
        if (hasChanges) {
            console.debug(`AGV ${agv.name} 資料已更新`);
        }
    }

    // 添加更新動畫效果
    function addUpdateAnimation(element) {
        if (!element) return;

        element.classList.add('agv-field-updated');
        setTimeout(() => {
            element.classList.remove('agv-field-updated');
        }, 1000);
    }

    // 從電量欄位提取數值
    function extractBatteryValue(batteryCell) {
        const progressElement = batteryCell.querySelector('progress');
        if (progressElement) {
            return progressElement.getAttribute('value');
        }
        return null;
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



    // 獲取啟用狀態資訊
    function getEnableInfo(enable) {
        if (enable === 1) {
            return { class: 'is-success', icon: 'mdi-check', text: '啟用' };
        } else {
            return { class: 'is-danger', icon: 'mdi-close', text: '停用' };
        }
    }

    // 獲取電量資訊
    function getBatteryInfo(battery) {
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
                              value="${battery}" max="100">${battery}%</progress>
                </div>
                <p class="is-size-7 has-text-centered">${battery.toFixed(1)}%</p>
            </div>
        `;
    }

    // 更新總數顯示
    function updateTotalCount(count) {
        const countElement = document.querySelector('.subtitle .level-item p');
        if (countElement) {
            countElement.innerHTML = `
                <span class="icon">
                    <i class="mdi mdi-information"></i>
                </span>
                總計 ${count} 個 AGV
            `;
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
