import { agvStore } from '../store/index.js';
import socket from './socket.js';
import { notify } from '../js/notify.js';

export const agvPage = (() => {

    function getLocalAgvId() {
        // 從隱藏欄位讀取本機 AGV ID
        const localAgvIdInput = document.getElementById("local-agv-id");
        return localAgvIdInput ? localAgvIdInput.value : '';
    }

    function bindRefreshButtonEvents() {
        const refreshButton = document.getElementById("refresh-button");
        if (refreshButton) {
            refreshButton.onclick = null; // 先移除舊事件
            refreshButton.onclick = () => {
                console.log('重新整理狀態');
                location.reload(); // 重新整理頁面
            };
        }
    }

    // 儲存已建立的 DOM 元素參考
    let domCache = {
        categories: {},
        initialized: false
    };

    function updateAgvStatus(agvStatus) {
        if (!agvStatus || typeof agvStatus !== 'object') {
            console.warn('Invalid AGV status data:', agvStatus);
            return;
        }
        
        // 使用本機 AGV ID 進行過濾
        const localAgvId = getLocalAgvId();
        const agvIdFromServer = agvStatus.AGV_ID || agvStatus.agv_id;//server傳回的AGVID的訊息

        console.debug("Local AGV ID:", localAgvId)
        console.debug("AGV ID from server:", agvIdFromServer)
        console.debug("Total attributes:", Object.keys(agvStatus).length) // 顯示屬性總數
        
        // 如果有本機 AGV ID，只顯示本機的狀態
        if (localAgvId && agvIdFromServer && agvIdFromServer !== localAgvId) {
            console.debug(`跳過非本機 AGV 狀態: ${agvIdFromServer} (本機: ${localAgvId})`);
            return;
        }
        
        // 更新摘要區域
        updateStatusSummary(agvStatus);

        // 更新多欄位網格顯示
        const categoriesContainer = document.getElementById('status-categories');
        if (!categoriesContainer) return;

        // 分類顯示狀態
        const categorizedStatus = categorizeStatus(agvStatus);
        
        // 如果是第一次或結構改變，建立 DOM
        if (!domCache.initialized || Object.keys(categorizedStatus).length !== Object.keys(domCache.categories).length) {
            createCategoryDOM(categoriesContainer, categorizedStatus);
            domCache.initialized = true;
        } else {
            // 只更新數值，不重建 DOM
            updateCategoryValues(categorizedStatus);
        }
    }
    
    function createCategoryDOM(container, categorizedStatus) {
        container.innerHTML = ''; // 只在初始化時清空
        domCache.categories = {};
        
        // 依照分類顯示 - 使用多欄位佈局
        for (const [category, items] of Object.entries(categorizedStatus)) {
            // 決定欄位寬度
            const itemCount = Object.keys(items).length;
            let columnClass = 'column is-4'; // 預設 3 欄
            
            // 根據項目數量調整欄位寬度
            if (category === '輸入狀態' || category === '輸出狀態' || category === '警報狀態') {
                columnClass = 'column is-12'; // IO/警報使用全寬，特殊顯示
            } else if (itemCount <= 10) {
                columnClass = 'column is-3'; // 小分類用 4 欄
            }
            
            // 建立分類區塊
            const categoryDiv = document.createElement('div');
            categoryDiv.className = columnClass;
            categoryDiv.setAttribute('data-category', category);
            
            // 分類卡片
            const card = document.createElement('div');
            card.className = 'card';
            card.style.height = '100%';
            
            // 卡片標題
            const cardHeader = document.createElement('header');
            cardHeader.className = 'card-header';
            cardHeader.innerHTML = `
                <p class="card-header-title">
                    ${category} 
                    <span class="tag is-light ml-2" data-count="${category}">${itemCount}</span>
                </p>
            `;
            
            // 卡片內容
            const cardContent = document.createElement('div');
            cardContent.className = 'card-content';
            cardContent.style.maxHeight = '400px';
            cardContent.style.overflowY = 'auto';
            cardContent.style.padding = '0.75rem';
            
            // 對於 IO 和警報狀態，使用特殊的網格顯示
            if (category === '輸入狀態' || category === '輸出狀態' || category === '警報狀態') {
                // 建立網格容器
                const gridContainer = document.createElement('div');
                gridContainer.style.display = 'grid';
                gridContainer.style.gridTemplateColumns = 'repeat(20, 1fr)';
                gridContainer.style.gap = '4px';
                gridContainer.style.padding = '8px';
                
                // 儲存到快取
                domCache.categories[category] = {
                    type: 'grid',
                    container: gridContainer,
                    indicators: {}
                };
                
                // 排序項目以確保順序正確
                const sortedEntries = Object.entries(items).sort((a, b) => {
                    const numA = parseInt(a[0].match(/\d+/)?.[0] || 0);
                    const numB = parseInt(b[0].match(/\d+/)?.[0] || 0);
                    return numA - numB;
                });
                
                // 建立每個 IO 指示器
                for (const [key, value] of sortedEntries) {
                    const indicator = document.createElement('div');
                    indicator.style.width = '100%';
                    indicator.style.paddingTop = '100%'; // 正方形
                    indicator.style.position = 'relative';
                    indicator.style.borderRadius = '4px';
                    indicator.style.border = '1px solid #ddd';
                    indicator.style.cursor = 'pointer';
                    indicator.setAttribute('data-key', key);
                    
                    // 根據值設定顏色
                    if (value === 1 || value === true) {
                        indicator.style.backgroundColor = '#48c774'; // 綠色 - ON
                        indicator.style.boxShadow = '0 0 8px rgba(72, 199, 116, 0.5)';
                    } else {
                        indicator.style.backgroundColor = '#f5f5f5'; // 灰色 - OFF
                    }
                    
                    // 內部標籤
                    const label = document.createElement('div');
                    label.style.position = 'absolute';
                    label.style.top = '50%';
                    label.style.left = '50%';
                    label.style.transform = 'translate(-50%, -50%)';
                    label.style.fontSize = '10px';
                    label.style.fontWeight = 'bold';
                    label.style.color = value ? 'white' : '#999';
                    
                    // 提取數字
                    const num = key.match(/\d+/)?.[0] || key;
                    label.textContent = num;
                    
                    // Tooltip
                    indicator.title = `${key}: ${value}`;
                    
                    indicator.appendChild(label);
                    gridContainer.appendChild(indicator);
                    
                    // 儲存參考
                    domCache.categories[category].indicators[key] = indicator;
                }
                
                cardContent.appendChild(gridContainer);
            } else {
                // 其他分類使用原本的表格顯示
                const table = document.createElement('table');
                table.className = 'table is-narrow is-fullwidth';
                table.style.fontSize = '0.875rem';
                
                const tbody = document.createElement('tbody');
                
                // 儲存到快取
                domCache.categories[category] = {
                    type: 'table',
                    tbody: tbody,
                    cells: {}
                };
                
                // 顯示該分類的項目
                for (const [key, value] of Object.entries(items)) {
                    const row = document.createElement('tr');
                    row.setAttribute('data-key', key);
                    
                    const keyCell = document.createElement('td');
                    keyCell.style.width = '60%';
                    keyCell.style.padding = '0.25rem';
                    keyCell.innerHTML = `<small>${key}</small>`;
                    
                    const valueCell = document.createElement('td');
                    valueCell.style.width = '40%';
                    valueCell.style.padding = '0.25rem';
                    valueCell.style.textAlign = 'right';
                    valueCell.setAttribute('data-value-cell', key);
                    
                    // 格式化顯示
                    formatValueCell(valueCell, value, key);
                    
                    row.appendChild(keyCell);
                    row.appendChild(valueCell);
                    tbody.appendChild(row);
                    
                    // 儲存參考
                    domCache.categories[category].cells[key] = valueCell;
                }
                
                table.appendChild(tbody);
                cardContent.appendChild(table);
            }
            
            card.appendChild(cardHeader);
            card.appendChild(cardContent);
            categoryDiv.appendChild(card);
            container.appendChild(categoryDiv);
        }
    }
    
    function updateCategoryValues(categorizedStatus) {
        // 只更新數值，不重建 DOM
        for (const [category, items] of Object.entries(categorizedStatus)) {
            const cached = domCache.categories[category];
            if (!cached) continue;
            
            if (cached.type === 'grid') {
                // 更新 IO 網格
                for (const [key, value] of Object.entries(items)) {
                    const indicator = cached.indicators[key];
                    if (indicator) {
                        // 更新顏色
                        if (value === 1 || value === true) {
                            indicator.style.backgroundColor = '#48c774';
                            indicator.style.boxShadow = '0 0 8px rgba(72, 199, 116, 0.5)';
                            indicator.querySelector('div').style.color = 'white';
                        } else {
                            indicator.style.backgroundColor = '#f5f5f5';
                            indicator.style.boxShadow = '';
                            indicator.querySelector('div').style.color = '#999';
                        }
                        // 更新 tooltip
                        indicator.title = `${key}: ${value}`;
                    }
                }
            } else if (cached.type === 'table') {
                // 更新表格數值
                for (const [key, value] of Object.entries(items)) {
                    const cell = cached.cells[key];
                    if (cell) {
                        formatValueCell(cell, value, key);
                    }
                }
            }
        }
    }
    
    function formatValueCell(cell, value, key) {
        // 格式化顯示
        if (value === null || value === undefined) {
            cell.innerHTML = '<small class="has-text-grey">-</small>';
        } else if (typeof value === 'boolean') {
            cell.innerHTML = value 
                ? '<span class="has-text-success">✓</span>' 
                : '<span class="has-text-grey-light">✗</span>';
        } else if (typeof value === 'number') {
            // 數值格式化
            let displayValue = value;
            if (key.includes('SPEED') || key.includes('SLAM')) {
                displayValue = value.toFixed(2);
            } else if (key.includes('POWER')) {
                displayValue = value.toFixed(1) + '%';
            }
            cell.innerHTML = `<small class="has-text-info has-text-weight-semibold">${displayValue}</small>`;
        } else {
            cell.innerHTML = `<small>${value}</small>`;
        }
    }
    
    function updateStatusSummary(status) {
        // 顯示摘要區域
        const summaryDiv = document.getElementById('status-summary');
        if (summaryDiv) {
            summaryDiv.style.display = 'block';
        }
        
        // 更新摘要資料
        const agvIdElem = document.getElementById('summary-agv-id');
        const totalAttrsElem = document.getElementById('summary-total-attrs');
        const positionElem = document.getElementById('summary-position');
        const powerElem = document.getElementById('summary-power');
        const modeElem = document.getElementById('summary-mode');
        
        if (agvIdElem) {
            const displayId = status.AGV_ID || '-';
            const localAgvId = getLocalAgvId();
            // 如果是本機 AGV，特別標示
            if (localAgvId && displayId === localAgvId) {
                agvIdElem.innerHTML = `<span class="has-text-primary">${displayId}</span> <small>(本機)</small>`;
            } else {
                agvIdElem.textContent = displayId;
            }
        }
        if (totalAttrsElem) totalAttrsElem.textContent = Object.keys(status).length;
        
        if (positionElem) {
            const x = status.AGV_SLAM_X || 0;
            const y = status.AGV_SLAM_Y || 0;
            positionElem.textContent = `(${x.toFixed(2)}, ${y.toFixed(2)})`;
        }
        
        if (powerElem) {
            const power = status.POWER || 0;
            powerElem.textContent = `${power.toFixed(1)}%`;
            powerElem.className = power > 50 ? 'title has-text-success' : 
                                 power > 20 ? 'title has-text-warning' : 'title has-text-danger';
        }
        
        if (modeElem) {
            const isAuto = status.AGV_Auto;
            const isManual = status.AGV_Manual;
            const mode = isAuto ? '自動' : isManual ? '手動' : '未知';
            modeElem.textContent = mode;
            modeElem.className = isAuto ? 'title has-text-success' : 
                                isManual ? 'title has-text-info' : 'title has-text-grey';
        }
    }
    
    function categorizeStatus(status) {
        // 將狀態分類以便更好地顯示
        const categories = {
            '基本資訊': {},
            '位置狀態': {},
            '速度狀態': {},
            '位元狀態': {},
            '門控狀態': {},
            '輸入狀態': {},
            '輸出狀態': {},
            '警報狀態': {},
            'PLC記憶體': {},
            '其他': {}
        };
        
        for (const [key, value] of Object.entries(status)) {
            if (key.includes('AGV_ID') || key.includes('MAGIC') || key.includes('timestamp') || key.includes('namespace')) {
                categories['基本資訊'][key] = value;
            } else if (key.includes('SLAM') || key.includes('PGV') || key.includes('POINT') || key.includes('ZONE')) {
                categories['位置狀態'][key] = value;
            } else if (key.includes('SPEED') || key.includes('POWER')) {
                categories['速度狀態'][key] = value;
            } else if (key.includes('AGV_') && (key.includes('Auto') || key.includes('Manual') || key.includes('MOVING') || key.includes('TURN'))) {
                categories['位元狀態'][key] = value;
            } else if (key.includes('DOOR')) {
                categories['門控狀態'][key] = value;
            } else if (key.includes('Input')) {
                categories['輸入狀態'][key] = value;
            } else if (key.includes('Output')) {
                categories['輸出狀態'][key] = value;
            } else if (key.includes('Alarm') || key.includes('ALARM')) {
                categories['警報狀態'][key] = value;
            } else if (key.includes('PLC')) {
                categories['PLC記憶體'][key] = value;
            } else {
                categories['其他'][key] = value;
            }
        }
        
        // 移除空的分類
        for (const category in categories) {
            if (Object.keys(categories[category]).length === 0) {
                delete categories[category];
            }
        }
        
        return categories;
    }

    function handleChange(newState) {
        // console.debug('agv store state changed:', newState);
        if (newState?.agvStatus) {
            updateAgvStatus(newState.agvStatus);
        }
    }

    function setup() {
        //可以用 clear() state資料清空
        agvStore.clear();
        
        // 取得並儲存本機 AGV ID
        const localAgvId = getLocalAgvId();
        if (localAgvId) {
            agvStore.setState({ agvId: localAgvId });
            console.log(`✅ 使用本機 AGV ID: ${localAgvId}`);
        } else {
            console.log('⚠️ 未偵測到本機 AGV ID，顯示所有 AGV 狀態');
        }

        // 綁定按鈕事件
        bindRefreshButtonEvents()

        // 資料監聽狀態改變
        agvStore.on('change', handleChange);
        // 設定初始狀態
        handleChange(agvStore.getState());
    }

    return {
        setup,
    };
})();