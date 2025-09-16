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
        
        // 處理嵌套的 JSON 結構，將其扁平化
        let flattenedStatus = {};
        
        // 如果有 metadata，提取它
        if (agvStatus.metadata) {
            Object.assign(flattenedStatus, agvStatus.metadata);
        }
        
        // 如果有 agv_status，提取它（這是主要的狀態資料）
        if (agvStatus.agv_status) {
            Object.assign(flattenedStatus, agvStatus.agv_status);
        }
        
        // 如果有 contexts，扁平化它 (base_context, agv_context, robot_context)
        if (agvStatus.contexts) {
            for (const [contextName, contextData] of Object.entries(agvStatus.contexts)) {
                if (typeof contextData === 'object' && contextData !== null) {
                    for (const [key, value] of Object.entries(contextData)) {
                        flattenedStatus[`${contextName}_${key}`] = value;
                    }
                }
            }
        }
        
        // 如果有 type_specific，扁平化它 (agv_ports, work_id, task_progress 等)
        if (agvStatus.type_specific) {
            for (const [key, value] of Object.entries(agvStatus.type_specific)) {
                if (value !== null && typeof value === 'object' && !Array.isArray(value)) {
                    // 巢狀物件扁平化（如 agv_ports: {port1: true, port2: false}）
                    for (const [subKey, subValue] of Object.entries(value)) {
                        flattenedStatus[`${key}_${subKey}`] = subValue;
                    }
                } else {
                    // 簡單值直接保留（如 work_id: null）
                    flattenedStatus[key] = value;
                }
            }
        }
        
        // 如果有 door_status，直接合併（保持原始鍵名）
        if (agvStatus.door_status) {
            Object.assign(flattenedStatus, agvStatus.door_status);
        }
        
        // 如果有 alarms，直接合併
        if (agvStatus.alarms) {
            Object.assign(flattenedStatus, agvStatus.alarms);
        }
        
        // 如果有 io_data，直接合併
        if (agvStatus.io_data) {
            Object.assign(flattenedStatus, agvStatus.io_data);
        }
        
        // 如果沒有嵌套結構，直接使用原始資料
        if (Object.keys(flattenedStatus).length === 0) {
            flattenedStatus = agvStatus;
        }
        
        // 使用本機 AGV ID 進行過濾
        const localAgvId = getLocalAgvId();
        const agvIdFromServer = flattenedStatus.AGV_ID || flattenedStatus.agv_id || flattenedStatus.agv_id;//server傳回的AGVID的訊息

        console.debug("Local AGV ID:", localAgvId)
        console.debug("AGV ID from server:", agvIdFromServer)
        console.debug("Total attributes:", Object.keys(flattenedStatus).length); // 顯示屬性總數
        // 顯示各類別的資料筆數（除錯用）
        const categorySummary = {
            metadata: Object.keys(agvStatus.metadata || {}).length,
            agv_status: Object.keys(agvStatus.agv_status || {}).length,
            contexts: Object.keys(agvStatus.contexts || {}).length,
            type_specific: Object.keys(agvStatus.type_specific || {}).length,
            io_data: Object.keys(agvStatus.io_data || {}).length,
            door_status: Object.keys(agvStatus.door_status || {}).length,
            alarms: Object.keys(agvStatus.alarms || {}).length
        };
        console.debug("Category summary:", categorySummary);
        
        // 如果有本機 AGV ID，只顯示本機的狀態
        if (localAgvId && agvIdFromServer && agvIdFromServer !== localAgvId) {
            console.debug(`跳過非本機 AGV 狀態: ${agvIdFromServer} (本機: ${localAgvId})`);
            return;
        }
        
        // 更新摘要區域（使用扁平化的資料）
        updateStatusSummary(flattenedStatus);

        // 更新多欄位網格顯示
        const categoriesContainer = document.getElementById('status-categories');
        if (!categoriesContainer) return;

        // 分類顯示狀態（使用扁平化的資料）
        const categorizedStatus = categorizeStatus(flattenedStatus);
        
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
        
        // 定義分類的顯示順序 - 其他放在最前面
        const categoryOrder = [
            '其他',           // 最先顯示
            '基本資訊',
            '位置狀態',
            '運動狀態',
            '控制狀態',
            '狀態機',
            '任務資訊',
            '門控狀態',
            '輸入狀態',
            '輸出狀態',
            '警報狀態',
            'PLC記憶體'
        ];
        
        // 根據定義的順序排序分類
        const sortedCategories = Object.entries(categorizedStatus).sort((a, b) => {
            const orderA = categoryOrder.indexOf(a[0]);
            const orderB = categoryOrder.indexOf(b[0]);
            // 如果都在順序列表中，按順序排序
            if (orderA !== -1 && orderB !== -1) {
                return orderA - orderB;
            }
            // 如果只有一個在列表中，列表中的優先
            if (orderA !== -1) return -1;
            if (orderB !== -1) return 1;
            // 都不在列表中，按字母順序
            return a[0].localeCompare(b[0]);
        });
        
        // 依照分類顯示 - 使用多欄位佈局
        for (const [category, items] of sortedCategories) {
            // 決定欄位寬度
            const itemCount = Object.keys(items).length;
            let columnClass = 'column is-4'; // 預設 3 欄
            
            // 根據項目數量調整欄位寬度
            if (category === '輸入狀態' || category === '輸出狀態') {
                columnClass = 'column is-6'; // IO 狀態使用半寬
            } else if (category === '警報狀態') {
                columnClass = 'column is-12'; // 警報使用全寬
            } else if (category === '門控狀態') {
                columnClass = 'column is-12'; // 門控狀態使用全寬
            } else if (category === '狀態機') {
                columnClass = 'column is-6'; // 狀態機使用半寬
            } else if (category === '其他' || itemCount <= 10) {
                columnClass = 'column is-3'; // 其他和小分類用 4 欄
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
            // 移除固定高度和捲軸，讓內容自動調整高度
            cardContent.style.padding = '0.75rem';
            
            // 對於門控狀態，使用特殊的橫向顯示
            if (category === '門控狀態') {
                // 建立門控顯示容器
                const doorContainer = document.createElement('div');
                doorContainer.style.padding = '10px';
                
                // 儲存到快取
                domCache.categories[category] = {
                    type: 'doors',
                    container: doorContainer,
                    doorElements: {}
                };
                
                // 將門控狀態按門號分組
                const doors = {};
                for (const [key, value] of Object.entries(items)) {
                    // 提取門號 (DOOR_1, DOOR_OPEN_1, DOOR_CLOSE_1 等)
                    const match = key.match(/DOOR.*?(\d+)/);
                    if (match) {
                        const doorNum = match[1];
                        if (!doors[doorNum]) {
                            doors[doorNum] = {};
                        }
                        
                        if (key.includes('OPEN')) {
                            doors[doorNum].open = value;
                            doors[doorNum].openKey = key;
                        } else if (key.includes('CLOSE')) {
                            doors[doorNum].close = value;
                            doors[doorNum].closeKey = key;
                        } else {
                            // 可能是狀態值 (DOOR_1: "OPEN", "CLOSED", "MOVING" 等)
                            doors[doorNum].status = value;
                            doors[doorNum].statusKey = key;
                        }
                    }
                }
                
                // 排序門號
                const sortedDoors = Object.entries(doors).sort((a, b) => {
                    return parseInt(a[0]) - parseInt(b[0]);
                });
                
                // 建立橫向顯示的門控狀態 - 使用 grid 佈局，每行 8 個門
                const doorsGrid = document.createElement('div');
                doorsGrid.style.display = 'grid';
                doorsGrid.style.gridTemplateColumns = 'repeat(8, 1fr)'; // 每行固定 8 個門
                doorsGrid.style.gap = '10px';
                doorsGrid.style.alignItems = 'center';
                
                for (const [doorNum, doorData] of sortedDoors) {
                    // 每個門的容器
                    const doorItem = document.createElement('div');
                    doorItem.style.display = 'flex';
                    doorItem.style.flexDirection = 'column';
                    doorItem.style.alignItems = 'center';
                    doorItem.style.justifyContent = 'center';
                    doorItem.style.padding = '8px';
                    doorItem.style.backgroundColor = '#1a1a1a';
                    doorItem.style.borderRadius = '6px';
                    doorItem.style.border = '1px solid #3a3a3a';
                    doorItem.style.width = '100%';
                    doorItem.style.aspectRatio = '1.2';
                    
                    // 門號標籤
                    const doorLabel = document.createElement('div');
                    doorLabel.style.fontSize = '0.85rem';
                    doorLabel.style.fontWeight = 'bold';
                    doorLabel.style.color = '#b0b0b0';
                    doorLabel.style.marginBottom = '6px';
                    doorLabel.textContent = `門 ${doorNum}`;
                    doorItem.appendChild(doorLabel);
                    
                    // 狀態指示器容器
                    const statusContainer = document.createElement('div');
                    statusContainer.style.display = 'flex';
                    statusContainer.style.gap = '4px';
                    
                    // 如果有狀態文字（如 "OPEN", "CLOSED", "MOVING"）
                    if (doorData.status !== undefined) {
                        const statusText = document.createElement('div');
                        statusText.style.padding = '4px 8px';
                        statusText.style.borderRadius = '4px';
                        statusText.style.fontSize = '0.75rem';
                        statusText.style.fontWeight = 'bold';
                        
                        // 根據狀態設定顏色
                        if (doorData.status === 'OPEN' || doorData.status === 1 || doorData.status === '1') {
                            statusText.style.backgroundColor = '#48c774';
                            statusText.style.color = 'white';
                            statusText.textContent = '開';
                        } else if (doorData.status === 'CLOSED' || doorData.status === 0 || doorData.status === '0') {
                            statusText.style.backgroundColor = '#2b2b2b';
                            statusText.style.color = '#aaa';
                            statusText.textContent = '關';
                        } else if (doorData.status === 'MOVING' || doorData.status === 2 || doorData.status === '2') {
                            statusText.style.backgroundColor = '#ffdd57';
                            statusText.style.color = '#363636';
                            statusText.textContent = '移動中';
                        } else {
                            statusText.style.backgroundColor = '#3a3a3a';
                            statusText.style.color = '#ccc';
                            statusText.textContent = String(doorData.status);
                        }
                        
                        statusContainer.appendChild(statusText);
                        
                        // 儲存參考
                        if (!domCache.categories[category].doorElements[doorNum]) {
                            domCache.categories[category].doorElements[doorNum] = {};
                        }
                        domCache.categories[category].doorElements[doorNum].status = statusText;
                    }
                    
                    // 如果有開/關狀態
                    if (doorData.open !== undefined || doorData.close !== undefined) {
                        // 開狀態
                        if (doorData.open !== undefined) {
                            const openIndicator = document.createElement('div');
                            openIndicator.style.width = '35px';
                            openIndicator.style.height = '35px';
                            openIndicator.style.borderRadius = '4px';
                            openIndicator.style.display = 'flex';
                            openIndicator.style.alignItems = 'center';
                            openIndicator.style.justifyContent = 'center';
                            openIndicator.style.fontSize = '0.75rem';
                            openIndicator.style.fontWeight = 'bold';
                            openIndicator.title = `${doorData.openKey}: ${doorData.open}`;
                            
                            const isOpen = doorData.open === true || doorData.open === 1 || doorData.open === '1';
                            if (isOpen) {
                                openIndicator.style.backgroundColor = '#48c774';
                                openIndicator.style.color = 'white';
                                openIndicator.style.boxShadow = '0 0 4px rgba(72, 199, 116, 0.5)';
                            } else {
                                openIndicator.style.backgroundColor = '#2b2b2b';
                                openIndicator.style.color = '#aaa';
                                openIndicator.style.border = '1px solid #3a3a3a';
                            }
                            openIndicator.textContent = '開';
                            statusContainer.appendChild(openIndicator);
                            
                            if (!domCache.categories[category].doorElements[doorNum]) {
                                domCache.categories[category].doorElements[doorNum] = {};
                            }
                            domCache.categories[category].doorElements[doorNum].open = openIndicator;
                        }
                        
                        // 關狀態
                        if (doorData.close !== undefined) {
                            const closeIndicator = document.createElement('div');
                            closeIndicator.style.width = '35px';
                            closeIndicator.style.height = '35px';
                            closeIndicator.style.borderRadius = '4px';
                            closeIndicator.style.display = 'flex';
                            closeIndicator.style.alignItems = 'center';
                            closeIndicator.style.justifyContent = 'center';
                            closeIndicator.style.fontSize = '0.75rem';
                            closeIndicator.style.fontWeight = 'bold';
                            closeIndicator.title = `${doorData.closeKey}: ${doorData.close}`;
                            
                            const isClosed = doorData.close === true || doorData.close === 1 || doorData.close === '1';
                            if (isClosed) {
                                closeIndicator.style.backgroundColor = '#ff3860';
                                closeIndicator.style.color = 'white';
                                closeIndicator.style.boxShadow = '0 0 4px rgba(255, 56, 96, 0.5)';
                            } else {
                                closeIndicator.style.backgroundColor = '#2b2b2b';
                                closeIndicator.style.color = '#aaa';
                                closeIndicator.style.border = '1px solid #3a3a3a';
                            }
                            closeIndicator.textContent = '關';
                            statusContainer.appendChild(closeIndicator);
                            
                            if (!domCache.categories[category].doorElements[doorNum]) {
                                domCache.categories[category].doorElements[doorNum] = {};
                            }
                            domCache.categories[category].doorElements[doorNum].close = closeIndicator;
                        }
                    }
                    
                    doorItem.appendChild(statusContainer);
                    doorsGrid.appendChild(doorItem);
                }
                
                doorContainer.appendChild(doorsGrid);
                cardContent.appendChild(doorContainer);
            }
            // 對於 IO 和警報狀態，使用特殊的網格顯示
            else if (category === '輸入狀態' || category === '輸出狀態' || category === '警報狀態') {
                // 建立網格容器
                const gridContainer = document.createElement('div');
                gridContainer.style.padding = '10px';
                
                // 儲存到快取
                domCache.categories[category] = {
                    type: 'grid',
                    container: gridContainer,
                    indicators: {}
                };
                
                // 將項目按群組分類 (AGV_INPUT_1_*, AGV_INPUT_2_*, 等)
                const groups = {};
                for (const [key, value] of Object.entries(items)) {
                    let groupKey = '其他';
                    let sortKey = 999;
                    let subKey = 999;
                    
                    if (key.includes('AGV_INPUT_') || key.includes('AGV_OUTPUT_')) {
                        // AGV_INPUT_1_1 -> group: 1, subKey: 1
                        const parts = key.split('_');
                        if (parts.length >= 4) {
                            groupKey = parts[2];
                            sortKey = parseInt(parts[2]) || 999;
                            subKey = parseInt(parts[3]) || 999;
                        }
                    } else if (key.includes('ALARM_STATUS_')) {
                        // ALARM_STATUS_1 -> group: 警報
                        groupKey = '警報';
                        sortKey = 0;
                        subKey = parseInt(key.split('_')[2]) || 999;
                    } else {
                        // 其他格式
                        const match = key.match(/\d+/);
                        if (match) {
                            subKey = parseInt(match[0]) || 999;
                        }
                    }
                    
                    if (!groups[groupKey]) {
                        groups[groupKey] = {
                            items: [],
                            sortKey: sortKey
                        };
                    }
                    groups[groupKey].items.push({ key, value, subKey });
                }
                
                // 排序群組
                const sortedGroups = Object.entries(groups).sort((a, b) => {
                    return a[1].sortKey - b[1].sortKey;
                });
                
                // 為每個群組建立一行
                for (const [groupName, group] of sortedGroups) {
                    // 排序群組內的項目
                    group.items.sort((a, b) => a.subKey - b.subKey);
                    
                    // 建立群組標籤
                    const groupLabel = document.createElement('div');
                    groupLabel.style.fontSize = '0.9rem'; // 使用 rem 單位
                    groupLabel.style.fontWeight = 'bold';
                    groupLabel.style.color = '#b0b0b0';
                    groupLabel.style.marginBottom = '5px';
                    groupLabel.style.marginTop = '10px';
                    groupLabel.textContent = `群組 ${groupName}`;
                    gridContainer.appendChild(groupLabel);
                    
                    // 建立群組的網格容器
                    const groupGrid = document.createElement('div');
                    groupGrid.style.display = 'grid';
                    groupGrid.style.gridTemplateColumns = 'repeat(16, 1fr)'; // 每行 16 個，適合半寬顯示
                    groupGrid.style.gap = '3px'; // 稍微增加間距
                    groupGrid.style.marginBottom = '8px';
                    
                    // 建立該群組的每個指示器
                    for (const item of group.items) {
                        const { key, value } = item;
                        const indicator = document.createElement('div');
                        indicator.style.width = '100%';
                        indicator.style.paddingTop = '100%'; // 正方形
                        indicator.style.position = 'relative';
                        indicator.style.borderRadius = '3px';
                        indicator.style.cursor = 'pointer';
                        indicator.setAttribute('data-key', key);
                        indicator.style.transition = 'all 0.2s ease'; // 添加過渡效果
                    
                        // 根據值設定顏色 (處理各種可能的值類型)
                        const isOn = value === 1 || value === '1' || value === true || value === 'true' || value > 0;
                        
                        // 特殊處理警報狀態的顏色
                        if (category === '警報狀態') {
                            if (isOn) {
                                indicator.style.backgroundColor = '#ff3860'; // 紅色 - 警報觸發
                                indicator.style.boxShadow = '0 0 6px rgba(255, 56, 96, 0.6)';
                                indicator.style.border = '1px solid #ff1443';
                            } else {
                                indicator.style.backgroundColor = '#2b2b2b'; // 深灰色 - 正常
                                indicator.style.border = '1px solid #3a3a3a';
                            }
                        } else {
                            // IO 狀態顏色
                            if (isOn) {
                                indicator.style.backgroundColor = '#48c774'; // 綠色 - ON
                                indicator.style.boxShadow = '0 0 4px rgba(72, 199, 116, 0.5)';
                                indicator.style.border = '1px solid #3abb67';
                            } else {
                                indicator.style.backgroundColor = '#2b2b2b'; // 更深的灰色 - OFF
                                indicator.style.border = '1px solid #3a3a3a';
                            }
                        }
                        
                        // 內部標籤
                        const label = document.createElement('div');
                        label.style.position = 'absolute';
                        label.style.top = '50%';
                        label.style.left = '50%';
                        label.style.transform = 'translate(-50%, -50%)';
                        label.style.fontSize = '0.75rem'; // 使用 rem 單位，約 12px
                        label.style.fontWeight = 'bold';
                        label.style.color = isOn ? 'white' : '#aaa'; // OFF 時用更亮的灰色
                        label.style.textShadow = isOn ? '0 1px 2px rgba(0,0,0,0.5)' : 'none'; // 添加文字陰影
                        
                        // 提取數字 - 只顯示子編號
                        let displayText = '';
                        if (key.includes('AGV_INPUT_') || key.includes('AGV_OUTPUT_')) {
                            // AGV_INPUT_1_1 -> 1 (只顯示最後的數字)
                            const parts = key.split('_');
                            if (parts.length >= 4) {
                                displayText = parts[3];
                            }
                        } else if (key.includes('ALARM_STATUS_')) {
                            // ALARM_STATUS_1 -> 1
                            displayText = key.split('_')[2] || key;
                        } else if (key.includes('IN_') || key.includes('OUT_')) {
                            // IN_1 -> 1
                            displayText = key.split('_')[1] || key;
                        } else {
                            // 其他格式
                            displayText = key.match(/\d+/)?.[0] || key;
                        }
                        label.textContent = displayText;
                        
                        // Tooltip
                        indicator.title = `${key}: ${value}`;
                        
                        indicator.appendChild(label);
                        groupGrid.appendChild(indicator);
                        
                        // 儲存參考
                        domCache.categories[category].indicators[key] = indicator;
                    }
                    
                    // 將群組網格加入主容器
                    gridContainer.appendChild(groupGrid);
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
                        // 判斷是否為開啟狀態
                        const isOn = value === 1 || value === '1' || value === true || value === 'true' || value > 0;
                        
                        // 根據類別設定不同的顏色
                        if (category === '警報狀態') {
                            // 警報狀態特殊處理
                            if (isOn) {
                                indicator.style.backgroundColor = '#ff3860'; // 紅色 - 警報觸發
                                indicator.style.boxShadow = '0 0 6px rgba(255, 56, 96, 0.6)';
                                indicator.style.border = '1px solid #ff1443';
                                indicator.querySelector('div').style.color = 'white';
                            } else {
                                indicator.style.backgroundColor = '#2b2b2b'; // 深灰色 - 正常
                                indicator.style.border = '1px solid #3a3a3a';
                                indicator.style.boxShadow = '';
                                indicator.querySelector('div').style.color = '#aaa';
                            }
                        } else {
                            // IO 狀態顏色
                            if (isOn) {
                                indicator.style.backgroundColor = '#48c774'; // 綠色 - ON
                                indicator.style.boxShadow = '0 0 4px rgba(72, 199, 116, 0.5)';
                                indicator.style.border = '1px solid #3abb67';
                                indicator.querySelector('div').style.color = 'white';
                            } else {
                                indicator.style.backgroundColor = '#2b2b2b'; // 深灰色 - OFF
                                indicator.style.border = '1px solid #3a3a3a';
                                indicator.style.boxShadow = '';
                                indicator.querySelector('div').style.color = '#aaa';
                            }
                        }
                        // 更新 tooltip
                        indicator.title = `${key}: ${value}`;
                    }
                }
            } else if (cached.type === 'doors') {
                // 更新門控狀態
                for (const [key, value] of Object.entries(items)) {
                    const match = key.match(/DOOR.*?(\d+)/);
                    if (match) {
                        const doorNum = match[1];
                        const doorElement = cached.doorElements[doorNum];
                        
                        if (doorElement) {
                            if (key.includes('OPEN') && doorElement.open) {
                                // 更新開狀態
                                if (value === true || value === 1 || value === '1') {
                                    doorElement.open.style.backgroundColor = '#48c774';
                                    doorElement.open.style.color = 'white';
                                    doorElement.open.style.boxShadow = '0 0 4px rgba(72, 199, 116, 0.5)';
                                    doorElement.open.style.border = 'none';
                                } else {
                                    doorElement.open.style.backgroundColor = '#2b2b2b';
                                    doorElement.open.style.color = '#aaa';
                                    doorElement.open.style.boxShadow = 'none';
                                    doorElement.open.style.border = '1px solid #3a3a3a';
                                }
                                doorElement.open.title = `${key}: ${value}`;
                            } else if (key.includes('CLOSE') && doorElement.close) {
                                // 更新關狀態
                                if (value === true || value === 1 || value === '1') {
                                    doorElement.close.style.backgroundColor = '#ff3860';
                                    doorElement.close.style.color = 'white';
                                    doorElement.close.style.boxShadow = '0 0 4px rgba(255, 56, 96, 0.5)';
                                    doorElement.close.style.border = 'none';
                                } else {
                                    doorElement.close.style.backgroundColor = '#2b2b2b';
                                    doorElement.close.style.color = '#aaa';
                                    doorElement.close.style.boxShadow = 'none';
                                    doorElement.close.style.border = '1px solid #3a3a3a';
                                }
                                doorElement.close.title = `${key}: ${value}`;
                            } else if (doorElement.status) {
                                // 更新狀態文字
                                if (value === 'OPEN' || value === 1 || value === '1') {
                                    doorElement.status.style.backgroundColor = '#48c774';
                                    doorElement.status.style.color = 'white';
                                    doorElement.status.textContent = '開';
                                } else if (value === 'CLOSED' || value === 0 || value === '0') {
                                    doorElement.status.style.backgroundColor = '#2b2b2b';
                                    doorElement.status.style.color = '#aaa';
                                    doorElement.status.textContent = '關';
                                } else if (value === 'MOVING' || value === 2 || value === '2') {
                                    doorElement.status.style.backgroundColor = '#ffdd57';
                                    doorElement.status.style.color = '#363636';
                                    doorElement.status.textContent = '移動中';
                                } else {
                                    doorElement.status.style.backgroundColor = '#3a3a3a';
                                    doorElement.status.style.color = '#ccc';
                                    doorElement.status.textContent = String(value);
                                }
                            }
                        }
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
            // 嘗試多種可能的欄位名稱
            const x = status.AGV_SLAM_X || status.X_DIST || status.x || 0;
            const y = status.AGV_SLAM_Y || status.Y_DIST || status.y || 0;
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
            '運動狀態': {},
            '狀態機': {},
            '控制狀態': {},
            '門控狀態': {},
            '輸入狀態': {},
            '輸出狀態': {},
            '警報狀態': {},
            '任務資訊': {},
            'PLC記憶體': {},
            '其他': {}
        };
        
        for (const [key, value] of Object.entries(status)) {
            // 基本資訊
            if (key.includes('agv_id') || key.includes('AGV_ID') || key.includes('MAGIC') || 
                key.includes('timestamp') || key.includes('namespace') || key.includes('version') ||
                key.includes('agv_type') || key.includes('node_name')) {
                categories['基本資訊'][key] = value;
            } 
            // 位置狀態
            else if (key.includes('SLAM') || key.includes('PGV') || key.includes('POINT') || 
                     key.includes('ZONE') || key.includes('X_DIST') || key.includes('Y_DIST') ||
                     key.includes('THETA')) {
                categories['位置狀態'][key] = value;
            } 
            // 運動狀態
            else if (key.includes('SPEED') || key.includes('POWER') || key.includes('MOVING')) {
                categories['運動狀態'][key] = value;
            } 
            // 狀態機相關
            else if (key.includes('context') || key.includes('current_state') || key.includes('_state')) {
                categories['狀態機'][key] = value;
            }
            // 控制狀態
            else if (key.includes('AGV_Auto') || key.includes('AGV_MANUAL') || key.includes('AGV_IDLE') ||
                     key.includes('AGV_ALARM') || key.includes('TURN')) {
                categories['控制狀態'][key] = value;
            } 
            // 門控狀態
            else if (key.includes('DOOR')) {
                categories['門控狀態'][key] = value;
            } 
            // 輸入狀態 (支援多種格式: Input1, AGV_INPUT_1_1, IN_1, DI_01 等)
            else if (key.includes('INPUT') || key.includes('Input') || (key.startsWith('IN_') && !key.includes('MISSION')) || key.startsWith('DI_')) {
                categories['輸入狀態'][key] = value;
            } 
            // 輸出狀態 (支援多種格式: Output1, AGV_OUTPUT_1_1, DO_01 等)
            else if (key.includes('OUTPUT') || key.includes('Output') || key.startsWith('DO_')) {
                categories['輸出狀態'][key] = value;
            } 
            // 警報狀態
            else if (key.includes('Alarm') || key.includes('ALARM')) {
                categories['警報狀態'][key] = value;
            } 
            // 任務相關
            else if (key.includes('work_id') || key.includes('task') || key.includes('action') ||
                     key.includes('equipment') || key.includes('port')) {
                categories['任務資訊'][key] = value;
            }
            // PLC 記憶體
            else if (key.includes('PLC') || key.includes('MR') || key.includes('DM')) {
                categories['PLC記憶體'][key] = value;
            } 
            // 其他
            else {
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