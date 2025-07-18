// dashboardPage.js - Dashboard 頁面 JavaScript 模組
// 實作 DOM 優化方法論和 miniStore 整合

import { agvsStore, signalsStore, racksStore, tasksStore, carriersStore, roomsStore } from '../store/index.js';
import {
    getTaskStatusStatistics,
    getTaskStatusInfo,
    isActiveStatus,
    isPendingStatus,
    isExecutingStatus
} from './taskStatus.js';

export const dashboardPage = (() => {
    // Dashboard 資料快取
    let dashboardData = {
        task: { total: 0, pending: 0, executing: 0, todayCompleted: 0, failed: 0 },
        rooms: {}, // 動態房間資料：{ roomId: { carriersInProcess: 0, ... } }
        equipment: { agvs: [], racks: [] }, // 設備資料
        system: { status: 'unknown', lastUpdate: null }
    };

    // 啟用的房間列表（從 DOM 中動態獲取）
    let enabledRooms = [];

    /**
     * 頁面初始化
     */
    function setup() {
        console.log('🚀 Dashboard 頁面初始化');

        // 初始化卡片結構
        initializeDashboardCards();

        // 設置 Store 監聽器
        setupStoreListeners();

        // 初始化展開/收合功能
        setupExpandCollapse();

        // 初始化載入狀態
        showLoadingState();

        console.log('✅ Dashboard 頁面初始化完成');
    }

    /**
     * 初始化 Dashboard 卡片結構
     */
    function initializeDashboardCards() {
        // 動態獲取啟用的房間
        enabledRooms = [];
        const roomCards = document.querySelectorAll('[id^="dashboard-card-room"]');
        roomCards.forEach(card => {
            const roomId = parseInt(card.id.replace('dashboard-card-room', ''));
            if (!isNaN(roomId)) {
                enabledRooms.push(roomId);
                dashboardData.rooms[roomId] = {
                    carriersInProcess: 0,
                    processStatus: '待機中',
                    entranceHasRack: false,
                    exitHasRack: false
                };
            }
        });

        // 設置初始載入狀態
        const cardIds = ['dashboard-card-task', 'dashboard-card-equipment', 'dashboard-card-rooms'];
        const allCardIds = [...cardIds];

        allCardIds.forEach(cardId => {
            const card = document.getElementById(cardId);
            if (card) {
                card.dataset.currentData = '';
            }
        });

        // 確保個別卡片容器存在
        const agvContainer = document.getElementById('dashboard-agv-cards-container');
        const rackContainer = document.getElementById('dashboard-rack-cards-container');

        if (!agvContainer) {
            console.warn('AGV 卡片容器未找到');
        }
        if (!rackContainer) {
            console.warn('貨架卡片容器未找到');
        }

        console.log('🏠 發現啟用的房間:', enabledRooms);
    }

    /**
     * 設置 Store 監聽器
     */
    function setupStoreListeners() {
        // AGV Store 監聽
        agvsStore.on('change', handleAgvsChange);

        // Rack Store 監聽
        racksStore.on('change', handleRacksChange);

        // Task Store 監聽
        tasksStore.on('change', handleTasksChange);

        // Carrier Store 監聽
        carriersStore.on('change', handleCarriersChange);

        // Room Store 監聽
        roomsStore.on('change', handleRoomsChange);

        console.log('📡 Dashboard Store 監聽器已設置');
    }

    /**
     * 處理 AGV 資料變化
     * @param {Object} newState - 新的 AGV 狀態
     */
    function handleAgvsChange(newState) {
        if (!newState?.agvs) return;

        const agvs = newState.agvs || [];
        console.debug('Dashboard 收到 AGV 更新:', agvs.length, '個 AGV');

        // 更新設備狀態卡片中的 AGV 列表
        updateEquipmentCard(agvs, null);

        // 更新系統狀態
        updateSystemStatus();
    }



    /**
     * 處理 Rack 資料變化
     * @param {Object} newState - 新的 Rack 狀態
     */
    function handleRacksChange(newState) {
        if (!newState?.racks) return;

        const racks = newState.racks || [];
        console.debug('Dashboard 收到 Rack 更新:', racks.length, '個貨架');

        // 更新設備狀態卡片中的 Rack 列表
        updateEquipmentCard(null, racks);

        // 更新房間狀態卡片中的貨架狀態
        updateRoomsCardRackStatus(racks);
    }

    /**
     * 處理 Task 資料變化
     * @param {Object} newState - 新的 Task 狀態
     */
    function handleTasksChange(newState) {
        if (!newState?.tasks) return;

        const tasks = newState.tasks || [];
        console.debug('Dashboard 收到 Task 更新:', tasks.length, '個任務');

        // 處理任務資料
        const taskData = processTaskData(tasks);

        // 更新 Task 卡片
        updateTaskStatusCard(taskData);
    }

    /**
     * 處理 Carrier 資料變化
     * @param {Object} newState - 新的 Carrier 狀態
     */
    function handleCarriersChange(newState) {
        if (!newState?.carriers) return;

        const carriers = newState.carriers || [];
        console.debug('Dashboard 收到 Carrier 更新:', carriers.length, '個載具');

        // 更新房間狀態卡片
        updateRoomsCard(carriers);
    }

    /**
     * 處理 Room 資料變化
     * @param {Object} newState - 新的 Room 狀態
     */
    function handleRoomsChange(newState) {
        if (!newState?.rooms) return;

        const rooms = newState.rooms || [];
        console.debug('Dashboard 收到 Room 更新:', rooms.length, '個房間');

        // 更新房間狀態卡片中的基本資訊
        updateRoomsCardBasicInfo(rooms);
    }

    /**
     * 更新設備狀態卡片
     * @param {Array|null} agvs - AGV 列表（null 表示不更新）
     * @param {Array|null} racks - Rack 列表（null 表示不更新）
     */
    function updateEquipmentCard(agvs, racks) {
        // 更新資料快取
        if (agvs !== null) {
            dashboardData.equipment.agvs = agvs;
        }
        if (racks !== null) {
            dashboardData.equipment.racks = racks;
        }

        const cardId = 'dashboard-card-equipment';
        const equipmentData = {
            agvs: dashboardData.equipment.agvs,
            racks: dashboardData.equipment.racks,
            totalCount: dashboardData.equipment.agvs.length + dashboardData.equipment.racks.length
        };

        if (!hasCardChanged(cardId, equipmentData)) return;

        // 更新狀態指示器
        let statusType = 'info';
        let statusText = '設備正常';

        const onlineAgvs = dashboardData.equipment.agvs.filter(agv => determineAgvOnlineStatus(agv));
        const totalAgvs = dashboardData.equipment.agvs.length;
        const totalRacks = dashboardData.equipment.racks.length;

        if (onlineAgvs.length < totalAgvs) {
            statusType = 'warning';
            statusText = `${onlineAgvs.length}/${totalAgvs} AGV 線上，${totalRacks} 個貨架`;
        } else {
            statusType = 'success';
            statusText = `${totalAgvs} AGV 全部線上，${totalRacks} 個貨架`;
        }

        updateStatusIndicator('dashboard-status-equipment', statusType, statusText);

        // 更新摘要內容
        updateEquipmentSummary(totalAgvs, onlineAgvs.length, totalRacks);

        // 更新設備列表（詳細內容）
        updateEquipmentLists(dashboardData.equipment.agvs, dashboardData.equipment.racks);

        // 更新卡片資料屬性
        const card = document.getElementById(cardId);
        if (card) {
            card.dataset.currentData = JSON.stringify(equipmentData);
            addUpdateAnimation(card);
        }

        hideLoadingState();
        console.debug('設備狀態卡片已更新:', equipmentData);
    }

    /**
     * 更新設備摘要內容
     * @param {number} totalAgvs - AGV總數
     * @param {number} onlineAgvs - 線上AGV數量
     * @param {number} totalRacks - 貨架總數
     */
    function updateEquipmentSummary(totalAgvs, onlineAgvs, totalRacks) {
        // 更新摘要指標
        updateElement('equipment-summary-agv-total', totalAgvs);
        updateElement('equipment-summary-agv-online', onlineAgvs);
        updateElement('equipment-summary-rack-total', totalRacks);

        // 更新摘要狀態
        const summaryStatusElement = document.getElementById('equipment-summary-status');
        if (summaryStatusElement) {
            const statusDot = summaryStatusElement.querySelector('.dashboard-summary-status-dot');
            const statusText = summaryStatusElement.querySelector('span');

            if (onlineAgvs < totalAgvs) {
                statusDot.style.backgroundColor = '#ff9800';
                statusText.textContent = `${onlineAgvs}/${totalAgvs} 線上`;
            } else if (totalAgvs > 0) {
                statusDot.style.backgroundColor = '#4caf50';
                statusText.textContent = '全部線上';
            } else {
                statusDot.style.backgroundColor = '#dbdbdb';
                statusText.textContent = '無設備';
            }
        }
    }

    /**
     * 更新設備列表
     * @param {Array} agvs - AGV 列表
     * @param {Array} racks - Rack 列表
     */
    function updateEquipmentLists(agvs, racks) {
        const agvList = document.getElementById('dashboard-agv-list');
        const rackList = document.getElementById('dashboard-rack-list');
        const equipmentContainer = document.getElementById('dashboard-equipment-container');
        const emptyState = document.getElementById('dashboard-equipment-empty');

        if (!agvList || !rackList || !equipmentContainer || !emptyState) {
            console.warn('設備列表元素未找到');
            return;
        }

        // 清空現有內容
        agvList.innerHTML = '';
        rackList.innerHTML = '';

        const totalEquipment = agvs.length + racks.length;

        if (totalEquipment === 0) {
            // 顯示空狀態
            equipmentContainer.style.display = 'none';
            emptyState.style.display = 'block';
            return;
        }

        // 隱藏空狀態，顯示設備列表
        emptyState.style.display = 'none';
        equipmentContainer.style.display = 'block';

        // 更新 AGV 列表
        const displayAgvs = agvs.slice(0, 10); // 限制顯示數量
        displayAgvs.forEach(agv => {
            const agvItem = createAgvListItem(agv);
            agvList.appendChild(agvItem);
        });

        // 更新 Rack 列表
        const displayRacks = racks.slice(0, 10); // 限制顯示數量
        displayRacks.forEach(rack => {
            const rackItem = createRackListItem(rack);
            rackList.appendChild(rackItem);
        });

        console.debug('設備列表已更新:', displayAgvs.length, '個 AGV,', displayRacks.length, '個貨架');
    }

    /**
     * 創建 AGV 列表項目
     * @param {Object} agv - AGV 資料
     * @returns {HTMLElement} AGV 列表項目元素
     */
    function createAgvListItem(agv) {
        const item = document.createElement('div');
        item.className = 'equipment-item';

        // 判斷 AGV 線上狀態
        const isOnline = determineAgvOnlineStatus(agv);
        const batteryLevel = agv.battery || 0;
        const isLowBattery = batteryLevel < 20;

        // 狀態樣式
        const statusClass = isOnline ? 'status-success' : 'status-danger';
        const statusText = isOnline ? '線上' : '離線';
        const batteryClass = isLowBattery ? 'status-warning' : 'status-success';

        item.innerHTML = `
            <div class="equipment-item-info">
                <div class="equipment-item-name">${agv.name || `AGV ${agv.id}`}</div>
                <div class="equipment-item-details">
                    <span>位置: (${(agv.x || 0).toFixed(1)}, ${(agv.y || 0).toFixed(1)})</span>
                    <span>型號: ${agv.model || 'N/A'}</span>
                </div>
            </div>
            <div class="equipment-item-status">
                <span class="equipment-status-badge ${statusClass}">${statusText}</span>
                <span class="equipment-status-badge ${batteryClass}">${batteryLevel}%</span>
            </div>
        `;

        return item;
    }

    /**
     * 創建 Rack 列表項目
     * @param {Object} rack - Rack 資料
     * @returns {HTMLElement} Rack 列表項目元素
     */
    function createRackListItem(rack) {
        const item = document.createElement('div');
        item.className = 'equipment-item';

        // 貨架狀態判斷
        const isInMap = rack.is_in_map || false;
        const isCarry = rack.is_carry || false;
        const carrierCount = rack.count || 0;
        const hasCarriers = carrierCount > 0;

        // 狀態樣式
        const mapStatusClass = isInMap ? 'status-success' : 'status-warning';
        const mapStatusText = isInMap ? '在地圖中' : '不在地圖';
        const carryStatusClass = isCarry ? 'status-info' : 'status-success';
        const carryStatusText = isCarry ? '搬運中' : '靜止';

        item.innerHTML = `
            <div class="equipment-item-info">
                <div class="equipment-item-name">${rack.name || `貨架 ${rack.id}`}</div>
                <div class="equipment-item-details">
                    <span>位置: ${rack.location_id || 'N/A'}</span>
                    <span>產品: ${rack.product_id || 'N/A'}</span>
                    <span>載具: ${carrierCount} 個</span>
                </div>
            </div>
            <div class="equipment-item-status">
                <span class="equipment-status-badge ${mapStatusClass}">${mapStatusText}</span>
                <span class="equipment-status-badge ${carryStatusClass}">${carryStatusText}</span>
                ${hasCarriers ? `<span class="equipment-status-badge status-info">${carrierCount} 載具</span>` : ''}
            </div>
        `;

        return item;
    }

    /**
     * 創建個別 AGV 卡片（保留用於向後相容）
     * @param {Object} agv - AGV 資料
     * @returns {HTMLElement} AGV 卡片元素
     */
    function createIndividualAgvCard(agv) {
        const card = document.createElement('div');
        card.className = 'dashboard-card dashboard-agv-individual-card';
        card.id = `dashboard-card-agv-${agv.id}`;

        // 判斷 AGV 線上狀態
        const isOnline = determineAgvOnlineStatus(agv);
        const batteryLevel = agv.battery || 0;
        const isLowBattery = batteryLevel < 20;

        // 狀態樣式
        const statusClass = isOnline ? 'status-success' : 'status-danger';
        const statusText = isOnline ? '線上' : '離線';
        const batteryClass = isLowBattery ? 'status-warning' : 'status-success';

        card.innerHTML = `
            <div class="dashboard-card-header">
                <div class="dashboard-card-title">
                    <span class="dashboard-card-icon">
                        <i class="mdi mdi-robot"></i>
                    </span>
                    ${agv.name || `AGV ${agv.id}`}
                </div>
            </div>

            <div class="dashboard-agv-details">
                <div class="dashboard-agv-detail-item">
                    <span class="dashboard-agv-detail-label">狀態:</span>
                    <span class="agv-status-badge ${statusClass}">${statusText}</span>
                </div>

                <div class="dashboard-agv-detail-item">
                    <span class="dashboard-agv-detail-label">電量:</span>
                    <span class="agv-battery-badge ${batteryClass}">${batteryLevel}%</span>
                </div>

                <div class="dashboard-agv-detail-item">
                    <span class="dashboard-agv-detail-label">型號:</span>
                    <span class="dashboard-agv-detail-value">${agv.model || '未知'}</span>
                </div>

                <div class="dashboard-agv-detail-item">
                    <span class="dashboard-agv-detail-label">位置:</span>
                    <span class="dashboard-agv-detail-value">
                        ${agv.x !== null && agv.y !== null ? `(${Math.round(agv.x)}, ${Math.round(agv.y)})` : '未知'}
                    </span>
                </div>

                <div class="dashboard-agv-detail-item">
                    <span class="dashboard-agv-detail-label">方向:</span>
                    <span class="dashboard-agv-detail-value">${Math.round(agv.heading || 0)}°</span>
                </div>
            </div>
        `;

        return card;
    }

    /**
     * 判斷 AGV 線上狀態
     * @param {Object} agv - AGV 資料
     * @returns {boolean} 是否線上
     */
    function determineAgvOnlineStatus(agv) {
        // 1. 首先檢查 enable 狀態（基本啟用狀態）
        if (agv.enable !== 1) return false;

        // 2. 檢查是否有位置資訊（表示有通訊）
        const hasPosition = (agv.x !== null && agv.x !== undefined) &&
            (agv.y !== null && agv.y !== undefined);

        // 3. 檢查是否有電量資訊（表示系統正常）
        const hasBattery = agv.battery !== null && agv.battery !== undefined;

        // 4. 檢查狀態ID（如果有的話）
        const hasValidStatus = !agv.status_id || agv.status_id > 0;

        // 綜合判斷：啟用 + (有位置 或 有電量) + 狀態正常
        return hasValidStatus && (hasPosition || hasBattery);
    }





    /**
     * 創建個別貨架卡片
     * @param {Object} rack - 貨架資料
     * @returns {HTMLElement} 貨架卡片元素
     */
    function createIndividualRackCard(rack) {
        const card = document.createElement('div');
        card.className = 'dashboard-card dashboard-rack-individual-card';
        card.id = `dashboard-card-rack-${rack.id}`;

        // 貨架狀態判斷
        const isInMap = rack.is_in_map || false;
        const isCarry = rack.is_carry || false;
        const carrierCount = rack.count || 0;
        const hasCarriers = carrierCount > 0;

        // 狀態樣式
        const mapStatusClass = isInMap ? 'status-success' : 'status-warning';
        const mapStatusText = isInMap ? '在地圖中' : '不在地圖';
        const carryStatusClass = isCarry ? 'status-info' : 'status-success';
        const carryStatusText = isCarry ? '搬運中' : '靜止';
        const carrierStatusClass = hasCarriers ? 'status-success' : 'status-light';

        card.innerHTML = `
            <div class="dashboard-card-header">
                <div class="dashboard-card-title">
                    <span class="dashboard-card-icon">
                        <i class="mdi mdi-archive"></i>
                    </span>
                    貨架 ${rack.name || rack.id}
                </div>
            </div>

            <div class="dashboard-rack-details">
                <div class="dashboard-rack-detail-item">
                    <span class="dashboard-rack-detail-label">載具數量:</span>
                    <span class="rack-carrier-badge ${carrierStatusClass}">${carrierCount} 個</span>
                </div>

                <div class="dashboard-rack-detail-item">
                    <span class="dashboard-rack-detail-label">產品:</span>
                    <span class="dashboard-rack-detail-value">${rack.product_name || '未設定'}</span>
                </div>

                <div class="dashboard-rack-detail-item">
                    <span class="dashboard-rack-detail-label">地圖狀態:</span>
                    <span class="rack-status-badge ${mapStatusClass}">${mapStatusText}</span>
                </div>

                <div class="dashboard-rack-detail-item">
                    <span class="dashboard-rack-detail-label">搬運狀態:</span>
                    <span class="rack-status-badge ${carryStatusClass}">${carryStatusText}</span>
                </div>

                <div class="dashboard-rack-detail-item">
                    <span class="dashboard-rack-detail-label">方向:</span>
                    <span class="dashboard-rack-detail-value">${rack.direction || 0}°</span>
                </div>

                <div class="dashboard-rack-detail-item">
                    <span class="dashboard-rack-detail-label">位置:</span>
                    <span class="dashboard-rack-detail-value">
                        ${rack.location_name || '未知位置'}
                    </span>
                </div>
            </div>
        `;

        return card;
    }

    /**
     * 篩選和處理任務資料
     * @param {Array} tasks - Task 列表
     * @returns {Object} 處理後的任務資料
     */
    function processTaskData(tasks) {
        // 使用新的狀態統計函數
        const stats = getTaskStatusStatistics(tasks);

        // 只保留活躍狀態的任務（請求中、待處理、待執行、執行中）
        const activeTasks = tasks.filter(task => {
            const status = task.status_id || task.status;
            return isActiveStatus(status);
        });

        // 按狀態和時間排序（執行中優先，然後按開始時間）
        activeTasks.sort((a, b) => {
            const statusA = a.status_id || a.status;
            const statusB = b.status_id || b.status;

            // 執行中的任務優先 (status_id = 3)
            if (isExecutingStatus(statusA) && !isExecutingStatus(statusB)) return -1;
            if (!isExecutingStatus(statusA) && isExecutingStatus(statusB)) return 1;

            // 相同狀態按時間排序（最新的在前）
            const timeA = new Date(a.created_at || a.updated_at || 0);
            const timeB = new Date(b.created_at || b.updated_at || 0);
            return timeB - timeA;
        });

        // 限制顯示數量（最多10個）
        const limitedTasks = activeTasks.slice(0, 10);

        // 統計數量 - 根據新的狀態定義
        // 待執行：狀態 1(待處理) + 2(待執行)
        // 執行中：狀態 3(執行中)
        const pendingCount = stats.pending + stats.ready; // 1 + 2
        const runningCount = stats.executing; // 3

        return {
            activeTasks: limitedTasks,
            totalActive: stats.active,
            totalCount: stats.total,
            pendingCount,
            runningCount,
            completedCount: stats.completed,
            errorCount: stats.error,
            cancelledCount: stats.cancelled
        };
    }

    /**
     * 更新房間狀態卡片
     * @param {Array} carriers - 載具列表
     */
    function updateRoomsCard(carriers) {
        const cardId = 'dashboard-card-rooms';

        // 計算所有房間的統計資料
        const roomsData = {};
        let totalCarriers = 0;
        let activeRooms = 0;

        enabledRooms.forEach(roomId => {
            const roomStats = calculateRoomCarrierStats(carriers, roomId);
            roomsData[roomId] = roomStats;
            totalCarriers += roomStats.carriersInProcess;
            if (roomStats.carriersInProcess > 0) {
                activeRooms++;
            }
        });

        const overallData = {
            totalCarriers,
            activeRooms,
            totalRooms: enabledRooms.length,
            roomsData
        };

        if (!hasCardChanged(cardId, overallData)) return;

        // 更新整體狀態指示器
        let statusType = 'info';
        let statusText = '所有房間待機中';

        if (activeRooms > 0) {
            statusType = 'warning';
            statusText = `${activeRooms}/${enabledRooms.length} 個房間處理中，共 ${totalCarriers} 個載具`;
        }

        updateStatusIndicator('dashboard-status-rooms', statusType, statusText);

        // 更新摘要內容
        updateRoomsSummary(roomsData);

        // 更新各房間的載具統計（詳細內容）
        enabledRooms.forEach(roomId => {
            const roomStats = roomsData[roomId];
            updateRoomSection(roomId, roomStats);
        });

        // 更新卡片資料屬性
        const card = document.getElementById(cardId);
        if (card) {
            card.dataset.currentData = JSON.stringify(overallData);
            addUpdateAnimation(card);
        }

        hideLoadingState();
        console.debug('房間狀態卡片已更新:', overallData);
    }

    /**
     * 更新房間摘要內容
     * @param {Object} roomsData - 房間資料
     */
    function updateRoomsSummary(roomsData) {
        // 更新各房間的摘要指標
        enabledRooms.forEach(roomId => {
            const roomStats = roomsData[roomId];
            const carriersCount = roomStats ? roomStats.carriersInProcess : 0;
            updateElement(`rooms-summary-${roomId}-carriers`, carriersCount);
        });

        // 更新摘要狀態
        const summaryStatusElement = document.getElementById('rooms-summary-status');
        if (summaryStatusElement) {
            const statusDot = summaryStatusElement.querySelector('.dashboard-summary-status-dot');
            const statusText = summaryStatusElement.querySelector('span');

            const totalCarriers = Object.values(roomsData).reduce((sum, room) => sum + (room.carriersInProcess || 0), 0);
            const activeRooms = Object.values(roomsData).filter(room => room.carriersInProcess > 0).length;

            if (activeRooms > 0) {
                statusDot.style.backgroundColor = '#ff9800';
                statusText.textContent = `${activeRooms} 房間處理中`;
            } else {
                statusDot.style.backgroundColor = '#4caf50';
                statusText.textContent = '全部待機';
            }
        }
    }

    /**
     * 計算房間載具統計
     * @param {Array} carriers - 載具列表
     * @param {number} roomId - 房間 ID
     * @returns {Object} 房間統計資料
     */
    function calculateRoomCarrierStats(carriers, roomId) {
        // 計算房間內處理中的載具數量
        // 根據 location_id 判斷房間：roomId*100 到 (roomId+1)*100-1
        const roomLocationRange = [roomId * 100, (roomId + 1) * 100 - 1];

        const roomCarriers = carriers.filter(carrier => {
            // 如果載具在設備端口，檢查設備的 location_id
            if (carrier.port_id && carrier.location_id) {
                const locationId = carrier.location_id;
                return locationId >= roomLocationRange[0] && locationId <= roomLocationRange[1];
            }
            // 如果載具有 room_id，直接檢查
            return carrier.room_id === roomId;
        });

        return {
            carriersInProcess: roomCarriers.length
        };
    }

    /**
     * 更新房間區塊
     * @param {number} roomId - 房間 ID
     * @param {Object} roomStats - 房間統計資料
     */
    function updateRoomSection(roomId, roomStats) {
        // 更新載具數量
        const carriersCountElement = document.getElementById(`room-carriers-count-${roomId}`);
        if (carriersCountElement) {
            carriersCountElement.textContent = `${roomStats.carriersInProcess} 個載具`;
        }

        // 更新狀態徽章
        const statusBadgeElement = document.getElementById(`room-status-badge-${roomId}`);
        if (statusBadgeElement) {
            let statusClass = '';
            let statusText = '待機中';

            if (roomStats.carriersInProcess > 0) {
                statusClass = 'status-warning';
                statusText = '處理中';
            } else {
                statusClass = '';
                statusText = '待機中';
            }

            statusBadgeElement.className = `room-status-badge ${statusClass}`;
            statusBadgeElement.textContent = statusText;
        }

        // 更新製程狀態
        const processStatusElement = document.getElementById(`room-process-status-${roomId}`);
        if (processStatusElement) {
            const processStatus = roomStats.carriersInProcess > 0 ? '處理中' : '待機中';
            processStatusElement.textContent = processStatus;
        }
    }

    /**
     * 更新房間狀態卡片中的基本資訊
     * @param {Array} rooms - 房間列表
     */
    function updateRoomsCardBasicInfo(rooms) {
        enabledRooms.forEach(roomId => {
            const room = rooms.find(r => r.id === roomId);
            if (!room) return;

            // 更新房間的製程描述
            const processDescElement = document.getElementById(`room-process-desc-${roomId}`);
            if (processDescElement && room.process_description) {
                processDescElement.textContent = room.process_description;
            }

            // 更新泡藥次數
            const soakingTimesElement = document.getElementById(`room-soaking-times-${roomId}`);
            if (soakingTimesElement && room.soaking_times) {
                soakingTimesElement.textContent = room.soaking_times;
            }
        });

        console.debug('房間基本資訊已更新:', rooms.length, '個房間');
    }

    /**
     * 更新房間狀態卡片中的貨架狀態
     * @param {Array} racks - 貨架列表
     */
    function updateRoomsCardRackStatus(racks) {
        enabledRooms.forEach(roomId => {
            // 定義房間的入口和出口位置ID（精確匹配）
            // 編碼格式：{房間ID}{位置類型}00
            // 位置類型：01 = 入口，02 = 出口
            const entranceLocationId = roomId * 10000 + 100; // 例：房間2入口 = 20100
            const exitLocationId = roomId * 10000 + 200;     // 例：房間2出口 = 20200

            // 檢查入口是否有貨架（精確匹配）
            const entranceRacks = racks.filter(rack =>
                rack.location_id === entranceLocationId && rack.count > 0
            );

            // 檢查出口是否有貨架（精確匹配）
            const exitRacks = racks.filter(rack =>
                rack.location_id === exitLocationId && rack.count > 0
            );

            // 更新入口狀態
            const entranceElement = document.getElementById(`room-entrance-${roomId}`);
            if (entranceElement) {
                const entranceStatus = entranceRacks.length > 0
                    ? `貨架 ${entranceRacks[0].name || entranceRacks[0].id}`
                    : '無貨架';
                const entranceClass = entranceRacks.length > 0 ? 'status-warning' : '';

                entranceElement.textContent = entranceStatus;
                entranceElement.className = `logistics-status ${entranceClass}`;
            }

            // 更新出口狀態
            const exitElement = document.getElementById(`room-exit-${roomId}`);
            if (exitElement) {
                const exitStatus = exitRacks.length > 0
                    ? `貨架 ${exitRacks[0].name || exitRacks[0].id}`
                    : '無貨架';
                const exitClass = exitRacks.length > 0 ? 'status-success' : '';

                exitElement.textContent = exitStatus;
                exitElement.className = `logistics-status ${exitClass}`;
            }
        });

        console.debug('房間貨架狀態已更新:', enabledRooms.length, '個房間');
    }

    /**
     * 更新元素內容的輔助函數
     * @param {string} elementId - 元素 ID
     * @param {string|number} value - 要設置的值
     */
    function updateElement(elementId, value) {
        const element = document.getElementById(elementId);
        if (element) {
            element.textContent = value;
        }
    }

    /**
     * 精確的變化檢測函數
     * @param {string} cardId - 卡片 ID
     * @param {Object} newData - 新資料
     * @returns {boolean} 是否有變化
     */
    function hasCardChanged(cardId, newData) {
        const element = document.getElementById(cardId);
        if (!element) return true;

        const currentData = element.dataset.currentData;
        const newDataStr = JSON.stringify(newData);

        return currentData !== newDataStr;
    }

    /**
     * 添加更新動畫效果（帶防重疊機制）
     * @param {Element} element - 要添加動畫的元素
     */
    function addUpdateAnimation(element) {
        if (!element) return;

        // 檢查是否已經在播放動畫
        if (element.classList.contains('field-updated')) {
            console.debug('Dashboard 動畫進行中，跳過重複添加');
            return;
        }

        element.classList.add('field-updated');
        setTimeout(() => {
            element.classList.remove('field-updated');
        }, 1000); // 與 CSS 動畫持續時間一致
    }



    /**
     * 顯示載入狀態
     */
    function showLoadingState() {
        // 更新為新的卡片ID
        const cardIds = ['dashboard-card-task', 'dashboard-card-equipment', 'dashboard-card-rooms'];

        cardIds.forEach(cardId => {
            const card = document.getElementById(cardId);
            if (card) {
                const loadingElement = card.querySelector('.dashboard-loading');
                if (loadingElement) {
                    loadingElement.style.display = 'flex';
                    // 確保載入狀態在摘要模式下也可見
                    loadingElement.style.zIndex = '10';
                }

                // 隱藏摘要內容，避免與載入狀態重疊
                const summaryContent = card.querySelector('.dashboard-summary-content');
                if (summaryContent) {
                    summaryContent.style.opacity = '0.3';
                }
            }
        });
    }

    /**
     * 隱藏載入狀態
     */
    function hideLoadingState() {
        // 更新為新的卡片ID
        const cardIds = ['dashboard-card-task', 'dashboard-card-equipment', 'dashboard-card-rooms'];

        cardIds.forEach(cardId => {
            const card = document.getElementById(cardId);
            if (card) {
                const loadingElement = card.querySelector('.dashboard-loading');
                if (loadingElement) {
                    loadingElement.style.display = 'none';
                }

                // 恢復摘要內容的可見性
                const summaryContent = card.querySelector('.dashboard-summary-content');
                if (summaryContent) {
                    summaryContent.style.opacity = '1';
                }
            }
        });
    }







    /**
     * 更新 Task 狀態卡片
     * @param {Object} taskData - 處理後的任務資料
     */
    function updateTaskStatusCard(taskData) {
        const cardId = 'dashboard-card-task';
        if (!hasCardChanged(cardId, taskData)) return;

        // 更新資料快取
        dashboardData.task = taskData;

        // 更新摘要內容
        updateTaskSummary(taskData);

        // 更新狀態指示器
        let statusType = 'info';
        let statusText = '系統正常';

        if (taskData.runningCount > 0) {
            statusType = 'warning';
            statusText = `${taskData.runningCount} 個任務執行中`;
        } else if (taskData.pendingCount > 0) {
            statusType = 'info';
            statusText = `${taskData.pendingCount} 個任務待執行`;
        } else {
            statusType = 'success';
            statusText = '無待處理任務';
        }

        updateStatusIndicator('dashboard-status-task', statusType, statusText);

        // 更新任務階層結構（詳細內容）
        updateTaskHierarchy(taskData.activeTasks);

        // 更新卡片資料屬性
        const card = document.getElementById(cardId);
        if (card) {
            card.dataset.currentData = JSON.stringify(taskData);
            addUpdateAnimation(card);
        }

        hideLoadingState();
        console.debug('Task 狀態卡片已更新:', taskData);
    }

    /**
     * 更新任務摘要內容
     * @param {Object} taskData - 任務資料
     */
    function updateTaskSummary(taskData) {
        // 更新摘要指標
        updateElement('task-summary-total', taskData.totalCount || 0);
        updateElement('task-summary-pending', taskData.pendingCount || 0);
        updateElement('task-summary-executing', taskData.runningCount || 0);

        // 更新摘要狀態
        const summaryStatusElement = document.getElementById('task-summary-status');
        if (summaryStatusElement) {
            const statusDot = summaryStatusElement.querySelector('.dashboard-summary-status-dot');
            const statusText = summaryStatusElement.querySelector('span');

            if (taskData.runningCount > 0) {
                statusDot.style.backgroundColor = '#ff9800';
                statusText.textContent = `${taskData.runningCount} 個執行中`;
            } else if (taskData.pendingCount > 0) {
                statusDot.style.backgroundColor = '#2196f3';
                statusText.textContent = `${taskData.pendingCount} 個待執行`;
            } else {
                statusDot.style.backgroundColor = '#4caf50';
                statusText.textContent = '全部完成';
            }
        }
    }

    /**
     * 更新任務階層結構
     * @param {Array} tasks - 活躍任務列表
     */
    function updateTaskHierarchy(tasks) {
        const hierarchyContainer = document.getElementById('dashboard-task-hierarchy');
        const containerWrapper = document.getElementById('dashboard-task-hierarchy-container');
        const emptyState = document.getElementById('dashboard-task-empty');

        if (!hierarchyContainer || !containerWrapper || !emptyState) {
            console.warn('任務階層結構元素未找到');
            return;
        }

        // 清空現有內容
        hierarchyContainer.innerHTML = '';

        if (tasks.length === 0) {
            // 顯示空狀態
            containerWrapper.style.display = 'none';
            emptyState.style.display = 'block';
            return;
        }

        // 隱藏空狀態，顯示階層結構
        emptyState.style.display = 'none';
        containerWrapper.style.display = 'block';

        // 建立任務階層結構
        const taskHierarchy = buildTaskHierarchy(tasks);

        // 渲染階層結構
        taskHierarchy.forEach(task => {
            const taskNode = createTaskHierarchyNode(task, 0);
            hierarchyContainer.appendChild(taskNode);
        });

        console.debug('任務階層結構已更新:', tasks.length, '個任務');
    }

    /**
     * 建立任務階層結構
     * @param {Array} tasks - 任務列表
     * @returns {Array} 階層結構的根任務列表
     */
    function buildTaskHierarchy(tasks) {
        const taskMap = {};
        const rootTasks = [];

        // 建立任務映射
        tasks.forEach(task => {
            taskMap[task.id] = { ...task, children: [] };
        });

        // 建立父子關係
        tasks.forEach(task => {
            const parentId = task.parent_id || task.parent_task_id;
            if (parentId && taskMap[parentId]) {
                taskMap[parentId].children.push(taskMap[task.id]);
            } else {
                rootTasks.push(taskMap[task.id]);
            }
        });

        return rootTasks;
    }

    /**
     * 創建任務階層節點
     * @param {Object} task - 任務資料
     * @param {number} level - 階層層級
     * @returns {HTMLElement} 任務節點元素
     */
    function createTaskHierarchyNode(task, level) {
        const node = document.createElement('div');
        node.className = `task-hierarchy-node level-${level}`;

        // 任務狀態
        const status = task.status_id || task.status;
        const statusInfo = getTaskStatusInfoLocal(status);

        // 時間格式化
        const timeStr = formatTaskTime(task.created_at || task.updated_at);

        // 節點內容
        node.innerHTML = `
            <div class="task-hierarchy-header">
                <div class="task-hierarchy-title">
                    <span class="icon has-text-${level === 0 ? 'primary' : 'info'}">
                        <i class="mdi mdi-${level === 0 ? 'folder' : 'file'}"></i>
                    </span>
                    <strong>ID: ${task.id || '-'}</strong>
                    <span>${task.name || '未命名任務'}</span>
                </div>
                <div class="task-hierarchy-meta">
                    <span class="tag ${statusInfo.class}">${statusInfo.text}</span>
                    ${task.agv_id ? `<span class="tag is-light">AGV ${task.agv_id}</span>` : ''}
                    ${task.children.length > 0 ? `<span class="task-children-count">${task.children.length} 子任務</span>` : ''}
                </div>
            </div>
            <div class="task-hierarchy-details">
                建立時間: ${timeStr}
                ${task.description ? ` | 描述: ${task.description}` : ''}
            </div>
        `;

        // 添加子任務
        task.children.forEach(child => {
            const childNode = createTaskHierarchyNode(child, level + 1);
            node.appendChild(childNode);
        });

        return node;
    }

    /**
     * 取得任務狀態資訊（使用統一的狀態定義）
     * @param {number} status - 狀態 ID
     * @returns {Object} 狀態資訊
     */
    function getTaskStatusInfoLocal(status) {
        const statusInfo = getTaskStatusInfo(status);
        return {
            class: statusInfo.color,
            text: statusInfo.name
        };
    }

    /**
     * 格式化任務時間
     * @param {string} timeStr - 時間字符串
     * @returns {string} 格式化後的時間
     */
    function formatTaskTime(timeStr) {
        if (!timeStr) return '-';

        try {
            const date = new Date(timeStr);
            const now = new Date();
            const diffMs = now - date;
            const diffMins = Math.floor(diffMs / (1000 * 60));
            const diffHours = Math.floor(diffMs / (1000 * 60 * 60));
            const diffDays = Math.floor(diffMs / (1000 * 60 * 60 * 24));

            if (diffMins < 1) {
                return '剛剛';
            } else if (diffMins < 60) {
                return `${diffMins}分鐘前`;
            } else if (diffHours < 24) {
                return `${diffHours}小時前`;
            } else if (diffDays < 7) {
                return `${diffDays}天前`;
            } else {
                // 超過一週顯示具體日期
                return date.toLocaleDateString('zh-TW', {
                    month: 'short',
                    day: 'numeric',
                    hour: '2-digit',
                    minute: '2-digit'
                });
            }
        } catch (e) {
            console.warn('時間格式化錯誤:', timeStr, e);
            return '-';
        }
    }



    /**
     * 更新指標數值
     * @param {string} elementId - 元素 ID
     * @param {string|number} value - 新值
     */
    function updateMetricValue(elementId, value) {
        const element = document.getElementById(elementId);
        if (element && element.textContent !== String(value)) {
            element.textContent = value;
        }
    }

    /**
     * 更新狀態指示器
     * @param {string} elementId - 元素 ID
     * @param {string} statusType - 狀態類型 (success, warning, danger, info)
     * @param {string} statusText - 狀態文字
     */
    function updateStatusIndicator(elementId, statusType, statusText) {
        const element = document.getElementById(elementId);
        if (!element) return;

        // 移除舊的狀態類別
        element.classList.remove('status-success', 'status-warning', 'status-danger', 'status-info');

        // 添加新的狀態類別
        element.classList.add(`status-${statusType}`);

        // 更新狀態文字
        const textElement = element.querySelector('.dashboard-status-text');
        if (textElement && textElement.textContent !== statusText) {
            textElement.textContent = statusText;
        }
    }

    /**
     * 更新進度條
     * @param {string} elementId - 元素 ID
     * @param {number} percentage - 百分比值
     */
    function updateProgressBar(elementId, percentage) {
        const element = document.getElementById(elementId);
        if (!element) return;

        const progressBar = element.querySelector('.progress');
        if (progressBar) {
            progressBar.value = percentage;

            // 根據百分比設置顏色
            progressBar.classList.remove('is-success', 'is-warning', 'is-danger');
            if (percentage >= 80) {
                progressBar.classList.add('is-success');
            } else if (percentage >= 50) {
                progressBar.classList.add('is-warning');
            } else {
                progressBar.classList.add('is-danger');
            }
        }
    }

    /**
     * 更新系統狀態
     */
    function updateSystemStatus() {
        dashboardData.system.lastUpdate = new Date();

        // 根據各子系統狀態判斷整體系統狀態
        let systemStatus = 'success';

        // 簡化系統狀態判斷，主要基於任務和房間狀態
        if (dashboardData.task.executing > 0) {
            systemStatus = 'info'; // 有任務執行中
        }

        // 可以根據需要添加更多狀態判斷邏輯
        dashboardData.system.status = systemStatus;

        // 更新系統狀態顯示
        updateStatusIndicator('dashboard-status-system', systemStatus, '系統運行中');
    }

    /**
     * 設置展開/收合功能
     */
    function setupExpandCollapse() {
        // 任務卡片展開/收合
        const taskExpandButton = document.getElementById('task-expand-button');
        if (taskExpandButton) {
            taskExpandButton.addEventListener('click', () => {
                toggleCardExpansion('dashboard-card-task', taskExpandButton);
            });
        }

        // 設備卡片展開/收合
        const equipmentExpandButton = document.getElementById('equipment-expand-button');
        if (equipmentExpandButton) {
            equipmentExpandButton.addEventListener('click', () => {
                toggleCardExpansion('dashboard-card-equipment', equipmentExpandButton);
            });
        }

        // 房間卡片展開/收合
        const roomsExpandButton = document.getElementById('rooms-expand-button');
        if (roomsExpandButton) {
            roomsExpandButton.addEventListener('click', () => {
                toggleCardExpansion('dashboard-card-rooms', roomsExpandButton);
            });
        }

        console.log('🔄 展開/收合功能已初始化');
    }

    /**
     * 切換卡片展開/收合狀態
     * @param {string} cardId - 卡片ID
     * @param {HTMLElement} button - 展開按鈕
     */
    function toggleCardExpansion(cardId, button) {
        const card = document.getElementById(cardId);
        if (!card) return;

        const isExpanded = card.classList.contains('dashboard-card-expanded');

        if (isExpanded) {
            // 收合卡片
            card.classList.remove('dashboard-card-expanded');
            card.classList.add('dashboard-card-compact');
            button.classList.remove('expanded');
        } else {
            // 展開卡片
            card.classList.add('dashboard-card-expanded');
            card.classList.remove('dashboard-card-compact');
            button.classList.add('expanded');
        }

        console.debug(`卡片 ${cardId} ${isExpanded ? '已收合' : '已展開'}`);
    }

    /**
     * 清理資源
     */
    function cleanup() {
        agvsStore.off('change', handleAgvsChange);
        racksStore.off('change', handleRacksChange);
        tasksStore.off('change', handleTasksChange);
        carriersStore.off('change', handleCarriersChange);
        roomsStore.off('change', handleRoomsChange);
        console.log('🧹 Dashboard 頁面資源已清理');
    }

    // 返回公開的方法
    return {
        setup,
        cleanup
    };
})();
