/**
 * 地圖互動功能模組
 * 提供地圖上的彈出視窗、側邊面板、工具列等互動功能
 */

import { userStore } from '../store/index.js';
import { notify } from './notify.js';
import { mapPermissions } from './mapPermissions.js';
import { getTaskStatusName } from './taskStatus.js';

export const mapInteraction = (() => {
    let currentPopup = null;
    let currentSidebar = null;
    let map = null;
    let currentPopupLatLng = null; // 新增：儲存當前 popup 的經緯度

    // 初始化
    function init(leafletMap) {
        map = leafletMap;
        mapPermissions.init();
        updatePermissions();
        initializeElements();
        setupEventListeners();
    }

    // 更新權限
    function updatePermissions() {
        // 權限更新由 mapPermissions 內部處理
        // 這裡可以添加 UI 更新邏輯
    }

    // 初始化現有元素
    function initializeElements() {
        // 獲取現有元素
        currentSidebar = document.getElementById('map-sidebar');
        currentPopup = document.getElementById('map-popup');

        // 綁定工具列事件
        const toolbar = document.querySelector('.map-toolbar');
        if (toolbar) {
            toolbar.addEventListener('click', handleToolbarClick);
        }

        // 使用事件委託綁定側邊面板關閉事件
        if (currentSidebar) {
            currentSidebar.addEventListener('click', (e) => {
                // 檢查點擊的是否為關閉按鈕或其子元素
                const closeButton = e.target.closest('#sidebar-close') ||
                    e.target.closest('.map-sidebar-close') ||
                    (e.target.id === 'sidebar-close') ||
                    (e.target.classList.contains('map-sidebar-close'));

                if (closeButton) {
                    e.preventDefault();
                    e.stopPropagation();
                    console.log('Sidebar close button clicked'); // 調試用
                    closeSidebar();
                }
            });
        }

        // 綁定彈出視窗關閉事件
        const popupCloseButton = document.getElementById('popup-close');
        if (popupCloseButton) {
            popupCloseButton.addEventListener('click', closePopup);
        }

        // 防止點擊彈出視窗時關閉
        if (currentPopup) {
            currentPopup.addEventListener('click', (e) => {
                e.stopPropagation();
            });
        }
    }

    // 設置事件監聽器
    function setupEventListeners() {
        // 監聽權限變化
        mapPermissions.onPermissionChange(updatePermissions);

        // 點擊地圖其他地方關閉彈出視窗
        map.on('click', () => {
            closePopup();
        });

        // 新增：地圖移動時更新 popup 位置
        map.on('move', updatePopupPosition);

        // ESC 鍵關閉所有彈出元素
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                closePopup();
                closeSidebar();
            }
        });
    }

    // 處理工具列點擊
    function handleToolbarClick(e) {
        const button = e.target.closest('button');
        if (!button) return;

        const toolId = button.id;
        console.log('Toolbar button clicked:', toolId); // 調試用

        // 檢查是否點擊了已經活動的按鈕
        const isCurrentlyActive = button.classList.contains('is-active');
        console.log('Button currently active:', isCurrentlyActive); // 調試用

        // 移除所有活動狀態
        document.querySelectorAll('.map-toolbar .button').forEach(btn => {
            btn.classList.remove('is-active');
        });

        // 如果點擊的是已經活動的按鈕，關閉側邊面板
        if (isCurrentlyActive) {
            console.log('Closing sidebar due to repeat click'); // 調試用
            closeSidebar();
            return;
        }

        // 設置當前按鈕為活動狀態
        button.classList.add('is-active');
        console.log('Button set to active:', toolId); // 調試用

        switch (toolId) {
            case 'map-tool-tasks':
                showTasksSidebar();
                break;
            case 'map-tool-racks':
                showRacksSidebar();
                break;
            case 'map-tool-carriers':
                showCarriersSidebar();
                break;
            case 'map-tool-agvs':
                showAgvsSidebar();
                break;
            case 'map-tool-legend':
                showLegend();
                // 圖例不需要保持按鈕活動狀態
                button.classList.remove('is-active');
                break;
            default:
                console.warn('Unknown tool ID:', toolId); // 調試用
        }
    }

    // 顯示彈出視窗
    function showPopup(latlng, title, content, actions = []) {
        if (!currentPopup) return;
        currentPopupLatLng = latlng; // 記錄當前經緯度

        // 設置標題
        const titleElement = document.getElementById('popup-title');
        if (titleElement) {
            titleElement.textContent = title;
        }

        // 設置內容
        const contentElement = document.getElementById('popup-content');
        if (contentElement) {
            contentElement.innerHTML = content;
        }

        // 設置操作按鈕
        const actionsElement = document.getElementById('popup-actions');
        if (actionsElement && actions.length > 0) {
            // 使用權限系統過濾操作
            const filteredActions = mapPermissions.filterActions(actions);

            if (filteredActions.length > 0) {
                actionsElement.innerHTML = '';
                filteredActions.forEach(action => {
                    const hasPermission = !action.permission || mapPermissions.hasPermission(action.permission);
                    const button = document.createElement('button');
                    button.className = `button ${action.class || 'is-primary'}`;
                    button.innerHTML = `
                        <span class="icon">
                            <i class="mdi ${action.icon}"></i>
                        </span>
                        <span>${action.text}</span>
                    `;

                    if (hasPermission) {
                        button.onclick = () => eval(action.onclick);
                    } else {
                        button.disabled = true;
                        button.title = mapPermissions.createPermissionTooltip(action.permission);
                    }

                    actionsElement.appendChild(button);
                });
                actionsElement.style.display = 'flex';
            } else {
                actionsElement.style.display = 'none';
            }
        } else if (actionsElement) {
            actionsElement.style.display = 'none';
        }

        // 計算位置
        const point = map.latLngToContainerPoint(latlng);
        currentPopup.style.left = `${point.x + 10}px`;
        currentPopup.style.top = `${point.y - 10}px`;
        currentPopup.style.display = 'block';

        return currentPopup;
    }

    // 關閉彈出視窗
    function closePopup() {
        if (currentPopup) {
            currentPopup.style.display = 'none';
        }
    }

    // 顯示側邊面板
    function showSidebar(title, content) {
        console.log('showSidebar called with title:', title); // 調試用
        if (!currentSidebar) {
            console.error('currentSidebar is null'); // 調試用
            return;
        }

        const titleElement = currentSidebar.querySelector('#sidebar-title');
        const contentElement = currentSidebar.querySelector('#sidebar-content');

        if (titleElement) {
            titleElement.textContent = title;
        } else {
            console.error('sidebar-title element not found'); // 調試用
        }

        if (contentElement) {
            // 保存原始模板（只在第一次保存）
            if (!contentElement._originalTemplates) {
                const templates = contentElement.querySelectorAll('[id$="-template"]');
                contentElement._originalTemplates = Array.from(templates).map(t => t.outerHTML).join('');
            }

            // 清空內容並設置新內容，然後恢復原始模板
            contentElement.innerHTML = content + contentElement._originalTemplates;
        } else {
            console.error('sidebar-content element not found'); // 調試用
        }

        console.log('Before adding class - classList:', currentSidebar.classList.toString()); // 調試用

        // 清除任何內聯樣式，讓 CSS class 生效
        currentSidebar.style.right = '';
        currentSidebar.classList.add('is-open');

        console.log('After adding class - classList:', currentSidebar.classList.toString()); // 調試用
        console.log('After adding class - computed style right:', window.getComputedStyle(currentSidebar).right); // 調試用

        // 如果 CSS 不生效，強制設置樣式
        setTimeout(() => {
            const computedRight = window.getComputedStyle(currentSidebar).right;
            if (computedRight !== '0px') {
                console.debug('CSS transition not complete, applying fallback positioning. Computed right:', computedRight); // 調試用
                currentSidebar.style.right = '0px';
                console.debug('Applied fallback positioning - new computed right:', window.getComputedStyle(currentSidebar).right); // 調試用
            }
        }, 50); // 給 CSS 一點時間生效

        console.log('Sidebar opened successfully'); // 調試用
    }

    // 關閉側邊面板
    function closeSidebar() {
        console.log('closeSidebar called'); // 調試用
        if (currentSidebar) {
            console.log('Before removing class - classList:', currentSidebar.classList.toString()); // 調試用
            console.log('Before removing class - computed style right:', window.getComputedStyle(currentSidebar).right); // 調試用

            // 同時使用 class 和直接設置 style 來確保關閉
            currentSidebar.classList.remove('is-open');
            currentSidebar.style.right = '-400px'; // 強制設置位置

            console.log('After removing class - classList:', currentSidebar.classList.toString()); // 調試用
            console.log('After removing class - computed style right:', window.getComputedStyle(currentSidebar).right); // 調試用
            console.log('After setting style - style.right:', currentSidebar.style.right); // 調試用

            // 移除工具列活動狀態
            document.querySelectorAll('.map-toolbar .button').forEach(btn => {
                btn.classList.remove('is-active');
            });
            console.log('Sidebar closed successfully'); // 調試用
        } else {
            console.warn('currentSidebar is null'); // 調試用
        }
    }

    // 顯示任務側邊面板
    function showTasksSidebar() {
        console.log('showTasksSidebar called'); // 調試用
        const template = document.getElementById('tasks-template');
        console.log('tasks-template found:', !!template); // 調試用

        if (template) {
            console.log('Cloning template content'); // 調試用
            // 克隆模板內容
            const clonedContent = template.cloneNode(true);
            clonedContent.style.display = 'block';
            clonedContent.removeAttribute('id'); // 移除重複的 ID

            // 檢查權限並顯示/隱藏操作按鈕
            const actionsElement = clonedContent.querySelector('#tasks-actions');
            if (actionsElement) {
                try {
                    const canCreate = mapPermissions.hasPermission('create_task');
                    console.log('Permission check result:', canCreate); // 調試用
                    actionsElement.style.display = canCreate ? 'flex' : 'none';
                    actionsElement.removeAttribute('id'); // 移除重複的 ID
                } catch (error) {
                    console.error('Permission check error:', error); // 調試用
                    actionsElement.style.display = 'none'; // 默認隱藏
                    actionsElement.removeAttribute('id');
                }
            }

            console.log('Calling showSidebar with tasks content'); // 調試用
            showSidebar('任務管理', clonedContent.outerHTML);

            // 延遲載入列表，確保 DOM 已更新
            setTimeout(() => {
                console.log('Loading tasks list after delay'); // 調試用
                // 優先使用 mapTaskManager 的 loadTasksList，如果不存在則使用本地的
                if (window.mapTaskManager && typeof window.mapTaskManager.loadTasksList === 'function') {
                    window.mapTaskManager.loadTasksList();
                } else {
                    loadTasksList();
                }
            }, 10);
        } else {
            console.error('tasks-template not found!'); // 調試用
        }
    }

    // 顯示貨架側邊面板
    function showRacksSidebar() {
        console.log('showRacksSidebar called'); // 調試用
        const template = document.getElementById('racks-template');
        console.log('racks-template found:', !!template); // 調試用

        if (template) {
            console.log('Cloning racks template content'); // 調試用
            // 克隆模板內容
            const clonedContent = template.cloneNode(true);
            clonedContent.style.display = 'block';
            clonedContent.removeAttribute('id'); // 移除重複的 ID

            // 檢查權限並顯示/隱藏操作按鈕
            const actionsElement = clonedContent.querySelector('#racks-actions');
            if (actionsElement) {
                try {
                    const canCreate = mapPermissions.hasPermission('create_rack');
                    console.log('Racks permission check result:', canCreate); // 調試用
                    actionsElement.style.display = canCreate ? 'flex' : 'none';
                    actionsElement.removeAttribute('id'); // 移除重複的 ID
                } catch (error) {
                    console.error('Racks permission check error:', error); // 調試用
                    actionsElement.style.display = 'none';
                    actionsElement.removeAttribute('id');
                }
            }

            console.log('Calling showSidebar with racks content'); // 調試用
            showSidebar('貨架管理', clonedContent.outerHTML);

            // 延遲載入列表，確保 DOM 已更新
            setTimeout(() => {
                console.log('Loading racks list after delay'); // 調試用
                // 優先使用 mapRackManager 的 loadRacksList，如果不存在則使用本地的
                if (window.mapRackManager && typeof window.mapRackManager.loadRacksList === 'function') {
                    console.log('Using mapRackManager.loadRacksList');
                    window.mapRackManager.loadRacksList();
                } else {
                    console.log('Using mapInteraction.loadRacksList');
                    loadRacksList();
                }
            }, 10);
        } else {
            console.error('racks-template not found!'); // 調試用
        }
    }

    // 顯示載具側邊面板
    function showCarriersSidebar() {
        const template = document.getElementById('carriers-template');

        if (template) {
            // 克隆模板內容
            const clonedContent = template.cloneNode(true);
            clonedContent.style.display = 'block';
            clonedContent.removeAttribute('id'); // 移除重複的 ID

            // 檢查權限並顯示/隱藏操作按鈕
            const actionsElement = clonedContent.querySelector('#carriers-actions');
            if (actionsElement) {
                const canCreate = mapPermissions.hasPermission('create_carrier');
                actionsElement.style.display = canCreate ? 'flex' : 'none';
                actionsElement.removeAttribute('id'); // 移除重複的 ID
            }

            showSidebar('載具管理', clonedContent.outerHTML);

            // 延遲載入列表，確保 DOM 已更新
            setTimeout(() => {
                // 優先使用 mapCarrierManager 的 loadCarriersList，如果不存在則使用本地的
                if (window.mapCarrierManager && typeof window.mapCarrierManager.loadCarriersList === 'function') {
                    console.log('Using mapCarrierManager.loadCarriersList');
                    window.mapCarrierManager.loadCarriersList();
                } else {
                    console.log('Using mapInteraction.loadCarriersList');
                    loadCarriersList();
                }
            }, 10);
        }
    }

    // 顯示 AGV 側邊面板
    function showAgvsSidebar() {
        const template = document.getElementById('agvs-template');

        if (template) {
            // 克隆模板內容
            const clonedContent = template.cloneNode(true);
            clonedContent.style.display = 'block';
            clonedContent.removeAttribute('id'); // 移除重複的 ID

            // 檢查權限並顯示/隱藏操作按鈕
            const actionsElement = clonedContent.querySelector('#agvs-actions');
            if (actionsElement) {
                const canCreate = mapPermissions.hasPermission('create_agv');
                actionsElement.style.display = canCreate ? 'flex' : 'none';
                actionsElement.removeAttribute('id'); // 移除重複的 ID
            }

            showSidebar('AGV 管理', clonedContent.outerHTML);

            // 延遲載入列表，確保 DOM 已更新
            setTimeout(() => {
                // 優先使用 mapAgvManager 的 loadAgvsList，如果不存在則使用本地的
                if (window.mapAgvManager && typeof window.mapAgvManager.loadAgvsList === 'function') {
                    console.log('Using mapAgvManager.loadAgvsList');
                    window.mapAgvManager.loadAgvsList();
                } else {
                    console.log('Using mapInteraction.loadAgvsList');
                    loadAgvsList();
                }
            }, 10);
        }
    }

    // 顯示圖例
    function showLegend() {
        const legend = document.getElementById('map-legend');
        if (!legend) return;

        // 切換顯示/隱藏
        if (legend.style.display === 'block') {
            legend.style.display = 'none';
        } else {
            legend.style.display = 'block';

            // 5秒後自動隱藏
            setTimeout(() => {
                legend.style.display = 'none';
            }, 5000);
        }
    }

    // 載入任務列表
    async function loadTasksList() {
        console.log('mapInteraction.loadTasksList: 開始載入任務列表');

        // 優先使用 mapTaskManager 的 loadTasksList
        if (window.mapTaskManager && typeof window.mapTaskManager.loadTasksList === 'function') {
            console.log('mapInteraction.loadTasksList: 使用 mapTaskManager.loadTasksList');
            window.mapTaskManager.loadTasksList();
            return;
        }

        // 備用邏輯（如果 mapTaskManager 不可用）
        console.warn('mapInteraction.loadTasksList: mapTaskManager 不可用，使用備用邏輯');

        const sidebarContent = document.getElementById('sidebar-content');
        if (!sidebarContent) {
            console.error('mapInteraction.loadTasksList: sidebar-content 元素未找到');
            return;
        }

        const listElement = sidebarContent.querySelector('#tasks-list') || sidebarContent.querySelector('[data-list="tasks"]');
        if (!listElement) {
            console.warn('mapInteraction.loadTasksList: tasks-list 元素未找到');
            return;
        }

        try {
            listElement.innerHTML = '<div class="has-text-centered"><span class="icon"><i class="mdi mdi-loading mdi-spin"></i></span> 載入中...</div>';

            if (window.tasksStore) {
                const tasksState = window.tasksStore.getState();
                const tasks = tasksState.tasks || [];

                if (tasks.length === 0) {
                    listElement.innerHTML = '<p class="has-text-grey">目前沒有任務</p>';
                    return;
                }

                // 按 ID 排序任務
                const sortedTasks = tasks.sort((a, b) => a.id - b.id);

                // 使用與 mapTaskManager 相同的樣式和結構，並使用原本的跳轉行為
                const tasksHtml = sortedTasks.map(task => `
                    <div class="task-item" onclick="window.mapInteraction.viewTaskDetails(${task.id})">
                        <div class="level is-mobile">
                            <div class="level-left">
                                <div class="level-item">
                                    <div>
                                        <p class="title is-6">${task.name || `任務 ${task.id}`}</p>
                                        <p class="subtitle is-7">
                                            <span class="tag is-small is-info">狀態 ${task.status_id ?? 'N/A'}</span>
                                            ${task.agv_id ? `<span class="tag is-small is-light">AGV ${task.agv_id}</span>` : ''}
                                        </p>
                                    </div>
                                </div>
                            </div>
                            <div class="level-right">
                                <div class="level-item">
                                    <span class="icon">
                                        <i class="mdi mdi-chevron-right"></i>
                                    </span>
                                </div>
                            </div>
                        </div>
                    </div>
                `).join('');

                // 新增任務總數顯示
                const totalInfo = `<div class="total-info has-text-grey is-size-7 mb-2">總計 ${tasks.length} 個任務</div>`;
                listElement.innerHTML = totalInfo + tasksHtml;

                console.log(`mapInteraction.loadTasksList: 已載入 ${tasks.length} 個任務（備用邏輯）`);
            } else {
                listElement.innerHTML = '<div class="has-text-danger">任務資料未載入</div>';
            }

        } catch (error) {
            console.error('mapInteraction.loadTasksList: 載入失敗', error);
            listElement.innerHTML = '<div class="has-text-danger">載入失敗</div>';
        }
    }

    // 載入貨架列表
    async function loadRacksList() {
        // 在側邊面板中查找貨架列表元素
        const sidebarContent = document.getElementById('sidebar-content');
        if (!sidebarContent) return;

        const listElement = sidebarContent.querySelector('#racks-list') || sidebarContent.querySelector('[data-list="racks"]');
        if (!listElement) {
            console.warn('Racks list element not found in sidebar');
            return;
        }

        try {
            listElement.innerHTML = '<div class="has-text-centered"><span class="icon"><i class="mdi mdi-loading mdi-spin"></i></span> 載入中...</div>';

            // 直接從 racksStore 載入資料
            if (window.racksStore) {
                const racksState = window.racksStore.getState();
                const racks = racksState.racks || [];

                if (racks.length === 0) {
                    listElement.innerHTML = '<p class="has-text-grey">目前沒有貨架</p>';
                    return;
                }

                // 按 ID 排序貨架
                const sortedRacks = racks.sort((a, b) => a.id - b.id);

                const racksHtml = sortedRacks.map(rack => `
                    <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapInteraction.viewRackDetails(${rack.id})">
                        <div class="map-info-card-header">
                            <span class="map-info-card-title">${rack.name || `貨架 ${rack.id}`}</span>
                            <span class="tag is-success">貨架 ${rack.id}</span>
                        </div>
                        <div class="is-size-7 has-text-grey">
                            位置: ${rack.location_id || 'N/A'} | 產品: ${rack.product_name || rack.product_id || 'N/A'}
                        </div>
                    </div>
                `).join('');

                // 新增貨架總數顯示
                const totalInfo = `<div class="has-text-grey is-size-7 mb-2">總計 ${racks.length} 個貨架</div>`;
                listElement.innerHTML = totalInfo + racksHtml;
            } else {
                listElement.innerHTML = '<div class="has-text-danger">貨架資料未載入</div>';
            }

        } catch (error) {
            console.error('Error loading racks:', error);
            listElement.innerHTML = '<div class="has-text-danger">載入失敗</div>';
        }
    }

    // 載入載具列表
    async function loadCarriersList() {
        // 在側邊面板中查找載具列表元素
        const sidebarContent = document.getElementById('sidebar-content');
        if (!sidebarContent) return;

        const listElement = sidebarContent.querySelector('#carriers-list') || sidebarContent.querySelector('[data-list="carriers"]');
        if (!listElement) {
            console.warn('Carriers list element not found in sidebar');
            return;
        }

        try {
            listElement.innerHTML = '<div class="has-text-centered"><span class="icon"><i class="mdi mdi-loading mdi-spin"></i></span> 載入中...</div>';

            // 直接從 carriersStore 載入資料
            if (window.carriersStore) {
                const carriersState = window.carriersStore.getState();
                const carriers = carriersState.carriers || [];

                if (carriers.length === 0) {
                    listElement.innerHTML = '<p class="has-text-grey">目前沒有載具</p>';
                    return;
                }

                // 按 ID 排序載具
                const sortedCarriers = carriers.sort((a, b) => a.id - b.id);

                const carriersHtml = sortedCarriers.map(carrier => `
                    <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapInteraction.viewCarrierDetails(${carrier.id})">
                        <div class="map-info-card-header">
                            <span class="map-info-card-title">載具 ${carrier.id}</span>
                            <span class="tag is-info">載具 ${carrier.id}</span>
                        </div>
                        <div class="is-size-7 has-text-grey">
                            貨架: ${carrier.rack_id || 'N/A'} | 位置: ${carrier.rack_index || 'N/A'}
                        </div>
                    </div>
                `).join('');

                // 新增載具總數顯示
                const totalInfo = `<div class="has-text-grey is-size-7 mb-2">總計 ${carriers.length} 個載具</div>`;
                listElement.innerHTML = totalInfo + carriersHtml;
            } else {
                listElement.innerHTML = '<div class="has-text-danger">載具資料未載入</div>';
            }

        } catch (error) {
            console.error('Error loading carriers:', error);
            listElement.innerHTML = '<div class="has-text-danger">載入失敗</div>';
        }
    }

    // 輔助函數 - 任務狀態
    function getTaskStatusClass(statusId) {
        switch (statusId) {
            case 1: return 'is-info';      // 待處理
            case 2: return 'is-warning';   // 進行中
            case 3: return 'is-success';   // 已完成
            case 4: return 'is-danger';    // 已取消
            default: return 'is-light';
        }
    }

    function getTaskStatusNameLocal(statusId) {
        return getTaskStatusName(statusId);
    }

    // 輔助函數 - 貨架狀態
    function getRackStatusClass(statusId) {
        switch (statusId) {
            case 1: return 'is-success';   // 正常
            case 2: return 'is-warning';   // 維護中
            case 3: return 'is-danger';    // 故障
            default: return 'is-light';
        }
    }

    function getRackStatusName(statusId) {
        switch (statusId) {
            case 1: return '正常';
            case 2: return '維護中';
            case 3: return '故障';
            default: return '未知';
        }
    }

    // 輔助函數 - 載具狀態
    function getCarrierStatusClass(statusId) {
        switch (statusId) {
            case 1: return 'is-success';   // 正常
            case 2: return 'is-info';      // 使用中
            case 3: return 'is-warning';   // 維護中
            case 4: return 'is-danger';    // 故障
            default: return 'is-light';
        }
    }

    function getCarrierStatusName(statusId) {
        switch (statusId) {
            case 1: return '正常';
            case 2: return '使用中';
            case 3: return '維護中';
            case 4: return '故障';
            default: return '未知';
        }
    }

    // 詳細檢視方法
    function viewTaskDetails(taskId) {
        mapPermissions.executeWithPermission('view_tasks', () => {
            window.open(`/tasks/${taskId}/edit`, '_blank');
        });
    }

    function viewRackDetails(rackId) {
        mapPermissions.executeWithPermission('view_racks', () => {
            window.open(`/racks/${rackId}/edit`, '_blank');
        });
    }

    function viewCarrierDetails(carrierId) {
        mapPermissions.executeWithPermission('view_carriers', () => {
            window.open(`/carriers/${carrierId}/edit`, '_blank');
        });
    }

    // 創建方法
    function createTask() {
        mapPermissions.executeWithPermission('create_task', () => {
            window.open('/tasks/create', '_blank');
        });
    }

    function createRack() {
        mapPermissions.executeWithPermission('create_rack', () => {
            window.open('/racks/create', '_blank');
        });
    }

    function createCarrier() {
        mapPermissions.executeWithPermission('create_carrier', () => {
            window.open('/carriers/create', '_blank');
        });
    }

    // AGV 相關方法
    function viewAgvDetails(agvId) {
        mapPermissions.executeWithPermission('view_agvs', () => {
            window.open(`/agvs/${agvId}/edit`, '_blank');
        });
    }

    function createAgv() {
        mapPermissions.executeWithPermission('create_agv', () => {
            window.open('/agvs/create', '_blank');
        });
    }

    // 載入 AGV 列表
    function loadAgvsList() {
        // 在側邊面板中查找 AGV 列表元素
        const sidebarContent = document.getElementById('sidebar-content');
        if (!sidebarContent) return;

        const listElement = sidebarContent.querySelector('#agvs-list') || sidebarContent.querySelector('[data-list="agvs"]');
        if (!listElement) {
            console.warn('AGVs list element not found in sidebar');
            return;
        }

        try {
            listElement.innerHTML = '<div class="has-text-centered"><span class="icon"><i class="mdi mdi-loading mdi-spin"></i></span> 載入中...</div>';

            // 直接從 agvsStore 載入資料
            if (window.agvsStore) {
                const agvsState = window.agvsStore.getState();
                const agvs = agvsState.agvs || [];

                if (agvs.length === 0) {
                    listElement.innerHTML = '<p class="has-text-grey">目前沒有 AGV</p>';
                    return;
                }

                // 按 ID 排序 AGV
                const sortedAgvs = agvs.sort((a, b) => a.id - b.id);

                const agvsHtml = sortedAgvs.map(agv => {
                    const statusInfo = getAgvStatusInfo(agv.status_id);
                    const batteryInfo = getAgvBatteryInfo(agv.battery);

                    return `
                    <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapInteraction.viewAgvDetails(${agv.id})">
                        <div class="map-info-card-header">
                            <span class="map-info-card-title">${agv.name}</span>
                            <span class="tag ${statusInfo.class}">${statusInfo.text}</span>
                        </div>
                        <div class="is-size-7 has-text-grey">
                            位置: (${agv.x.toFixed(1)}, ${agv.y.toFixed(1)}) | ${batteryInfo}
                        </div>
                    </div>
                `;
                }).join('');

                // 新增 AGV 總數顯示
                const totalInfo = `<div class="has-text-grey is-size-7 mb-2">總計 ${agvs.length} 個 AGV</div>`;
                listElement.innerHTML = totalInfo + agvsHtml;
            } else {
                listElement.innerHTML = '<div class="has-text-danger">AGV 資料未載入</div>';
            }

        } catch (error) {
            console.error('Error loading AGVs:', error);
            listElement.innerHTML = '<div class="has-text-danger">載入失敗</div>';
        }
    }

    // 獲取 AGV 狀態資訊
    function getAgvStatusInfo(status) {
        // 處理 null, undefined 或空值
        if (status === null || status === undefined) {
            return { class: 'is-light', text: '未設定' };
        }

        switch (status) {
            case 1:
                return { class: 'is-dark', text: '離場' };
            case 2:
                return { class: 'is-danger', text: '離線' };
            case 3:
                return { class: 'is-success', text: '空閒' };
            case 4:
                return { class: 'is-info', text: '任務中' };
            case 5:
                return { class: 'is-warning', text: '充電中' };
            case 6:
                return { class: 'is-link', text: '更新中' };
            case 7:
                return { class: 'is-danger', text: '異常' };
            case 8:
                return { class: 'is-warning', text: '維護中' };
            case 9:
                return { class: 'is-light', text: '待機' };
            case 10:
                return { class: 'is-info', text: '初始化' };
            default:
                return { class: 'is-light', text: `未知(${status})` };
        }
    }

    // 獲取 AGV 電量資訊
    function getAgvBatteryInfo(battery) {
        if (battery === null || battery === undefined) {
            return '電量: 未知';
        }
        return `電量: ${battery.toFixed(1)}%`;
    }

    // 更新 popup 位置
    function updatePopupPosition() {
        if (!currentPopupLatLng || !currentPopup || !map) return;
        const point = map.latLngToContainerPoint(currentPopupLatLng);
        currentPopup.style.left = `${point.x + 10}px`;
        currentPopup.style.top = `${point.y - 10}px`;
    }

    // 公開方法
    return {
        init,
        showPopup,
        closePopup,
        showSidebar,
        closeSidebar,
        updatePermissions,
        viewTaskDetails,
        viewRackDetails,
        viewCarrierDetails,
        viewAgvDetails,
        createTask,
        createRack,
        createCarrier,
        createAgv
    };
})();

// 全域暴露以供 HTML onclick 使用
window.mapInteraction = mapInteraction;
