/**
 * 地圖互動功能模組
 * 提供地圖上的彈出視窗、側邊面板、工具列等互動功能
 */

import { userStore, mapStore } from '../store/index.js';
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

        // 綁定底部檢視控制事件
        const viewControls = document.querySelector('.map-view-controls');
        if (viewControls) {
            viewControls.addEventListener('click', handleToolbarClick);
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
            case 'map-tool-nodes':
                showNodesSidebar();
                break;
            case 'map-tool-legend':
                showLegend();
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

    // 顯示路徑節點側邊面板
    function showNodesSidebar() {
        console.log('showNodesSidebar called'); // 調試用
        const template = document.getElementById('nodes-template');

        if (template) {
            // 克隆模板內容
            const clonedContent = template.cloneNode(true);
            clonedContent.style.display = 'block';
            clonedContent.removeAttribute('id'); // 移除重複的 ID

            // 檢查權限並顯示/隱藏操作按鈕
            const actionsElement = clonedContent.querySelector('#nodes-actions');
            if (actionsElement) {
                const canCreate = mapPermissions.hasPermission('create_node');
                actionsElement.style.display = canCreate ? 'flex' : 'none';
                actionsElement.removeAttribute('id'); // 移除重複的 ID
            }

            showSidebar('路徑節點管理', clonedContent.outerHTML);

            // 延遲載入列表，確保 DOM 已更新
            setTimeout(() => {
                console.log('Loading nodes list after delay'); // 調試用
                loadNodesList();
            }, 10);
        } else {
            console.error('nodes-template not found!'); // 調試用
        }
    }

    // 顯示圖例
    function showLegend() {
        console.log('showLegend() called'); // 調試用
        const legend = document.getElementById('map-legend');
        const legendBtn = document.getElementById('map-tool-legend');
        console.log('legend element:', legend); // 調試用
        console.log('legendBtn element:', legendBtn); // 調試用
        if (!legend) return;

        // 切換顯示/隱藏
        if (legend.style.display === 'block') {
            legend.style.display = 'none';
            // 移除按鈕激活狀態
            if (legendBtn) {
                legendBtn.setAttribute('data-show', 'false');
            }
        } else {
            legend.style.display = 'block';
            // 添加按鈕激活狀態
            if (legendBtn) {
                legendBtn.setAttribute('data-show', 'true');
            }
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

    // 載入路徑節點列表
    async function loadNodesList() {
        console.log('loadNodesList called'); // 調試用
        const sidebarContent = document.getElementById('sidebar-content');
        if (!sidebarContent) {
            console.error('sidebar-content element not found');
            return;
        }

        const listElement = sidebarContent.querySelector('#nodes-list') || sidebarContent.querySelector('[data-list="nodes"]');
        if (!listElement) {
            console.warn('Nodes list element not found in sidebar');
            return;
        }

        try {
            listElement.innerHTML = '<div class="has-text-centered"><span class="icon"><i class="mdi mdi-loading mdi-spin"></i></span> 載入中...</div>';

            // 從後端 API 載入節點資料
            const response = await fetch('/path-nodes/api/path-nodes');
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const nodes = await response.json();
            console.log('Loaded nodes:', nodes);

            if (!nodes || nodes.length === 0) {
                listElement.innerHTML = '<p class="has-text-grey">目前沒有路徑節點</p>';
                return;
            }

            // 按 ID 排序節點
            const sortedNodes = nodes.sort((a, b) => a.id - b.id);

            const nodesHtml = sortedNodes.map(node => {
                const typeText = getNodeTypeText(node.type);
                const pgvText = node.pgv === '前' ? '前' : '後';

                // 安全處理可能為 null 的數值欄位
                const x = node.x != null ? node.x.toFixed(1) : 'N/A';
                const y = node.y != null ? node.y.toFixed(1) : 'N/A';
                const theta = node.theta != null ? node.theta.toFixed(1) : 'N/A';

                return `
                <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapInteraction.viewNodeDetails(${node.id})">
                    <div class="map-info-card-header">
                        <span class="map-info-card-title">節點 ${node.id}</span>
                        <span class="tag is-info">${typeText}</span>
                    </div>
                    <div class="is-size-7 has-text-grey">
                        座標: (${x}, ${y}, ${theta}°) | PGV: ${pgvText}
                    </div>
                </div>
                `;
            }).join('');

            // 新增節點總數顯示
            const totalInfo = `<div class="has-text-grey is-size-7 mb-2">總計 ${nodes.length} 個節點</div>`;
            listElement.innerHTML = totalInfo + nodesHtml;

        } catch (error) {
            console.error('Error loading nodes:', error);
            listElement.innerHTML = '<div class="has-text-danger">載入失敗: ' + error.message + '</div>';
        }
    }

    // 取得節點類型文字
    function getNodeTypeText(type) {
        switch (type) {
            case 'NONE': return '一般';
            case '休息區': return '休息區';
            case '充電區': return '充電區';
            case '搬運點': return '搬運點';
            default: return type || '一般';
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

    // 匯入 LabVIEW 路徑檔
    function importLabVIEWPaths() {
        console.log('importLabVIEWPaths called');

        // 創建檔案輸入元素
        const fileInput = document.createElement('input');
        fileInput.type = 'file';
        fileInput.accept = '.json';
        fileInput.style.display = 'none';

        fileInput.onchange = async function(event) {
            const file = event.target.files[0];
            if (!file) {
                return;
            }

            try {
                // 讀取檔案內容
                const fileContent = await readFileContent(file);
                console.log('File content loaded:', fileContent.substring(0, 200) + '...');

                // 解析 JSON
                let labViewData;
                try {
                    labViewData = JSON.parse(fileContent);
                } catch (error) {
                    console.error('JSON parsing error:', error);
                    notify.showErrorMessage('檔案格式錯誤：無法解析 JSON 格式');
                    return;
                }

                // 驗證檔案格式
                if (!Array.isArray(labViewData)) {
                    notify.showErrorMessage('檔案格式錯誤：根元素必須是陣列');
                    return;
                }

                // 調用預覽 API 獲取差異
                notify.showNotifyMessage(`正在分析匯入差異...`, 'is-info');

                const previewResponse = await fetch('/path-nodes/preview-labview-import', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ labview_data: labViewData })
                });

                if (previewResponse.ok) {
                    const diffData = await previewResponse.json();
                    console.log('Preview data:', diffData);

                    // 總是顯示差異預覽 Modal（即使沒有差異）
                    showLabVIEWDiffModal(diffData, labViewData);
                } else {
                    const error = await previewResponse.json();
                    console.error('Preview error:', error);
                    notify.showErrorMessage(`預覽失敗: ${error.detail}`);
                }

            } catch (error) {
                console.error('File processing error:', error);
                notify.showErrorMessage('處理檔案時發生錯誤');
            }
        };

        // 觸發檔案選擇對話框
        document.body.appendChild(fileInput);
        fileInput.click();
        document.body.removeChild(fileInput);
    }

    // 讀取檔案內容的輔助函數
    function readFileContent(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = event => resolve(event.target.result);
            reader.onerror = error => reject(error);
            reader.readAsText(file);
        });
    }

    // 重新載入地圖節點顯示
    async function reloadMapNodes() {
        console.log('reloadMapNodes called');

        try {
            // 從後端 API 獲取最新的節點資料
            const nodesResponse = await fetch('/path-nodes/api/path-nodes');
            if (!nodesResponse.ok) {
                throw new Error(`HTTP error! status: ${nodesResponse.status}`);
            }

            const nodes = await nodesResponse.json();
            console.log('Fetched latest nodes for map:', nodes.length, 'nodes');

            // 從後端 API 獲取最新的邊資料
            const edgesResponse = await fetch('/path-nodes/api/path-edges');
            if (!edgesResponse.ok) {
                throw new Error(`HTTP error! status: ${edgesResponse.status}`);
            }

            const edges = await edgesResponse.json();
            console.log('Fetched latest edges for map:', edges.length, 'edges');

            // 獲取當前地圖狀態
            const currentMapState = window.mapStore.getState();
            console.log('Current map state:', currentMapState);

            // 更新地圖狀態中的節點和邊資料
            const updatedMapState = {
                ...currentMapState,
                nodes: nodes || [],
                edges: edges || []
            };

            // 更新地圖狀態，這會觸發 MapChangehandler 重新渲染節點和邊
            window.mapStore.setState(updatedMapState);
            console.log('Map state updated with', nodes.length, 'nodes and', edges.length, 'edges');

            // 顯示成功訊息
            notify.showNotifyMessage(`地圖已更新，顯示 ${nodes.length} 個節點和 ${edges.length} 個邊`, 'is-success');

        } catch (error) {
            console.error('Error reloading map nodes:', error);
            notify.showErrorMessage('重新載入地圖節點失敗: ' + error.message);
        }
    }

    // 創建新節點
    function createNode() {
        console.log('createNode called');

        // 顯示節點參數設定模態視窗
        const modal = document.getElementById('node-settings-modal');
        if (modal) {
            // 重設表單
            resetNodeForm();
            // 初始化群組設定
            initializeGroupSettings();

            // 清除編輯模式標記
            delete modal.dataset.mode;
            delete modal.dataset.nodeId;

            // 確保 ID 欄位可編輯
            const nodeIdInput = document.getElementById('node-id');
            if (nodeIdInput) {
                nodeIdInput.disabled = false;
            }

            // 顯示模態視窗
            modal.classList.add('is-active');

            // 綁定模態視窗事件
            setupNodeModalEvents();
        } else {
            console.error('node-settings-modal not found');
        }
    }

    // 查看節點詳細資訊
    function viewNodeDetails(nodeId) {
        console.log('viewNodeDetails called with ID:', nodeId);
        // 打開節點設定 modal 以查看/編輯節點
        editNode(nodeId);
    }

    // 編輯節點
    async function editNode(nodeId) {
        console.log('editNode called with ID:', nodeId);

        try {
            // 1. 從 API 獲取節點資料
            const response = await fetch(`/path-nodes/api/path-nodes/${nodeId}`);
            if (!response.ok) {
                throw new Error(`Failed to fetch node data: ${response.status}`);
            }

            const nodeData = await response.json();
            console.log('Fetched node data:', nodeData);

            // 2. 開啟 modal
            const modal = document.getElementById('node-settings-modal');
            if (!modal) {
                console.error('node-settings-modal not found');
                return;
            }

            // 3. 填入表單資料
            document.getElementById('node-id').value = nodeData.id || '';
            document.getElementById('node-id').disabled = true; // 編輯模式下 ID 不可修改
            document.getElementById('node-x').value = nodeData.x || 0;
            document.getElementById('node-y').value = nodeData.y || 0;
            document.getElementById('node-theta').value = nodeData.theta || 0;
            document.getElementById('node-type').value = nodeData.type || 'NONE';
            document.getElementById('node-pgv').value = nodeData.pgv || '前';

            // 4. 填入群組資料 (group_1 ~ group_5)
            initializeGroupSettings();
            for (let i = 1; i <= 5; i++) {
                const groupData = nodeData[`group_${i}`];
                if (groupData) {
                    fillGroupData(i, groupData);
                }
            }

            // 5. 設定為編輯模式 (在 modal 上加 data 屬性)
            modal.dataset.mode = 'edit';
            modal.dataset.nodeId = nodeId;

            // 6. 顯示 modal
            modal.classList.add('is-active');
            setupNodeModalEvents();

        } catch (error) {
            console.error('Error loading node data:', error);
            if (window.notify) {
                notify.showErrorMessage('載入節點資料失敗: ' + error.message);
            }
        }
    }

    // 輔助函數：填入群組資料
    function fillGroupData(groupNumber, groupData) {
        const prefix = `group-${groupNumber}`;

        // 如果是字串，嘗試解析為 JSON
        if (typeof groupData === 'string') {
            try {
                groupData = JSON.parse(groupData);
            } catch (e) {
                console.warn(`Failed to parse group_${groupNumber} data:`, e);
                return;
            }
        }

        if (!groupData || typeof groupData !== 'object') {
            console.warn(`Invalid group_${groupNumber} data:`, groupData);
            return;
        }

        console.log(`Filling group ${groupNumber} data:`, groupData);

        // 填入各欄位 - 使用實際資料中的鍵名
        const fieldMap = {
            '可移動點': 'movable-point',
            '動作模式': 'action-mode',
            '速度設定': 'speed',
            '向量角度': 'vector-angle',
            '區域防護': 'area-protection',
            'PGV': 'pgv',
            '加權': 'weight'
        };

        for (const [dataKey, elementKey] of Object.entries(fieldMap)) {
            const element = document.getElementById(`${prefix}-${elementKey}`);
            if (element && groupData[dataKey] !== undefined) {
                element.value = groupData[dataKey];
                console.log(`Set ${prefix}-${elementKey} = ${groupData[dataKey]}`);
            }
        }
    }

    // === 獨立節點檢視控制 ===
    // CT 和 KUKA 各有獨立的 toggle 按鈕

    function toggleCtNodes() {
        const currentState = mapStore.getState();
        const newState = !currentState.showCtNodes;

        // 更新 store（這會觸發 handler 重繪地圖）
        mapStore.setState({ showCtNodes: newState });

        // 更新按鈕視覺狀態
        const toggleBtn = document.getElementById('ct-nodes-toggle');
        if (toggleBtn) {
            toggleBtn.setAttribute('data-show', newState ? 'true' : 'false');
        }

        console.log(`CT 節點: ${newState ? '顯示' : '隱藏'}`);

        if (window.notify) {
            window.notify.info(`CT 節點: ${newState ? '顯示' : '隱藏'}`);
        }
    }

    function toggleKukaNodes() {
        const currentState = mapStore.getState();
        const newState = !currentState.showKukaNodes;

        // 更新 store（這會觸發 handler 重繪地圖）
        mapStore.setState({ showKukaNodes: newState });

        // 更新按鈕視覺狀態
        const toggleBtn = document.getElementById('kuka-nodes-toggle');
        if (toggleBtn) {
            toggleBtn.setAttribute('data-show', newState ? 'true' : 'false');
        }

        console.log(`KUKA 節點: ${newState ? '顯示' : '隱藏'}`);

        if (window.notify) {
            window.notify.info(`KUKA 節點: ${newState ? '顯示' : '隱藏'}`);
        }
    }

    // === 節點切換控制初始化 ===
    function initializeNodeToggleControl() {
        const currentState = mapStore.getState();

        // 初始化 CT 節點切換按鈕
        const ctToggleBtn = document.getElementById('ct-nodes-toggle');
        if (ctToggleBtn) {
            ctToggleBtn.addEventListener('click', toggleCtNodes);
            ctToggleBtn.setAttribute('data-show', currentState.showCtNodes ? 'true' : 'false');
            console.log('CT 節點切換按鈕初始化完成');
        } else {
            console.warn('找不到 CT 節點切換按鈕 #ct-nodes-toggle');
        }

        // 初始化 KUKA 節點切換按鈕
        const kukaToggleBtn = document.getElementById('kuka-nodes-toggle');
        if (kukaToggleBtn) {
            kukaToggleBtn.addEventListener('click', toggleKukaNodes);
            kukaToggleBtn.setAttribute('data-show', currentState.showKukaNodes ? 'true' : 'false');
            console.log('KUKA 節點切換按鈕初始化完成');
        } else {
            console.warn('找不到 KUKA 節點切換按鈕 #kuka-nodes-toggle');
        }

        // 初始化圖例按鈕
        const legendToggleBtn = document.getElementById('map-tool-legend');
        if (legendToggleBtn) {
            legendToggleBtn.addEventListener('click', showLegend);
            legendToggleBtn.setAttribute('data-show', 'false');  // 初始為隱藏
            console.log('圖例切換按鈕初始化完成');
        } else {
            console.warn('找不到圖例切換按鈕 #map-tool-legend');
        }
    }

    // 重設節點表單
    function resetNodeForm() {
        const form = document.getElementById('node-settings-form');
        if (form) {
            form.reset();
            // 設置預設值
            document.getElementById('node-id').value = '';
            document.getElementById('node-x').value = '0.0';
            document.getElementById('node-y').value = '0.0';
            document.getElementById('node-theta').value = '0.0';
            document.getElementById('node-type').value = 'NONE';
            document.getElementById('node-pgv').value = 'FRONT';
        }
    }

    // 初始化群組設定
    function initializeGroupSettings() {
        const groupSettings = document.getElementById('group-settings');
        if (!groupSettings) return;

        // 清空現有內容
        groupSettings.innerHTML = '';

        // 創建5個群組的設定面板，預設顯示群組1
        for (let i = 1; i <= 5; i++) {
            const groupPanel = createGroupPanel(i);
            groupPanel.style.display = i === 1 ? 'block' : 'none';
            groupSettings.appendChild(groupPanel);
        }
    }

    // 創建群組設定面板
    function createGroupPanel(groupNumber) {
        const panel = document.createElement('div');
        panel.id = `group-${groupNumber}-panel`;
        panel.innerHTML = `
            <div class="field is-grouped">
                <div class="control">
                    <label class="label">可移動點</label>
                    <input class="input" type="number" id="group-${groupNumber}-movable" step="0.1" value="0.0">
                </div>
                <div class="control">
                    <label class="label">動作模式</label>
                    <div class="select">
                        <select id="group-${groupNumber}-action">
                            <option value="向量">向量</option>
                            <option value="開門">開門</option>
                            <option value="關門">關門</option>
                        </select>
                    </div>
                </div>
            </div>
            <div class="field is-grouped">
                <div class="control">
                    <label class="label">速度設定</label>
                    <input class="input" type="number" id="group-${groupNumber}-speed" step="0.1" value="1.0">
                </div>
                <div class="control">
                    <label class="label">向量角度</label>
                    <input class="input" type="number" id="group-${groupNumber}-angle" step="0.1" value="0.0">
                </div>
            </div>
            <div class="field">
                <label class="label">區域防護</label>
                <input class="input" type="number" id="group-${groupNumber}-protection" step="0.1" value="0.5">
            </div>
        `;
        return panel;
    }

    // 設定節點模態視窗事件
    function setupNodeModalEvents() {
        const modal = document.getElementById('node-settings-modal');
        if (!modal) return;

        // 關閉模態視窗
        const closeButtons = modal.querySelectorAll('.delete, #close-node-modal, #cancel-node');
        closeButtons.forEach(button => {
            button.addEventListener('click', () => {
                modal.classList.remove('is-active');
            });
        });

        // 儲存節點
        const saveButton = modal.querySelector('#save-node');
        if (saveButton) {
            saveButton.addEventListener('click', saveNodeData);
        }

        // 模態背景點擊關閉
        const background = modal.querySelector('.modal-background');
        if (background) {
            background.addEventListener('click', () => {
                modal.classList.remove('is-active');
            });
        }
    }

    // 儲存節點資料
    async function saveNodeData() {
        console.log('saveNodeData called');

        // 獲取 modal 和模式
        const modal = document.getElementById('node-settings-modal');
        const mode = modal?.dataset.mode || 'create'; // 'create' 或 'edit'
        const editNodeId = modal?.dataset.nodeId;

        try {
            // 收集表單資料
            const nodeId = document.getElementById('node-id').value.trim();
            const xValue = document.getElementById('node-x').value;
            const yValue = document.getElementById('node-y').value;
            const thetaValue = document.getElementById('node-theta').value;

            const nodeData = {
                x: xValue ? parseFloat(xValue) : 0.0,
                y: yValue ? parseFloat(yValue) : 0.0,
                theta: thetaValue ? parseFloat(thetaValue) : 0.0,
                type: document.getElementById('node-type').value,
                pgv: document.getElementById('node-pgv').value,
                group_1: collectGroupData(1),
                group_2: collectGroupData(2),
                group_3: collectGroupData(3),
                group_4: collectGroupData(4),
                group_5: collectGroupData(5)
            };

            // 如果是創建模式且提供了節點ID，將其加入資料中
            if (mode === 'create' && nodeId && !isNaN(parseInt(nodeId))) {
                nodeData.id = parseInt(nodeId);
                console.log(`使用指定的節點ID: ${nodeId}`);
            } else if (mode === 'create' && nodeId) {
                console.log(`⚠️ 節點ID "${nodeId}" 不是有效的數字，系統將自動分配ID`);
            }

            // 驗證數值是否有效
            if (isNaN(nodeData.x) || isNaN(nodeData.y) || isNaN(nodeData.theta)) {
                throw new Error('座標或角度必須為有效的數值');
            }

            console.log(`Node data to ${mode}:`, nodeData);

            // 根據模式選擇 API 端點和方法
            let response;
            if (mode === 'edit' && editNodeId) {
                // 編輯模式：使用 PUT
                response = await fetch(`/path-nodes/api/path-nodes/${editNodeId}`, {
                    method: 'PUT',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(nodeData)
                });
            } else {
                // 創建模式：使用 POST
                response = await fetch('/path-nodes/api/path-nodes', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(nodeData)
                });
            }

            if (response.ok) {
                const result = await response.json();
                console.log(`Node ${mode}d successfully:`, result);

                // 關閉模態視窗
                modal.classList.remove('is-active');

                // 清除編輯模式標記
                delete modal.dataset.mode;
                delete modal.dataset.nodeId;

                // 重新啟用 ID 欄位
                const nodeIdInput = document.getElementById('node-id');
                if (nodeIdInput) {
                    nodeIdInput.disabled = false;
                }

                // 重新載入地圖節點和節點列表
                reloadMapNodes();
                setTimeout(() => {
                    loadNodesList();
                }, 100);

                // 顯示成功訊息
                if (window.notify) {
                    notify.showNotifyMessage(
                        mode === 'edit' ? '節點更新成功!' : '節點創建成功!',
                        'is-success'
                    );
                }
            } else {
                const error = await response.text();
                throw new Error(`HTTP ${response.status}: ${error}`);
            }

        } catch (error) {
            console.error('Error saving node:', error);
            if (window.notify) {
                notify.showErrorMessage('儲存節點失敗: ' + error.message);
            }
        }
    }

    // 收集群組資料
    function collectGroupData(groupNumber) {
        const movable = document.getElementById(`group-${groupNumber}-movable`);
        const action = document.getElementById(`group-${groupNumber}-action`);
        const speed = document.getElementById(`group-${groupNumber}-speed`);
        const angle = document.getElementById(`group-${groupNumber}-angle`);
        const protection = document.getElementById(`group-${groupNumber}-protection`);

        // 檢查所有欄位是否都有值
        if (!movable || !action || !speed || !angle || !protection) {
            return null;
        }

        return {
            "可移動點": parseFloat(movable.value) || 0.0,
            "動作模式": action.value || "向量",
            "速度設定": parseFloat(speed.value) || 1.0,
            "向量角度": parseFloat(angle.value) || 0.0,
            "區域防護": parseFloat(protection.value) || 0.5
        };
    }

    // 更新 popup 位置
    function updatePopupPosition() {
        if (!currentPopupLatLng || !currentPopup || !map) return;
        const point = map.latLngToContainerPoint(currentPopupLatLng);
        currentPopup.style.left = `${point.x + 10}px`;
        currentPopup.style.top = `${point.y - 10}px`;
    }

    // ========== LabVIEW 差異預覽 Modal 處理 ==========

    let currentLabVIEWData = null; // 儲存當前的 LabVIEW 資料用於執行

    function showLabVIEWDiffModal(diffData, labViewData) {
        currentLabVIEWData = labViewData;

        // 更新計數
        document.getElementById('new-count').textContent = diffData.new_nodes.length;
        document.getElementById('modified-count').textContent = diffData.modified_nodes.length;
        document.getElementById('deleted-count').textContent = diffData.deleted_nodes.length;

        // 檢查是否有差異
        const totalChanges = diffData.new_nodes.length +
                           diffData.modified_nodes.length +
                           diffData.deleted_nodes.length;

        // 更新 Modal 標題
        const modalTitle = document.querySelector('#labview-diff-modal .modal-card-title');
        if (totalChanges === 0) {
            modalTitle.textContent = 'LabVIEW 匯入預覽 - ✅ 無差異';
        } else {
            modalTitle.textContent = `LabVIEW 匯入預覽 - 共 ${totalChanges} 項變更`;
        }

        // 填充三個列表
        populateNewNodesList(diffData.new_nodes);
        populateModifiedNodesList(diffData.modified_nodes);
        populateDeletedNodesList(diffData.deleted_nodes);

        // 顯示 Modal
        document.getElementById('labview-diff-modal').classList.add('is-active');

        // 綁定事件處理器
        setupDiffModalEvents();
    }

    function populateNewNodesList(newNodes) {
        const tbody = document.getElementById('new-nodes-list');
        tbody.innerHTML = '';

        newNodes.forEach(node => {
            const tr = document.createElement('tr');
            tr.innerHTML = `
                <td><input type="checkbox" class="node-checkbox" data-node-id="${node.id}" checked></td>
                <td>${node.id}</td>
                <td>${node.x_mm != null ? node.x_mm.toFixed(1) : 'N/A'}</td>
                <td>${node.y_mm != null ? node.y_mm.toFixed(1) : 'N/A'}</td>
                <td>${node.x != null ? node.x.toFixed(1) : 'N/A'}</td>
                <td>${node.y != null ? node.y.toFixed(1) : 'N/A'}</td>
            `;
            tbody.appendChild(tr);
        });
    }

    function populateModifiedNodesList(modifiedNodes) {
        const tbody = document.getElementById('modified-nodes-list');
        tbody.innerHTML = '';

        modifiedNodes.forEach(item => {
            // 檢查哪些欄位有變化
            const changes = [];
            if (item.old.x_mm !== item.new.x_mm) changes.push({ field: 'X (mm)', old: item.old.x_mm, new: item.new.x_mm });
            if (item.old.y_mm !== item.new.y_mm) changes.push({ field: 'Y (mm)', old: item.old.y_mm, new: item.new.y_mm });
            if (item.old.x !== item.new.x) changes.push({ field: 'X (px)', old: item.old.x, new: item.new.x });
            if (item.old.y !== item.new.y) changes.push({ field: 'Y (px)', old: item.old.y, new: item.new.y });

            changes.forEach((change, index) => {
                const tr = document.createElement('tr');
                tr.innerHTML = `
                    <td>${index === 0 ? `<input type="checkbox" class="node-checkbox" data-node-id="${item.id}" checked>` : ''}</td>
                    <td>${index === 0 ? item.id : ''}</td>
                    <td>${change.field}</td>
                    <td class="diff-old-value">${change.old != null ? change.old.toFixed(2) : 'N/A'}</td>
                    <td>→</td>
                    <td class="diff-new-value">${change.new != null ? change.new.toFixed(2) : 'N/A'}</td>
                `;
                tbody.appendChild(tr);
            });
        });
    }

    function populateDeletedNodesList(deletedNodes) {
        const tbody = document.getElementById('deleted-nodes-list');
        tbody.innerHTML = '';

        deletedNodes.forEach(node => {
            const tr = document.createElement('tr');
            tr.innerHTML = `
                <td><input type="checkbox" class="node-checkbox" data-node-id="${node.id}"></td>
                <td>${node.id}</td>
                <td>${node.name || 'N/A'}</td>
                <td>${node.x_mm != null ? node.x_mm.toFixed(1) : 'N/A'}</td>
                <td>${node.y_mm != null ? node.y_mm.toFixed(1) : 'N/A'}</td>
                <td>${node.description || 'N/A'}</td>
            `;
            tbody.appendChild(tr);
        });
    }

    function setupDiffModalEvents() {
        // 標籤頁切換
        document.querySelectorAll('#diff-tabs li').forEach(tab => {
            tab.onclick = () => {
                const tabName = tab.dataset.tab;

                // 更新標籤狀態
                document.querySelectorAll('#diff-tabs li').forEach(t => t.classList.remove('is-active'));
                tab.classList.add('is-active');

                // 顯示對應面板
                document.querySelectorAll('.diff-panel').forEach(panel => panel.style.display = 'none');
                document.getElementById(`${tabName}-nodes-panel`).style.display = 'block';
            };
        });

        // 全選/全不選
        setupCheckboxHandlers('new');
        setupCheckboxHandlers('modified');
        setupCheckboxHandlers('deleted');

        // 關閉按鈕
        document.getElementById('close-diff-modal').onclick = closeDiffModal;
        document.getElementById('cancel-diff').onclick = closeDiffModal;

        // 執行匯入按鈕
        document.getElementById('execute-import').onclick = executeLabVIEWImport;
    }

    function setupCheckboxHandlers(type) {
        // 全選按鈕
        document.getElementById(`select-all-${type}`).onclick = () => {
            document.querySelectorAll(`#${type}-nodes-panel .node-checkbox`).forEach(cb => cb.checked = true);
        };

        // 全不選按鈕
        document.getElementById(`deselect-all-${type}`).onclick = () => {
            document.querySelectorAll(`#${type}-nodes-panel .node-checkbox`).forEach(cb => cb.checked = false);
        };

        // 全選 checkbox
        document.getElementById(`check-all-${type}`).onchange = (e) => {
            document.querySelectorAll(`#${type}-nodes-panel .node-checkbox`).forEach(cb => cb.checked = e.target.checked);
        };
    }

    function closeDiffModal() {
        document.getElementById('labview-diff-modal').classList.remove('is-active');
        currentLabVIEWData = null;
    }

    async function executeLabVIEWImport() {
        // 收集用戶選擇
        const selectedNew = [];
        const selectedModified = [];
        const selectedDeleted = [];

        document.querySelectorAll('#new-nodes-panel .node-checkbox:checked').forEach(cb => {
            selectedNew.push(parseInt(cb.dataset.nodeId));
        });

        document.querySelectorAll('#modified-nodes-panel .node-checkbox:checked').forEach(cb => {
            selectedModified.push(parseInt(cb.dataset.nodeId));
        });

        document.querySelectorAll('#deleted-nodes-panel .node-checkbox:checked').forEach(cb => {
            selectedDeleted.push(parseInt(cb.dataset.nodeId));
        });

        // 保存 labViewData（關閉 Modal 前先保存，因為 closeDiffModal 會清空 currentLabVIEWData）
        const labViewDataToSend = currentLabVIEWData;

        // 關閉 Modal
        closeDiffModal();

        // 顯示處理中訊息
        notify.showNotifyMessage(`正在執行匯入操作...`, 'is-info');

        try {
            const response = await fetch('/path-nodes/execute-labview-import', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    labview_data: labViewDataToSend,
                    selected_operations: {
                        add_node_ids: selectedNew,
                        update_node_ids: selectedModified,
                        delete_node_ids: selectedDeleted
                    }
                })
            });

            if (response.ok) {
                const result = await response.json();
                notify.showNotifyMessage(result.message, 'is-success');

                // 重新載入節點列表和地圖
                loadNodesList();
                reloadMapNodes();
            } else {
                const error = await response.json();
                console.error('Execute error:', error);
                notify.showErrorMessage(`執行失敗: ${error.detail}`);
            }
        } catch (error) {
            console.error('Execute error:', error);
            notify.showErrorMessage('執行匯入時發生錯誤');
        }
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
        viewNodeDetails,
        createTask,
        createRack,
        createCarrier,
        createAgv,
        createNode,
        editNode,
        importLabVIEWPaths,
        showLabVIEWDiffModal,  // 導出 Modal 顯示函數供測試使用
        // 獨立節點切換控制
        toggleCtNodes,
        toggleKukaNodes,
        initializeNodeToggleControl
    };
})();

// 全域暴露以供 HTML onclick 使用
window.mapInteraction = mapInteraction;
