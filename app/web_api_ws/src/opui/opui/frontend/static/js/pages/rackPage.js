// 料架頁面模組
import { userStore, operationStore, dataStore } from '../store.js';
import { socketAPI } from '../api.js';
import { notify } from '../notify.js';

export const rackPage = (() => {
    // 頁面狀態
    let isInitialized = false;
    let boundEvents = new Set();

    /**
     * 頁面初始化
     */
    function setup() {
        if (isInitialized) {
            //console.log('⚠️ Rack 頁面已初始化，跳過重複初始化');
            return;
        }

        //console.log('🚀 Rack 頁面初始化');

        // 綁定事件
        bindEvents();

        // 設置 Store 監聽器
        setupStoreListeners();

        // 初始化 UI
        initializeUI();

        isInitialized = true;
        //console.log('✅ Rack 頁面初始化完成');
    }

    /**
     * 綁定頁面事件
     */
    function bindEvents() {
        // 料架管理事件
        bindRackManagementEvents();

        // 料架選擇事件
        bindRackSelectionEvents();

        // 模態框事件
        bindModalEvents();
    }

    /**
     * 綁定料架管理事件
     */
    function bindRackManagementEvents() {
        if (boundEvents.has('rackManagement')) return;

        // 綁定新增料架按鈕
        document.querySelectorAll('.rack-add-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                // 使用 currentTarget 而不是 target，確保獲取到按鈕元素
                const button = e.currentTarget;
                const side = button.getAttribute('data-side');
                const input = document.querySelector(`.rack-add[data-side="${side}"]`);

                //console.log(`➕ 點擊新增料架按鈕: ${side} 側`);
                //console.log(`📝 輸入框值:`, input?.value);

                if (input && input.value.trim()) {
                    const rackId = input.value.trim();
                    handleAddRack(side, rackId);
                    input.value = ''; // 清空輸入框
                } else {
                    notify.showErrorMessage('請輸入料架編號');
                }
            });
        });

        // 綁定新增料架輸入框的 Enter 鍵
        document.querySelectorAll('.rack-add').forEach(input => {
            input.addEventListener('keypress', (e) => {
                if (e.key === 'Enter') {
                    const side = e.target.getAttribute('data-side');
                    const rackId = e.target.value.trim();

                    if (rackId) {
                        handleAddRack(side, rackId);
                        e.target.value = ''; // 清空輸入框
                    } else {
                        notify.showErrorMessage('請輸入料架編號');
                    }
                }
            });
        });

        // 綁定刪除料架事件（使用事件委派）
        document.addEventListener('click', (e) => {
            if (e.target.classList.contains('is-delete') && e.target.hasAttribute('data-rack-id')) {
                const rackId = e.target.getAttribute('data-rack-id');
                //console.log(`🗑️ 點擊刪除料架: 料架ID: ${rackId}`);
                handleDeleteRack(rackId);
            }
        });

        boundEvents.add('rackManagement');
    }

    /**
     * 綁定料架選擇事件
     */
    function bindRackSelectionEvents() {
        if (boundEvents.has('rackSelection')) return;

        // 使用事件委託處理動態生成的料架按鈕
        document.addEventListener('click', (e) => {
            // 處理料架按鈕
            if (e.target.classList.contains('rack-btn')) {
                const rackId = parseInt(e.target.getAttribute('data-rackid'));
                const side = e.target.getAttribute('data-side');

                if (!side || isNaN(rackId)) return;

                const operationState = operationStore.getState();
                const productIndex = operationState[side].productSelected;

                // 更新料架選擇
                const newProducts = [...operationState[side].products];
                newProducts[productIndex] = { ...newProducts[productIndex], rackId: rackId };

                const newState = {
                    ...operationState,
                    [side]: {
                        ...operationState[side],
                        products: newProducts
                    }
                };
                operationStore.setState(newState);

                // 同步到後端
                const allStates = socketAPI.getAllStates();
                socketAPI.updateClient(allStates);
            }

            // 處理料架刪除按鈕
            if (e.target.classList.contains('rack-delete-btn')) {
                const rackId = parseInt(e.target.getAttribute('data-rackid'));

                if (isNaN(rackId)) return;

                showDeleteRackModal(rackId);
            }
        });

        boundEvents.add('rackSelection');
    }

    /**
     * 綁定模態框事件
     */
    function bindModalEvents() {
        if (boundEvents.has('modalEvents')) return;

        // 綁定刪除確認模態框事件
        bindDeleteModalEvents();

        boundEvents.add('modalEvents');
    }

    /**
     * 設置 Store 監聽器
     */
    function setupStoreListeners() {
        // 監聽使用者狀態變更
        userStore.on('change', handleUserChange);
        
        // 監聽操作狀態變更
        operationStore.on('change', handleOperationChange);
        
        // 監聽資料變更
        dataStore.on('change', handleDataChange);
    }

    /**
     * 處理使用者狀態變更
     */
    function handleUserChange() {
        // 料架頁面不需要處理使用者狀態變更
    }

    /**
     * 處理操作狀態變更
     */
    function handleOperationChange() {
        // 料架頁面不需要處理操作狀態變更
    }

    /**
     * 處理資料狀態變更
     */
    function handleDataChange(newState) {
        updateParkingAreas(newState.parking);
    }

    /**
     * 初始化 UI
     */
    function initializeUI() {
        const dataState = dataStore.getState();
        updateParkingAreas(dataState.parking);
    }

    /**
     * 更新停車區域顯示
     */
    function updateParkingAreas(parkingData) {
        if (!parkingData) return;

        ['left', 'right'].forEach(side => {
            const container = document.querySelector(`.parking-area[data-side="${side}"]`);
            if (!container) return;

            const racks = parkingData[side] || [];
            
            container.innerHTML = '';

            racks.forEach(rack => {
                const control = document.createElement('div');
                control.className = 'control';

                const tags = document.createElement('div');
                tags.className = 'tags has-addons';

                const nameTag = document.createElement('span');
                nameTag.className = 'tag is-large is-primary';
                nameTag.textContent = rack.name;

                const deleteTag = document.createElement('span');
                deleteTag.className = 'tag is-large is-delete';
                deleteTag.setAttribute('data-rack-id', rack.id);
                deleteTag.setAttribute('data-side', side);

                tags.appendChild(nameTag);
                tags.appendChild(deleteTag);
                control.appendChild(tags);
                container.appendChild(control);
            });
        });
    }

    /**
     * 處理新增料架
     */
    function handleAddRack(side, rackId) {
        //console.log(`➕ 處理新增料架: ${side} 側, 料架ID: ${rackId}`);

        try {
            // 驗證料架名稱（不轉換為數字，保持原始字串）
            if (!rackId || rackId.trim() === '') {
                console.warn(`⚠️ 請輸入料架名稱: ${rackId}`);
                notify.showErrorMessage('請輸入料架名稱');
                return;
            }

            const rackName = rackId.trim();
            //console.log(`✅ 料架名稱: ${rackName}`);

            // 檢查是否已存在（根據料架名稱檢查）
            const dataState = dataStore.getState();
            const existingRacks = dataState.parking?.[side] || [];

            if (existingRacks.some(rack => rack.name === rackName)) {
                notify.showErrorMessage('料架已存在於此停車位');
                return;
            }

            // 發送料架分配請求（發送料架名稱，讓後端根據 name 查找對應的 rack）
            socketAPI.addRack(side, rackName)
                .then(() => {
                    notify.showNotifyMessage(`已將料架 ${rackName} 分配到 ${side} 側`);
                })
                .catch(error => {
                    console.error('料架分配失敗:', error);

                    // 如果是料架不存在的錯誤，提供更清楚的指引
                    if (error.message && error.message.includes('不存在於系統中')) {
                        notify.showErrorMessage(`料架 ${rackName} 不存在。請先在料架管理系統中新增此料架，然後再進行分配。`);
                    } else {
                        // 顯示後端返回的具體錯誤訊息
                        notify.showErrorMessage(error.message || '料架分配失敗');
                    }
                });

        } catch (error) {
            console.error('新增料架失敗:', error);
            notify.showErrorMessage('新增料架失敗');
        }
    }

    /**
     * 處理刪除料架
     */
    function handleDeleteRack(rackId) {
        try {
            // 直接刪除，不需要確認彈窗
            socketAPI.deleteRack(parseInt(rackId))
                .then(() => {
                    notify.showNotifyMessage(`已刪除料架 ${rackId}`);
                })
                .catch(error => {
                    console.error('刪除料架失敗:', error);
                    notify.showErrorMessage('刪除料架失敗');
                });
        } catch (error) {
            console.error('刪除料架失敗:', error);
            notify.showErrorMessage('刪除料架失敗');
        }
    }

    /**
     * 顯示刪除料架模態框
     */
    function showDeleteRackModal(rackId) {
        const modal = document.getElementById('deleteModal');
        if (modal) {
            // 設置刪除項目名稱
            const nameElement = document.getElementById('deleteEntityName') ||
                               document.getElementById('deleteItemName');
            if (nameElement) {
                nameElement.textContent = `料架 ${rackId}`;
            }

            // 設置確認按鈕事件
            const form = document.getElementById('deleteForm');
            if (form) {
                // 移除舊的事件監聽器
                const newForm = form.cloneNode(true);
                form.parentNode.replaceChild(newForm, form);

                // 添加新的事件監聽器
                newForm.addEventListener('submit', (e) => {
                    e.preventDefault();
                    handleDeleteRack(rackId);
                    hideDeleteModal();
                });
            }

            // 顯示模態框
            modal.classList.add('is-active');
        } else {
            // 備用方案：使用 confirm
            if (confirm(`確定要刪除料架 ${rackId} 嗎？`)) {
                handleDeleteRack(rackId);
            }
        }
    }

    /**
     * 隱藏刪除模態框
     */
    function hideDeleteModal() {
        const modal = document.getElementById('deleteModal');
        if (modal) {
            modal.classList.remove('is-active');
        }
    }

    /**
     * 綁定刪除模態框事件
     */
    function bindDeleteModalEvents() {
        // 綁定模態框關閉按鈕
        document.querySelectorAll('#deleteModal .delete, #deleteModal .modal-background').forEach(element => {
            element.addEventListener('click', hideDeleteModal);
        });

        // 綁定取消按鈕
        const cancelBtn = document.querySelector('#deleteModal .button:not(.is-danger)');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', hideDeleteModal);
        }
    }

    /**
     * 頁面清理
     */
    function cleanup() {
        // 移除事件監聽器
        userStore.off('change', handleUserChange);
        operationStore.off('change', handleOperationChange);
        dataStore.off('change', handleDataChange);
        
        // 清除綁定標記
        boundEvents.clear();
        
        isInitialized = false;
        //console.log('🧹 Rack 頁面已清理');
    }

    // 公開介面
    return {
        setup,
        cleanup
    };
})();
