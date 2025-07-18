// settingPage.js - Setting 頁面 JavaScript 模組
// 參考 AGVCUI 架構設計，專門處理設定頁面的配置邏輯

import { userStore, operationStore, dataStore, uiStore } from '../store.js';
import { socketAPI } from '../api.js';
import { notify } from '../notify.js';

export const settingPage = (() => {
    // 頁面狀態
    let isInitialized = false;
    let boundEvents = new Set();

    /**
     * 頁面初始化
     */
    function setup() {
        if (isInitialized) {
            //console.log('⚠️ Setting 頁面已初始化，跳過重複初始化');
            return;
        }

        //console.log('🚀 Setting 頁面初始化');

        // 綁定事件
        bindEvents();

        // 設置 Store 監聽器
        setupStoreListeners();

        // 初始化 UI
        initializeUI();

        isInitialized = true;
        //console.log('✅ Setting 頁面初始化完成');
    }

    /**
     * 綁定頁面事件
     */
    function bindEvents() {
        // 機台按鈕事件
        bindMachineButtons();

        // 產品輸入事件
        bindProductInputs();

        // 恢復原廠設定事件
        bindFactoryResetButton();

        // 模態框事件
        bindModalEvents();
    }

    /**
     * 綁定機台按鈕事件
     */
    function bindMachineButtons() {
        if (boundEvents.has('machineButtons')) return;

        document.querySelectorAll('.machine-btn').forEach(btn => {
            const machineId = parseInt(btn.getAttribute('data-machine-id'));
            
            if (!isNaN(machineId)) {
                btn.addEventListener('click', () => {
                    // 更新機台 ID
                    const currentState = userStore.getState();
                    userStore.setState({
                        ...currentState,
                        machineId
                    });

                    // 發送更新請求
                    const allStates = socketAPI.getAllStates();
                    socketAPI.updateClient(allStates);
                });
            }
        });

        boundEvents.add('machineButtons');
    }

    /**
     * 綁定產品輸入事件
     */
    function bindProductInputs() {
        if (boundEvents.has('productInputs')) return;

        document.querySelectorAll('.product-input').forEach(input => {
            input.addEventListener('input', (e) => {
                const side = e.target.getAttribute('data-side');
                const index = parseInt(e.target.getAttribute('data-index'));
                let value = e.target.value;

                // 允許小寫字母輸入，然後轉換為大寫，並過濾其他特殊字符
                value = value.replace(/[^A-Za-z0-9_]/g, '').toUpperCase();

                // 更新輸入框值（保持游標位置）
                if (e.target.value !== value) {
                    const cursorPos = e.target.selectionStart;
                    e.target.value = value;
                    e.target.setSelectionRange(cursorPos, cursorPos);
                }

                // 更新狀態
                updateProductName(side, index, value);

                // 更新驗證狀態
                updateProductValidation(e.target, value);
            });

            // 綁定失去焦點事件，保存資料
            input.addEventListener('blur', () => {
                const allStates = socketAPI.getAllStates();
                socketAPI.updateClient(allStates);
            });
        });

        boundEvents.add('productInputs');
    }



    /**
     * 綁定恢復原廠設定按鈕
     */
    function bindFactoryResetButton() {
        if (boundEvents.has('factoryReset')) return;

        const resetBtn = document.querySelector('#factory-restore');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                if (confirm('確定要恢復原廠設定嗎？這將清除所有自訂設定。')) {
                    executeFactoryReset();
                }
            });
        }

        boundEvents.add('factoryReset');
    }



    /**
     * 綁定模態框事件
     */
    function bindModalEvents() {
        if (boundEvents.has('modalEvents')) return;

        // 綁定恢復原廠模態框事件
        bindFactoryRestoreModal();

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
    function handleUserChange(newState) {
        updateMachineSelection(newState.machineId);
    }

    /**
     * 處理操作狀態變更
     */
    function handleOperationChange(newState) {
        updateProductInputs(newState);
    }

    /**
     * 處理資料變更
     */
    function handleDataChange(newState) {
        updateMachineButtons(newState.machines);
        updateProductValidationAll(newState.products);
    }

    /**
     * 初始化 UI
     */
    function initializeUI() {
        const userState = userStore.getState();
        const operationState = operationStore.getState();
        const dataState = dataStore.getState();

        updateMachineSelection(userState.machineId);
        updateMachineButtons(dataState.machines);
        updateProductInputs(operationState);
        updateProductValidationAll();
    }

    // ===== UI 更新方法 =====

    /**
     * 更新機台選擇狀態
     */
    function updateMachineSelection(selectedMachineId) {
        document.querySelectorAll('.machine-btn').forEach(btn => {
            const machineId = parseInt(btn.getAttribute('data-machine-id'));
            btn.classList.toggle('is-primary', machineId === selectedMachineId);
            btn.classList.toggle('is-light', machineId !== selectedMachineId);
        });

        // 更新導航欄機台編號顯示 已由 index.js中更新 navbar 中的機台編號顯示
        // updateMachineDisplay(selectedMachineId);
    }

    /**
     * 更新機台按鈕狀態
     */
    function updateMachineButtons(machines) {
        if (!machines) return;

        document.querySelectorAll('.machine-btn').forEach(btn => {
            const machineId = parseInt(btn.getAttribute('data-machine-id'));
            const machine = machines.find(m => m.id === machineId);

            if (machine) {
                btn.disabled = !machine.enable;
                btn.classList.toggle('is-disabled', !machine.enable);
            }
        });
    }

    /**
     * 更新產品輸入框
     */
    function updateProductInputs(state) {
        ['left', 'right'].forEach(side => {
            const sideData = state[side];
            if (!sideData || !sideData.products) return;

            sideData.products.forEach((product, index) => {
                const input = document.querySelector(`input[data-side="${side}"][data-index="${index}"].product-input`);
                if (input && document.activeElement !== input) {
                    // 只在用戶沒有焦點時更新，避免干擾輸入
                    input.value = product.name || '';
                }
            });
        });
    }

    /**
     * 更新單個產品驗證狀態
     */
    function updateProductValidation(input, productName) {
        const dataState = dataStore.getState();
        const availableProducts = dataState.products || [];
        const isValid = !productName || availableProducts.some(p => p.name === productName);

        // 更新輸入框樣式
        input.classList.toggle('is-danger', !isValid);
        input.classList.toggle('is-success', isValid && productName);

        // 更新檢查圖示
        const checkIcon = input.parentElement.querySelector('.product-checked');
        if (checkIcon) {
            checkIcon.classList.toggle('is-visible', isValid && productName);
        }

        // 更新幫助文字
        const helpText = input.parentElement.querySelector('.help');
        if (helpText) {
            if (!isValid && productName) {
                helpText.textContent = '產品名稱不存在';
                helpText.classList.add('is-danger');
            } else {
                helpText.textContent = '';
                helpText.classList.remove('is-danger');
            }
        }
    }

    /**
     * 更新所有產品驗證狀態
     */
    function updateProductValidationAll() {
        document.querySelectorAll('.product-input').forEach(input => {
            const productName = input.value.trim();
            updateProductValidation(input, productName);
        });
    }

    // ===== 業務邏輯方法 =====

    /**
     * 更新產品名稱
     */
    function updateProductName(side, index, name) {
        const currentState = operationStore.getState();
        const products = [...currentState[side].products];
        products[index] = { ...products[index], name };

        operationStore.setState({
            ...currentState,
            [side]: {
                ...currentState[side],
                products
            }
        });
    }



    /**
     * 執行恢復原廠設定
     */
    function executeFactoryReset() {
        try {
            // 清除本機儲存
            localStorage.clear();

            // 重置所有 Store 狀態
            userStore.clear();
            operationStore.clear();
            dataStore.clear();
            uiStore.clear();

            // 顯示成功訊息
            notify.showNotifyMessage('恢復原廠設定完成，頁面將重新載入');

            // 延遲重新載入頁面
            setTimeout(() => {
                window.location.reload();
            }, 1500);

        } catch (error) {
            console.error('恢復原廠設定失敗:', error);
            notify.showErrorMessage('恢復原廠設定失敗');
        }
    }



    /**
     * 綁定恢復原廠模態框事件
     */
    function bindFactoryRestoreModal() {
        const modal = document.getElementById('factory-restore-modal');
        if (!modal) return;

        // 綁定確認按鈕
        const confirmBtn = document.getElementById('factory-restore-confirm');
        if (confirmBtn) {
            confirmBtn.addEventListener('click', () => {
                executeFactoryReset();
                hideFactoryRestoreModal();
            });
        }

        // 綁定取消按鈕
        const cancelBtn = document.getElementById('factory-restore-cancel');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', () => {
                hideFactoryRestoreModal();
            });
        }

        // 綁定關閉按鈕
        const closeBtn = modal.querySelector('.delete');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => {
                hideFactoryRestoreModal();
            });
        }

        // 綁定背景點擊關閉
        const background = modal.querySelector('.modal-background');
        if (background) {
            background.addEventListener('click', () => {
                hideFactoryRestoreModal();
            });
        }
    }

    /**
     * 綁定刪除模態框事件
     */
    function bindDeleteModalEvents() {
        const modal = document.getElementById('deleteModal');
        if (!modal) return;

        // 綁定關閉按鈕
        const closeBtn = modal.querySelector('.delete');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => {
                hideDeleteModal();
            });
        }

        // 綁定取消按鈕
        const cancelBtn = modal.querySelector('button[type="button"]');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', () => {
                hideDeleteModal();
            });
        }

        // 綁定背景點擊關閉
        const background = modal.querySelector('.modal-background');
        if (background) {
            background.addEventListener('click', () => {
                hideDeleteModal();
            });
        }

        // ESC 鍵關閉
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && modal.classList.contains('is-active')) {
                hideDeleteModal();
            }
        });
    }



    /**
     * 隱藏恢復原廠模態框
     */
    function hideFactoryRestoreModal() {
        const modal = document.getElementById('factory-restore-modal');
        if (modal) {
            modal.classList.remove('is-active');
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
        //console.log('🧹 Setting 頁面已清理');
    }

    // 返回公開的 API
    return {
        setup,
        cleanup
    };
})();
