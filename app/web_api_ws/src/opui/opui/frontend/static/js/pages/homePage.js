// homePage.js - Home 頁面 JavaScript 模組
// 參考 AGVCUI 架構設計，專門處理首頁的操作界面邏輯

import { userStore, operationStore, dataStore, tasksStore, uiStore } from '../store.js';
import { socketAPI } from '../api.js';
import { notify } from '../notify.js';

export const homePage = (() => {
    // 頁面狀態
    let isInitialized = false;
    let boundEvents = new Set();

    /**
     * 頁面初始化
     */
    function setup() {
        if (isInitialized) {
            //console.log('⚠️ Home 頁面已初始化，跳過重複初始化');
            return;
        }



        // 綁定事件
        bindEvents();

        // 設置 Store 監聽器
        setupStoreListeners();

        // 初始化 UI
        initializeUI();

        isInitialized = true;

    }

    /**
     * 綁定頁面事件
     */
    function bindEvents() {
        // 產品按鈕事件
        bindProductButtons();

        // 操作按鈕事件
        bindActionButtons();

        // 料架選擇事件
        bindRackButtons();

        // 數量矩陣按鈕事件
        bindNumberButtons();

        // 房間按鈕事件
        bindRoomButtons();

        // 料架選擇區域事件
        bindRackSelectionArea();
        
        // Modal 事件
        bindModalEvents();
    }

    /**
     * 綁定產品按鈕事件
     */
    function bindProductButtons() {
        if (boundEvents.has('productButtons')) return;

        document.querySelectorAll('.product-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-product-side');

                if (side) {
                    const operationState = operationStore.getState();
                    const currentIndex = operationState[side].productSelected;
                    const maxIndex = operationState[side].products.length - 1;
                    const nextIndex = currentIndex >= maxIndex ? 0 : currentIndex + 1;

                    //console.log(`🔄 ${side} 側產品切換: ${currentIndex} → ${nextIndex}`);

                    // 更新 store
                    const newState = {
                        ...operationState,
                        [side]: {
                            ...operationState[side],
                            productSelected: nextIndex
                        }
                    };
                    operationStore.setState(newState);

                    // 立即更新UI顯示
                    updateProductButtons(newState);
                    updateNumberButtons(newState);

                    // 同步到後端
                    socketAPI.updateClient(newState);
                }
            });
        });

        boundEvents.add('productButtons');
    }

    /**
     * 綁定操作按鈕事件
     */
    function bindActionButtons() {
        if (boundEvents.has('actionButtons')) return;

        // 加入料架按鈕
        document.querySelectorAll('[data-add-rack]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.currentTarget.getAttribute('data-add-rack');
                handleAddRack(side);
            });
        });

        // 叫滿車按鈕
        document.querySelectorAll('[data-call-full]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-call-full');
                const buttonText = e.target.textContent.trim();



                if (buttonText === '取消') {
                    handleCancelTask(side);
                } else {
                    handleCallFull(side);
                }
            });
        });

        // 派車按鈕
        document.querySelectorAll('[data-dispatch-full]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-dispatch-full');
                const buttonText = e.target.textContent.trim();



                if (buttonText === '取消') {
                    handleCancelTask(side);
                } else {
                    handleDispatchFull(side);
                }
            });
        });

        boundEvents.add('actionButtons');

    }

    /**
     * 綁定料架選擇事件
     */
    function bindRackButtons() {
        if (boundEvents.has('rackButtons')) return;

        document.addEventListener('click', (e) => {
            if (e.target.classList.contains('rack-btn') && e.target.hasAttribute('data-rackid')) {
                const rackId = parseInt(e.target.getAttribute('data-rackid'));
                const side = e.target.getAttribute('data-side');

                if (!side || isNaN(rackId)) return;

                const currentState = operationStore.getState();
                const productIndex = currentState[side].productSelected;

                // 更新選中的料架
                const products = [...currentState[side].products];
                products[productIndex] = { ...products[productIndex], rackId };

                operationStore.setState({
                    ...currentState,
                    [side]: {
                        ...currentState[side],
                        products
                    }
                });
            }
        });

        boundEvents.add('rackButtons');
    }

    /**
     * 綁定數量矩陣按鈕事件
     */
    function bindNumberButtons() {
        if (boundEvents.has('numberButtons')) return;

        document.querySelectorAll('.matrix-space button').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const matrixSpace = e.target.closest('.matrix-space');
                if (matrixSpace) {
                    const side = matrixSpace.getAttribute('data-side');
                    const number = parseInt(matrixSpace.getAttribute('data-num'));

                    if (side && !isNaN(number)) {
                        handleNumberButtonClick(side, number);
                    }
                }
            });
        });

        boundEvents.add('numberButtons');
    }

    /**
     * 綁定房間按鈕事件
     */
    function bindRoomButtons() {
        if (boundEvents.has('roomButtons')) return;

        document.querySelectorAll('.room-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-side');
                const room = parseInt(e.target.getAttribute('data-room'));

                if (side && !isNaN(room)) {
                    const operationState = operationStore.getState();
                    const sideData = operationState[side];
                    const productIndex = sideData.productSelected || 0;

                    // 🔧 修復：檢查products陣列是否存在
                    if (!sideData.products || sideData.products.length === 0) {
                        console.warn(`⚠️ ${side} 側products陣列為空，無法設定房號`);
                        notify.showErrorMessage('請先選擇產品');
                        return;
                    }

                    // 更新產品房間
                    const newProducts = [...sideData.products];
                    if (newProducts[productIndex]) {
                        newProducts[productIndex] = { ...newProducts[productIndex], room: room };

                        const newState = {
                            ...operationState,
                            [side]: {
                                ...sideData,
                                products: newProducts
                            }
                        };
                        operationStore.setState(newState);

                        // 同步到後端
                        socketAPI.updateClient(newState);

                        notify.showNotifyMessage(`已設定 ${side} 側房號為 ${room}`);
                    } else {
                        console.error(`❌ ${side} 側產品索引 ${productIndex} 不存在`);
                        notify.showErrorMessage('產品選擇錯誤');
                    }
                }
            });
        });

        boundEvents.add('roomButtons');
    }

    /**
     * 綁定料架選擇區域事件
     */
    function bindRackSelectionArea() {
        if (boundEvents.has('rackSelectionArea')) return;

        // 使用事件委託處理動態生成的料架選擇按鈕
        document.addEventListener('click', (e) => {
            // 處理 home 頁面的料架選擇按鈕
            if (e.target.classList.contains('rack-option-btn')) {
                const rackId = parseInt(e.target.getAttribute('data-rackid'));
                const side = e.target.getAttribute('data-side');

                if (!side || isNaN(rackId)) return;

                selectRack(side, rackId);
            }
        });

        boundEvents.add('rackSelectionArea');
    }

    /**
     * 設置 Store 監聽器
     */
    function setupStoreListeners() {
        // 監聽操作狀態變更
        operationStore.on('change', handleOperationChange);
        
        // 監聽任務狀態變更
        tasksStore.on('change', handleTasksChange);
        
        // 監聽資料變更
        dataStore.on('change', handleDataChange);
    }

    /**
     * 處理操作狀態變更
     */
    function handleOperationChange(newState) {
        updateProductButtons(newState);
        updateNumberButtons(newState);
        updateProductSelection(newState);
        updateRackSelection(newState);
        updateRoomButtons(newState);
    }

    /**
     * 處理任務狀態變更
     */
    function handleTasksChange(newState) {

        updateTaskButtons(newState);
        updateTaskStatus(newState);
    }

    /**
     * 處理資料變更
     */
    function handleDataChange(newState) {
        // 更新停車格列表和料架選擇區域
        updateParkingList(newState.parking);

        // 檢查派車任務是否應該自動完成（rack 被取走）
        checkDispatchTaskCompletion(newState.parking);

        // 當資料變更時，也需要更新房間按鈕的啟用狀態
        const operationState = operationStore.getState();
        updateRoomButtons(operationState);
    }

    /**
     * 初始化 UI
     */
    function initializeUI() {
        // 更新初始狀態
        const operationState = operationStore.getState();
        const tasksState = tasksStore.getState();
        const dataState = dataStore.getState();
        const userState = userStore.getState();
        const uiState = uiStore.getState();

        // 調用共用的UI更新函數（定義在index.js中）
        if (typeof updateConnectionStatus === 'function') {
            updateConnectionStatus(userState.isConnected);
        }
        if (typeof updateMachineDisplay === 'function') {
            updateMachineDisplay(userState.machineId);
        }

        // 調用home頁面專用的UI更新函數
        updateProductButtons(operationState);
        updateNumberButtons(operationState);
        updateProductSelection(operationState);
        updateTaskButtons(tasksState);
        updateParkingList(dataState.parking);
        updateRackSelection(operationState);
        updateRoomButtons(operationState);

        // 調用共用的UI更新函數（如果存在）
        if (typeof updateLoadingState === 'function') {
            updateLoadingState(uiState.loading);
        }
        if (typeof updateNotifications === 'function') {
            updateNotifications(uiState.notifications);
        }
    }

    // ===== UI 更新方法 =====

    /**
     * 更新產品按鈕顯示
     */
    function updateProductButtons(operationState) {
        // 從 dataStore 獲取可用產品清單
        const dataState = dataStore.getState();
        const availableProducts = dataState.products || [];

        ['left', 'right'].forEach(side => {
            const btn = document.querySelector(`.product-btn[data-product-side="${side}"]`);
            if (btn && operationState[side]) {
                const selectedIndex = operationState[side].productSelected;
                const operationProduct = operationState[side].products[selectedIndex];

                let productName = '未設定';
                let needsUpdate = false;
                let updatedProduct = { ...operationProduct };

                if (operationProduct?.id) {
                    const fullProduct = availableProducts.find(p => p.id === operationProduct.id);
                    if (fullProduct) {
                        productName = fullProduct.name;
                        // 同步更新產品的完整資訊到operationStore
                        if (operationProduct.name !== fullProduct.name || operationProduct.size !== fullProduct.size) {
                            updatedProduct = {
                                ...operationProduct,
                                name: fullProduct.name,
                                size: fullProduct.size
                            };
                            needsUpdate = true;
                        }
                    } else {
                        productName = operationProduct.name || '未設定';
                    }
                } else if (operationProduct?.name) {
                    productName = operationProduct.name;
                }

                btn.textContent = productName;

                // 如果需要更新產品資訊，同步到store
                if (needsUpdate) {
                    const newProducts = [...operationState[side].products];
                    newProducts[selectedIndex] = updatedProduct;

                    const newState = {
                        ...operationState,
                        [side]: {
                            ...operationState[side],
                            products: newProducts
                        }
                    };

                    // 靜默更新，不觸發後端同步（避免無限循環）
                    operationStore.setState(newState);
                    //console.log(`🔄 已同步 ${side} 側產品 ${selectedIndex} 的完整資訊:`, updatedProduct);
                }
            }
        });
    }

    /**
     * 更新數量選擇矩陣按鈕的顯示/隱藏
     */
    function updateNumberButtons(operationState) {
        // 從 dataStore 獲取可用產品清單以獲取完整的產品資訊
        const dataState = dataStore.getState();
        const availableProducts = dataState.products || [];

        ['left', 'right'].forEach(side => {
            const sideData = operationState[side];
            if (sideData) {
                // 🔧 修復：檢查products陣列是否為空
                if (!sideData.products || sideData.products.length === 0) {
                    console.warn(`⚠️ ${side} 側products陣列為空，嘗試初始化預設產品`);

                    // 從dataStore獲取可用產品來初始化
                    if (availableProducts.length > 0) {
                        const defaultProducts = [];
                        for (let i = 0; i < Math.min(2, availableProducts.length); i++) {
                            const product = availableProducts[i];
                            defaultProducts.push({
                                ...product,
                                count: product.size === 'S' ? 32 : 16,
                                room: 2,
                                rackId: null
                            });
                        }

                        // 更新operationStore
                        const newState = {
                            ...operationState,
                            [side]: {
                                ...sideData,
                                products: defaultProducts
                            }
                        };
                        operationStore.setState(newState);

                        // 使用新的狀態繼續處理
                        const selectedProduct = defaultProducts[sideData.productSelected || 0];
                        const currentCount = selectedProduct?.count || 0;

                        // 繼續處理顯示邏輯
                        updateMatrixButtons(side, selectedProduct, currentCount, availableProducts);
                        return;
                    } else {
                        // 如果沒有可用產品，隱藏所有按鈕
                        hideAllMatrixButtons(side);
                        return;
                    }
                }

                const selectedProduct = sideData.products[sideData.productSelected];
                const currentCount = selectedProduct?.count || 0;

                // 處理矩陣按鈕顯示
                updateMatrixButtons(side, selectedProduct, currentCount, availableProducts);
            }
        });
    }

    /**
     * 隱藏指定側的所有矩陣按鈕
     */
    function hideAllMatrixButtons(side) {
        const matrices = document.querySelectorAll(`.matrix-space[data-side="${side}"]`);
        matrices.forEach(matrix => {
            matrix.classList.add('hidden');
        });
    }

    /**
     * 更新矩陣按鈕的顯示邏輯
     */
    function updateMatrixButtons(side, selectedProduct, currentCount, availableProducts) {
        // 優先從dataStore獲取產品尺寸，確保資訊正確
        let productSize = (selectedProduct?.size || '').toUpperCase();

        // 如果operationStore中沒有size資訊，從dataStore中查找
        if (!productSize && selectedProduct?.id) {
            const fullProduct = availableProducts.find(p => p.id === selectedProduct.id);
            if (fullProduct) {
                productSize = (fullProduct.size || '').toUpperCase();
                //console.log(`🔍 從dataStore獲取 ${side} 側產品尺寸: ${productSize}`);
            }
        }

        // 如果還是沒有尺寸資訊，根據產品名稱推斷（備用方案）
        if (!productSize && selectedProduct?.name) {
            const fullProduct = availableProducts.find(p => p.name === selectedProduct.name);
            if (fullProduct) {
                productSize = (fullProduct.size || '').toUpperCase();
                //console.log(`🔍 根據產品名稱推斷 ${side} 側產品尺寸: ${productSize}`);
            }
        }

        //console.log(`🔢 更新 ${side} 側數量按鈕: 產品尺寸=${productSize}, 當前數量=${currentCount}`);

        // 根據產品尺寸顯示/隱藏矩陣按鈕
        const matrices = document.querySelectorAll(`.matrix-space[data-side="${side}"]`);

        matrices.forEach(matrix => {
            const btnSize = matrix.dataset.size;
            const btnNum = parseInt(matrix.dataset.num);
            const btn = matrix.querySelector('button');
            const shouldHide = btnSize !== productSize;

            matrix.classList.toggle('hidden', shouldHide);

            if (btn) {
                if (btnSize === productSize && btnNum <= currentCount) {
                    btn.classList.add('is-selected', 'is-primary');
                } else {
                    btn.classList.remove('is-selected', 'is-primary');
                }
            }
        });

        // 統計顯示的按鈕數量（用於除錯）
        const visibleMatrices = document.querySelectorAll(`.matrix-space[data-side="${side}"]:not(.hidden)`);
        //console.log(`📊 ${side} 側顯示 ${visibleMatrices.length} 個數量按鈕 (產品尺寸: ${productSize})`);

        // 驗證按鈕數量是否符合規格
        const expectedCount = productSize === 'S' ? 32 : (productSize === 'L' ? 16 : 0);
        if (visibleMatrices.length === expectedCount && productSize) {
            //console.log(`✅ ${side} 側按鈕數量符合規格: ${expectedCount}個`);
        } else if (productSize) {
            console.warn(`⚠️ ${side} 側按鈕數量不符合規格: 期望${expectedCount}個，實際${visibleMatrices.length}個`);
        }
    }

    /**
     * 更新產品選擇 UI（更新數量按鈕的選中狀態）
     */
    function updateProductSelection(state) {
        ['left', 'right'].forEach(side => {
            const sideData = state[side];
            if (!sideData) return;

            // 更新數量按鈕的選中狀態（基於當前選中的產品）
            const selectedProduct = sideData.products[sideData.productSelected];
            if (selectedProduct) {
                // 清除所有數量按鈕的選中狀態
                document.querySelectorAll(`.num-btn[data-side="${side}"]`).forEach(btn => {
                    btn.classList.remove('is-primary');
                });

                // 設置選中的數量按鈕
                if (selectedProduct.count > 0) {
                    const selectedBtn = document.querySelector(`.num-btn[data-side="${side}"][data-num="${selectedProduct.count}"]`);
                    if (selectedBtn) {
                        selectedBtn.classList.add('is-primary');
                    }
                }

                // 更新產品資訊顯示
                updateProductInfo(side, selectedProduct);
            }
        });
    }

    /**
     * 更新產品資訊顯示
     */
    function updateProductInfo(side, product) {
        const infoElement = document.querySelector(`#${side}-product-info`);
        if (infoElement) {
            infoElement.innerHTML = `
                <div class="has-text-weight-bold">${product.name}</div>
                <div class="is-size-7 has-text-grey">
                    尺寸: ${product.size} | 數量: ${product.count} | 房間: ${product.room}
                </div>
            `;
        }
    }

    /**
     * 更新料架選擇 UI
     */
    function updateRackSelection(state) {
        //console.log('🏷️ 更新料架選擇顯示');

        ['left', 'right'].forEach(side => {
            const sideData = state[side];
            if (sideData) {
                const selectedProduct = sideData.products[sideData.productSelected];
                const selectedRackId = selectedProduct?.rackId;

                // 更新料架選擇區域中按鈕的選中狀態
                const rackSelectedContainer = document.querySelector(`.rack-selected[data-side="${side}"]`);
                if (rackSelectedContainer) {
                    // 清除所有按鈕的選中狀態
                    const allButtons = rackSelectedContainer.querySelectorAll('.rack-option-btn');
                    allButtons.forEach(btn => {
                        btn.classList.remove('is-primary', 'is-selected');
                        btn.classList.add('is-light');
                    });

                    // 如果有選中的料架，高亮對應的按鈕
                    if (selectedRackId) {
                        const selectedButton = rackSelectedContainer.querySelector(`[data-rackid="${selectedRackId}"]`);
                        if (selectedButton) {
                            selectedButton.classList.remove('is-light');
                            selectedButton.classList.add('is-primary', 'is-selected');
                        }
                    }

                    //console.log(`🏷️ ${side} 側料架選擇已更新: ${selectedRackId || '未選擇'}`);
                }
            }
        });
    }

    /**
     * 更新任務按鈕狀態
     */
    function updateTaskButtons(state) {


        ['left', 'right'].forEach(side => {
            const activeTask = state.active?.[side];
            const callFullBtn = document.querySelector(`[data-call-full="${side}"]`);
            const dispatchFullBtn = document.querySelector(`[data-dispatch-full="${side}"]`);



            // 加入料架按鈕不需要根據任務狀態更新

            // 更新叫滿車按鈕
            if (callFullBtn) {
                if (activeTask) {
                    if (activeTask.type === 'call_full') {
                        callFullBtn.textContent = '取消';
                        callFullBtn.className = 'button is-danger';
                        callFullBtn.disabled = false;
                    } else {
                        callFullBtn.textContent = '叫滿車';
                        callFullBtn.className = 'button is-warning';
                        callFullBtn.disabled = true; // 有其他任務時禁用
                    }
                } else {
                    callFullBtn.textContent = '叫滿車';
                    callFullBtn.className = 'button is-warning';
                    callFullBtn.disabled = false;
                }
            }

            // 更新派車按鈕
            if (dispatchFullBtn) {
                if (activeTask) {
                    if (activeTask.type === 'dispatch_full') {
                        dispatchFullBtn.textContent = '取消';
                        dispatchFullBtn.className = 'button is-danger';
                        dispatchFullBtn.disabled = false;
                    } else {
                        // 保留圖標結構，只更新文字
                        const textSpan = dispatchFullBtn.querySelector('span:not(.icon)');
                        if (textSpan) {
                            textSpan.textContent = '派車';
                        } else if (!dispatchFullBtn.querySelector('.icon')) {
                            dispatchFullBtn.innerHTML = '<span class="icon"><i class="mdi mdi-truck-delivery"></i></span><span>派車</span>';
                        } else {
                            dispatchFullBtn.textContent = '派車';
                        }
                        dispatchFullBtn.className = 'button is-warning';
                        dispatchFullBtn.disabled = true; // 有其他任務時禁用
                    }
                } else {
                    // 保留圖標結構，只更新文字
                    const textSpan = dispatchFullBtn.querySelector('span:not(.icon)');
                    if (textSpan) {
                        textSpan.textContent = '派車';
                    } else if (!dispatchFullBtn.querySelector('.icon')) {
                        dispatchFullBtn.innerHTML = '<span class="icon"><i class="mdi mdi-truck-delivery"></i></span><span>派車</span>';
                    } else {
                        dispatchFullBtn.textContent = '派車';
                    }
                    dispatchFullBtn.className = 'button is-warning';
                    dispatchFullBtn.disabled = false;
                }
            }


        });
    }

    /**
     * 更新任務狀態顯示
     */
    function updateTaskStatus(state) {
        ['left', 'right'].forEach(side => {
            const activeTask = state.active?.[side];
            const statusElement = document.querySelector(`#${side}-task-status`);

            if (statusElement) {
                if (activeTask) {
                    statusElement.innerHTML = `
                        <span class="tag is-warning">
                            ${getTaskTypeText(activeTask.type)} - ${activeTask.status}
                        </span>
                    `;
                } else {
                    statusElement.innerHTML = `
                        <span class="tag is-light">待機中</span>
                    `;
                }
            }
        });
    }

    /**
     * 更新停車格列表和料架選擇區域
     */
    function updateParkingList(parking) {
        if (!parking) return;

        ['left', 'right'].forEach(side => {
            const parkingList = parking[side];

            // 更新料架選擇區域（home 頁面的料架選擇按鈕）
            updateRackSelectionArea(side, parkingList);
        });

        // 更新完料架列表後，確保料架選擇狀態也同步更新
        const operationState = operationStore.getState();
        if (operationState) {
            updateRackSelection(operationState);
        }
    }

    /**
     * 更新 home 頁面的料架選擇區域
     * 顯示可用的料架列表，並正確顯示當前選擇狀態
     */
    function updateRackSelectionArea(side, parkingList) {
        const rackSelectedContainer = document.querySelector(`.rack-selected[data-side="${side}"]`);
        if (!rackSelectedContainer) {
            // 不在 home 頁面，跳過
            return;
        }

        //console.log(`🏷️ 更新 ${side} 側料架選擇區域`);

        // 獲取當前選擇的料架
        const operationState = operationStore.getState();
        const selectedProduct = operationState[side]?.products[operationState[side]?.productSelected];
        const selectedRackId = selectedProduct?.rackId;

        // 清空現有內容
        rackSelectedContainer.innerHTML = '';

        if (parkingList && parkingList.length > 0) {
            // 顯示可用的料架列表
            const rackButtons = parkingList.map(rack => {
                const isSelected = selectedRackId === rack.id;
                const buttonClass = isSelected ? 'button is-small is-primary rack-option-btn' : 'button is-small is-light rack-option-btn';

                return `
                    <p class="control">
                        <button class="${buttonClass}" data-side="${side}" data-rackid="${rack.id}">
                            ${rack.name || rack.id}
                        </button>
                    </p>
                `;
            }).join('');

            rackSelectedContainer.innerHTML = rackButtons;
            //console.log(`🏷️ ${side} 側顯示 ${parkingList.length} 個可用料架，當前選擇: ${selectedRackId || '無'}`);
        } else {
            // 沒有可用料架
            rackSelectedContainer.innerHTML = `
                <p class="control">
                    <button class="button is-small is-light" disabled>無可用料架</button>
                </p>
            `;
            //console.log(`🏷️ ${side} 側無可用料架`);
        }
    }

    // ===== 事件處理方法 =====

    /**
     * 載入可用料架列表
     */
    async function loadAvailableRacks() {
        const select = document.getElementById('addRackSelect');
        const noRacksMessage = document.getElementById('noRacksMessage');
        const confirmBtn = document.getElementById('confirmAddRack');
        
        if (!select) return;
        
        // 顯示載入中
        select.innerHTML = '<option value="">載入中...</option>';
        select.disabled = true;
        
        try {
            // 調用 API 獲取可用料架
            const response = await fetch('/api/rack/available');
            const data = await response.json();
            
            if (data.success && data.racks && data.racks.length > 0) {
                // 清空並填充選項
                select.innerHTML = '<option value="">請選擇料架</option>';
                
                data.racks.forEach(rack => {
                    const option = document.createElement('option');
                    option.value = rack.name;
                    option.textContent = rack.name;
                    select.appendChild(option);
                });
                
                select.disabled = false;
                if (confirmBtn) confirmBtn.disabled = false;
                
                // 隱藏無料架訊息
                if (noRacksMessage) {
                    noRacksMessage.style.display = 'none';
                }
            } else {
                // 沒有可用料架
                select.innerHTML = '<option value="">沒有可用料架</option>';
                select.disabled = true;
                if (confirmBtn) confirmBtn.disabled = true;
                
                // 顯示無料架訊息
                if (noRacksMessage) {
                    noRacksMessage.style.display = 'block';
                }
            }
        } catch (error) {
            console.error('載入可用料架失敗:', error);
            select.innerHTML = '<option value="">載入失敗</option>';
            select.disabled = true;
            if (confirmBtn) confirmBtn.disabled = true;
        }
    }
    
    /**
     * 處理加入料架
     */
    async function handleAddRack(side) {
        //console.log(`📦 處理加入料架: ${side} 側`);
        
        // 儲存當前操作的側邊
        window.currentAddRackSide = side;
        
        // 顯示 Modal
        const modal = document.getElementById('addRackModal');
        const select = document.getElementById('addRackSelect');
        const helpText = document.getElementById('addRackHelp');
        const noRacksMessage = document.getElementById('noRacksMessage');
        
        if (modal && select) {
            // 更新提示文字
            helpText.textContent = `選擇要加入到${side === 'left' ? '左側' : '右側'}停車格的料架`;
            helpText.classList.remove('is-danger');
            
            // 顯示 Modal
            modal.classList.add('is-active');
            
            // 載入可用料架列表
            await loadAvailableRacks();
        }
    }

    /**
     * 處理叫滿車
     */
    function handleCallFull(side) {
        //console.log(`🚛 處理叫滿車: ${side} 側`);

        const operationState = operationStore.getState();
        const selectedProduct = operationState[side].products[operationState[side].productSelected];

        if (selectedProduct && selectedProduct.rackId) {
            socketAPI.callFull({ side, rackId: selectedProduct.rackId });
            //console.log(`✅ 叫滿車請求已發送: ${side} 側, 料架: ${selectedProduct.rackId}`);
        } else {
            console.warn(`⚠️ 叫滿車失敗: ${side} 側未選擇料架`);
            notify.showErrorMessage('請先選擇料架');
        }
    }

    /**
     * 處理派車
     */
    function handleDispatchFull(side) {
        console.log(`🚚 處理派車: ${side} 側`);

        const operationState = operationStore.getState();
        const selectedProduct = operationState[side].products[operationState[side].productSelected];

        // 檢查必要條件
        if (!selectedProduct) {
            console.warn(`⚠️ 派車失敗: ${side} 側未選擇產品`);
            notify.showErrorMessage('請先選擇產品');
            return;
        }

        if (!selectedProduct.rackId) {
            console.warn(`⚠️ 派車失敗: ${side} 側未選擇料架`);
            notify.showErrorMessage('請先選擇料架');
            return;
        }

        if (!selectedProduct.name) {
            console.warn(`⚠️ 派車失敗: ${side} 側產品名稱為空`);
            notify.showErrorMessage('產品名稱不能為空');
            return;
        }

        if (!selectedProduct.count || selectedProduct.count <= 0) {
            console.warn(`⚠️ 派車失敗: ${side} 側未設定數量`);
            notify.showErrorMessage('請先設定數量');
            return;
        }

        if (!selectedProduct.room) {
            console.warn(`⚠️ 派車失敗: ${side} 側未選擇房間`);
            notify.showErrorMessage('請先選擇房間');
            return;
        }

        // 準備完整的派車資料
        const dispatchData = {
            side: side,
            productName: selectedProduct.name,
            count: selectedProduct.count,
            rackId: selectedProduct.rackId,
            room: selectedProduct.room
        };

        console.log(`✅ 派車請求資料:`, dispatchData);
        socketAPI.dispatchFull(dispatchData);
    }

    /**
     * 綁定 Modal 事件
     */
    function bindModalEvents() {
        if (boundEvents.has('modalEvents')) return;
        
        const modal = document.getElementById('addRackModal');
        const confirmBtn = document.getElementById('confirmAddRack');
        const cancelBtn = document.getElementById('cancelAddRack');
        const closeBtn = modal?.querySelector('.modal-card-head .delete');
        const background = modal?.querySelector('.modal-background');
        const select = document.getElementById('addRackSelect');
        
        // 確認按鈕
        if (confirmBtn) {
            confirmBtn.addEventListener('click', confirmAddRack);
        }
        
        // 取消按鈕
        if (cancelBtn) {
            cancelBtn.addEventListener('click', closeAddRackModal);
        }
        
        // 關閉按鈕
        if (closeBtn) {
            closeBtn.addEventListener('click', closeAddRackModal);
        }
        
        // 背景點擊關閉
        if (background) {
            background.addEventListener('click', closeAddRackModal);
        }
        
        // 選擇框 Change 事件（可選）
        if (select) {
            select.addEventListener('change', (e) => {
                // 當選擇改變時可以做一些處理
                const helpText = document.getElementById('addRackHelp');
                if (helpText) {
                    helpText.classList.remove('is-danger');
                }
            });
        }
        
        boundEvents.add('modalEvents');
    }
    
    /**
     * 確認加入 Rack
     */
    function confirmAddRack() {
        const select = document.getElementById('addRackSelect');
        const helpText = document.getElementById('addRackHelp');
        const side = window.currentAddRackSide;
        
        if (!select || !side) return;
        
        const rackName = select.value;
        
        if (!rackName) {
            helpText.textContent = '請選擇一個料架';
            helpText.classList.add('is-danger');
            return;
        }
        
        // 發送加入 Rack 請求
        socketAPI.addRack(side, rackName)
            .then(() => {
                notify.showNotifyMessage(`已將料架 ${rackName} 加入到 ${side === 'left' ? '左側' : '右側'}`);
                closeAddRackModal();
            })
            .catch(error => {
                console.error('加入料架失敗:', error);
                
                // 如果是料架不存在的錯誤，提供更清楚的指引
                if (error.message && error.message.includes('不存在於系統中')) {
                    helpText.textContent = `料架 ${rackName} 不存在，請先在 AGVCUI 創建此料架`;
                    helpText.classList.add('is-danger');
                } else {
                    helpText.textContent = error.message || '加入料架失敗';
                    helpText.classList.add('is-danger');
                }
            });
    }
    
    /**
     * 關閉加入 Rack Modal
     */
    function closeAddRackModal() {
        const modal = document.getElementById('addRackModal');
        const select = document.getElementById('addRackSelect');
        const helpText = document.getElementById('addRackHelp');
        const noRacksMessage = document.getElementById('noRacksMessage');
        
        if (modal) {
            modal.classList.remove('is-active');
        }
        
        if (select) {
            select.value = '';
        }
        
        if (helpText) {
            helpText.textContent = '選擇要加入的料架';
            helpText.classList.remove('is-danger');
        }
        
        if (noRacksMessage) {
            noRacksMessage.style.display = 'none';
        }
        
        // 清除暫存的側邊資訊
        window.currentAddRackSide = null;
    }

    /**
     * 處理取消任務
     */
    async function handleCancelTask(side) {
        //console.log(`❌ 處理取消任務: ${side} 側`);

        try {
            // 發送取消請求到後端並等待回應
            const response = await socketAPI.cancelTask({ side });

            if (response && response.success) {
                //console.log(`✅ ${side} 側任務取消成功`);
                // 後端會推送任務狀態更新，前端會自動更新 UI
            } else {
                console.warn(`⚠️ ${side} 側任務取消失敗:`, response?.message || '未知錯誤');
                notify.showErrorMessage(`取消任務失敗: ${response?.message || '請重試'}`);
            }
        } catch (error) {
            console.error(`❌ ${side} 側任務取消錯誤:`, error);
            notify.showErrorMessage(`取消任務失敗: ${error.message}`);
        }
    }

    /**
     * 處理確認送達
     */
    function handleConfirmDelivery(side) {
        //console.log(`✅ 處理確認送達: ${side} 側`);
        socketAPI.confirmDelivery({ side });
        //console.log(`📤 確認送達請求已發送: ${side} 側`);
    }

    /**
     * 處理數量按鈕點擊
     */
    function handleNumberButtonClick(side, number) {
        //console.log(`數量按鈕點擊: ${side}, 數量: ${number}`);

        const operationState = operationStore.getState();
        const sideData = operationState[side];
        const selectedProductIndex = sideData.productSelected || 0;

        // 🔧 修復：檢查products陣列是否存在
        if (!sideData.products || sideData.products.length === 0) {
            console.warn(`⚠️ ${side} 側products陣列為空，無法設定數量`);
            notify.showErrorMessage('請先選擇產品');
            return;
        }

        // 更新選中產品的數量
        const newProducts = [...sideData.products];
        if (newProducts[selectedProductIndex]) {
            newProducts[selectedProductIndex] = {
                ...newProducts[selectedProductIndex],
                count: number
            };

            const newState = {
                ...operationState,
                [side]: {
                    ...sideData,
                    products: newProducts
                }
            };

            operationStore.setState(newState);

            // 更新UI顯示
            updateNumberButtons(newState);

            // 同步到後端
            socketAPI.updateClient(newState);

            notify.showNotifyMessage(`已設定 ${side} 側數量為 ${number}`);
        } else {
            console.error(`❌ ${side} 側產品索引 ${selectedProductIndex} 不存在`);
            notify.showErrorMessage('產品選擇錯誤');
        }
    }

    // ===== 輔助方法 =====

    /**
     * 根據側邊獲取停車格編號（從 machine 表獲取）
     * 叫空車功能應該直接使用 machine.parking_space_1 和 parking_space_2
     */
    function getParkingSpaceBySide(side) {
        const userState = userStore.getState();
        const dataState = dataStore.getState();
        const machineId = userState.machineId;

        //console.log(`🔍 獲取 ${side} 側停車格，機台ID: ${machineId}`);

        // 從 machines 資料中找到當前機台
        const machines = dataState.machines;
        const currentMachine = machines?.find(m => m.id === machineId);

        if (!currentMachine) {
            console.warn(`⚠️ 找不到機台 ${machineId}`);
            return null;
        }

        //console.log(`🔍 機台資料:`, currentMachine);
        //console.log(`🔍 機台停車格配置: parking_space_1=${currentMachine.parking_space_1}, parking_space_2=${currentMachine.parking_space_2}`);

        // 根據側邊獲取對應的停車格編號
        let parkingSpaceId = null;
        if (side === 'left') {
            parkingSpaceId = currentMachine.parking_space_1;
            //console.log(`🔍 左側停車格編號: ${parkingSpaceId}`);
        } else if (side === 'right') {
            parkingSpaceId = currentMachine.parking_space_2;
            //console.log(`🔍 右側停車格編號: ${parkingSpaceId}`);
        }

        if (parkingSpaceId) {
            //console.log(`✅ 找到 ${side} 側停車格編號: ${parkingSpaceId}`);
            return parkingSpaceId;
        } else {
            console.warn(`⚠️ 機台 ${machineId} 的 ${side} 側沒有配置停車格`);
            return null;
        }
    }

    /**
     * 獲取任務類型文字
     */
    function getTaskTypeText(type) {
        const typeMap = {
            'call_full': '叫滿車',
            'dispatch_full': '派送滿車'
        };
        return typeMap[type] || type;
    }

    /**
     * 更新房間按鈕狀態
     */
    function updateRoomButtons(operationState) {
        // 首先更新房間按鈕的啟用/禁用狀態
        const dataState = dataStore.getState();
        const rooms = dataState.rooms;

        document.querySelectorAll('.room-btn').forEach(btn => {
            const roomId = parseInt(btn.getAttribute('data-room'));
            const room = rooms && rooms.find(r => r.id === roomId);
            if (room && room.enable) {
                btn.disabled = false;
            } else {
                btn.disabled = true;
            }
        });

        // 然後更新房間按鈕的選中狀態
        ['left', 'right'].forEach(side => {
            const sideData = operationState[side];
            if (sideData) {
                // 🔧 修復：處理products陣列為空的情況
                let selectedProduct = null;
                if (sideData.products && sideData.products.length > 0) {
                    selectedProduct = sideData.products[sideData.productSelected || 0];
                } else {
                    // 如果products陣列為空，使用預設房號
                    console.warn(`⚠️ ${side} 側products陣列為空，房號按鈕使用預設狀態`);
                }

                const selectedRoom = selectedProduct?.room;

                document.querySelectorAll(`.room-btn[data-side="${side}"]`).forEach(btn => {
                    const btnRoom = parseInt(btn.getAttribute('data-room'));
                    btn.classList.toggle('is-selected', btnRoom === selectedRoom);
                    btn.classList.toggle('is-primary', btnRoom === selectedRoom);
                });
            }
        });
    }


    /**
     * 檢查派車任務是否應該自動完成（rack 被取走）
     */
    function checkDispatchTaskCompletion(parkingData) {
        const tasksState = tasksStore.getState();
        const operationState = operationStore.getState();

        ['left', 'right'].forEach(side => {
            const activeTask = tasksState.active?.[side];

            // 只檢查派車任務
            if (activeTask && activeTask.type === 'dispatch_full') {
                const selectedProduct = operationState[side].products[operationState[side].productSelected];
                const selectedRackId = selectedProduct?.rackId;

                if (selectedRackId) {
                    // 檢查該 rack 是否還在當前 location
                    const parkingList = parkingData?.[side];
                    const rackStillThere = parkingList?.some(rack => rack.id === selectedRackId);

                    if (!rackStillThere) {
                        //console.log(`🚚 檢測到 ${side} 側料架 ${selectedRackId} 已被取走，自動完成派車任務`);

                        // 清除派車任務
                        stateHelpers.updateTask(side, null);

                        // 顯示通知
                        notify.showNotifyMessage(`${side} 側派車完成，料架已被取走`);
                    } else {
                        //console.log(`🚚 ${side} 側料架 ${selectedRackId} 仍在位置上，派車任務繼續`);
                    }
                }
            }
        });
    }

    /**
     * 選擇料架
     */
    function selectRack(side, rackId) {
        //console.log(`🏷️ 選擇料架: ${side} 側, 料架編號: ${rackId}`);

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
        socketAPI.updateClient(newState);

        notify.showNotifyMessage(`已選擇 ${side} 側料架 ${rackId}`);
    }

    /**
     * 頁面清理
     */
    function cleanup() {
        // 移除事件監聽器
        operationStore.off('change', handleOperationChange);
        tasksStore.off('change', handleTasksChange);
        dataStore.off('change', handleDataChange);
        
        // 清除綁定標記
        boundEvents.clear();
        
        isInitialized = false;
        //console.log('🧹 Home 頁面已清理');
    }

    // 返回公開的 API
    return {
        setup,
        cleanup
    };
})();
