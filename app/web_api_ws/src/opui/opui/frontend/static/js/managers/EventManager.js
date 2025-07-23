/**
 * 事件管理器
 * 統一處理所有事件綁定和監聽，提高程式碼組織性
 */
import { socketAPI } from '../api.js';
import { stateHelpers } from '../store.js';
import { notify } from '../notify.js';

export class EventManager {
    constructor() {
        this.boundEvents = new Set(); // 追蹤已綁定的事件，避免重複綁定
    }

    /**
     * 綁定首頁事件
     */
    bindHomePageEvents() {
        // console.log("🔗 綁定首頁事件");

        this.bindProductButtons();
        this.bindNumberButtons();
        this.bindActionButtons();
        this.bindRackSelection();
        this.bindRoomButtons();
        this.bindDynamicRackSelection();
    }

    /**
     * 綁定設定頁面事件
     */
    bindSettingsPageEvents() {
        // console.log("🔗 綁定設定頁面事件");

        this.bindMachineButtons();
        this.bindProductInputs();
        this.bindFactoryRestoreButton();
        this.bindRackAddButtons();
        this.bindRackDeleteEvents();
    }

    /**
     * 綁定全域事件
     */
    bindGlobalEvents() {
        if (this.boundEvents.has('global')) return;

        // console.log("🔗 綁定全域事件");

        // 頁面卸載時清理
        window.addEventListener('beforeunload', () => {
            // console.log("🧹 清理應用程式資源");
        });

        // 錯誤處理
        window.addEventListener('error', (e) => {
            console.error("全域錯誤:", e.error);
            notify.showErrorMessage("發生未預期的錯誤");
        });

        this.boundEvents.add('global');
    }

    /**
     * 綁定產品按鈕事件（簡化架構版）
     */
    bindProductButtons() {
        document.querySelectorAll('.product-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-product-side');
                if (!side) return;

                // console.log(`🔄 產品切換: ${side} - 採用簡化單向資料流`);

                // 簡化架構：
                // 1. 立即更新本地狀態（提供即時 UI 反饋）
                const currentState = window.appStore.getState();
                const currentSelected = currentState.operation[side].productSelected;
                const nextSelected = (currentSelected + 1) % 2;

                stateHelpers.updateProductSelection(side, nextSelected);

                // 2. 直接向後端發送更新請求
                const updatedState = window.appStore.getState();
                socketAPI.updateClient(updatedState)
                    .then(() => {
                        // console.log(`✅ 產品切換到 ${side} 索引 ${nextSelected} 同步完成`);
                    })
                    .catch((error) => {
                        console.error(`❌ 產品切換同步失敗:`, error);
                        notify.showErrorMessage('產品切換失敗，請重試');
                    });

                // 3. 後端會透過 Socket 事件回傳最新資料，自動更新 store 和 UI
                // console.log(`🔄 等待後端 Socket 事件確認產品切換`);
            });
        });
    }

    /**
     * 綁定數量按鈕事件（簡化架構版）
     */
    bindNumberButtons() {
        document.querySelectorAll('.num-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-side');
                const num = parseInt(e.target.getAttribute('data-num'));
                const currentState = window.appStore.getState();
                const productIndex = currentState.operation[side].productSelected;

                // console.log(`🔄 數量設定: ${side} 產品 ${productIndex} 數量 ${num}`);

                // 簡化架構：
                // 1. 立即更新本地狀態（提供即時 UI 反饋）
                stateHelpers.updateProduct(side, productIndex, { count: num });

                // 2. 直接向後端發送更新請求
                const updatedState = window.appStore.getState();
                socketAPI.updateClient(updatedState)
                    .then(() => {
                        // console.log(`✅ 數量設定同步完成: ${side} 產品 ${productIndex} 數量 ${num}`);
                    })
                    .catch((error) => {
                        console.error(`❌ 數量設定同步失敗:`, error);
                        notify.showErrorMessage('數量設定失敗，請重試');
                    });
            });
        });
    }

    /**
     * 綁定操作按鈕事件
     */
    bindActionButtons() {
        // 叫空車按鈕
        document.querySelectorAll('[data-call-empty]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-call-empty');
                const buttonText = e.target.textContent.trim();

                if (buttonText === '取消') {
                    this.handleCancelTask(side);
                } else if (buttonText === '確認送達') {
                    this.handleConfirmDelivery(side);
                } else {
                    const parkingSpace = this.getParkingSpaceBySide(side);
                    if (parkingSpace) {
                        socketAPI.callEmpty({ side, parkingSpace });
                    } else {
                        notify.showErrorMessage('找不到對應的停車格');
                    }
                }
            });
        });

        // 派滿車按鈕
        document.querySelectorAll('[data-dispatch-full]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-dispatch-full');
                const buttonText = e.target.textContent.trim();

                if (buttonText === '取消') {
                    this.handleCancelTask(side);
                } else {
                    // 派車邏輯：只有「派車」和「取消」兩種狀態
                    const parkingSpace = this.getParkingSpaceBySide(side);
                    const currentState = window.appStore.getState();
                    const sideData = currentState.operation[side];
                    const productIndex = sideData.productSelected;
                    const product = sideData.products[productIndex];

                    if (parkingSpace && product) {
                        // console.log(`🔄 派車: ${side} 停車格 ${parkingSpace}`);
                        socketAPI.dispatchFull({
                            side,
                            parkingSpace,
                            name: product.name,
                            rackId: product.rackId,
                            count: product.count,
                            room: product.room
                        });
                    } else {
                        notify.showErrorMessage('找不到對應的停車格或產品資料');
                    }
                }
            });
        });
    }

    /**
     * 綁定料架選擇事件（含料架驗證）
     */
    bindRackSelection() {
        document.querySelectorAll('.rack-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const rackId = parseInt(btn.getAttribute('data-rackid'));
                const side = btn.getAttribute('data-side');

                if (!side || isNaN(rackId)) return;

                const currentState = window.appStore.getState();
                const productIndex = currentState.operation[side].productSelected;

                // console.log(`🔄 料架選擇 (靜態): ${side} 產品 ${productIndex} 料架 ${rackId}`);

                // 驗證料架是否存在於當前停車格的 parking_list 中
                if (!this.validateRackSelection(rackId, side, currentState)) {
                    notify.showErrorMessage(`料架 ${rackId} 不存在於當前停車格，請重新選擇`);
                    return;
                }

                // 更新狀態並同步到後端
                stateHelpers.updateProduct(side, productIndex, { rackId });

                // 同步到後端
                const updatedState = window.appStore.getState();
                socketAPI.updateClient(updatedState)
                    .then(() => {
                        // console.log(`✅ 料架選擇同步完成: ${side} 產品 ${productIndex} 料架 ${rackId}`);
                    })
                    .catch((error) => {
                        console.error(`❌ 料架選擇同步失敗:`, error);
                        notify.showErrorMessage('料架選擇失敗，請重試');
                    });
            });
        });
    }

    /**
     * 綁定房號按鈕事件（簡化架構版）
     */
    bindRoomButtons() {
        document.querySelectorAll('.room-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const side = e.target.getAttribute('data-side');
                const room = parseInt(e.target.getAttribute('data-room'));
                const currentState = window.appStore.getState();
                const productIndex = currentState.operation[side].productSelected;

                // console.log(`🔄 房號設定: ${side} 產品 ${productIndex} 房號 ${room}`);

                // 簡化架構：
                // 1. 立即更新本地狀態（提供即時 UI 反饋）
                stateHelpers.updateProduct(side, productIndex, { room });

                // 2. 直接向後端發送更新請求
                const updatedState = window.appStore.getState();
                socketAPI.updateClient(updatedState)
                    .then(() => {
                        // console.log(`✅ 房號設定同步完成: ${side} 產品 ${productIndex} 房號 ${room}`);
                    })
                    .catch((error) => {
                        console.error(`❌ 房號設定同步失敗:`, error);
                        notify.showErrorMessage('房號設定失敗，請重試');
                    });
            });
        });
    }

    /**
     * 綁定動態料架選擇事件（簡化架構版 + 料架驗證）
     */
    bindDynamicRackSelection() {
        if (this.boundEvents.has('dynamicRack')) return;

        document.addEventListener('click', (e) => {
            if (e.target.classList.contains('rack-btn') && e.target.hasAttribute('data-rackid')) {
                const rackId = parseInt(e.target.getAttribute('data-rackid'));
                const side = e.target.getAttribute('data-side');

                if (!side || isNaN(rackId)) return;

                const currentState = window.appStore.getState();
                const productIndex = currentState.operation[side].productSelected;

                // console.log(`🔄 料架選擇: ${side} 產品 ${productIndex} 料架 ${rackId}`);

                // 驗證料架是否存在於當前停車格的 parking_list 中
                if (!this.validateRackSelection(rackId, side, currentState)) {
                    notify.showErrorMessage(`料架 ${rackId} 不存在於當前停車格，請重新選擇`);
                    return;
                }

                // 簡化架構：
                // 1. 立即更新本地狀態（提供即時 UI 反饋）
                stateHelpers.updateProduct(side, productIndex, { rackId });

                // 2. 直接向後端發送更新請求
                const updatedState = window.appStore.getState();
                socketAPI.updateClient(updatedState)
                    .then(() => {
                        // console.log(`✅ 料架選擇同步完成: ${side} 產品 ${productIndex} 料架 ${rackId}`);
                    })
                    .catch((error) => {
                        console.error(`❌ 料架選擇同步失敗:`, error);
                        notify.showErrorMessage('料架選擇失敗，請重試');
                    });
            }
        });

        this.boundEvents.add('dynamicRack');
    }

    /**
     * 驗證料架選擇是否有效
     */
    validateRackSelection(rackId, side, currentState) {
        try {
            // 獲取當前停車格的料架列表
            const parkingList = currentState.data.parking || {};
            const availableRacks = parkingList[side] || [];

            // 檢查選擇的料架是否存在於可用列表中
            const isValidRack = availableRacks.some(rack => rack.id === rackId);

            if (!isValidRack) {
                // console.warn(`⚠️ 料架驗證失敗: rack ${rackId} 不存在於 ${side} 側停車格`);
                // console.log(`可用料架:`, availableRacks.map(r => `${r.id}(${r.name})`).join(', '));
                return false;
            }

            // console.log(`✅ 料架驗證通過: rack ${rackId} 存在於 ${side} 側停車格`);
            return true;

        } catch (error) {
            console.error(`❌ 料架驗證過程發生錯誤:`, error);
            return false; // 發生錯誤時拒絕選擇，保守處理
        }
    }

    /**
     * 綁定機台按鈕事件
     */
    bindMachineButtons() {
        const state = window.appStore.getState();
        const machines = state.data.machines || [];

        document.querySelectorAll('.machine-btn').forEach(btn => {
            const machineId = parseInt(btn.getAttribute('data-machine'));
            const machine = machines.find(m => Number(m.id) === machineId);

            btn.onclick = null; // 移除舊事件

            if (machine && machine.enable) {
                btn.disabled = false;
                btn.addEventListener('click', () => {
                    // console.log(`🔄 機台切換: ${machineId} - 採用簡化單向資料流`);

                    // 簡化架構：
                    // 1. 立即更新本地狀態（提供即時 UI 反饋）
                    stateHelpers.setUser({ machineId });

                    // 2. 直接向後端發送更新請求
                    const currentState = window.appStore.getState();
                    socketAPI.updateClient(currentState)
                        .then(() => {
                            // console.log(`✅ 機台切換到 ${machineId} 同步完成`);
                        })
                        .catch((error) => {
                            console.error(`❌ 機台切換同步失敗:`, error);
                            notify.showErrorMessage('機台切換失敗，請重試');
                        });

                    // 3. 後端會透過 Socket 事件回傳最新資料，自動更新 store 和 UI
                    // console.log(`🔄 等待後端 Socket 事件更新停車格資料`);
                });
            } else {
                btn.disabled = true;
            }
        });
    }

    /**
     * 綁定產品輸入事件
     */
    bindProductInputs() {
        // console.log("🔗 綁定產品輸入事件");
        const groups = document.querySelectorAll('.product-group');
        // console.log(`找到 ${groups.length} 個產品組`);

        groups.forEach(group => {
            const side = group.getAttribute('data-side');
            if (!side) return;

            group.addEventListener('input', (e) => {
                if (!e.target.classList.contains('product-input')) return;

                const productIndex = parseInt(e.target.getAttribute('data-index'));
                let productName = e.target.value.trim();

                // 自動轉換為大寫
                const upperCaseProductName = productName.toUpperCase();
                if (productName !== upperCaseProductName) {
                    // 保存游標位置
                    const cursorPosition = e.target.selectionStart;
                    e.target.value = upperCaseProductName;
                    // 恢復游標位置
                    e.target.setSelectionRange(cursorPosition, cursorPosition);
                    productName = upperCaseProductName;
                }

                // console.log(`📝 產品輸入變更: ${side}[${productIndex}] = "${productName}"`);

                // 立即更新 UI 驗證
                this.updateSingleProductValidation(e.target, productName);

                stateHelpers.updateProduct(side, productIndex, { name: productName });
            });
        });
    }

    /**
     * 更新單個產品輸入的驗證狀態
     */
    updateSingleProductValidation(input, productName) {
        // 獲取當前可用的產品列表
        const currentState = window.appStore.getState();
        const availableProductNames = currentState.data.products?.map(p => p.name) || [];

        const isValid = !productName || availableProductNames.includes(productName);

        // 更新輸入框樣式
        input.classList.toggle('is-danger', !isValid);

        // 更新檢查圖示
        const checkIcon = input.parentElement.querySelector('.product-checked');
        if (checkIcon) {
            checkIcon.classList.toggle('is-visible', isValid && productName);
        }

        // console.log(`🔍 產品驗證: "${productName}" -> ${isValid ? '有效' : '無效'}`);
    }

    /**
     * 綁定恢復原廠按鈕事件
     */
    bindFactoryRestoreButton() {
        const restoreBtn = document.getElementById('factory-restore');
        if (restoreBtn) {
            restoreBtn.addEventListener('click', () => {
                this.showFactoryRestoreModal();
            });
        }

        this.bindFactoryRestoreModal();
    }

    /**
     * 綁定料架新增按鈕事件
     */
    bindRackAddButtons() {
        // console.log("🔗 綁定料架新增按鈕事件");
        const buttons = document.querySelectorAll('.rack-add-btn');
        // console.log(`找到 ${buttons.length} 個料架新增按鈕`);

        buttons.forEach(btn => {
            const side = btn.getAttribute('data-side');
            // console.log(`綁定料架新增按鈕: ${side}`);

            btn.addEventListener('click', (e) => {
                // console.log(`🖱️ 料架新增按鈕被點擊: ${side}`);
                const clickedSide = e.target.getAttribute('data-side') || e.target.closest('.rack-add-btn').getAttribute('data-side');
                const input = document.querySelector(`input[data-side="${clickedSide}"].rack-add`);

                if (input && input.value.trim()) {
                    const rackName = input.value.trim();
                    // console.log(`➕ 新增料架: ${clickedSide} - ${rackName}`);
                    this.handleRackAddClick(clickedSide, rackName);
                    input.value = '';
                } else {
                    // console.log("⚠️ 輸入框為空或找不到");
                }
            });
        });
    }

    /**
     * 綁定料架刪除事件
     */
    bindRackDeleteEvents() {
        if (this.boundEvents.has('rackDelete')) return;

        document.addEventListener('click', (e) => {
            if (e.target.classList.contains('is-delete') && e.target.hasAttribute('data-rackid')) {
                const side = e.target.getAttribute('data-side');
                const rackId = parseInt(e.target.getAttribute('data-rackid'));
                this.handleRackDeleteClick(side, rackId);
            }
        });

        this.boundEvents.add('rackDelete');
    }

    // === 輔助方法 ===

    /**
     * 根據產品選擇同步產品資料
     */
    syncProductDataFromSelection(side, productIndex) {
        const currentState = window.appStore.getState();
        const currentProduct = currentState.operation[side].products[productIndex];

        if (currentProduct && currentProduct.name) {
            const availableProduct = currentState.data.products.find(p => p.name === currentProduct.name);
            if (availableProduct) {
                const updatedProduct = {
                    ...currentProduct,
                    size: availableProduct.size,
                    id: availableProduct.id
                };
                stateHelpers.updateProduct(side, productIndex, updatedProduct);
            }
        }
    }

    /**
     * 處理取消任務
     */
    handleCancelTask(side) {
        const parkingSpace = this.getParkingSpaceBySide(side);
        if (parkingSpace) {
            socketAPI.cancelTask({ side, parkingSpace });
        } else {
            notify.showErrorMessage('找不到對應的停車格');
        }
    }

    /**
     * 處理確認送達
     */
    handleConfirmDelivery(side) {
        const parkingSpace = this.getParkingSpaceBySide(side);
        if (parkingSpace) {
            socketAPI.confirmDelivery({ side, parkingSpace });
        } else {
            notify.showErrorMessage('找不到對應的停車格');
        }
    }

    /**
     * 處理料架新增
     */
    handleRackAddClick(side, rackName) {
        // console.log(`🔧 呼叫 addRack API: side=${side}, rackName=${rackName}`);

        // 驗證料架名稱格式
        if (!rackName || rackName.length === 0) {
            notify.showErrorMessage("請輸入料架編號");
            return;
        }

        socketAPI.addRack(side, rackName)
            .then((response) => {
                // console.log('✅ 料架新增成功:', response);
                notify.showNotifyMessage(response.message || "料架已分配到停車格");
            })
            .catch(error => {
                console.error('料架新增失敗:', error);
                const errorMessage = error.message || "料架新增失敗";
                notify.showErrorMessage(errorMessage);

                // 如果是料架不存在的錯誤，提供更詳細的說明
                if (errorMessage.includes("不存在於系統中")) {
                    setTimeout(() => {
                        notify.showNotifyMessage("提示：請先到料架管理頁面新增此料架", "info");
                    }, 2000);
                }
            });
    }

    /**
     * 處理料架刪除
     */
    handleRackDeleteClick(side, rackId) {
        socketAPI.deleteRack(rackId)
            .then(() => {
                notify.showNotifyMessage("料架已刪除");
            })
            .catch(error => {
                console.error('料架刪除失敗:', error);
                notify.showErrorMessage("料架刪除失敗");
            });
    }

    /**
     * 顯示恢復原廠確認 modal
     */
    showFactoryRestoreModal() {
        const modal = document.getElementById('factory-restore-modal');
        if (modal) {
            modal.classList.add('is-active');
        }
    }

    /**
     * 綁定恢復原廠 modal 事件
     */
    bindFactoryRestoreModal() {
        // 綁定取消按鈕
        const cancelBtn = document.getElementById('factory-restore-cancel');
        if (cancelBtn) {
            cancelBtn.addEventListener('click', () => {
                this.hideFactoryRestoreModal();
            });
        }

        // 綁定確認按鈕
        const confirmBtn = document.getElementById('factory-restore-confirm');
        if (confirmBtn) {
            confirmBtn.addEventListener('click', () => {
                this.executeFactoryRestore();
                this.hideFactoryRestoreModal();
            });
        }

        // 綁定背景點擊關閉
        const modal = document.getElementById('factory-restore-modal');
        if (modal) {
            modal.addEventListener('click', (e) => {
                if (e.target === modal) {
                    this.hideFactoryRestoreModal();
                }
            });
        }
    }

    /**
     * 隱藏恢復原廠 modal
     */
    hideFactoryRestoreModal() {
        const modal = document.getElementById('factory-restore-modal');
        if (modal) {
            modal.classList.remove('is-active');
        }
    }

    /**
     * 執行恢復原廠操作
     */
    executeFactoryRestore() {
        // 這個方法會在 StateManager 中實作
        if (window.opuiApp && window.opuiApp.stateManager) {
            window.opuiApp.stateManager.executeFactoryRestore();
        }
    }

    /**
     * 根據側邊獲取停車格 node_id
     */
    getParkingSpaceBySide(side) {
        const state = window.appStore.getState();
        const machineId = state.user.machineId;
        const machine = state.data.machines?.find(m => Number(m.id) === Number(machineId));

        if (!machine) {
            // console.warn('找不到對應的機台:', machineId);
            return null;
        }

        return side === 'left' ? machine.parking_space_1 : machine.parking_space_2;
    }
}
