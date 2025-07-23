/**
 * UI 更新管理器
 * 統一處理所有 UI 更新邏輯，提高程式碼可維護性
 */
import { PARKING_STATUS_ID } from '../constants/parkingStatus.js';

export class UIManager {
    constructor() {
        this.currentPage = null;
    }

    /**
     * 設定當前頁面
     */
    setCurrentPage(page) {
        this.currentPage = page;
    }

    /**
     * 主要 UI 更新協調器
     */
    updateUI(state) {
        // 更新所有頁面共用的 UI 元素
        this.updateNavbar(state.user);
        this.updateMachineSelection(state.user.machineId);

        // 根據當前頁面更新對應的 UI 元素
        if (this.currentPage === 'home') {
            this.updateHomePage(state);
        } else if (this.currentPage === 'settings') {
            this.updateSettingsPage(state);
        }
    }

    /**
     * 更新首頁 UI
     */
    updateHomePage(state) {
        this.updateProductButtonText(state.operation);
        this.updateNumberButtonStyles(state.operation);
        this.updateRoomButtonStates(state.data.rooms, state.operation);
        this.updateRackSelection(state.data.parking, state.operation);
        this.updateCallEmptyButtons(state.data.machines, state.user.machineId);
        this.updateDispatchFullButtons(state.data.machines, state.user.machineId);
    }

    /**
     * 更新特定側的數量按鈕樣式
     */
    updateSideNumberButtonStyles(side, operation) {
        const sideData = operation[side];
        if (sideData && sideData.products) {
            const selectedProduct = sideData.products[sideData.productSelected];
            const currentCount = selectedProduct?.count || 0;
            const productSize = (selectedProduct?.size || '').toUpperCase();

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
        }
    }

    /**
     * 更新設定頁面 UI
     */
    updateSettingsPage(state) {
        this.updateMachineButtonStates(state.data.machines);
        this.updateMachineSelection(state.user.machineId);
        this.updateProductInputs(state.operation);
        this.updateProductValidation(state.data.products);
        this.updateParkingArea(state.data.parking);
    }

    /**
     * 更新導航列狀態
     */
    updateNavbar(userState) {
        // 更新機台顯示
        const navMachineNumber = document.getElementById('nav-machine-number');
        if (navMachineNumber) {
            navMachineNumber.textContent = 'Machine ' + userState.machineId;
        }

        // 更新連線狀態
        const wifiIcon = document.querySelector('.mdi-wifi');
        if (wifiIcon) {
            wifiIcon.classList.toggle('is-connected', !!userState.isConnected);
            wifiIcon.classList.toggle('is-disconnected', !userState.isConnected);
        }
    }

    /**
     * 更新機台選擇狀態
     */
    updateMachineSelection(machineId) {
        document.querySelectorAll('.machine-btn').forEach(btn => {
            const btnMachineId = parseInt(btn.getAttribute('data-machine'));
            btn.classList.toggle('is-selected', btnMachineId === machineId);
            btn.classList.toggle('is-primary', btnMachineId === machineId);
        });
    }

    /**
     * 更新產品按鈕文字
     */
    updateProductButtonText(operation) {
        // 從 appStore 獲取可用產品清單
        const state = window.appStore?.getState() || {};
        const availableProducts = state.data?.products || [];

        ['left', 'right'].forEach(side => {
            const btn = document.querySelector(`.product-btn[data-product-side="${side}"]`);
            if (btn && operation[side]) {
                const selectedIndex = operation[side].productSelected;
                const operationProduct = operation[side].products[selectedIndex];

                let productName = '未設定';
                if (operationProduct?.id) {
                    const fullProduct = availableProducts.find(p => p.id === operationProduct.id);
                    productName = fullProduct?.name || operationProduct.name || '未設定';
                } else if (operationProduct?.name) {
                    productName = operationProduct.name;
                }

                btn.textContent = productName;
            }
        });
    }

    /**
     * 更新數量按鈕樣式
     */
    updateNumberButtonStyles(operation) {
        ['left', 'right'].forEach(side => {
            const sideData = operation[side];
            if (sideData && sideData.products) {
                const selectedProduct = sideData.products[sideData.productSelected];
                const currentCount = selectedProduct?.count || 0;
                const productSize = (selectedProduct?.size || '').toUpperCase();

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
            }
        });
    }

    /**
     * 更新房號按鈕狀態
     */
    updateRoomButtonStates(rooms, operation) {
        // 啟用/禁用房號按鈕
        document.querySelectorAll('.room-btn').forEach(btn => {
            const roomId = parseInt(btn.getAttribute('data-room'));
            const room = rooms && rooms.find(r => r.id === roomId);
            if (room && room.enable) {
                btn.disabled = false;
            } else {
                btn.disabled = true;
            }
        });

        // 更新房號按鈕選中狀態
        ['left', 'right'].forEach(side => {
            const sideData = operation[side];
            if (sideData) {
                const selectedProduct = sideData.products[sideData.productSelected];
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
     * 更新料架選擇區域（含驗證邏輯）
     */
    updateRackSelection(parking, operation) {
        ['left', 'right'].forEach(side => {
            const rackContainer = document.querySelector(`.rack-selected[data-side="${side}"]`);
            if (!rackContainer) return;

            rackContainer.innerHTML = '';

            const sideData = operation[side];
            const selectedProduct = sideData?.products[sideData.productSelected];
            const selectedRackId = selectedProduct?.rackId;

            const racks = parking[side] || [];

            // 檢查當前選擇的料架是否仍然有效（UI 層面的額外檢查）
            if (selectedRackId && !racks.some(rack => rack.id === selectedRackId)) {
                // console.warn(`⚠️ UI 檢測到無效料架選擇: ${side} 側料架 ${selectedRackId} 不在停車格中`);
                // 注意：這裡不直接清除選擇，因為 API 層已經處理了
                // 這裡只是記錄日誌，實際清除由 parking_list 事件處理
            }

            racks.forEach(rack => {
                const rackElement = document.createElement('button');
                rackElement.className = 'button is-small rack-btn';
                rackElement.textContent = rack.name;
                rackElement.setAttribute('data-rackid', rack.id);
                rackElement.setAttribute('data-side', side);

                if (rack.id === selectedRackId) {
                    rackElement.classList.add('is-selected', 'is-primary');
                }

                rackContainer.appendChild(rackElement);
            });

            // 顯示可用料架數量
            // console.log(`🔄 更新 ${side} 側料架選擇: ${racks.length} 個可用料架`);
        });

        // 重新綁定料架選擇事件（確保新生成的按鈕有事件處理）
        if (window.opuiApp && window.opuiApp.eventManager) {
            window.opuiApp.eventManager.bindRackSelection();
        }
    }

    /**
     * 更新叫車按鈕狀態（修復 Bulma 樣式覆蓋問題）
     */
    updateCallEmptyButtons(machines, machineId) {
        const machine = machines?.find(m => Number(m.id) === Number(machineId));
        if (!machine) return;

        ['left', 'right'].forEach(side => {
            const btn = document.querySelector(`.button[data-call-empty='${side}']`);
            if (!btn) return;

            const status = side === 'left' ? machine.parking_space_1_status : machine.parking_space_2_status;
            const hasRack = side === 'left' ? machine.parking_space_1_has_rack : machine.parking_space_2_has_rack;

            // 清除所有狀態相關的 Bulma 顏色類別
            btn.classList.remove('is-danger', 'is-success', 'is-warning', 'is-info');
            btn.disabled = false;

            if (status === PARKING_STATUS_ID.TASK_ACTIVE) {
                // 任務進行中：紅色取消按鈕
                btn.textContent = '取消';
                btn.classList.add('is-danger');
            } else if (status === PARKING_STATUS_ID.TASK_COMPLETED && hasRack) {
                // 任務完成且有料架：綠色確認送達按鈕
                btn.textContent = '確認送達';
                btn.classList.add('is-success');
            } else {
                // 正常狀態：恢復原始的藍色資訊樣式
                btn.textContent = '叫車';
                btn.classList.add('is-info');
            }
        });
    }

    /**
     * 更新派車按鈕狀態（修復 Bulma 樣式覆蓋問題）
     */
    updateDispatchFullButtons(machines, machineId) {
        const machine = machines?.find(m => Number(m.id) === Number(machineId));
        if (!machine) return;

        ['left', 'right'].forEach(side => {
            const btn = document.querySelector(`.button[data-dispatch-full='${side}']`);
            if (!btn) return;

            const status = side === 'left' ? machine.parking_space_1_status : machine.parking_space_2_status;
            const hasRack = side === 'left' ? machine.parking_space_1_has_rack : machine.parking_space_2_has_rack;

            // 清除所有狀態相關的 Bulma 顏色類別
            btn.classList.remove('is-danger', 'is-success', 'is-warning', 'is-disabled');
            btn.disabled = false;

            if (status === PARKING_STATUS_ID.TASK_ACTIVE) {
                // 派車任務進行中：紅色取消按鈕
                btn.textContent = '取消';
                btn.classList.add('is-danger');
            } else if (hasRack) {
                // 有料架時才能派車：恢復原始的橙色警告樣式
                btn.textContent = '派車';
                btn.classList.add('is-warning');
            } else {
                // 沒有料架時禁用派車按鈕：保持橙色但禁用
                btn.textContent = '派車';
                btn.classList.add('is-warning', 'is-disabled');
                btn.disabled = true;
            }
        });
    }

    /**
     * 更新機台按鈕狀態
     */
    updateMachineButtonStates(machines) {
        document.querySelectorAll('.machine-btn').forEach(btn => {
            const machineId = parseInt(btn.getAttribute('data-machine'));
            const machine = machines?.find(m => Number(m.id) === machineId);

            if (machine && machine.enable) {
                btn.disabled = false;
                btn.classList.remove('is-disabled');
            } else {
                btn.disabled = true;
                btn.classList.add('is-disabled');
            }
        });
    }

    /**
     * 更新產品輸入
     */
    updateProductInputs(operation) {
        ['left', 'right'].forEach(side => {
            const sideData = operation[side];
            if (sideData && sideData.products) {
                sideData.products.forEach((product, index) => {
                    const input = document.querySelector(`input[data-side="${side}"][data-index="${index}"]`);
                    if (input) {
                        input.value = product.name || '';
                    }
                });
            }
        });
    }

    /**
     * 更新產品驗證
     */
    updateProductValidation(products) {
        const availableProductNames = products?.map(p => p.name) || [];

        document.querySelectorAll('.product-input').forEach(input => {
            const productName = input.value.trim();
            const isValid = !productName || availableProductNames.includes(productName);

            input.classList.toggle('is-danger', !isValid);

            // 更新檢查圖示
            const checkIcon = input.parentElement.querySelector('.product-checked');
            if (checkIcon) {
                checkIcon.classList.toggle('is-visible', isValid && productName);
            }

            const helpText = input.parentElement.querySelector('.help');
            if (helpText) {
                helpText.textContent = isValid ? '' : '產品名稱不存在';
                helpText.classList.toggle('is-danger', !isValid);
            }
        });
    }

    /**
     * 更新停車區域
     */
    updateParkingArea(parking) {
        ['left', 'right'].forEach(side => {
            const container = document.querySelector(`.parking-area[data-side="${side}"]`);
            if (!container) return;

            container.innerHTML = '';
            const racks = parking[side] || [];

            racks.forEach(rack => {
                const rackElement = document.createElement('div');
                rackElement.className = 'control';
                rackElement.innerHTML = `
                    <div class="tags has-addons">
                        <span class="tag is-info is-large">${rack.name}</span>
                        <button class="tag is-delete is-large" data-rackid="${rack.id}" data-side="${side}"></button>
                    </div>
                `;
                container.appendChild(rackElement);
            });
        });
    }
}
