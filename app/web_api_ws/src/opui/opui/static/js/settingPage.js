// settingPage.js
import { clientStore, productsStore, machinesStore, parkingStore } from '../store/index.js';
import { notify } from './notify.js';
import socket from './socket.js';

export const settingPage = (() => {

    function updateProductInputText(op) {

        ["left", "right"].forEach(side => {
            const group = document.querySelector(`.product-group[data-side='${side}']`);
            if (!group) return;
            const entries = group.querySelectorAll('.product-entry');
            const sideData = op[side];
            if (sideData && Array.isArray(sideData.product)) {
                entries.forEach((entry, idx) => {
                    const input = entry.querySelector('.product-input');
                    if (input && sideData.product[idx]) {
                        input.value = sideData.product[idx].name || '';
                    }
                });
            }
        });
    }

    function updateMachineBtnClass(machineId) {
        const machineButtons = getMachineButtons();
        machineButtons.forEach(btn => {
            const btnId = Number(btn.getAttribute('data-machine'));
            if (btnId === Number(machineId)) {
                btn.classList.add('is-selected');
            } else {
                btn.classList.remove('is-selected');
            }
        });
    }

    function handleClientChange(newState) {
        if (!newState?.op?.left || !newState?.op?.right) return;
        const op = newState.op;
        updateProductInputText(op);
        updateMachineBtnClass(newState.machineId);
    }
    function handleParkingChange(newState) {
        if (!newState?.parkingSpace) return;
        const parkingSpace = newState?.parkingSpace
        updateParkingArea(parkingSpace);
    }
    function handleProductsChange(newState) {
        if (!newState?.products) return;
        const products = newState.products || [];
        updateProductCheckedIcons();
    }
    function handleMachinesChange(newState) {
        if (!newState?.machines) return;
        const machines = newState.machines || [];
        enableMachineButtons();
    }

    function getMachineButtons() {
        return document.querySelectorAll('.machine-btn');
    }
    function getProductGroups() {
        return document.querySelectorAll('.product-group');
    }

    function enableMachineButtons() {
        const machineButtons = getMachineButtons();
        const machines = machinesStore.getState().machines || [];
        machineButtons.forEach(btn => {
            const machineId = Number(btn.getAttribute('data-machine'));
            const machine = machines.find(m => Number(m.id) === machineId);
            if (machine && machine.enable) {
                btn.disabled = false;
            } else {
                btn.disabled = true;
            }
        });
    }

    function bindMachineButtonEvents() {
        const machineButtons = getMachineButtons();
        const machines = machinesStore.getState().machines || [];
        machineButtons.forEach(btn => {
            const machineId = Number(btn.getAttribute('data-machine'));
            const machine = machines.find(m => Number(m.id) === machineId);
            btn.onclick = null; // 先移除舊事件
            if (machine && machine.enable) {
                btn.addEventListener('click', () => {
                    console.log('機台按鈕點擊:', machineId);
                    clientStore.setState({ machineId })

                    //送到server更新
                    socket.api.clientUpdate(clientStore.getState());
                });
            }
        });
    }

    //function handleProductAddClick(side, index, inputText) {
    //    // 未來可在這裡呼叫 API
    //    console.debug('product-add click:', { side, index, inputText });
    //    const product = { name: inputText, size: 'S', process: 1 }
    //    // 設定這 side 的 op 的 product[index].name = inputText
    //    const state = clientStore.getState();
    //    if (!state.op || !state.op[side] || !Array.isArray(state.op[side].product)) return;
    //    const products = [...state.op[side].product];
    //    const idx = Number(index);
    //    if (products[idx]) {
    //        products[idx] = { ...products[idx], name: product.name };
    //        const newOp = {
    //            ...state.op,
    //            [side]: {
    //                ...state.op[side],
    //                product: products
    //            }
    //        };
    //        clientStore.setState({ op: newOp });
    //    }
    //    // TODO: 呼叫 API 並傳遞 side、inputText、index
    //    socket.api.addProduct(product);
    //}

    function updateProductCheckedIcon(productEntry) {
        const input = productEntry.querySelector('.product-input');
        const checkedIcon = productEntry.querySelector('.product-checked');
        if (input && checkedIcon) {
            // 初始化時也做一次字元過濾
            input.value = input.value.toUpperCase().replace(/[^A-Z0-9]/g, '');
            const inputValue = input.value.trim();
            const products = productsStore.getState()?.products || [];
            //如果輸入字元不為空或不在已有的產品清單中,不顯示checkedIcon
            if (inputValue === '' || !products.some(p => p.name === inputValue)) {
                checkedIcon.style.display = 'none';
            } else {
                checkedIcon.style.display = '';
            }
        }
    }
    function updateProductCheckedIcons() {
        const productGroups = getProductGroups();
        productGroups.forEach(group => {
            const entries = group.querySelectorAll('.product-entry');
            entries.forEach(entry => updateProductCheckedIcon(entry));
        });
    }

    function bindProductInputEvents() {
        const productGroups = getProductGroups();
        productGroups.forEach(group => {
            const side = group.getAttribute('data-side');
            group.addEventListener('input', (e) => {
                if (!e.target.classList.contains('product-input')) return;
                // 僅允許大寫英文字母及數字
                const original = e.target.value;
                const filteredProductName = original.toUpperCase().replace(/[^A-Z0-9]/g, '');
                if (original !== filteredProductName) e.target.value = filteredProductName;
                const entry = e.target.closest('.product-entry');
                if (!entry) return;
                //更新顯示checkedIcon
                updateProductCheckedIcon(entry);
                const index = entry.getAttribute('data-index');
                const idx = Number(index);
                // 新增：即時同步 input 到 clientStore.op[side].product[index].name
                // 如果有 clientStore.op[side].product[index] 且 filteredProductName 在product 中，則更新其 name
                const state = clientStore.getState();
                if (state.op && state.op[side] && Array.isArray(state.op[side].product)) {
                    const allProducts = productsStore.getState()?.products || [];

                    console.log('products', allProducts, filteredProductName);

                    const selectedProduct = allProducts.find(p => p.name === filteredProductName);
                    if (selectedProduct) {
                        //if (allProducts.some(p => p.name === filteredProductName)) {
                        console.log('filteredProductName', selectedProduct);
                        // 預設值 產品S最多32個 L 最多16個
                        const defaultMaxCount = selectedProduct.size === 'S' ? 32 : 16;
                        const products = [...state.op[side].product];
                        if (products[idx]) {
                            products[idx] = { ...products[idx], name: filteredProductName, size: selectedProduct.size, count: defaultMaxCount };
                        }
                        const newOp = {
                            ...state.op,
                            [side]: {
                                ...state.op[side],
                                product: products
                            }
                        };
                        clientStore.setState({ op: newOp });
                        console.log('newOp', newOp);

                        //送到server更新
                        socket.api.clientUpdate(clientStore.getState());
                    }
                }
            });
        });
    }

    function handleSaveClick() {
        console.debug('[設定儲存] 使用者點擊了儲存按鈕');

        const state = clientStore.getState();

        if (!state || Object.keys(state).length === 0) {
            notify.showErrorMessage("無可儲存的設定資料");
            return;
        }

        socket.api.clientUpdate(state);
    }

    function handleResetFactoryClick() {
        // 顯示 modal
        const modal = document.getElementById('factory-restore-modal');
        if (modal) {
            modal.classList.add('is-active');
        }
    }

    // Modal 關閉邏輯
    function bindFactoryRestoreModalEvents() {
        const modal = document.getElementById('factory-restore-modal');
        if (!modal) return;
        // 關閉按鈕
        const closeBtn = modal.querySelector('.delete');
        const cancelBtn = modal.querySelector('#factory-restore-cancel');
        const bg = modal.querySelector('.modal-background');
        [closeBtn, cancelBtn, bg].forEach(btn => {
            if (btn) btn.onclick = () => modal.classList.remove('is-active');
        });
        // 確認按鈕可在這裡加 API 呼叫
        const confirmBtn = modal.querySelector('#factory-restore-confirm');
        if (confirmBtn) {
            confirmBtn.onclick = () => {
                // TODO: 呼叫 API 恢復原廠
                localStorage.clear();
                clientStore.clear();
                socket.api.clientUpdate(clientStore.getState());
                modal.classList.remove('is-active');
                console.debug('factory restore confirmed');
            };
        }
    }

    function bindGlobalButtons() {
        //const saveBtn = document.getElementById('save');
        //if (saveBtn) saveBtn.onclick = handleSaveClick;
        const resetBtn = document.getElementById('factory-restore');
        if (resetBtn) resetBtn.onclick = handleResetFactoryClick;
    }

    function updateParkingArea(parkingSpace) {

        // 左右側
        ["left", "right"].forEach(side => {
            const root = document.querySelector(`.parking-area[data-side='${side}']`);
            if (root) {
                root.innerHTML = "";
                (parkingSpace[side] || []).forEach(rack => {
                    const control = document.createElement("div");
                    control.className = "control";
                    control.innerHTML = `
                        <div class="tags has-addons">
                            <a class="tag is-danger is-light is-large">${rack.name}</a>
                            <a class="tag is-delete is-large is-danger" data-side="${side}" data-idx="${rack.id}"></a>
                        </div>
                    `;
                    root.appendChild(control);
                });
            }
        });
    }

    function handleDeleteParkingClick(side, idx) {
        // 之後可在這裡呼叫 API
        console.debug('delete parking click:', { side, idx });
        // TODO: 呼叫 API 刪除料架
        socket.api.delRack(idx);
    }

    function bindParkingDeleteEvents() {
        document.querySelectorAll('.parking-area').forEach(area => {
            area.addEventListener('click', (e) => {
                const target = e.target;
                if (target.classList.contains('is-delete')) {
                    const side = target.getAttribute('data-side');
                    const idx = target.getAttribute('data-idx');
                    handleDeleteParkingClick(side, idx);
                }
            });
        });
    }

    function handleRackAddClick(side) {
        // 取得對應 input
        const input = document.querySelector(`.rack-add[data-side='${side}']`);
        if (!input) return;
        const rack_name = input.value.trim();
        if (!rack_name) return;
        // 之後可在這裡呼叫 API
        console.debug('rack-add click:', { side, rack_name });
        // TODO: 呼叫 API 新增料架
        socket.api.addRack(side, rack_name);
    }

    function bindRackAddButtons() {
        document.querySelectorAll('.rack-add-btn').forEach(btn => {
            const side = btn.getAttribute('data-side');
            btn.onclick = () => handleRackAddClick(side);
        });
    }

    function setup() {

        enableMachineButtons();
        bindMachineButtonEvents();
        updateProductCheckedIcons();
        bindProductInputEvents();
        bindGlobalButtons();

        bindParkingDeleteEvents();
        bindRackAddButtons();
        bindFactoryRestoreModalEvents();

        clientStore.on('change', handleClientChange);
        parkingStore.on('change', handleParkingChange)
        productsStore.on('change', handleProductsChange);
        machinesStore.on('change', handleMachinesChange);

        handleClientChange(clientStore.getState())
        handleParkingChange(parkingStore.getState())
        handleProductsChange(productsStore.getState())
        handleMachinesChange(machinesStore.getState())
        return clientStore;
    }

    return {
        setup,
    };
})();
