import { clientStore, roomsStore, parkingStore, machinesStore } from './store/index.js';
import { notify } from './notify.js';
import socket from './socket.js';

export const homePage = (() => {
    function getNumButtons() {
        return document.querySelectorAll('.num-btn');
    }
    function getProductBtns() {
        return document.querySelectorAll('.product-btn');
    }
    function getCallEmptyBtns() {
        return document.querySelectorAll('.button[data-call-empty]');
    }
    function getDispatchFullBtns() {
        return document.querySelectorAll('.button[data-dispatch-full]');
    }
    function getRoomBtns() {
        return document.querySelectorAll('.room-btn');
    }

    function handleNumBtnClick(side, num) {
        // 更新 store.op[side].product[productSelected].count
        const state = clientStore.getState();
        const op = state.op || {};
        const sideData = op[side];  // <-- 加上這行才有 sideData
        if (sideData && Array.isArray(sideData.product)) {
            const idx = sideData.productSelected || 0;
            const products = [...sideData.product];
            // 只更新 count
            products[idx] = { ...products[idx], count: Number(num) };
            const newOp = {
                ...op,
                [side]: {
                    ...sideData,
                    product: products
                }
            };
            clientStore.setState({ op: newOp });
        }
        console.debug('num-btn click:', { side, num });
    }

    function handleProductBtnClick(side) {
        // 產品按鈕點擊，切換 productSelected
        const op = clientStore.getState().op || {};
        const sideData = op[side];
        if (sideData && Array.isArray(sideData.product)) {
            const current = sideData.productSelected || 0;
            const next = (current + 1) % 2;
            const newOp = { ...op, [side]: { ...sideData, productSelected: next } };
            clientStore.setState({ op: newOp });
        }
        console.debug('product-btn click:', { side });
    }
    function handleCallEmptyClick(side) {
        const parkingSpace = getParkingSpaceBySide(side);
        if (!parkingSpace) {
            showErrorMessage('找不到對應的停車格');
            return;
        }
        console.debug('call-empty click:', { side, parkingSpace });
        socket.api.callEmpty({ side, parkingSpace });
    }

    function handleDispatchFullClick(side) {
        const parkingSpace = getParkingSpaceBySide(side);
        if (!parkingSpace) {
            showErrorMessage('找不到對應的停車格');
            return;
        }
        // 取得 op 的產品資訊
        const op = clientStore.getState().op || {};
        const sideData = op[side];
        let product = {};
        if (sideData && Array.isArray(sideData.product)) {
            const idx = sideData.productSelected || 0;
            const p = sideData.product[idx] || {};
            product = {
                name: p.name,
                rackId: p.rackId,
                count: p.count,
                room: p.room
            };
        }
        console.debug('dispatch-full click:', { side, parkingSpace, ...product });
        socket.api.dispatchFull({ side, parkingSpace, ...product });
    }
    // 依據 side 取得對應的 parkingSpace
    function getParkingSpaceBySide(side) {
        const machineId = clientStore.getState().machineId;
        const machines = machinesStore.getState().machines || [];
        const machine = machines.find(m => Number(m.id) === Number(machineId));
        if (!machine) return null;
        if (side === 'left' && machine.parking_space_1) {
            return machine.parking_space_1;
        } else if (side === 'right' && machine.parking_space_2) {
            return machine.parking_space_2;
        }
        return null;
    }

    function handleRoomBtnClick(side, room) {
        // 房號按鈕點擊，更新 store.op[side].product[productSelected].room
        const state = clientStore.getState();
        const op = state.op || {};
        const sideData = op[side];
        if (sideData && Array.isArray(sideData.product)) {
            const idx = sideData.productSelected || 0;
            const products = [...sideData.product];
            products[idx] = { ...products[idx], room: Number(room) };
            const newOp = {
                ...op,
                [side]: {
                    ...sideData,
                    product: products
                }
            };
            clientStore.setState({ op: newOp });
        }
        console.debug('room-btn click:', { side, room });
    }

    function bindNumButtonEvents() {
        getNumButtons().forEach(btn => {
            const side = btn.getAttribute('data-side');
            const num = btn.getAttribute('data-num');
            btn.onclick = () => handleNumBtnClick(side, num);
        });
    }
    function bindProductBtnEvents() {
        getProductBtns().forEach(btn => {
            const side = btn.getAttribute('data-product-side');
            btn.onclick = () => handleProductBtnClick(side);
        });
    }
    function bindCallEmptyBtnEvents() {
        getCallEmptyBtns().forEach(btn => {
            const side = btn.getAttribute('data-call-empty');
            btn.onclick = () => handleCallEmptyClick(side);
        });
    }
    function bindDispatchFullBtnEvents() {
        getDispatchFullBtns().forEach(btn => {
            const side = btn.getAttribute('data-dispatch-full');
            btn.onclick = () => handleDispatchFullClick(side);
        });
    }
    function bindRoomBtnEvents() {
        getRoomBtns().forEach(btn => {
            const side = btn.getAttribute('data-side');
            const room = btn.getAttribute('data-room');
            btn.onclick = () => handleRoomBtnClick(side, room);
        });
    }

    // --- UI 更新區 ---
    function updateProductBtnText(op) {
        ["left", "right"].forEach(side => {
            const btn = document.querySelector(`.product-btn[data-product-side='${side}']`);
            const sideData = op[side];
            if (btn && sideData && Array.isArray(sideData.product)) {
                const idx = sideData.productSelected || 0;
                const product = sideData.product[idx];
                btn.textContent = product && product.name ? product.name : '';
            }
        });
    }

    function updateNumBtnClass(op) {
        ["left", "right"].forEach(side => {
            const sideData = op[side];
            if (sideData && Array.isArray(sideData.product)) {
                const idx = sideData.productSelected || 0;
                const product = sideData.product[idx];
                const count = product && typeof product.count === 'number' ? product.count : 0;
                const btns = document.querySelectorAll(`.num-btn[data-side='${side}']`);
                btns.forEach(btn => {
                    const btnNum = Number(btn.getAttribute('data-num'));
                    if (btnNum <= count) {
                        btn.classList.add('is-success');
                    } else {
                        btn.classList.remove('is-success');
                    }
                });
            }
        });
    }

    function updateRoomBtnEnable() {
        const rooms = roomsStore.getState().rooms || [];
        document.querySelectorAll('.room-btn').forEach(btn => {
            const idx = Number(btn.getAttribute('data-room')) - 1;
            if (rooms[idx] && rooms[idx].enable) {
                btn.disabled = false;
            } else {
                btn.disabled = true;
            }
        });
    }

    function updateRoomBtnClass(op) {
        ["left", "right"].forEach(side => {
            const sideData = op[side];
            if (sideData && Array.isArray(sideData.product)) {
                const idx = sideData.productSelected || 0;
                const product = sideData.product[idx];
                const selectedRoom = product && typeof product.room === 'number' ? product.room : null;
                document.querySelectorAll(`.room-btn[data-side='${side}']`).forEach(btn => {
                    const btnRoom = Number(btn.getAttribute('data-room'));
                    if (btnRoom === selectedRoom) {
                        btn.classList.add('is-success');
                    } else {
                        btn.classList.remove('is-success');
                    }
                });
            }
        });
    }
    // 動態產生 rack-selected 內的 button，格式與需求一致
    function updateRackSelected() {
        ['left', 'right'].forEach(side => {
            const rack = document.querySelector(`.rack-selected[data-side="${side}"]`);
            if (!rack) return;
            rack.innerHTML = '';

            const spaces = parkingStore.getState().parkingSpace[side];
            // 依據目前選中的 product 的 rackId
            const op = clientStore.getState().op || {};
            const sideData = op[side] || {};
            let selectedRackId = null;
            if (sideData && Array.isArray(sideData.product)) {
                const idx = sideData.productSelected || 0;
                selectedRackId = sideData.product[idx]?.rackId || null;
            }

            spaces.forEach(space => {
                const p = document.createElement('p');
                p.className = 'control';
                const btn = document.createElement('button');
                btn.className = 'button';
                btn.setAttribute('data-rackid', space.id);
                btn.setAttribute('data-side', side);
                if (selectedRackId === space.id) {
                    btn.classList.add('is-success');
                }
                btn.textContent = space.name;
                p.appendChild(btn);
                rack.appendChild(p);
            });
        });
    }

    // 事件委派：只需 setup 時綁一次
    function bindRackSelectedEvents() {
        document.querySelectorAll('.rack-selected').forEach(rackEl => {
            //綁click event
            rackEl.addEventListener('click', function (e) {
                const btn = e.target.closest('button[data-rackid]');
                if (!btn) return;

                const rackId = Number(btn.getAttribute('data-rackid'));
                const side = btn.getAttribute('data-side');
                if (!side || isNaN(rackId)) return;

                const state = clientStore.getState();
                const op = state.op || {};
                const sideData = op[side] || {};

                if (sideData && Array.isArray(sideData.product)) {
                    const idx = sideData.productSelected || 0;
                    const products = [...sideData.product];
                    products[idx] = { ...products[idx], rackId: rackId };
                    const newOp = {
                        ...op,
                        [side]: {
                            ...sideData,
                            product: products
                        }
                    };
                    clientStore.setState({ op: newOp });
                }
            });
        });
    }

    function handleChange(newState) {
        console.debug('狀態變更:', newState);
        if (!newState?.op?.left || !newState?.op?.right) return;
        const op = newState.op;
        updateProductBtnText(op);
        updateRackSelected();
        updateNumBtnClass(op);
        updateRoomBtnEnable();
        updateRoomBtnClass(op);

        //socket.api.clientUpdate(newState);
    }
    function handleRoomsChange(newState) {
        if (!newState?.rooms) return;
        const rooms = newState.rooms || [];
        updateRoomBtnEnable();
    }

    function setup() {

        bindNumButtonEvents();
        bindProductBtnEvents();
        bindCallEmptyBtnEvents();
        bindDispatchFullBtnEvents();
        bindRoomBtnEvents();
        bindRackSelectedEvents(); // 只需 setup 時綁一次
        // 可根據 store 狀態初始化 UI
        //clientStore.clear();

        clientStore.on('change', handleChange);
        roomsStore.on('change', handleRoomsChange);

        //第一次打開時更新 測試用
        handleChange(clientStore.getState());

        return clientStore;
    }

    return {
        setup,
    };
})();