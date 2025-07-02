import { clientStore, productsStore, machinesStore, roomsStore, parkingStore } from '../store/index.js';
import { notify } from './notify.js';

let socket = null;

function setup() {
    if (socket) return; // 不回傳 socket，保持私有

    if (typeof io === "undefined") {
        console.warn("Socket.IO 尚未載入，稍後重試");
        setTimeout(setup, 50);
        return;
    }

    socket = io();

    socket.on("connect", () => {
        clientStore.setState({ isConnected: true, userAgent: navigator.userAgent })
        console.log("✅ Socket.IO 連線成功，socket.id:", socket.id);

        api.login(clientStore.getState())
        notify.showErrorMessage(`✅ AGVC 已連線`, "is-success");
    });

    socket.on("disconnect", () => {
        clientStore.setState({ isConnected: false })
        console.warn("❌ 與 AGVC 斷線");
        notify.showErrorMessage("與 AGVC 連線中斷");
    });

    socket.io.on("reconnect_attempt", (attempt) => {
        notify.showErrorMessage(`與 AGVC 斷線重連中（第 ${attempt} 次）`);
    });

    socket.io.on("reconnect", (attempt) => {
        notify.showErrorMessage(`✅ 重新連線成功（第 ${attempt} 次）`, "is-success");
    });

    socket.on("notify_message", (msg) => {
        const message = (msg.message || "通知").replace(/\n/g, "<br/>");
        notify.showNotifyMessage(message);
    });

    socket.on("error_message", (msg) => {
        const message = (msg.message || "錯誤").replace(/\n/g, "<br/>");
        notify.showErrorMessage(message);
    });

    socket.on("parking_list", (parkingList) => {
        console.log("Received parking list:", parkingList);
        parkingStore.setState({ parkingSpace: parkingList });
    });

    socket.on("product_list", (msg) => {
        productsStore.setState({ "products": msg.products });
    });
    socket.on("machine_list", (msg) => {
        machinesStore.setState({ "machines": msg.machines });
    });
    socket.on("room_list", (msg) => {
        roomsStore.setState({ "rooms": msg.rooms });
    });

}

const api = {
    login(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return;
        }
        socket.emit("login", data, (res) => {
            if (res.success) {
                console.log('login success', res.client);
                clientStore.setState(
                    {
                        "clientId": res.client.clientId,
                        "userAgent": res.client.userAgent,
                        "op": res.client.op,
                        "machineId": res.client.machineId
                    })
            }
            res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
        });
    },
    callEmpty(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return;
        }
        socket.emit("call_empty", data, (res) => {
            res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
        });
    },

    dispatchFull(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return;
        }
        socket.emit("dispatch_full", data, (res) => {
            res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
        });
    },

    //不再讓opui新增product
    //addProduct(productData) {
    //    if (!socket?.connected) {
    //        notify.showErrorMessage("Socket 尚未連線");
    //        return;
    //    }
    //    socket.emit("add_product", productData, (res) => {
    //        res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
    //        console.log(res.product)
    //    });
    //},

    addRack(side, rackName) {
        if (!rackName) {
            notify.showErrorMessage("請輸入料架編號");
            return;
        }

        socket.emit("add_rack", { side, rack: rackName }, (res) => {
            res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
        });
    },

    delRack(rackId) {
        if (!rackId) {
            notify.showErrorMessage("請輸入料架編號(rack.id)");
            return;
        }

        socket.emit("del_rack", { rackId }, (res) => {
            res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
        });
    },

    clientUpdate(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return;
        }
        socket.emit("client_update", data, (res) => {
            res?.success ? notify.showNotifyMessage("設定已儲存") : notify.showErrorMessage(res?.message || "儲存失敗");
        });
    },
    cancelTask(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return;
        }
        socket.emit("cancel_task", data, (res) => {
            res.success ? notify.showNotifyMessage(res.message) : notify.showErrorMessage(res.message);
        });
    },
};

// 匯出封裝好的模組
export default {
    setup,
    api
};