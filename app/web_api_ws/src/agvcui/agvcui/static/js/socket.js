import { mapStore, signalsStore, carriersStore, machinesStore, roomsStore, racksStore, tasksStore, agvsStore, userStore, locationsStore } from '../store/index.js';
import { notify } from './notify.js';
import { validateTaskStatus } from './taskStatus.js';

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
        console.log("✅ Socket.IO 連線成功，socket.id:", socket.id);

        // 更新用戶連線狀態
        const currentUserState = userStore.getState();
        if (currentUserState.isLoggedIn) {
            userStore.setState({ isConnected: true });
        }

        notify.showErrorMessage(`✅ AGVC 已連線`, "is-success");

        // 請求交管區數據
        socket.emit("request_traffic_zones");
    });

    socket.on("disconnect", () => {
        console.warn("❌ 與 AGVC 斷線");

        // 更新用戶連線狀態
        const currentUserState = userStore.getState();
        if (currentUserState.isLoggedIn) {
            userStore.setState({ isConnected: false });
        }

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

    socket.on("map_info", (mapInfo) => {
        //console.log("Received mapInfo:", mapInfo);
        mapStore.setState(mapInfo);
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
    socket.on("signal_list", (msg) => {
        signalsStore.setState({ "signals": msg.signals });
    });
    socket.on("carrier_list", (msg) => {
        carriersStore.setState({ "carriers": msg.carriers });
    });
    socket.on("rack_list", (msg) => {
        racksStore.setState({ "racks": msg.racks });
    });
    socket.on("task_list", (msg) => {
        console.log('🔍 DEBUG: 收到 task_list 事件 (原始)', msg);

        // 🔧 新增：資料格式驗證
        if (!msg || !Array.isArray(msg.tasks)) {
            console.error('任務資料格式錯誤:', msg);
            return;
        }

        // 🔧 新增：狀態驗證和修正
        const validatedTasks = msg.tasks.map(task => {
            const statusValidation = validateTaskStatus(task.status_id);
            if (!statusValidation.isValid) {
                console.warn(`任務 ${task.id} 狀態不一致:`, statusValidation.error);
                console.warn(`有效狀態 ID:`, statusValidation.validStatusIds);
                // 使用降級狀態
                task.status_id = statusValidation.fallbackStatus;
                task._statusCorrected = true; // 標記狀態已修正
            }

            return task;
        });

        console.debug(`Socket.IO 接收任務資料: ${validatedTasks.length} 個任務`);
        tasksStore.setState({ "tasks": validatedTasks });
    });
    socket.on("location_list", (msg) => {
        console.log("收到 location_list 事件:", msg);
        locationsStore.updateLocations(msg.locations);
        console.log("locationsStore 狀態:", locationsStore.getState());
    });
    socket.on("agv_list", (msg) => {
        agvsStore.setState({ "agvs": msg.agvs });
    });
    socket.on("traffic_zones_update", (msg) => {
        console.log("收到 traffic_zones_update 事件:", msg);
        trafficZonesStore.setState({ "trafficZones": msg.traffic_zones });
    });

}

const api = {
    userLogin(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return Promise.reject(new Error("Socket 尚未連線"));
        }

        return new Promise((resolve, reject) => {
            socket.emit("user_login", data, (res) => {
                if (res.success) {
                    console.log('用戶登入成功', res.user);

                    // 設置 JWT token cookie（用於服務器端認證）
                    if (res.access_token) {
                        document.cookie = `access_token=${res.access_token}; path=/; max-age=${7 * 24 * 60 * 60}; SameSite=Lax`;
                        console.log('JWT token 已設置');
                    }

                    // 更新 userStore（用於客戶端狀態管理）
                    userStore.setState(res.user);
                    notify.showNotifyMessage(res.message);
                    resolve(res);
                } else {
                    notify.showErrorMessage(res.message);
                    reject(new Error(res.message));
                }
            });
        });
    },

    userLogout() {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return Promise.reject(new Error("Socket 尚未連線"));
        }

        return new Promise((resolve, reject) => {
            socket.emit("user_logout", {}, (res) => {
                if (res.success) {
                    console.log('用戶登出成功');

                    // 清除 JWT token cookie
                    document.cookie = 'access_token=; path=/; expires=Thu, 01 Jan 1970 00:00:00 GMT';
                    console.log('JWT token 已清除');

                    // 清空 userStore
                    userStore.clear();
                    notify.showNotifyMessage(res.message);
                    resolve(res);
                } else {
                    notify.showErrorMessage(res.message);
                    reject(new Error(res.message));
                }
            });
        });
    }
};

// 匯出封裝好的模組
export default {
    get socket() {
        return socket;
    },
    setup,
    api
};