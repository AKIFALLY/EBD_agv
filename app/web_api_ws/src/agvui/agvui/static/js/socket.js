import { agvStore } from '../store/index.js';
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
        agvStore.setState({ isConnected: true })
        console.log("✅ Socket.IO 連線成功，socket.id:", socket.id);
        notify.showErrorMessage(`✅ AGVC 已連線`, "is-success");
    });

    socket.on("disconnect", () => {
        agvStore.setState({ isConnected: false })
        console.warn("❌ 與 AGVC 斷線");
        notify.showErrorMessage("與 AGVC 連線中斷");
    });

    socket.io.on("reconnect_attempt", (attempt) => {
        notify.showErrorMessage(`與 AGVC 斷線重連中（第 ${attempt} 次）`);
    });

    socket.io.on("reconnect", (attempt) => {
        notify.showErrorMessage(`✅ 重新連線成功（第 ${attempt} 次）`, "is-success");
    });

    // from server -> client
    socket.on("notify_message", (msg) => {
        const message = (msg.message || "通知").replace(/\n/g, "<br/>");
        notify.showNotifyMessage(message);
    });

    // from server -> client
    socket.on("error_message", (msg) => {
        const message = (msg.message || "錯誤").replace(/\n/g, "<br/>");
        notify.showErrorMessage(message);
    });

    // from server -> client
    socket.on("agv_status_update", (data) => {
        // console.log(data);
        agvStore.setState({ agvStatus: data.agv_status });
    });

}

const api = {
    // client -> server -> response -> client
    sendToBackendExample(data) {
        if (!socket?.connected) {
            notify.showErrorMessage("Socket 尚未連線");
            return;
        }
        socket.emit("send_to_backend", data, (res) => {
            res?.success ? notify.showNotifyMessage("已送到伺服器(成功)" + res?.message) : notify.showErrorMessage(res?.message || "(失敗)");
        });
    }
};

// 匯出封裝好的模組
export default {
    setup,
    api
};