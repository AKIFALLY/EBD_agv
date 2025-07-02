import { agvStore } from '../store/index.js';
import socket from './socket.js';
import { notify } from '../js/notify.js';

export const agvPage = (() => {

    function bindAgvIdInputEvents() {
        const input = document.getElementById("agv-id-input");
        input.oninput = null; // 先移除舊事件
        input.oninput = (e) => {
            console.log('輸入:', e);
            agvStore.setState({ agvId: input.value }); // 記錄目前所選的AGVID
        };
        input.value = agvStore.getState().agvId; // 記錄目前所選的AGVID
    }


    function bindExampleButtonEvents() {
        const exampleButton = document.getElementById("example-button");
        exampleButton.onclick = null; // 先移除舊事件
        exampleButton.onclick = () => {
            console.log('按鈕點擊:', exampleButton);

            //送訊息到server
            socket.api.sendToBackendExample({
                "message": "======================================test======================================"
            });
        };
    }

    function updateAgvStatus(agvStatus) {
        if (!agvStatus || typeof agvStatus !== 'object') {
            console.warn('Invalid AGV status data:', agvStatus);
            return;
        }
        const agvId = agvStore.getState().agvId;//目前所選的AGVID
        const agvIdFromServer = agvStatus.agv_id;//server傳回的AGVID的訊息

        console.debug("agvId:", agvId)
        console.debug("agvIdFromServer:", agvIdFromServer)
        if (agvIdFromServer !== agvId) return;

        const tbody = document.getElementById('agv_status_body');
        if (!tbody) return;
        tbody.innerHTML = ''; // Clear existing data

        for (const [key, value] of Object.entries(agvStatus)) {
            const row = document.createElement('tr');

            const keyCell = document.createElement('td');
            keyCell.textContent = key;

            const valueCell = document.createElement('td');
            valueCell.textContent = JSON.stringify(value, null, 2);//印出json格式，才可看出字串或數字

            row.appendChild(keyCell);
            row.appendChild(valueCell);

            tbody.appendChild(row);
        }
    }

    function handleChange(newState) {
        // console.debug('agv store state changed:', newState);
        if (newState?.agvStatus) {
            updateAgvStatus(newState.agvStatus);
        }
    }

    function setup() {
        //可以用 clear() state資料清空
        agvStore.clear();
        // 綁定輸入AGV ID
        bindAgvIdInputEvents()

        // 綁定按鈕事件
        bindExampleButtonEvents()

        // 資料監聽狀態改變
        agvStore.on('change', handleChange);
        // 設定初始狀態
        handleChange(agvStore.getState());
    }

    return {
        setup,
    };
})();