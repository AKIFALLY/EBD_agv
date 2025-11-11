import { BaseObject } from './BaseObject.js';

/**
 * 門狀態顯示物件
 * 用於在地圖上顯示自動門的實時狀態
 */
export class DoorStatusObject extends BaseObject {
    /**
     * 創建門狀態顯示物件
     * @param {L.Map} map - Leaflet 地圖實例
     * @param {L.LatLng} latlng - 地圖坐標
     * @param {number} doorId - 門ID (1-4)
     * @param {string} doorName - 門名稱，默認為"門{doorId}"
     */
    constructor(map, latlng, doorId, doorName = null) {
        const displayName = doorName || `Door[${doorId}]`;

        const html = `
<div id="door-status-${doorId}" class="door-status-container">
    <div class="door-status-title">
        <span>${displayName}</span>
    </div>
    <div class="door-status-value">未知</div>
</div>
        `;

        // 調用父類構造函數
        // iconSize: [100, 60] - 寬100px, 高60px
        // iconAnchor: [50, 30] - 錨點在中心
        // zIndex: 2000 - 與 EqpInfoObject 相同層級
        super(map, latlng, html, [100, 60], [50, 30], 2000);

        this.doorId = doorId;
        this.doorName = displayName;

        // 添加點擊事件監聽器
        this.addClickHandler();
    }

    /**
     * 添加點擊事件處理器
     */
    addClickHandler() {
        // 獲取門狀態容器元素
        const elem = document.getElementById(`door-status-${this.doorId}`);
        if (!elem) {
            console.warn(`Door status element not found for click handler: door-status-${this.doorId}`);
            return;
        }

        // 添加點擊樣式
        elem.style.cursor = 'pointer';

        // 添加點擊事件
        elem.addEventListener('click', () => {
            // 檢查 mapDoorControlModal 是否存在
            if (window.mapDoorControlModal && typeof window.mapDoorControlModal.openModal === 'function') {
                window.mapDoorControlModal.openModal(this.doorId, this.doorName);
            } else {
                console.error('mapDoorControlModal 未初始化或 openModal 方法不存在');
            }
        });
    }

    /**
     * 更新門狀態顯示
     * @param {string} signalValue - 信號值 ("0"=關閉, "1"=開啟)
     */
    updateStatus(signalValue) {
        const elem = document.getElementById(`door-status-${this.doorId}`);
        if (!elem) {
            console.warn(`Door status element not found: door-status-${this.doorId}`);
            return;
        }

        const valueElem = elem.querySelector('.door-status-value');
        if (!valueElem) {
            console.warn(`Door status value element not found for door ${this.doorId}`);
            return;
        }

        // 根據信號值設置狀態
        const isOpen = signalValue === "1";
        valueElem.textContent = isOpen ? "開" : "關";
        valueElem.className = `door-status-value ${isOpen ? 'opened' : 'closed'}`;
    }

    /**
     * 獲取門ID
     * @returns {number} 門ID
     */
    getDoorId() {
        return this.doorId;
    }

    /**
     * 獲取門名稱
     * @returns {string} 門名稱
     */
    getDoorName() {
        return this.doorName;
    }
}
