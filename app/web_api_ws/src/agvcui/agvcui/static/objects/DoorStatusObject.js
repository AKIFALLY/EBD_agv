import { BaseObject } from './BaseObject.js';

/**
 * 门状态显示物件
 * 用于在地图上显示自动门的实时状态
 */
export class DoorStatusObject extends BaseObject {
    /**
     * 创建门状态显示物件
     * @param {L.Map} map - Leaflet 地图实例
     * @param {L.LatLng} latlng - 地图坐标
     * @param {number} doorId - 门ID (1-4)
     * @param {string} doorName - 门名称，默认为"门{doorId}"
     */
    constructor(map, latlng, doorId, doorName = null) {
        const displayName = doorName || `Door[${doorId}]`;

        const html = `
<div id="door-status-${doorId}" class="door-status-container">
    <div class="door-status-title">
        <span>${displayName}</span>
    </div>
    <div class="door-status-value">
        <span class="tag is-light">未知</span>
    </div>
</div>
        `;

        // 调用父类构造函数
        // iconSize: [100, 60] - 宽100px, 高60px
        // iconAnchor: [50, 30] - 锚点在中心
        // zIndex: 2000 - 与 EqpInfoObject 相同层级
        super(map, latlng, html, [100, 60], [50, 30], 2000);

        this.doorId = doorId;
        this.doorName = displayName;
    }

    /**
     * 更新门状态显示
     * @param {string} signalValue - 信号值 ("0"=关闭, "1"=开启)
     */
    updateStatus(signalValue) {
        const elem = document.getElementById(`door-status-${this.doorId}`);
        if (!elem) {
            console.warn(`Door status element not found: door-status-${this.doorId}`);
            return;
        }

        const tag = elem.querySelector('.tag');
        if (!tag) {
            console.warn(`Door status tag not found for door ${this.doorId}`);
            return;
        }

        // 根据信号值设置状态
        const isOpen = signalValue === "1";
        tag.textContent = isOpen ? "開" : "關";
        tag.className = `tag ${isOpen ? 'is-success' : 'is-danger'}`;
    }

    /**
     * 获取门ID
     * @returns {number} 门ID
     */
    getDoorId() {
        return this.doorId;
    }

    /**
     * 获取门名称
     * @returns {string} 门名称
     */
    getDoorName() {
        return this.doorName;
    }
}
