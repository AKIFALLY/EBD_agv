import { BaseObject } from './BaseObject.js';

/**
 * TrafficZoneInfoObject - 交管區狀態显示物件
 *
 * 显示内容：
 * - 标题：交管區名称（例如"交管區1"）
 * - 内容：狀態图标 + 占用者 AGV 名称
 *
 * 狀態类型：
 * - free: 空闲（绿色 ✓）
 * - controlled: 被控制/占用（橙色 ⚠）- 主要狀態
 * - occupied: 占用（橙色 ⚠）- 向後兼容
 * - disabled: 禁用（灰色 ✕）
 */
export class TrafficZoneInfoObject extends BaseObject {
    constructor(map, latlng, id, name = "交管區") {
        const html = `
<div id="traffic-zone-${id}" class="traffic-zone-info">
    <div class="traffic-zone-title has-radius-top-left has-radius-top-right">
        <span>${name}</span>
    </div>
    <div class="traffic-zone-content has-radius-bottom-left has-radius-bottom-right">
        <span class="traffic-zone-status-icon">✓</span>
        <span class="traffic-zone-owner">---</span>
    </div>
</div>
`;
        // iconSize: [120, 80], iconAnchor: [60, 40], zIndex: 1800
        super(map, latlng, html, [120, 80], [60, 40], 1800);

        this.id = `traffic-zone-${id}`;
        this.trafficZoneId = id;
        this.trafficZoneName = name;
        this.currentStatus = 'free';

        // 添加點擊事件以打開控制 Modal
        this.addClickHandler(() => {
            // 動態導入 modal 模組並打開
            import('../js/mapTrafficZoneModal.js').then(module => {
                const { mapTrafficZoneModal } = module;
                mapTrafficZoneModal.openModal(this.trafficZoneId, this.trafficZoneName);
            }).catch(error => {
                console.error('無法載入 mapTrafficZoneModal:', error);
            });
        });
    }

    /**
     * 更新交管區狀態
     * @param {Object} data - 交管區数据
     * @param {string} data.status - 狀態：'free' | 'controlled' | 'occupied' | 'disabled'
     * @param {string|null} data.ownerAgvName - 占用者 AGV 名称
     * @param {boolean} data.enabled - 是否启用
     */
    update(data) {
        const infoElem = document.getElementById(this.id);
        if (!infoElem) return;

        const statusIcon = infoElem.querySelector('.traffic-zone-status-icon');
        const ownerElem = infoElem.querySelector('.traffic-zone-owner');
        const contentDiv = infoElem.querySelector('.traffic-zone-content');

        if (!statusIcon || !ownerElem || !contentDiv) return;

        // 确定实际狀態
        let actualStatus = data.status || 'free';
        if (data.enabled === false) {
            actualStatus = 'disabled';
        }

        // 更新狀態图标和样式
        switch (actualStatus) {
            case 'free':
                statusIcon.textContent = '✓';
                contentDiv.classList.remove('status-controlled', 'status-occupied', 'status-disabled');
                contentDiv.classList.add('status-free');
                ownerElem.textContent = '';
                break;

            case 'controlled':  // TrafficController 使用的狀態（主要）
            case 'occupied':    // 向後兼容
                statusIcon.textContent = '⚠';
                contentDiv.classList.remove('status-free', 'status-disabled');
                contentDiv.classList.add('status-controlled');
                ownerElem.textContent = data.ownerAgvName || '???';
                break;

            case 'disabled':
                statusIcon.textContent = '✕';
                contentDiv.classList.remove('status-free', 'status-controlled', 'status-occupied');
                contentDiv.classList.add('status-disabled');
                ownerElem.textContent = '禁用';
                break;
        }

        this.currentStatus = actualStatus;

        // 存储数据到物件
        this.data = {
            status: actualStatus,
            ownerAgvName: data.ownerAgvName,
            enabled: data.enabled
        };
    }

    /**
     * 获取当前狀態
     * @returns {string} 当前狀態
     */
    getStatus() {
        return this.currentStatus;
    }

    /**
     * 获取占用者 AGV 名称
     * @returns {string|null} AGV 名称
     */
    getOwnerAgvName() {
        return this.data?.ownerAgvName || null;
    }
}
