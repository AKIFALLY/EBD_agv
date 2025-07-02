/**
 * AGV 狀態統一管理模組
 * 統一管理所有 AGV 狀態相關的定義和邏輯
 */

export const AGV_STATUS = {
    // 狀態定義
    OFFLINE: { id: 1, name: '離場', description: 'AGV 已離開工作區域', color: 'is-dark' },
    DISCONNECTED: { id: 2, name: '離線', description: 'AGV 離線或無法通訊', color: 'is-danger' },
    IDLE: { id: 3, name: '空閒', description: 'AGV 空閒，可接受新任務', color: 'is-success' },
    WORKING: { id: 4, name: '任務中', description: 'AGV 正在執行任務', color: 'is-info' },
    CHARGING: { id: 5, name: '充電中', description: 'AGV 正在充電', color: 'is-warning' },
    UPDATING: { id: 6, name: '更新中', description: 'AGV 正在進行軟體更新', color: 'is-link' },
    ERROR: { id: 7, name: '異常', description: 'AGV 發生異常或錯誤', color: 'is-danger' },
    MAINTENANCE: { id: 8, name: '維護中', description: 'AGV 正在進行維護', color: 'is-warning' },
    STANDBY: { id: 9, name: '待機', description: 'AGV 待機狀態', color: 'is-light' },
    INITIALIZING: { id: 10, name: '初始化', description: 'AGV 正在初始化', color: 'is-info' }
};

/**
 * 根據狀態 ID 獲取狀態資訊
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {Object} 狀態資訊物件
 */
export function getStatusInfo(statusId) {
    // 處理 null, undefined 或空值
    if (statusId === null || statusId === undefined) {
        return { id: null, name: '未設定', description: '狀態未設定', color: 'is-light' };
    }

    // 查找對應的狀態
    const status = Object.values(AGV_STATUS).find(s => s.id === statusId);
    
    if (status) {
        return status;
    }

    // 未知狀態
    return { 
        id: statusId, 
        name: `未知(${statusId})`, 
        description: '未知狀態', 
        color: 'is-light' 
    };
}

/**
 * 獲取所有可用的狀態選項（用於下拉選單）
 * @returns {Array} 狀態選項陣列
 */
export function getStatusOptions() {
    return Object.values(AGV_STATUS).map(status => ({
        value: status.id,
        text: status.name,
        description: status.description
    }));
}

/**
 * 根據狀態 ID 獲取狀態名稱
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {string} 狀態名稱
 */
export function getStatusName(statusId) {
    return getStatusInfo(statusId).name;
}

/**
 * 根據狀態 ID 獲取狀態顏色類別
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {string} Bulma CSS 顏色類別
 */
export function getStatusColor(statusId) {
    return getStatusInfo(statusId).color;
}

/**
 * 檢查狀態是否為錯誤狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為錯誤狀態
 */
export function isErrorStatus(statusId) {
    return statusId === AGV_STATUS.DISCONNECTED.id || statusId === AGV_STATUS.ERROR.id;
}

/**
 * 檢查狀態是否為可用狀態（可接受任務）
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為可用狀態
 */
export function isAvailableStatus(statusId) {
    return statusId === AGV_STATUS.IDLE.id || statusId === AGV_STATUS.STANDBY.id;
}

/**
 * 檢查狀態是否為忙碌狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為忙碌狀態
 */
export function isBusyStatus(statusId) {
    return statusId === AGV_STATUS.WORKING.id || 
           statusId === AGV_STATUS.CHARGING.id || 
           statusId === AGV_STATUS.UPDATING.id || 
           statusId === AGV_STATUS.MAINTENANCE.id;
}
