/**
 * 停車格狀態常數定義
 * 與後端 parking_status.py 保持同步
 */

export const PARKING_STATUS_ID = {
    AVAILABLE: 0,       // 可用 - 停車格空閒，可以叫車
    TASK_ACTIVE: 1,     // 任務進行中 - 已叫車，等待AGV送達
    TASK_COMPLETED: 2   // 任務完成 - 車輛已送達，等待確認取貨
};

export const PARKING_STATUS_INFO = {
    [PARKING_STATUS_ID.AVAILABLE]: {
        id: PARKING_STATUS_ID.AVAILABLE,
        name: '可用',
        description: '停車格空閒，可以叫車',
        color: 'is-light'
    },
    [PARKING_STATUS_ID.TASK_ACTIVE]: {
        id: PARKING_STATUS_ID.TASK_ACTIVE,
        name: '任務進行中',
        description: '已叫車，等待AGV送達',
        color: 'is-warning'
    },
    [PARKING_STATUS_ID.TASK_COMPLETED]: {
        id: PARKING_STATUS_ID.TASK_COMPLETED,
        name: '任務完成',
        description: '車輛已送達，等待確認取貨',
        color: 'is-success'
    }
};

/**
 * 根據狀態 ID 獲取狀態資訊
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {Object} 狀態資訊物件
 */
export function getParkingStatusInfo(statusId) {
    // 處理 null, undefined 或空值
    if (statusId === null || statusId === undefined) {
        return { id: null, name: '未設定', description: '狀態未設定', color: 'is-light' };
    }

    // 查找對應的狀態
    const status = PARKING_STATUS_INFO[statusId];
    
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
 * 檢查停車格是否可用
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否可用
 */
export function isParkingAvailable(statusId) {
    return statusId === PARKING_STATUS_ID.AVAILABLE;
}

/**
 * 檢查停車格是否有進行中的任務
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否有進行中的任務
 */
export function isParkingTaskActive(statusId) {
    return statusId === PARKING_STATUS_ID.TASK_ACTIVE;
}

/**
 * 檢查停車格任務是否已完成
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 任務是否已完成
 */
export function isParkingTaskCompleted(statusId) {
    return statusId === PARKING_STATUS_ID.TASK_COMPLETED;
}
