/**
 * 任務狀態常數定義
 * 與資料庫 13_works_tasks.py 和後端 task_status.py 保持同步
 */

export const TASK_STATUS_ID = {
    // 基本任務流程狀態
    REQUESTING: 0,    // 請求中 - UI-請求執行任務
    PENDING: 1,       // 待處理 - WCS-任務已接受，待處理
    READY: 2,         // 待執行 - RCS-任務已派發，待執行
    EXECUTING: 3,     // 執行中 - AGV-任務正在執行
    COMPLETED: 4,     // 已完成 - AGV-任務已完成
    
    // 取消相關狀態
    CANCELLING: 5,        // 取消中 - 任務取消
    WCS_CANCELLING: 51,   // WCS-取消中 - WCS-任務取消中，待處理
    RCS_CANCELLING: 52,   // RCS-取消中 - RCS-任務取消中，取消中
    AGV_CANCELLING: 53,   // AGV-取消中 - AGV-取消完成
    CANCELLED: 54,        // 已取消 - 任務已取消
    
    // 錯誤狀態
    ERROR: 6          // 錯誤
};

export const TASK_STATUS_INFO = {
    [TASK_STATUS_ID.REQUESTING]: {
        id: TASK_STATUS_ID.REQUESTING,
        name: '請求中',
        description: 'UI-請求執行任務',
        color: 'is-info'
    },
    [TASK_STATUS_ID.PENDING]: {
        id: TASK_STATUS_ID.PENDING,
        name: '待處理',
        description: 'WCS-任務已接受，待處理',
        color: 'is-warning'
    },
    [TASK_STATUS_ID.READY]: {
        id: TASK_STATUS_ID.READY,
        name: '待執行',
        description: 'RCS-任務已派發，待執行',
        color: 'is-warning'
    },
    [TASK_STATUS_ID.EXECUTING]: {
        id: TASK_STATUS_ID.EXECUTING,
        name: '執行中',
        description: 'AGV-任務正在執行',
        color: 'is-info'
    },
    [TASK_STATUS_ID.COMPLETED]: {
        id: TASK_STATUS_ID.COMPLETED,
        name: '已完成',
        description: 'AGV-任務已完成',
        color: 'is-success'
    },
    [TASK_STATUS_ID.CANCELLING]: {
        id: TASK_STATUS_ID.CANCELLING,
        name: '取消中',
        description: '任務取消',
        color: 'is-warning'
    },
    [TASK_STATUS_ID.WCS_CANCELLING]: {
        id: TASK_STATUS_ID.WCS_CANCELLING,
        name: 'WCS-取消中',
        description: 'WCS-任務取消中，待處理',
        color: 'is-warning'
    },
    [TASK_STATUS_ID.RCS_CANCELLING]: {
        id: TASK_STATUS_ID.RCS_CANCELLING,
        name: 'RCS-取消中',
        description: 'RCS-任務取消中，取消中',
        color: 'is-warning'
    },
    [TASK_STATUS_ID.AGV_CANCELLING]: {
        id: TASK_STATUS_ID.AGV_CANCELLING,
        name: 'AGV-取消中',
        description: 'AGV-取消完成',
        color: 'is-warning'
    },
    [TASK_STATUS_ID.CANCELLED]: {
        id: TASK_STATUS_ID.CANCELLED,
        name: '已取消',
        description: '任務已取消',
        color: 'is-danger'
    },
    [TASK_STATUS_ID.ERROR]: {
        id: TASK_STATUS_ID.ERROR,
        name: '錯誤',
        description: '錯誤',
        color: 'is-danger'
    }
};

/**
 * 根據狀態 ID 獲取狀態資訊
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {Object} 狀態資訊物件
 */
export function getTaskStatusInfo(statusId) {
    // 處理 null, undefined 或空值
    if (statusId === null || statusId === undefined) {
        return { id: null, name: '未設定', description: '狀態未設定', color: 'is-light' };
    }

    // 查找對應的狀態
    const status = TASK_STATUS_INFO[statusId];
    
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
 * 根據狀態名稱獲取狀態 ID
 * @param {string} statusName - 狀態名稱
 * @returns {number|null} 狀態 ID
 */
export function getTaskStatusIdByName(statusName) {
    const status = Object.values(TASK_STATUS_INFO).find(s => s.name === statusName);
    return status ? status.id : null;
}

/**
 * 檢查狀態是否為活躍狀態（未完成且未取消）
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為活躍狀態
 */
export function isActiveStatus(statusId) {
    return [
        TASK_STATUS_ID.REQUESTING,
        TASK_STATUS_ID.PENDING,
        TASK_STATUS_ID.READY,
        TASK_STATUS_ID.EXECUTING
    ].includes(statusId);
}

/**
 * 檢查狀態是否為取消相關狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為取消相關狀態
 */
export function isCancellingStatus(statusId) {
    return [
        TASK_STATUS_ID.CANCELLING,
        TASK_STATUS_ID.WCS_CANCELLING,
        TASK_STATUS_ID.RCS_CANCELLING,
        TASK_STATUS_ID.AGV_CANCELLING,
        TASK_STATUS_ID.CANCELLED
    ].includes(statusId);
}

/**
 * 檢查狀態是否為已完成
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為已完成
 */
export function isCompletedStatus(statusId) {
    return statusId === TASK_STATUS_ID.COMPLETED;
}

/**
 * 檢查狀態是否為錯誤狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為錯誤狀態
 */
export function isErrorStatus(statusId) {
    return statusId === TASK_STATUS_ID.ERROR;
}

/**
 * 獲取所有可用的狀態選項（用於下拉選單）
 * @returns {Array} 狀態選項陣列
 */
export function getTaskStatusOptions() {
    return Object.values(TASK_STATUS_INFO).map(status => ({
        value: status.id,
        text: status.name,
        description: status.description
    }));
}
