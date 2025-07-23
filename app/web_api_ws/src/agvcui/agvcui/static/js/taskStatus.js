/**
 * 任務狀態定義模組
 * 與資料庫 13_works_tasks.py 中的 default_task_status 保持同步
 */

export const TASK_STATUS = {
    // 基本任務流程狀態
    REQUESTING: { id: 0, name: '請求中', description: 'UI-請求執行任務', color: 'is-info' },
    PENDING: { id: 1, name: '待處理', description: 'WCS-任務已接受，待處理', color: 'is-warning' },
    READY_TO_EXECUTE: { id: 2, name: '待執行', description: 'RCS-任務已派發，待執行', color: 'is-warning' },
    EXECUTING: { id: 3, name: '執行中', description: 'AGV-任務正在執行', color: 'is-info' },
    COMPLETED: { id: 4, name: '已完成', description: 'AGV-任務已完成', color: 'is-success' },

    // 取消相關狀態
    CANCELLING: { id: 5, name: '取消中', description: '任務取消', color: 'is-warning' },
    WCS_CANCELLING: { id: 51, name: 'WCS-取消中', description: 'WCS-任務取消中，待處理', color: 'is-warning' },
    RCS_CANCELLING: { id: 52, name: 'RCS-取消中', description: 'RCS-任務取消中，取消中', color: 'is-warning' },
    AGV_CANCELLING: { id: 53, name: 'AGV-取消中', description: 'AGV-取消完成', color: 'is-warning' },
    CANCELLED: { id: 54, name: '已取消', description: '任務已取消', color: 'is-danger' },

    // 錯誤狀態
    ERROR: { id: 6, name: '錯誤', description: '錯誤', color: 'is-danger' }
};

// 🔧 新增：向後相容性別名，確保現有程式碼不會中斷
export const TASK_STATUS_ALIASES = {
    READY: TASK_STATUS.READY_TO_EXECUTE  // 向後相容舊的 READY 常數
};

/**
 * 🔧 新增：驗證任務狀態 ID 的有效性
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {Object} 驗證結果物件
 */
export function validateTaskStatus(statusId) {
    const validStatusIds = Object.values(TASK_STATUS).map(status => status.id);
    const isValid = validStatusIds.includes(statusId);

    return {
        isValid: isValid,
        error: isValid ? null : `無效的狀態 ID: ${statusId}`,
        fallbackStatus: isValid ? statusId : TASK_STATUS.REQUESTING.id, // 預設為 REQUESTING
        validStatusIds: validStatusIds
    };
}

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

    // 🔧 增強：使用狀態驗證函數
    const validation = validateTaskStatus(statusId);
    if (!validation.isValid) {
        console.warn(`getTaskStatusInfo: ${validation.error}`);
        // 返回降級狀態
        const fallbackStatus = Object.values(TASK_STATUS).find(s => s.id === validation.fallbackStatus);
        return {
            ...fallbackStatus,
            name: `${fallbackStatus.name}(降級)`,
            description: `原狀態無效(${statusId})，已降級為${fallbackStatus.description}`
        };
    }

    // 查找對應的狀態
    const status = Object.values(TASK_STATUS).find(s => s.id === statusId);

    if (status) {
        return status;
    }

    // 未知狀態（理論上不應該到達這裡，因為已經過驗證）
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
export function getTaskStatusOptions() {
    return Object.values(TASK_STATUS).map(status => ({
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
export function getTaskStatusName(statusId) {
    return getTaskStatusInfo(statusId).name;
}

/**
 * 根據狀態名稱獲取狀態 ID
 * @param {string} statusName - 狀態名稱
 * @returns {number|null} 狀態 ID
 */
export function getTaskStatusIdByName(statusName) {
    const status = Object.values(TASK_STATUS).find(s => s.name === statusName);
    return status ? status.id : null;
}

/**
 * 根據狀態 ID 獲取狀態顏色類別
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {string} Bulma CSS 顏色類別
 */
export function getTaskStatusColor(statusId) {
    return getTaskStatusInfo(statusId).color;
}

/**
 * 檢查狀態是否為錯誤狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為錯誤狀態
 */
export function isErrorStatus(statusId) {
    return statusId === TASK_STATUS.ERROR.id;
}

/**
 * 檢查狀態是否為取消相關狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為取消相關狀態
 */
export function isCancellingStatus(statusId) {
    return statusId === TASK_STATUS.CANCELLING.id ||
        statusId === TASK_STATUS.WCS_CANCELLING.id ||
        statusId === TASK_STATUS.RCS_CANCELLING.id ||
        statusId === TASK_STATUS.AGV_CANCELLING.id ||
        statusId === TASK_STATUS.CANCELLED.id;
}

/**
 * 檢查狀態是否為活躍狀態（未完成且未取消）
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為活躍狀態
 */
export function isActiveStatus(statusId) {
    return statusId === TASK_STATUS.REQUESTING.id ||
        statusId === TASK_STATUS.PENDING.id ||
        statusId === TASK_STATUS.READY.id ||
        statusId === TASK_STATUS.EXECUTING.id;
}

/**
 * 檢查狀態是否為待執行狀態（包含待處理和待執行）
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為待執行狀態
 */
export function isPendingStatus(statusId) {
    return statusId === TASK_STATUS.PENDING.id ||
        statusId === TASK_STATUS.READY.id;
}

/**
 * 檢查狀態是否為執行中狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為執行中狀態
 */
export function isExecutingStatus(statusId) {
    return statusId === TASK_STATUS.EXECUTING.id;
}

/**
 * 檢查狀態是否為完成狀態
 * @param {number|null|undefined} statusId - 狀態 ID
 * @returns {boolean} 是否為完成狀態
 */
export function isCompletedStatus(statusId) {
    return statusId === TASK_STATUS.COMPLETED.id;
}

/**
 * 獲取狀態分組（用於統計）
 * @param {Array} tasks - 任務列表
 * @returns {Object} 狀態統計物件
 */
export function getTaskStatusStatistics(tasks) {
    const stats = {
        total: tasks.length,
        requesting: 0,
        pending: 0,
        ready: 0,
        executing: 0,
        completed: 0,
        cancelled: 0,
        error: 0,
        active: 0
    };

    tasks.forEach(task => {
        const statusId = task.status_id || task.status;

        switch (statusId) {
            case TASK_STATUS.REQUESTING.id:
                stats.requesting++;
                stats.active++;
                break;
            case TASK_STATUS.PENDING.id:
                stats.pending++;
                stats.active++;
                break;
            case TASK_STATUS.READY.id:
                stats.ready++;
                stats.active++;
                break;
            case TASK_STATUS.EXECUTING.id:
                stats.executing++;
                stats.active++;
                break;
            case TASK_STATUS.COMPLETED.id:
                stats.completed++;
                break;
            case TASK_STATUS.ERROR.id:
                stats.error++;
                break;
            default:
                if (isCancellingStatus(statusId)) {
                    stats.cancelled++;
                }
                break;
        }
    });

    return stats;
}
