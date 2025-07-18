import { tasksStore } from '../store/index.js';
import { notify } from './notify.js';
import { getTaskStatusInfo, getTaskStatusName, getTaskStatusIdByName } from './taskStatus.js';

export const tasksPage = (() => {
    let currentTasks = []; // 當前顯示的任務列表
    let currentAgvId = null; // 當前篩選的 AGV ID

    /**
     * 處理 tasksStore 變化事件
     * @param {Object} newState - 新的任務狀態
     */
    function handleTasksChange(newState) {
        if (!newState?.tasks) return;

        const allTasks = newState.tasks || [];
        console.debug('收到任務更新:', allTasks.length, '個任務');

        // 根據當前篩選條件更新任務列表
        updateTasksDisplay(allTasks);
    }

    /**
     * 更新任務顯示
     * @param {Array} allTasks - 所有任務數據
     */
    function updateTasksDisplay(allTasks) {
        // 根據當前 AGV ID 篩選任務
        let filteredTasks = allTasks;
        if (currentAgvId) {
            filteredTasks = allTasks.filter(task => task.agv_id === currentAgvId);
        }

        currentTasks = filteredTasks;

        // 更新頁面上的任務值
        updateTaskValues(filteredTasks);

        // 更新統計信息
        updateTaskStats(filteredTasks, allTasks);
    }

    /**
     * 更新頁面上的任務值顯示（優化版本 - 使用唯一 ID 和精確變化檢測）
     * @param {Array} tasks - 要顯示的任務列表
     */
    function updateTaskValues(tasks) {
        tasks.forEach(task => {
            updateTaskRowOptimized(task.id, task);
        });
    }

    /**
     * 優化的任務行更新函數（只更新變化的欄位，不重建 DOM 結構）
     * @param {number} taskId - 任務 ID
     * @param {Object} newTask - 新的任務資料
     */
    function updateTaskRowOptimized(taskId, newTask) {
        let hasChanges = false;

        // 更新任務狀態（帶變化檢測和詳細 debug）
        const statusElement = document.getElementById(`task-status-${taskId}`);
        if (statusElement) {
            const oldStatus = extractStatusFromTag(statusElement);
            const newStatus = newTask.status_id;

            console.debug(`任務 ${taskId} 狀態檢測: 舊值="${oldStatus}" (${typeof oldStatus}), 新值="${newStatus}" (${typeof newStatus})`);

            if (hasChanged(oldStatus, newStatus)) {
                updateTaskStatusTag(statusElement, newTask.status_id);
                // 統一動畫目標：應用到 td 元素
                const statusTdElement = statusElement.closest('td');
                addUpdateAnimation(statusTdElement);
                hasChanges = true;
                console.debug(`任務 ${newTask.name || taskId} 狀態更新: "${oldStatus}" → "${newStatus}"`);
            } else {
                console.debug(`任務 ${taskId} 狀態無變化，跳過動畫`);
            }
        }

        // 更新 AGV 分配（帶變化檢測）
        const agvElement = document.getElementById(`task-agv-${taskId}`);
        if (agvElement) {
            const oldAgvId = extractAgvFromTag(agvElement);
            const newAgvId = newTask.agv_id;

            if (hasChanged(oldAgvId, newAgvId)) {
                updateTaskAgvTag(agvElement, newTask.agv, newTask.agv_id);
                // 統一動畫目標：應用到 td 元素
                const agvTdElement = agvElement.closest('td');
                addUpdateAnimation(agvTdElement);
                hasChanges = true;
                console.debug(`任務 ${newTask.name || taskId} AGV 更新: ${oldAgvId} → ${newAgvId}`);
            }
        }

        // 更新任務代碼（帶變化檢測）
        const missionElement = document.getElementById(`task-mission-${taskId}`);
        if (missionElement) {
            const oldMission = extractMissionFromTag(missionElement);
            const newMission = newTask.mission_code;

            if (hasChanged(oldMission, newMission)) {
                updateTaskMissionTag(missionElement, newTask.mission_code);
                // 統一動畫目標：應用到 td 元素
                const missionTdElement = missionElement.closest('td');
                addUpdateAnimation(missionTdElement);
                hasChanges = true;
            }
        }

        // 更新時間戳（帶變化檢測）
        const timestampElement = document.getElementById(`task-timestamp-${taskId}`);
        if (timestampElement && newTask.updated_at) {
            const oldTimestamp = timestampElement.textContent;
            const newTimestamp = new Date(newTask.updated_at).toLocaleString();

            if (hasChanged(oldTimestamp, newTimestamp)) {
                timestampElement.textContent = newTimestamp;
                // 統一動畫目標：應用到 td 元素
                const timestampTdElement = timestampElement.closest('td');
                addUpdateAnimation(timestampTdElement);
                hasChanges = true;
            }
        }

        // 記錄變化但不添加整行動畫
        if (hasChanges) {
            console.debug(`任務 ${newTask.name || taskId} 資料已更新`);
        }
    }

    /**
     * 精確的變化檢測函數
     * @param {any} oldValue - 舊值
     * @param {any} newValue - 新值
     * @returns {boolean} 是否有變化
     */
    function hasChanged(oldValue, newValue) {
        return oldValue !== newValue;
    }

    /**
     * 添加更新動畫效果（帶防重疊機制）
     * @param {Element} element - 要添加動畫的元素
     */
    function addUpdateAnimation(element) {
        if (!element) return;

        // 檢查是否已經在播放動畫
        if (element.classList.contains('task-updated')) {
            console.debug('Task 動畫進行中，跳過重複添加');
            return;
        }

        element.classList.add('task-updated');
        setTimeout(() => {
            element.classList.remove('task-updated');
        }, 1000); // 與 CSS 動畫持續時間一致
    }

    /**
     * 從狀態標籤中提取狀態 ID
     * @param {Element} statusElement - 狀態標籤元素
     * @returns {number} 狀態 ID
     */
    function extractStatusFromTag(statusElement) {
        const text = statusElement.textContent.trim();
        // 使用統一的狀態映射函數
        return getTaskStatusIdByName(text);
    }

    /**
     * 從 AGV 標籤中提取 AGV ID
     * @param {Element} agvElement - AGV 標籤元素
     * @returns {number} AGV ID
     */
    function extractAgvFromTag(agvElement) {
        const text = agvElement.textContent.trim();
        if (text === '未分配') return null;
        const match = text.match(/AGV (\d+)/);
        return match ? parseInt(match[1]) : null;
    }

    /**
     * 從任務代碼標籤中提取代碼
     * @param {Element} missionElement - 任務代碼標籤元素
     * @returns {string} 任務代碼
     */
    function extractMissionFromTag(missionElement) {
        const text = missionElement.textContent.trim();
        return text === '未設定' ? null : text;
    }

    /**
     * 更新任務狀態標籤
     * @param {Element} statusElement - 狀態標籤元素
     * @param {number} statusId - 狀態 ID
     */
    function updateTaskStatusTag(statusElement, statusId) {
        const statusInfo = getTaskStatusInfo(statusId);
        statusElement.className = `tag ${statusInfo.color}`;
        statusElement.textContent = statusInfo.name;
    }

    /**
     * 更新任務 AGV 標籤
     * @param {Element} agvElement - AGV 標籤元素
     * @param {Object} agv - AGV 對象
     * @param {number} agvId - AGV ID
     */
    function updateTaskAgvTag(agvElement, agv, agvId) {
        if (agv && agv.name) {
            agvElement.className = 'tag is-success';
            agvElement.innerHTML = `
                <span class="icon">
                    <i class="mdi mdi-robot"></i>
                </span>
                <span>${agv.name}</span>
            `;
        } else if (agvId) {
            agvElement.className = 'tag is-warning';
            agvElement.textContent = `AGV ${agvId}`;
        } else {
            agvElement.className = 'tag is-light';
            agvElement.textContent = '未分配';
        }
    }

    /**
     * 更新任務代碼標籤
     * @param {Element} missionElement - 任務代碼標籤元素
     * @param {string} missionCode - 任務代碼
     */
    function updateTaskMissionTag(missionElement, missionCode) {
        if (missionCode) {
            missionElement.className = 'tag is-info';
            missionElement.innerHTML = `
                <span class="icon">
                    <i class="mdi mdi-barcode"></i>
                </span>
                <span>${missionCode}</span>
            `;
        } else {
            missionElement.className = 'tag is-light';
            missionElement.textContent = '未設定';
        }
    }

    /**
     * 更新統計信息
     * @param {Array} filteredTasks - 當前篩選的任務
     * @param {Array} allTasks - 所有任務
     */
    function updateTaskStats(filteredTasks, allTasks) {
        // 更新標題中的任務數量
        const titleTag = document.querySelector('.level-item .tag');
        if (titleTag) {
            const count = filteredTasks.length;
            const agvName = currentAgvId ? `AGV ${currentAgvId}` : '所有任務';
            titleTag.innerHTML = `
                <span class="icon">
                    <i class="mdi ${currentAgvId ? 'mdi-robot' : 'mdi-format-list-checks'}"></i>
                </span>
                <span>${agvName} (${count} 個任務)</span>
            `;
        }
    }

    /**
     * 初始化頁面
     */
    function setup() {
        console.log('🔧 初始化 Tasks 頁面');

        // 獲取當前選中的 AGV ID（從URL參數）
        const urlParams = new URLSearchParams(window.location.search);
        currentAgvId = urlParams.get('agv_id') ? parseInt(urlParams.get('agv_id')) : null;

        // 監聽 tasksStore 變化
        tasksStore.on('change', handleTasksChange);

        // 初始化時獲取當前狀態並顯示
        const currentState = tasksStore.getState();
        if (currentState.tasks) {
            handleTasksChange(currentState);
        }

        console.log('✅ Tasks 頁面初始化完成');
    }

    /**
     * 清理資源
     */
    function cleanup() {
        tasksStore.off('change', handleTasksChange);
        console.log('🧹 Tasks 頁面資源已清理');
    }

    // 返回公開的方法
    return {
        setup,
        cleanup
    };
})();
