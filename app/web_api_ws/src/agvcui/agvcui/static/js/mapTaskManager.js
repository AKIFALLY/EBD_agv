/**
 * 地圖任務管理器
 * 處理地圖上的任務相關功能
 */

import { getTaskStatusInfo, getTaskStatusName } from './taskStatus.js';

export const mapTaskManager = (() => {
    let taskData = new Map();

    // 初始化
    function init() {
        loadTaskData();

        // 監聽 tasksStore 的變化
        if (window.tasksStore) {
            window.tasksStore.on('change', handleTasksChange);
            console.debug('mapTaskManager: 已訂閱 tasksStore 變化');
        }
    }

    // 處理 tasksStore 變化
    function handleTasksChange(newState) {
        if (!newState?.tasks) {
            console.debug('mapTaskManager.handleTasksChange: 無效的狀態資料');
            return;
        }

        const tasks = newState.tasks || [];
        console.debug(`mapTaskManager.handleTasksChange: 收到任務更新，共 ${tasks.length} 個任務`);

        // 更新本地資料
        taskData.clear();
        tasks.forEach(task => {
            taskData.set(task.id, task);
        });

        // 如果側邊面板正在顯示任務列表，只更新內容
        const tasksList = document.getElementById('tasks-list');
        if (tasksList && tasksList.children.length > 0) {
            console.debug('mapTaskManager.handleTasksChange: 側邊面板正在顯示，更新任務列表內容');
            updateTasksListContent(tasksList, tasks);
        } else {
            console.debug('mapTaskManager.handleTasksChange: 側邊面板未顯示或為空，跳過更新');
        }
    }

    // 載入任務資料
    async function loadTaskData() {
        try {
            // 使用現有的 store 資料而不是 API 調用
            if (window.tasksStore) {
                const tasksState = window.tasksStore.getState();
                const tasks = tasksState.tasks || [];

                taskData.clear();
                tasks.forEach(task => {
                    taskData.set(task.id, task);
                });

                console.debug(`Loaded ${tasks.length} tasks from store`);
            } else {
                console.warn('tasksStore not available');
            }
        } catch (error) {
            console.error('Error loading task data:', error);
        }
    }

    // 獲取任務資料
    function getTaskData(taskId) {
        return taskData.get(taskId);
    }

    // 獲取所有任務資料
    function getAllTaskData() {
        return Array.from(taskData.values());
    }

    // 根據狀態獲取任務
    function getTasksByStatus(statusId) {
        return Array.from(taskData.values()).filter(task => task.status_id === statusId);
    }

    // 根據 AGV 獲取任務
    function getTasksByAgv(agvId) {
        return Array.from(taskData.values()).filter(task => task.agv_id === agvId);
    }

    // 顯示任務彈出視窗
    function showTaskPopup(task, position) {
        if (!window.mapInteraction) {
            console.warn('mapInteraction not available');
            return;
        }

        const title = `任務: ${task.name || task.id}`;

        const content = `
            <div class="map-info-card">
                <div class="field">
                    <label class="label">任務 ID</label>
                    <div class="control">
                        <span class="tag is-info">${task.id}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">任務名稱</label>
                    <div class="control">
                        <span>${task.name || '未命名'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">狀態</label>
                    <div class="control">
                        <span class="tag ${getTaskStatusColor(task.status_id)}">${getTaskStatusNameLocal(task.status_id)}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">AGV</label>
                    <div class="control">
                        <span>${task.agv_id ? `AGV ${task.agv_id}` : '未分配'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">起始位置</label>
                    <div class="control">
                        <span>${task.start_location_id || '未設定'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">目標位置</label>
                    <div class="control">
                        <span>${task.end_location_id || '未設定'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">優先級</label>
                    <div class="control">
                        <span class="tag ${getPriorityColor(task.priority)}">${task.priority || 'Normal'}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">創建時間</label>
                    <div class="control">
                        <span>${task.created_at ? new Date(task.created_at).toLocaleString() : '未知'}</span>
                    </div>
                </div>
                ${task.description ? `
                <div class="field">
                    <label class="label">描述</label>
                    <div class="control">
                        <span>${task.description}</span>
                    </div>
                </div>
                ` : ''}
            </div>
        `;

        const actions = [];

        // 根據權限和任務狀態顯示不同的操作按鈕
        if (window.mapPermissions) {
            if (window.mapPermissions.hasPermission('edit_task')) {
                actions.push({
                    text: '編輯任務',
                    icon: 'mdi-pencil',
                    class: 'is-primary',
                    onclick: `mapTaskManager.editTask(${task.id})`,
                    permission: 'edit_task'
                });
            }

            if (window.mapPermissions.hasPermission('delete_task')) {
                actions.push({
                    text: '刪除任務',
                    icon: 'mdi-delete',
                    class: 'is-danger',
                    onclick: `mapTaskManager.deleteTask(${task.id})`,
                    permission: 'delete_task'
                });
            }

            // 根據任務狀態顯示不同操作
            if (task.status_id === 1) { // 待執行
                actions.push({
                    text: '開始執行',
                    icon: 'mdi-play',
                    class: 'is-success',
                    onclick: `mapTaskManager.startTask(${task.id})`,
                    permission: 'execute_task'
                });
            } else if (task.status_id === 2) { // 執行中
                actions.push({
                    text: '暫停任務',
                    icon: 'mdi-pause',
                    class: 'is-warning',
                    onclick: `mapTaskManager.pauseTask(${task.id})`,
                    permission: 'execute_task'
                });
            }
        }

        actions.push({
            text: '查看詳情',
            icon: 'mdi-information',
            class: 'is-info',
            onclick: `window.open('/tasks/${task.id}', '_blank')`
        });

        window.mapInteraction.showPopup(position, title, content, actions);
    }

    // 獲取任務狀態顏色（使用統一的狀態定義）
    function getTaskStatusColor(statusId) {
        return getTaskStatusInfo(statusId).color;
    }

    // 獲取任務狀態名稱（使用統一的狀態定義）
    function getTaskStatusNameLocal(statusId) {
        return getTaskStatusName(statusId);
    }

    // 獲取優先級顏色
    function getPriorityColor(priority) {
        switch (priority?.toLowerCase()) {
            case 'high': return 'is-danger';
            case 'medium': return 'is-warning';
            case 'low': return 'is-success';
            default: return 'is-info';
        }
    }

    // 編輯任務
    function editTask(taskId) {
        if (window.mapPermissions && window.mapPermissions.hasPermission('edit_task')) {
            window.open(`/tasks/${taskId}/edit`, '_blank');
        }
    }

    // 刪除任務
    function deleteTask(taskId) {
        if (window.mapPermissions && window.mapPermissions.hasPermission('delete_task')) {
            if (confirm('確定要刪除這個任務嗎？')) {
                // 這裡可以調用 API 刪除任務
                console.debug('Delete task:', taskId);
                // TODO: 實作刪除 API 調用
            }
        }
    }

    // 開始任務
    function startTask(taskId) {
        if (window.mapPermissions && window.mapPermissions.hasPermission('execute_task')) {
            console.debug('Start task:', taskId);
            // TODO: 實作開始任務 API 調用
        }
    }

    // 暫停任務
    function pauseTask(taskId) {
        if (window.mapPermissions && window.mapPermissions.hasPermission('execute_task')) {
            console.debug('Pause task:', taskId);
            // TODO: 實作暫停任務 API 調用
        }
    }

    // 更新任務列表內容（保持 DOM 結構）
    function updateTasksListContent(tasksList, tasks) {
        console.debug('mapTaskManager.updateTasksListContent: 開始更新任務列表內容');

        // 防止重複更新
        if (tasksList._isUpdating) {
            console.debug('mapTaskManager.updateTasksListContent: 正在更新中，跳過此次更新');
            return;
        }
        tasksList._isUpdating = true;

        try {
            // 保存當前滾動位置
            const scrollTop = tasksList.scrollTop;

            // 按 ID 排序任務
            const sortedTasks = tasks.sort((a, b) => a.id - b.id);

            // 更新總數資訊
            let totalInfoElement = tasksList.querySelector('.total-info');
            if (!totalInfoElement) {
                totalInfoElement = document.createElement('div');
                totalInfoElement.className = 'total-info has-text-grey is-size-7 mb-2';
                tasksList.insertBefore(totalInfoElement, tasksList.firstChild);
            }
            totalInfoElement.textContent = `總計 ${tasks.length} 個任務`;

            // 簡化更新邏輯：直接重建任務項目
            // 移除所有現有的任務項目（保留總數資訊）
            const existingItems = Array.from(tasksList.querySelectorAll('.task-item'));
            existingItems.forEach(item => item.remove());

            // 重新創建所有任務項目
            sortedTasks.forEach(task => {
                const itemElement = document.createElement('div');
                itemElement.className = 'task-item';

                // 使用原本的跳轉行為而不是彈出視窗
                itemElement.setAttribute('onclick', `window.mapInteraction.viewTaskDetails(${task.id})`);

                itemElement.innerHTML = `
                    <div class="level is-mobile">
                        <div class="level-left">
                            <div class="level-item">
                                <div>
                                    <p class="title is-6">${task.name || `任務 ${task.id}`}</p>
                                    <p class="subtitle is-7">
                                        <span class="tag is-small ${getTaskStatusColor(task.status_id)}">${getTaskStatusNameLocal(task.status_id)}</span>
                                        ${task.agv_id ? `<span class="tag is-small is-light">AGV ${task.agv_id}</span>` : ''}
                                    </p>
                                </div>
                            </div>
                        </div>
                        <div class="level-right">
                            <div class="level-item">
                                <span class="icon">
                                    <i class="mdi mdi-chevron-right"></i>
                                </span>
                            </div>
                        </div>
                    </div>
                `;

                tasksList.appendChild(itemElement);
            });

            // 恢復滾動位置
            tasksList.scrollTop = scrollTop;

            console.debug(`mapTaskManager.updateTasksListContent: 已更新 ${tasks.length} 個任務項目`);
        } catch (error) {
            console.error('mapTaskManager.updateTasksListContent: 更新失敗', error);
        } finally {
            // 重置更新標誌
            tasksList._isUpdating = false;
        }
    }

    // 載入任務列表到側邊面板
    function loadTasksList() {
        console.debug('mapTaskManager.loadTasksList: 開始載入任務列表');

        const tasks = getAllTaskData();
        const tasksList = document.getElementById('tasks-list');

        if (!tasksList) {
            console.warn('mapTaskManager.loadTasksList: tasks-list 元素未找到');
            return;
        }

        console.debug(`mapTaskManager.loadTasksList: 找到 ${tasks.length} 個任務`);

        if (tasks.length === 0) {
            tasksList.innerHTML = '<p class="has-text-grey">目前沒有任務</p>';
            console.debug('mapTaskManager.loadTasksList: 沒有任務，顯示空狀態');
            return;
        }

        // 如果列表已經有內容，使用更新方式
        if (tasksList.children.length > 0) {
            console.debug('mapTaskManager.loadTasksList: 列表已有內容，使用更新方式');
            updateTasksListContent(tasksList, tasks);
            return;
        }

        // 初次載入時使用完整重建
        console.debug('mapTaskManager.loadTasksList: 初次載入，使用完整重建');
        const sortedTasks = tasks.sort((a, b) => a.id - b.id);

        const tasksHtml = sortedTasks.map(task => `
            <div class="task-item" onclick="window.mapInteraction.viewTaskDetails(${task.id})">
                <div class="level is-mobile">
                    <div class="level-left">
                        <div class="level-item">
                            <div>
                                <p class="title is-6">${task.name || `任務 ${task.id}`}</p>
                                <p class="subtitle is-7">
                                    <span class="tag is-small ${getTaskStatusColor(task.status_id)}">${getTaskStatusNameLocal(task.status_id)}</span>
                                    ${task.agv_id ? `<span class="tag is-small is-light">AGV ${task.agv_id}</span>` : ''}
                                </p>
                            </div>
                        </div>
                    </div>
                    <div class="level-right">
                        <div class="level-item">
                            <span class="icon">
                                <i class="mdi mdi-chevron-right"></i>
                            </span>
                        </div>
                    </div>
                </div>
            </div>
        `).join('');

        const totalInfo = `<div class="total-info has-text-grey is-size-7 mb-2">總計 ${tasks.length} 個任務</div>`;
        tasksList.innerHTML = totalInfo + tasksHtml;

        console.debug(`mapTaskManager.loadTasksList: 已載入 ${tasks.length} 個任務（完整重建）`);
    }

    // 公開方法
    return {
        init,
        loadTaskData,
        getTaskData,
        getAllTaskData,
        getTasksByStatus,
        getTasksByAgv,
        showTaskPopup,
        editTask,
        deleteTask,
        startTask,
        pauseTask,
        loadTasksList
    };
})();

// 全域暴露
window.mapTaskManager = mapTaskManager;
