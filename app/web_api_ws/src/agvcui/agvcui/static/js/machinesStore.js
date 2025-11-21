// machinesStore.js - 射出机配置状态管理
// 接收 Socket.IO 推送的 machines 数据，提供 workspace 配置信息

/**
 * Machines Store - 管理射出机配置数据
 * 主要用于 workspace 货架分组显示
 */
export const machinesStore = (() => {
    // 内部状态
    let state = {
        machines: [],
        loading: false,
        error: null,
        lastUpdated: null
    };

    // 事件监听器
    const listeners = new Map();

    /**
     * 添加事件监听器
     * @param {string} event - 事件名称
     * @param {Function} callback - 回调函数
     */
    function on(event, callback) {
        if (!listeners.has(event)) {
            listeners.set(event, new Set());
        }
        listeners.get(event).add(callback);

        // 如果已有数据，立即触发一次回调
        if (event === 'change' && state.machines.length > 0) {
            try {
                callback(state);
            } catch (error) {
                console.error('Machines Store 事件处理错误:', error);
            }
        }
    }

    /**
     * 移除事件监听器
     * @param {string} event - 事件名称
     * @param {Function} callback - 回调函数
     */
    function off(event, callback) {
        if (listeners.has(event)) {
            listeners.get(event).delete(callback);
        }
    }

    /**
     * 触发事件
     * @param {string} event - 事件名称
     * @param {any} data - 事件数据
     */
    function emit(event, data) {
        if (listeners.has(event)) {
            listeners.get(event).forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    console.error(`Machines Store 事件处理错误 (${event}):`, error);
                }
            });
        }
    }

    /**
     * 获取当前状态
     * @returns {Object} 当前状态
     */
    function getState() {
        return { ...state };
    }

    /**
     * 更新射出机列表
     * @param {Array} machines - 射出机数组
     */
    function updateMachines(machines) {
        if (!Array.isArray(machines)) {
            console.error('Machines Store: updateMachines 需要数组参数');
            return;
        }

        state = {
            ...state,
            machines: machines,
            error: null,
            lastUpdated: new Date()
        };

        console.debug(`Machines Store: 更新了 ${machines.length} 台射出机配置`);
        console.debug('Machines 数据:', machines);

        // 触发变化事件
        emit('change', state);
    }

    /**
     * 根据 ID 获取射出机
     * @param {number} machineId - 射出机 ID
     * @returns {Object|null} 射出机对象或 null
     */
    function getMachineById(machineId) {
        return state.machines.find(m => m.id === machineId) || null;
    }

    /**
     * 获取所有启用的射出机
     * @returns {Array} 启用的射出机数组
     */
    function getEnabledMachines() {
        return state.machines.filter(m => m.enable === 1);
    }

    /**
     * 获取某个 location 所属的射出机和 workspace
     * @param {number} locationId - location ID
     * @returns {Object|null} {machine, workspace: 'workspace_1'|'workspace_2'} 或 null
     */
    function getMachineByLocationId(locationId) {
        for (const machine of state.machines) {
            if (machine.workspace_1 && machine.workspace_1.includes(locationId)) {
                return { machine, workspace: 'workspace_1' };
            }
            if (machine.workspace_2 && machine.workspace_2.includes(locationId)) {
                return { machine, workspace: 'workspace_2' };
            }
        }
        return null;
    }

    /**
     * 清空所有数据
     */
    function clear() {
        state = {
            machines: [],
            loading: false,
            error: null,
            lastUpdated: null
        };
        console.debug('Machines Store: 已清空所有数据');
    }

    /**
     * 设置加载状态
     * @param {boolean} loading - 是否加载中
     */
    function setLoading(loading) {
        state = { ...state, loading };
    }

    /**
     * 设置错误状态
     * @param {string|null} error - 错误信息
     */
    function setError(error) {
        state = { ...state, error };
    }

    /**
     * 初始化 Store
     */
    function init() {
        console.log('🔧 初始化 Machines Store');
        console.log('✅ Machines Store 初始化完成');
    }

    /**
     * 清理资源
     */
    function destroy() {
        listeners.clear();
        clear();
        console.log('🧹 Machines Store 资源已清理');
    }

    // 自动初始化
    init();

    // 返回公开的 API
    return {
        // 状态管理
        getState,
        updateMachines,
        setLoading,
        setError,

        // 事件系统
        on,
        off,
        emit,

        // 查询方法
        getMachineById,
        getEnabledMachines,
        getMachineByLocationId,

        // 工具方法
        clear,
        destroy
    };
})();

// 将 machinesStore 挂载到全局，方便其他模块使用
if (typeof window !== 'undefined') {
    window.machinesStore = machinesStore;
}
