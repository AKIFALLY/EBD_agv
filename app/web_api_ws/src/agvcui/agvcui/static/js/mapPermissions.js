/**
 * 地圖權限管理模組
 * 統一管理地圖上所有操作的權限控制
 */

import { userStore } from '../store/index.js';

export const mapPermissions = (() => {
    let currentPermissions = {};
    let permissionChangeCallbacks = [];

    // 權限等級定義
    const PERMISSION_LEVELS = {
        NONE: 0,
        VIEW: 1,
        OPERATE: 2,
        ADMIN: 3
    };

    // 角色權限映射
    const ROLE_PERMISSIONS = {
        'guest': PERMISSION_LEVELS.NONE,
        'user': PERMISSION_LEVELS.VIEW,
        'operator': PERMISSION_LEVELS.OPERATE,
        'admin': PERMISSION_LEVELS.ADMIN
    };

    // 操作權限要求
    const OPERATION_REQUIREMENTS = {
        // 查看操作
        'view_tasks': PERMISSION_LEVELS.VIEW,
        'view_racks': PERMISSION_LEVELS.VIEW,
        'view_carriers': PERMISSION_LEVELS.VIEW,
        'view_equipment': PERMISSION_LEVELS.VIEW,
        'view_agvs': PERMISSION_LEVELS.VIEW,
        'view_agv_status': PERMISSION_LEVELS.VIEW,
        'view_nodes': PERMISSION_LEVELS.VIEW,
        'view_kuka_nodes': PERMISSION_LEVELS.VIEW,

        // 創建操作
        'create_task': PERMISSION_LEVELS.OPERATE,
        'create_rack': PERMISSION_LEVELS.OPERATE,
        'create_carrier': PERMISSION_LEVELS.OPERATE,
        'create_agv': PERMISSION_LEVELS.ADMIN,
        'create_node': PERMISSION_LEVELS.OPERATE,
        'assign_task': PERMISSION_LEVELS.OPERATE,

        // 編輯操作
        'edit_task': PERMISSION_LEVELS.OPERATE,
        'edit_rack': PERMISSION_LEVELS.OPERATE,
        'edit_carrier': PERMISSION_LEVELS.OPERATE,
        'edit_node': PERMISSION_LEVELS.OPERATE,
        'control_equipment': PERMISSION_LEVELS.OPERATE,
        'control_door': PERMISSION_LEVELS.OPERATE,
        'move_agv': PERMISSION_LEVELS.OPERATE,

        // 刪除操作
        'delete_task': PERMISSION_LEVELS.ADMIN,
        'delete_rack': PERMISSION_LEVELS.ADMIN,
        'delete_carrier': PERMISSION_LEVELS.ADMIN,
        'delete_node': PERMISSION_LEVELS.ADMIN,
        'emergency_stop': PERMISSION_LEVELS.ADMIN,

        // 系統管理
        'manage_users': PERMISSION_LEVELS.ADMIN,
        'system_config': PERMISSION_LEVELS.ADMIN,
        'view_logs': PERMISSION_LEVELS.ADMIN
    };

    // 初始化
    function init() {
        updatePermissions();

        // 監聽用戶狀態變化
        userStore.on('change', () => {
            updatePermissions();
            notifyPermissionChange();
        });
    }

    // 更新權限
    function updatePermissions() {
        const userState = userStore.getState();
        const userRole = userState.role || 'guest';
        const userLevel = ROLE_PERMISSIONS[userRole] || PERMISSION_LEVELS.NONE;

        currentPermissions = {
            // 基本資訊
            isLoggedIn: userState.isLoggedIn || false,
            role: userRole,
            level: userLevel,
            username: userState.username || null,

            // 具體權限
            canView: userLevel >= PERMISSION_LEVELS.VIEW,
            canOperate: userLevel >= PERMISSION_LEVELS.OPERATE,
            canAdmin: userLevel >= PERMISSION_LEVELS.ADMIN,

            // 詳細操作權限
            operations: {}
        };

        // 計算每個操作的權限
        Object.keys(OPERATION_REQUIREMENTS).forEach(operation => {
            currentPermissions.operations[operation] = userLevel >= OPERATION_REQUIREMENTS[operation];
        });
    }

    // 檢查操作權限
    function hasPermission(operation) {
        if (!currentPermissions.operations) {
            return false;
        }
        return currentPermissions.operations[operation] || false;
    }

    // 檢查權限等級
    function hasLevel(requiredLevel) {
        return currentPermissions.level >= requiredLevel;
    }

    // 獲取當前權限
    function getPermissions() {
        return { ...currentPermissions };
    }

    // 獲取用戶資訊
    function getUserInfo() {
        return {
            isLoggedIn: currentPermissions.isLoggedIn,
            role: currentPermissions.role,
            username: currentPermissions.username,
            level: currentPermissions.level
        };
    }

    // 過濾操作按鈕
    function filterActions(actions) {
        return actions.filter(action => {
            if (!action.permission) return true;
            return hasPermission(action.permission);
        });
    }

    // 為 HTML 元素設置權限
    function applyPermissionsToElement(element, requiredPermission) {
        if (!element) return;

        const hasAccess = hasPermission(requiredPermission);

        if (hasAccess) {
            element.removeAttribute('disabled');
            element.classList.remove('is-disabled', 'permission-denied');
        } else {
            element.setAttribute('disabled', 'true');
            element.classList.add('is-disabled', 'permission-denied');
        }
    }

    // 為按鈕組設置權限
    function applyPermissionsToButtons(container, permissionMap) {
        if (!container) return;

        Object.keys(permissionMap).forEach(selector => {
            const elements = container.querySelectorAll(selector);
            elements.forEach(element => {
                applyPermissionsToElement(element, permissionMap[selector]);
            });
        });
    }

    // 創建權限提示
    function createPermissionTooltip(requiredPermission) {
        const hasAccess = hasPermission(requiredPermission);

        if (hasAccess) {
            return '';
        }

        const requiredLevel = OPERATION_REQUIREMENTS[requiredPermission];
        const requiredRole = Object.keys(ROLE_PERMISSIONS).find(
            role => ROLE_PERMISSIONS[role] >= requiredLevel
        );

        return `需要 ${requiredRole || '更高'} 權限`;
    }

    // 顯示權限不足提示
    function showPermissionDenied(operation) {
        const requiredLevel = OPERATION_REQUIREMENTS[operation];
        const requiredRole = Object.keys(ROLE_PERMISSIONS).find(
            role => ROLE_PERMISSIONS[role] >= requiredLevel
        );

        // 這裡可以整合 notify 系統
        console.warn(`權限不足：需要 ${requiredRole || '更高'} 權限才能執行 ${operation}`);

        // 可以顯示更友好的提示
        if (window.notify) {
            window.notify.warning(`權限不足：需要 ${requiredRole || '更高'} 權限`);
        }
    }

    // 安全執行操作
    function executeWithPermission(operation, callback, ...args) {
        if (hasPermission(operation)) {
            try {
                return callback(...args);
            } catch (error) {
                console.error(`執行操作 ${operation} 時發生錯誤:`, error);
                if (window.notify) {
                    window.notify.error('操作執行失敗');
                }
            }
        } else {
            showPermissionDenied(operation);
        }
    }

    // 註冊權限變化回調
    function onPermissionChange(callback) {
        if (typeof callback === 'function') {
            permissionChangeCallbacks.push(callback);
        }
    }

    // 移除權限變化回調
    function offPermissionChange(callback) {
        const index = permissionChangeCallbacks.indexOf(callback);
        if (index > -1) {
            permissionChangeCallbacks.splice(index, 1);
        }
    }

    // 通知權限變化
    function notifyPermissionChange() {
        permissionChangeCallbacks.forEach(callback => {
            try {
                callback(currentPermissions);
            } catch (error) {
                console.error('權限變化回調執行錯誤:', error);
            }
        });
    }

    // 獲取操作按鈕配置
    function getActionButtonConfig(operation, baseConfig = {}) {
        const hasAccess = hasPermission(operation);

        return {
            ...baseConfig,
            disabled: !hasAccess,
            title: hasAccess ? baseConfig.title : createPermissionTooltip(operation),
            className: `${baseConfig.className || ''} ${hasAccess ? '' : 'permission-denied'}`.trim()
        };
    }

    // 創建權限感知的按鈕
    function createPermissionButton(operation, config) {
        const buttonConfig = getActionButtonConfig(operation, config);
        const button = document.createElement('button');

        button.className = `button ${buttonConfig.className || ''}`;
        button.innerHTML = `
            <span class="icon">
                <i class="mdi ${config.icon}"></i>
            </span>
            <span>${config.text}</span>
        `;

        if (buttonConfig.disabled) {
            button.disabled = true;
            button.title = buttonConfig.title;
        } else {
            button.addEventListener('click', config.onClick);
        }

        return button;
    }

    // 公開 API
    return {
        init,
        hasPermission,
        hasLevel,
        getPermissions,
        getUserInfo,
        filterActions,
        applyPermissionsToElement,
        applyPermissionsToButtons,
        createPermissionTooltip,
        showPermissionDenied,
        executeWithPermission,
        onPermissionChange,
        offPermissionChange,
        getActionButtonConfig,
        createPermissionButton,

        // 權限等級常量
        LEVELS: PERMISSION_LEVELS,

        // 常用權限檢查方法
        canView: () => hasPermission('view_tasks'),
        canCreate: () => hasPermission('create_task'),
        canEdit: () => hasPermission('edit_task'),
        canDelete: () => hasPermission('delete_task'),
        canAdmin: () => hasLevel(PERMISSION_LEVELS.ADMIN)
    };
})();

// 全域暴露
window.mapPermissions = mapPermissions;
