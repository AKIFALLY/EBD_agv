/**
 * Auth Manager - 統一管理用戶認證狀態
 * 在 Socket.IO 連線後初始化，負責用戶狀態同步
 */

import { userStore } from '../store/index.js';
import socket from './socket.js';

export const authManager = (() => {
    let isInitialized = false;

    async function initializeUserState() {
        if (isInitialized) return;

        console.log('AuthManager: 開始初始化用戶狀態');

        // 檢查當前用戶狀態
        const currentUserState = userStore.getState();
        console.log('AuthManager: 當前用戶狀態', currentUserState);

        // 如果用戶已登入，更新連線狀態
        if (currentUserState.isLoggedIn) {
            userStore.setState({
                isConnected: socket.socket && socket.socket.connected
            });
            console.log('AuthManager: 用戶已登入，更新連線狀態');
        } else {
            console.log('AuthManager: 用戶未登入');
        }

        // 監聽用戶狀態變化
        userStore.on('change', handleUserStateChange);

        isInitialized = true;
        console.log('AuthManager: 初始化完成');
    }



    function handleUserStateChange(newState) {
        console.log('AuthManager: 用戶狀態變化', newState);

        // 可以在這裡添加其他需要響應用戶狀態變化的邏輯
        // 例如：權限檢查、頁面重定向等

        // 如果用戶登出，可能需要清理一些狀態
        if (!newState.isLoggedIn) {
            console.log('AuthManager: 用戶已登出，清理狀態');
            // 這裡可以添加登出後的清理邏輯
        }
    }

    function getCurrentUser() {
        const userState = userStore.getState();
        return userState.isLoggedIn ? userState : null;
    }

    function isLoggedIn() {
        const userState = userStore.getState();
        return userState.isLoggedIn === true;
    }

    function hasRole(requiredRole) {
        const user = getCurrentUser();
        if (!user) return false;

        const roleHierarchy = {
            'admin': 3,
            'operator': 2,
            'user': 1
        };

        const userLevel = roleHierarchy[user.role] || 0;
        const requiredLevel = roleHierarchy[requiredRole] || 0;

        return userLevel >= requiredLevel;
    }

    function requireAuth() {
        if (!isLoggedIn()) {
            console.log('AuthManager: 需要登入，重定向到登入頁面');
            window.location.href = '/login?redirect=' + encodeURIComponent(window.location.pathname);
            return false;
        }
        return true;
    }

    function requireRole(requiredRole) {
        if (!requireAuth()) return false;

        if (!hasRole(requiredRole)) {
            console.log('AuthManager: 權限不足，需要角色:', requiredRole);
            alert('權限不足，需要 ' + requiredRole + ' 權限');
            return false;
        }
        return true;
    }

    function setup() {
        console.log('AuthManager: 開始設置');

        // 直接初始化用戶狀態（因為這個函數只會在 socket 連線後被調用）
        initializeUserState();
    }

    return {
        setup,
        getCurrentUser,
        isLoggedIn,
        hasRole,
        requireAuth,
        requireRole,
        initializeUserState
    };
})();
