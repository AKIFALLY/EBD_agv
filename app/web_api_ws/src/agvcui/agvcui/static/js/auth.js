// 認證相關的 JavaScript 功能

class AuthManager {
    constructor() {
        this.checkAuthStatus();
        this.setupEventListeners();
    }

    // 檢查認證狀態
    checkAuthStatus() {
        // 由於 cookie 設置為 HttpOnly，JavaScript 無法讀取
        // 認證檢查由服務器端中間件處理
        // 這裡只做基本的用戶狀態檢查
        console.log('Auth status check: relying on server-side middleware');

        // 如果頁面中有用戶信息，說明已經通過認證
        const user = this.getCurrentUser();
        if (user) {
            console.log(`User authenticated: ${user.username}`);
        }
    }

    // 獲取 cookie 值
    getCookie(name) {
        const value = `; ${document.cookie}`;
        const parts = value.split(`; ${name}=`);
        if (parts.length === 2) return parts.pop().split(';').shift();
        return null;
    }

    // 檢查是否為受保護頁面
    isProtectedPage() {
        const protectedPaths = ['/map', '/tasks', '/devices', '/signals', '/clients',
            '/racks', '/products', '/carriers', '/rosout_logs', '/runtime_logs'];
        const currentPath = window.location.pathname;
        return protectedPaths.some(path => currentPath.startsWith(path));
    }

    // 設置事件監聽器
    setupEventListeners() {
        // 監聽登出按鈕
        document.addEventListener('click', (e) => {
            if (e.target.closest('a[href="/logout"]')) {
                e.preventDefault();
                this.logout();
            }
        });

        // 監聽 401 錯誤（未授權）
        window.addEventListener('unhandledrejection', (event) => {
            if (event.reason && event.reason.status === 401) {
                this.handleUnauthorized();
            }
        });
    }

    // 處理登出
    logout() {
        console.log('🚪 前端登出處理開始');

        // 多種方式清除 cookie
        document.cookie = 'access_token=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/; max-age=0;';
        document.cookie = 'access_token=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/app; max-age=0;';
        document.cookie = 'access_token=; expires=Thu, 01 Jan 1970 00:00:00 UTC; max-age=0;';

        // 清除 localStorage 和 sessionStorage
        localStorage.clear();
        sessionStorage.clear();

        // 清除頁面狀態
        if (typeof window.currentUser !== 'undefined') {
            delete window.currentUser;
        }

        console.log('🧹 已清除前端狀態');

        // 重定向到登入頁面
        window.location.href = '/logout';  // 讓服務器處理登出
    }

    // 處理未授權錯誤
    handleUnauthorized() {
        // 清除無效的 token
        document.cookie = 'access_token=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/;';

        // 顯示提示訊息
        if (typeof notify !== 'undefined') {
            notify.showErrorMessage('登入已過期，請重新登入');
        }

        // 重定向到登入頁面
        setTimeout(() => {
            window.location.href = `/login?redirect=${encodeURIComponent(window.location.pathname)}`;
        }, 2000);
    }

    // 檢查 token 是否即將過期
    checkTokenExpiration() {
        // 由於 cookie 是 HttpOnly，JavaScript 無法讀取
        // Token 過期檢查由服務器端處理
        // 如果 token 過期，服務器會自動重定向到登入頁面
        console.log('Token expiration check: handled by server-side middleware');

        // 檢查用戶是否仍然存在於頁面中
        const user = this.getCurrentUser();
        if (!user && this.isProtectedPage()) {
            console.warn('User info missing on protected page');
        }
    }

    // 自動刷新 token（如果需要）
    refreshToken() {
        // 這裡可以實現 token 刷新邏輯
        // 目前的實現不支持 refresh token，所以暫時跳過
        console.log('Token refresh not implemented');
    }

    // 獲取當前用戶信息（從 userStore 中獲取）
    getCurrentUser() {
        // 動態導入 userStore 並獲取用戶信息
        try {
            // 如果 userStore 已經可用，直接使用
            if (typeof window.userStore !== 'undefined') {
                const userState = window.userStore.getState();
                return userState.isLoggedIn ? userState : null;
            }

            // 否則嘗試從 localStorage 讀取
            const userState = localStorage.getItem('userState');
            if (userState) {
                const parsed = JSON.parse(userState);
                return parsed.isLoggedIn ? parsed : null;
            }
        } catch (error) {
            console.warn('無法獲取用戶信息:', error);
        }
        return null;
    }

    // 檢查用戶權限
    hasPermission(requiredRole) {
        const user = this.getCurrentUser();
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
}

// 初始化認證管理器
const authManager = new AuthManager();

// 移除自動登出功能 - 不再定期檢查 token 過期
// 用戶登入狀態現在由 Socket.IO 和 userStore 管理

// 導出給其他模組使用
if (typeof window !== 'undefined') {
    window.authManager = authManager;
}
