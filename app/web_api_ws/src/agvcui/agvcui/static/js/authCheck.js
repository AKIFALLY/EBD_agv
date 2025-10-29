/**
 * 認證狀態檢查模組
 *
 * 功能：
 * - 頁面載入時檢查 Cookie 和 localStorage 的一致性
 * - 當 Cookie 過期但 localStorage 仍保留登入狀態時，自動清除並導向登入頁
 * - 輕量級實作，只在頁面載入時執行一次
 */

import { userStore } from '/static/store/index.js';

/**
 * 檢查 Cookie 中是否存在指定的 key
 * @param {string} name - Cookie 名稱
 * @returns {boolean} - Cookie 是否存在
 */
function hasCookie(name) {
    const cookies = document.cookie.split(';');
    for (let cookie of cookies) {
        const trimmed = cookie.trim();
        if (trimmed.startsWith(name + '=')) {
            return true;
        }
    }
    return false;
}

/**
 * 清除用戶登入狀態
 * - 清除 localStorage 中的 userState
 * - 重置 userStore 為初始狀態
 */
function clearUserState() {
    console.log('[authCheck] 清除用戶狀態');

    // 使用 userStore 的 clear() 方法重置為初始狀態
    userStore.clear();

    console.log('[authCheck] 用戶狀態已清除');
}

/**
 * 檢查認證狀態
 *
 * 邏輯：
 * 1. 檢查 localStorage 中的 userState.isLoggedIn
 * 2. 檢查 Cookie 中是否有 access_token
 * 3. 如果 localStorage 說已登入但 Cookie 不存在 → 狀態不一致
 * 4. 清除 localStorage 並導向登入頁
 */
function checkAuthStatus() {
    console.log('[authCheck] 開始檢查認證狀態');

    // 獲取當前用戶狀態
    const userState = userStore.getState();
    console.log('[authCheck] userState:', userState);

    // 檢查 Cookie 是否存在
    const hasAccessToken = hasCookie('access_token');
    console.log('[authCheck] access_token Cookie 存在:', hasAccessToken);

    // 如果 localStorage 顯示已登入，但 Cookie 不存在 → 狀態不一致
    if (userState.isLoggedIn && !hasAccessToken) {
        console.warn('[authCheck] ⚠️ 偵測到狀態不一致：localStorage 顯示已登入，但 Cookie 已過期或被清除');
        console.log('[authCheck] 正在清除 localStorage 並導向登入頁...');

        // 清除用戶狀態
        clearUserState();

        // 延遲一點點時間確保狀態清除完成，然後導向登入頁
        setTimeout(() => {
            const currentPath = window.location.pathname;
            // 如果不在登入頁，導向登入頁
            if (currentPath !== '/login') {
                console.log('[authCheck] 導向登入頁');
                window.location.href = '/login?redirect=' + encodeURIComponent(currentPath);
            }
        }, 100);

        return false;
    }

    console.log('[authCheck] ✅ 認證狀態一致');
    return true;
}

/**
 * 初始化認證檢查
 * 在頁面載入時自動執行一次
 */
function initAuthCheck() {
    // 如果在登入頁面，不需要檢查（避免無限循環）
    if (window.location.pathname === '/login') {
        console.log('[authCheck] 在登入頁面，跳過認證檢查');
        return;
    }

    console.log('[authCheck] 初始化認證檢查');
    checkAuthStatus();
}

// 導出函數供外部使用（如果需要）
export {
    checkAuthStatus,
    clearUserState,
    initAuthCheck
};

// 頁面載入完成後自動執行檢查
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initAuthCheck);
} else {
    // 如果文檔已經載入完成，立即執行
    initAuthCheck();
}
