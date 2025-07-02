/**
 * Login 頁面功能
 * 使用 Socket.IO 處理登入，不使用傳統表單提交
 */

import socket from './socket.js';

let loginForm = null;
let submitButton = null;
let errorContainer = null;

function setupAutoFocus() {
    // 自動聚焦到用戶名輸入框
    const usernameInput = document.querySelector('input[name="username"]');
    if (usernameInput) {
        usernameInput.focus();
    }
}

function showError(message) {
    if (!errorContainer) {
        // 創建錯誤容器
        errorContainer = document.createElement('div');
        errorContainer.className = 'error-message';
        errorContainer.innerHTML = `
            <span class="icon">
                <i class="mdi mdi-alert-circle"></i>
            </span>
            <span class="error-text"></span>
        `;

        // 插入到表單前面
        const form = document.querySelector('.login-form');
        form.parentNode.insertBefore(errorContainer, form);
    }

    errorContainer.querySelector('.error-text').textContent = message;
    errorContainer.style.display = 'flex';
}

function hideError() {
    if (errorContainer) {
        errorContainer.style.display = 'none';
    }
}

function setLoading(loading) {
    if (!submitButton) return;

    if (loading) {
        submitButton.classList.add('is-loading');
        submitButton.disabled = true;
    } else {
        submitButton.classList.remove('is-loading');
        submitButton.disabled = false;
    }
}

async function handleLogin(event) {
    event.preventDefault();

    const formData = new FormData(loginForm);
    const username = formData.get('username');
    const password = formData.get('password');
    const redirect = formData.get('redirect') || '/';

    if (!username || !password) {
        showError('請輸入用戶名和密碼');
        return;
    }

    hideError();
    setLoading(true);

    try {
        // 等待 Socket.IO 連線
        await waitForSocketConnection();

        // 使用 Socket.IO 進行登入
        await socket.api.userLogin({
            username: username,
            password: password
        });

        // 登入成功，延遲跳轉到目標頁面
        setTimeout(() => {
            window.location.href = redirect;
        }, 500);

    } catch (error) {
        console.error('登入失敗:', error);
        showError(error.message || '登入失敗，請稍後再試');
    } finally {
        setLoading(false);
    }
}

function waitForSocketConnection() {
    return new Promise((resolve, reject) => {
        // 檢查 Socket.IO 是否已連線
        if (socket && socket.socket && socket.socket.connected) {
            resolve();
            return;
        }

        // 等待連線，最多等待 5 秒
        let attempts = 0;
        const maxAttempts = 50; // 5 秒 / 100ms

        const checkConnection = () => {
            attempts++;

            if (socket && socket.socket && socket.socket.connected) {
                resolve();
            } else if (attempts >= maxAttempts) {
                reject(new Error('無法連接到服務器，請檢查網路連線'));
            } else {
                setTimeout(checkConnection, 100);
            }
        };

        checkConnection();
    });
}

function setupFormSubmit() {
    loginForm = document.querySelector('.login-form');
    submitButton = document.querySelector('.login-form button[type="submit"]');

    if (loginForm) {
        loginForm.addEventListener('submit', handleLogin);
    }
}



export const loginPage = (() => {
    function setup() {
        console.log('LoginPage: 開始初始化');

        // 設置自動聚焦
        setupAutoFocus();

        // 設置表單提交處理（使用 Socket.IO）
        setupFormSubmit();

        console.log('LoginPage: 初始化完成');
    }

    return {
        setup,
    };
})();


