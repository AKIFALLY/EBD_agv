
import { userStore } from '../store/index.js';

function setupDropdowns() {
    const dropdowns = document.querySelectorAll('.navbar-item.has-dropdown');
    dropdowns.forEach(dropdown => {
        const link = dropdown.querySelector('.navbar-link');
        if (link) {
            link.addEventListener('click', function (e) {
                e.preventDefault();
                // 關閉其他 dropdown
                dropdowns.forEach(d => {
                    if (d !== dropdown) {
                        d.classList.remove('is-active');
                    }
                });
                // 切換當前 dropdown
                dropdown.classList.toggle('is-active');
            });
        }
    });
    // 點擊外部時收起所有 dropdown
    document.addEventListener('click', function (e) {
        if (!e.target.closest('.navbar-item.has-dropdown')) {
            dropdowns.forEach(d => d.classList.remove('is-active'));
        }
    });
}

async function handleLogout(event) {
    event.preventDefault();

    try {
        // 使用 Socket.IO 進行登出
        const socket = (await import('./socket.js')).default;
        await socket.api.userLogout();

        // 登出成功，跳轉到登入頁面
        window.location.href = '/login';

    } catch (error) {
        console.error('登出失敗:', error);
        // 即使 Socket.IO 登出失敗，也跳轉到登入頁面
        window.location.href = '/login';
    }
}

function setupLogout() {
    const logoutBtn = document.getElementById('logout-btn');
    if (logoutBtn) {
        logoutBtn.addEventListener('click', handleLogout);
    }
}

function updateNavbarUserInfo(userState) {

    const userDropdown = document.getElementById('user-dropdown');
    const loginArea = document.getElementById('login-area');
    const userDisplayName = document.getElementById('user-display-name');
    const adminBadge = document.getElementById('admin-badge');
    const userUsername = document.getElementById('user-username');
    const userRole = document.getElementById('user-role');
    const userStatus = document.getElementById('user-status');
    const userManagementLink = document.getElementById('user-management-link');
    const userManagementDivider = document.getElementById('user-management-divider');

    if (userState && userState.isLoggedIn === true) {
        // 顯示已登入狀態
        if (userDropdown) userDropdown.style.display = 'block';
        if (loginArea) loginArea.style.display = 'none';

        // 更新用戶顯示名稱
        if (userDisplayName) userDisplayName.textContent = userState.full_name || userState.username || '用戶';

        // 顯示/隱藏管理員標籤
        if (userState.role === 'admin') {
            if (adminBadge) adminBadge.style.display = 'inline-block';
            if (userManagementLink) userManagementLink.style.display = 'block';
            if (userManagementDivider) userManagementDivider.style.display = 'block';
        } else {
            if (adminBadge) adminBadge.style.display = 'none';
            if (userManagementLink) userManagementLink.style.display = 'none';
            if (userManagementDivider) userManagementDivider.style.display = 'none';
        }

        // 更新用戶詳細信息
        if (userUsername) userUsername.textContent = userState.username || '-';
        if (userRole) userRole.textContent = getRoleDisplayName(userState.role);
        if (userStatus) userStatus.textContent = userState.isConnected ? '已連線' : '離線';

    } else {
        // 顯示未登入狀態
        if (userDropdown) userDropdown.style.display = 'none';
        if (loginArea) loginArea.style.display = 'block';
    }
}

function getRoleDisplayName(role) {
    const roleMap = {
        'admin': '管理員',
        'operator': '操作員',
        'user': '一般用戶'
    };
    return roleMap[role] || role || '-';
}

function handleUserStateChange(newState) {
    console.log('Navbar: 用戶狀態變化', newState);
    updateNavbarUserInfo(newState);
}

export const navbar = (() => {
    function setup() {
        console.log('Navbar: 開始初始化');

        // 設置下拉選單
        setupDropdowns();

        // 設置登出功能
        setupLogout();

        // 監聽用戶狀態變化
        userStore.on('change', handleUserStateChange);

        updateNavbarUserInfo(userStore.getState());
    }

    return {
        setup,
        updateNavbarUserInfo,
    };
})();