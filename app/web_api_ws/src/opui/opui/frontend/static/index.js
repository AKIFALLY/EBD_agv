/**
 * OPUI - 參考 AGVCUI 的簡單架構
 * 直接使用 store 監聽和 socket API，不使用複雜的管理器
 * 重構後只保留共用功能：全域初始化、Store 狀態管理、Socket 連線處理等
 */

// ========== 導入 Store 和 API ==========
import { userStore, operationStore, dataStore, tasksStore, uiStore } from './js/store.js';
import { socketAPI } from './js/api.js';
import { notify } from './js/notify.js';

// ========== 導入頁面模組 ==========
import { homePage } from './js/pages/homePage.js';
import { settingPage } from './js/pages/settingPage.js';

// ========== Socket 事件已在 socketAPI 內部處理 ==========
// socketAPI 會自動處理連線事件並更新 userStore
// socketAPI 會自動處理資料更新事件並更新對應的 store
// 我們只需要監聽 store 的變更即可

// ========== Store 變更處理（共用部分） ==========
function setupStoreListeners() {
    // 監聽 userStore 變更
    userStore.on('change', handleUserChange);

    // 監聽 operationStore 變更
    operationStore.on('change', handleOperationChange);

    // 監聽 dataStore 變更
    dataStore.on('change', handleDataChange);

    // 監聽 tasksStore 變更
    tasksStore.on('change', handleTasksChange);


}

function handleUserChange(newState) {
    if (!newState) return;
    //console.log('👤 用戶狀態變更:', newState);
    updateConnectionStatus(newState.isConnected);
    updateMachineDisplay(newState.machineId);
}

function handleOperationChange(newState) {
    if (!newState) return;
    //console.log('⚙️ 操作狀態變更:', newState);
    // 操作狀態變更的具體處理已移動到各頁面模組中
    // 這裡只保留共用的處理邏輯
}

function handleDataChange(newState) {
    if (!newState) return;
    //console.log('📊 資料變更:', newState);
    // 資料變更的具體處理已移動到各頁面模組中
    // 這裡只保留共用的處理邏輯
}

function handleTasksChange(newState) {
    if (!newState) return;
    //console.log('📋 任務狀態變更:', newState);
    // 任務狀態變更的具體處理已移動到各頁面模組中
}

// ========== 共用 UI 更新函數 ==========
function updateConnectionStatus(isConnected) {
    //console.log(`📡 更新連線狀態: ${isConnected ? '已連接' : '未連接'}`);

    // 更新 navbar 中的連線狀態圖標
    const navWifiIcon = document.querySelector('.navbar .mdi-wifi');
    if (navWifiIcon) {
        if (isConnected) {
            navWifiIcon.classList.remove('is-disconnected');
            navWifiIcon.classList.add('is-connected');
        } else {
            navWifiIcon.classList.remove('is-connected');
            navWifiIcon.classList.add('is-disconnected');
        }
        //console.log(`📡 Navbar 連線圖標已更新: ${isConnected ? 'connected' : 'disconnected'}`);
    }

    // 更新其他可能的連線狀態元素
    const statusElement = document.querySelector('#connection-status');
    if (statusElement) {
        statusElement.className = isConnected ? 'tag is-success' : 'tag is-danger';
        statusElement.textContent = isConnected ? '已連接' : '未連接';
    }
}

function updateMachineDisplay(machineId) {
    //console.log(`🔧 更新機台編號顯示: ${machineId}`);

    // 更新 navbar 中的機台編號顯示
    const navMachineElement = document.querySelector('#nav-machine-number');
    if (navMachineElement) {
        navMachineElement.textContent = `Machine ${machineId}`;
        //console.log(`🔧 Navbar 機台編號已更新: 機台 ${machineId}`);
    }

    // 同時更新機台選擇狀態
    updateMachineSelection(machineId);
}

function updateMachineSelection(machineId) {
    //console.log(`🔧 更新機台選擇狀態: ${machineId}`);

    const machineButtons = document.querySelectorAll('.machine-btn');
    //console.log(`🔧 找到 ${machineButtons.length} 個機台按鈕`);

    machineButtons.forEach(btn => {
        const btnMachineId = parseInt(btn.getAttribute('data-machine') || btn.getAttribute('data-machine-id'));
        const isSelected = btnMachineId === machineId;

        btn.classList.toggle('is-selected', isSelected);
        btn.classList.toggle('is-primary', isSelected);

        //console.log(`🔧 機台按鈕 ${btnMachineId}: selected=${isSelected}`);
    });
}

// ========== 輔助函數 ==========
async function syncToBackend() {
    // 🔧 修復：只傳送operation狀態，而不是整個狀態物件
    const operationState = operationStore.getState();

    try {
        await socketAPI.updateClient(operationState);
        //console.log('✅ 狀態同步成功');
    } catch (error) {
        console.error('❌ 同步到後端失敗:', error);
    }
}

function getCurrentPage() {
    const path = window.location.pathname;
    if (path.includes('setting')) return 'setting';
    if (path.includes('rack')) return 'rack';
    return 'home';
}

/**
 * 初始化頁面模組
 */
async function initPageModules() {
    const currentPageType = getCurrentPage();
    //console.log(`📄 初始化頁面模組: ${currentPageType}`);

    try {
        if (currentPageType === 'home') {
            // 初始化 home 頁面模組
            homePage.setup();
            //console.log('✅ Home 頁面模組初始化完成');
        } else if (currentPageType === 'setting') {
            // 初始化 setting 頁面模組
            settingPage.setup();
            //console.log('✅ Setting 頁面模組初始化完成');
        } else if (currentPageType === 'rack') {
            // 初始化 rack 頁面模組
            const { rackPage } = await import('./js/pages/rackPage.js');
            rackPage.setup();
            //console.log('✅ Rack 頁面模組初始化完成');
        }
    } catch (error) {
        console.error(`❌ 頁面模組初始化失敗: ${error}`);
    }
}

// ========== 主要初始化函數 ==========
async function initOPUI() {
    //console.log('🚀 OPUI 初始化開始');

    try {
        // 設定必要的全域變數（僅用於除錯和向後相容）
        window.socketAPI = socketAPI;
        window.notify = notify;
        window.userStore = userStore;

        // 初始化通知系統
        notify.setup();

        // 初始化 Socket.IO 連接（等待連線完成）
        try {
            await socketAPI.initAsync();
            //console.log('✅ Socket.IO 連線成功');
        } catch (error) {
            console.warn('⚠️ Socket.IO 連線失敗，將在背景重試:', error.message);
            // 不阻止初始化繼續進行
        }

        // 設定 Store 監聽器
        setupStoreListeners();

        // 初始化頁面模組
        await initPageModules();

        // 初始化 UI（觸發初始狀態更新）
        const userState = userStore.getState();
        const operationState = operationStore.getState();
        const dataState = dataStore.getState();
        const tasksState = tasksStore.getState();

        handleUserChange(userState);
        handleOperationChange(operationState);
        handleDataChange(dataState);
        handleTasksChange(tasksState);

        // 同步初始狀態到後端（如果 Socket 已連線）
        try {
            await syncToBackend();
            //console.log('✅ 初始狀態同步成功');
        } catch (error) {
            console.warn('⚠️ 初始狀態同步失敗，將在 Socket 連線後自動重試:', error.message);
            // 設置連線成功後的自動同步
            socketAPI.onConnected(() => {
                syncToBackend().catch(err => {
                    console.error('❌ 重試同步失敗:', err.message);
                });
            });
        }

        //console.log('✅ OPUI 初始化完成');

    } catch (error) {
        console.error('❌ OPUI 初始化失敗:', error);
        notify.showErrorMessage('應用程式初始化失敗');
    }
}

// DOM 載入完成後初始化
document.addEventListener("DOMContentLoaded", () => {
    initOPUI();
});
