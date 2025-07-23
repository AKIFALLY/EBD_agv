/**
 * OPUI 主應用程式 (重構版)
 * 簡化的應用程式協調器，使用模組化架構
 */
import { socketAPI } from './api.js';
import { appStore, stateHelpers } from './store.js';
import { notify } from './notify.js';

// 導入管理器
import { EventManager } from './managers/EventManager.js';
import { UIManager } from './managers/UIManager.js';
import { PageManager } from './managers/PageManager.js';
import { StateManager } from './managers/StateManager.js';

/**
 * 簡化的應用程式管理器
 * 只負責協調各個管理器，不直接處理具體業務邏輯
 */
class OPUIApp {
    constructor() {
        // 初始化各個管理器
        this.eventManager = new EventManager();
        this.uiManager = new UIManager();
        this.pageManager = new PageManager();
        this.stateManager = new StateManager();

        // 應用程式狀態
        this.isInitialized = false;

        // console.log("🏗️ OPUI 應用程式管理器已創建");
    }

    /**
     * 初始化應用程式
     */
    async init() {
        if (this.isInitialized) {
            // console.log("⚠️ 應用程式已初始化，跳過重複初始化");
            return;
        }

        // console.log("🚀 開始初始化 OPUI 應用程式");

        try {
            // 設定全域參考，供其他模組使用
            window.opuiApp = this;
            window.appStore = appStore; // 確保全域可訪問
            window.stateHelpers = stateHelpers; // 確保全域可訪問

            // 1. 立即使用 localStorage 資料初始化 UI（快速顯示）
            this.initializeLocalUI();

            // 2. 初始化當前頁面
            this.initializePage();

            // 3. 設定事件監聽
            this.setupEventListeners();

            // 4. 在背景初始化 Socket 連線和同步
            this.initializeBackgroundSync();

            this.isInitialized = true;
            // console.log("✅ OPUI 應用程式初始化完成");

        } catch (error) {
            console.error("❌ 應用程式初始化失敗:", error);
            notify.showErrorMessage("應用程式初始化失敗");
            throw error;
        }
    }

    /**
     * 初始化 Socket 連線
     */
    async initializeSocket() {
        // console.log("🔌 初始化 Socket 連線");

        // 初始化 Socket
        socketAPI.init();

        // 等待 Socket 連線完成
        return new Promise((resolve, reject) => {
            const timeout = setTimeout(() => {
                reject(new Error("Socket 連線超時"));
            }, 10000); // 10秒超時

            // 如果已經連線，直接解析
            if (socketAPI.isConnected) {
                clearTimeout(timeout);
                resolve();
                return;
            }

            // 監聽連線事件
            const onConnect = () => {
                clearTimeout(timeout);
                socketAPI.socket.off('connect', onConnect);
                socketAPI.socket.off('connect_error', onError);
                resolve();
            };

            const onError = (error) => {
                clearTimeout(timeout);
                socketAPI.socket.off('connect', onConnect);
                socketAPI.socket.off('connect_error', onError);
                reject(error);
            };

            socketAPI.socket.on('connect', onConnect);
            socketAPI.socket.on('connect_error', onError);
        });
    }

    /**
     * 設定狀態管理
     */
    setupStateManagement() {
        // console.log("📊 設定狀態管理");

        // 設定狀態變更監聽
        this.stateManager.setupStateListeners(appStore);

        // 檢查狀態一致性
        if (!this.stateManager.checkStateConsistency(appStore)) {
            // console.warn("⚠️ 狀態一致性檢查失敗，嘗試修復");
            this.stateManager.repairState(appStore);
        }
    }

    /**
     * 初始化當前頁面
     */
    initializePage() {
        // console.log("📄 初始化當前頁面");

        // 初始化頁面並獲取當前頁面名稱
        const currentPage = this.pageManager.initCurrentPage();

        // 設定 UI 管理器的當前頁面
        this.uiManager.setCurrentPage(currentPage);

        // 檢查頁面 DOM 完整性
        if (!this.pageManager.checkPageDOMIntegrity(currentPage)) {
            // console.warn("⚠️ 頁面 DOM 完整性檢查失敗");
            this.pageManager.repairPageDOM(currentPage);
        }
    }

    /**
     * 設定事件監聽
     */
    setupEventListeners() {
        // console.log("🎧 設定全域事件監聽");

        // 只綁定全域事件，頁面特定事件由 PageManager 處理
        this.eventManager.bindGlobalEvents();
    }

    /**
     * 使用 localStorage 資料立即初始化 UI（避免畫面跳動）
     */
    initializeLocalUI() {
        // console.log("🎨 使用 localStorage 資料立即初始化 UI");

        // 獲取當前 localStorage 中的狀態並立即更新 UI
        const currentState = appStore.getState();
        this.uiManager.updateUI(currentState);
    }

    /**
     * 初始化 UI 狀態（同步後更新）
     */
    initializeUI() {
        // console.log("🎨 同步後更新 UI 狀態");

        // 獲取同步後的狀態並更新 UI
        const currentState = appStore.getState();
        this.uiManager.updateUI(currentState);
    }

    /**
     * 在背景初始化 Socket 連線和同步（簡化版）
     */
    async initializeBackgroundSync() {
        // console.log("🔄 在背景初始化 Socket 連線和同步 - 簡化架構");

        try {
            // 簡化架構：移除複雜的 UI 更新控制邏輯
            // 採用單向資料流，讓 UI 自然響應狀態變更

            // 1. 初始化 Socket 連線
            await this.initializeSocket();

            // 2. 設定狀態管理
            this.setupStateManagement();

            // 3. 啟動狀態同步
            this.startStateSync();

            // 4. 等待同步完成後更新 UI
            setTimeout(() => {
                this.initializeUI();
                // console.log("✅ 背景同步完成，UI 已更新");
            }, 2000); // 給足夠時間讓同步完成

            // console.log("✅ 背景同步初始化完成");
        } catch (error) {
            console.error("❌ 背景同步初始化失敗:", error);
            notify.showErrorMessage("背景同步初始化失敗");
        }
    }

    /**
     * 啟動狀態同步
     */
    startStateSync() {
        // console.log("🔄 啟動狀態同步");

        // 初始化狀態同步
        this.stateManager.initializeSync(appStore);
    }

    /**
     * 獲取應用程式狀態（簡化版）
     */
    getAppStatus() {
        return {
            isInitialized: this.isInitialized,
            currentPage: this.pageManager.getCurrentPage(),
            pageInfo: this.pageManager.getPageInfo(),
            // 簡化：移除複雜的同步狀態，只保留基本的連線狀態
            isConnected: appStore.getState().user.isConnected,
            storeState: appStore.getState()
        };
    }

    /**
     * 重新初始化應用程式（簡化版）
     */
    async reinitialize() {
        // console.log("🔄 重新初始化應用程式");

        try {
            // 重置初始化狀態
            this.isInitialized = false;

            // 重置頁面管理器
            this.pageManager.resetAllPages();

            // 簡化：不需要重置複雜的同步狀態
            // 重新初始化
            await this.init();

            // console.log("✅ 應用程式重新初始化完成");
        } catch (error) {
            console.error("❌ 應用程式重新初始化失敗:", error);
            notify.showErrorMessage("應用程式重新初始化失敗");
        }
    }

    /**
     * 切換頁面
     */
    switchPage(pageName) {
        // console.log(`📄 切換到 ${pageName} 頁面`);

        try {
            // 使用頁面管理器切換頁面
            this.pageManager.switchToPage(pageName);

            // 重新綁定事件
            if (pageName === 'home') {
                this.eventManager.bindHomePageEvents();
            } else if (pageName === 'settings') {
                this.eventManager.bindSettingsPageEvents();
            }

            // 更新 UI
            const currentState = appStore.getState();
            this.uiManager.updateUI(currentState);

            // console.log(`✅ 成功切換到 ${pageName} 頁面`);
        } catch (error) {
            console.error(`❌ 切換到 ${pageName} 頁面失敗:`, error);
            notify.showErrorMessage(`切換到 ${pageName} 頁面失敗`);
        }
    }

    /**
     * 強制同步狀態
     */
    forceSync() {
        // console.log("🔄 強制同步狀態");
        this.stateManager.forceSyncToServer(appStore);
    }

    /**
     * 執行健康檢查
     */
    healthCheck() {
        // console.log("🏥 執行應用程式健康檢查");

        const issues = [];

        // 檢查初始化狀態
        if (!this.isInitialized) {
            issues.push("應用程式未初始化");
        }

        // 檢查狀態一致性
        if (!this.stateManager.checkStateConsistency(appStore)) {
            issues.push("狀態一致性檢查失敗");
        }

        // 檢查當前頁面 DOM
        const currentPage = this.pageManager.getCurrentPage();
        if (!this.pageManager.checkPageDOMIntegrity(currentPage)) {
            issues.push(`${currentPage} 頁面 DOM 完整性檢查失敗`);
        }

        // 檢查 Socket 連線
        const currentState = appStore.getState();
        if (!currentState.user.isConnected) {
            issues.push("Socket 連線中斷");
        }

        if (issues.length === 0) {
            // console.log("✅ 應用程式健康檢查通過");
            return { healthy: true, issues: [] };
        } else {
            // console.warn("⚠️ 應用程式健康檢查發現問題:", issues);
            return { healthy: false, issues };
        }
    }

    /**
     * 修復應用程式問題
     */
    async repair() {
        // console.log("🔧 嘗試修復應用程式問題");

        const healthCheck = this.healthCheck();
        if (healthCheck.healthy) {
            // console.log("✅ 應用程式狀態正常，無需修復");
            return;
        }

        try {
            // 修復狀態問題
            this.stateManager.repairState(appStore);

            // 修復頁面問題
            const currentPage = this.pageManager.getCurrentPage();
            this.pageManager.repairPageDOM(currentPage);

            // 重新初始化（如果需要）
            if (!this.isInitialized) {
                await this.reinitialize();
            }

            // console.log("✅ 應用程式修復完成");
            notify.showNotifyMessage("應用程式已修復");
        } catch (error) {
            console.error("❌ 應用程式修復失敗:", error);
            notify.showErrorMessage("應用程式修復失敗");
        }
    }

    /**
     * 清理資源（簡化版）
     */
    cleanup() {
        // console.log("🧹 清理應用程式資源");

        // 重置狀態
        this.isInitialized = false;

        // 清理頁面管理器
        this.pageManager.resetAllPages();

        // 簡化：不需要清理複雜的同步狀態
        // 移除全域參考
        if (window.opuiApp === this) {
            delete window.opuiApp;
        }

        // console.log("✅ 應用程式資源清理完成");
    }
}

// 建立全域應用程式實例
const opuiApp = new OPUIApp();

// 當 DOM 載入完成時初始化應用程式
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => opuiApp.init());
} else {
    opuiApp.init();
}

// 匯出應用程式實例
export { opuiApp };
