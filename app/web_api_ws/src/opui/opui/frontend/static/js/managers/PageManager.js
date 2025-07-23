/**
 * 頁面管理器
 * 處理不同頁面的初始化和管理邏輯
 */
import { stateHelpers } from '../store.js';

export class PageManager {
    constructor() {
        this.currentPage = null;
        this.initializedPages = new Set();
    }

    /**
     * 初始化當前頁面
     */
    initCurrentPage() {
        const path = window.location.pathname;

        if (path === '/setting') {
            this.currentPage = 'settings';
            this.initSettingsPage();
        } else {
            this.currentPage = 'home';
            this.initHomePage();
        }

        // 更新狀態中的當前頁面
        stateHelpers.setUser({ currentPage: this.currentPage });

        // console.log(`📄 當前頁面: ${this.currentPage}`);
        return this.currentPage;
    }

    /**
     * 獲取當前頁面
     */
    getCurrentPage() {
        return this.currentPage;
    }

    /**
     * 初始化首頁
     */
    initHomePage() {
        if (this.initializedPages.has('home')) {
            // console.log("📱 首頁已初始化，跳過重複初始化");
            return;
        }

        // console.log("📱 初始化首頁");

        // 綁定首頁事件
        if (window.opuiApp && window.opuiApp.eventManager) {
            window.opuiApp.eventManager.bindHomePageEvents();
        }

        // 初始化首頁 UI
        this.setupHomePageUI();

        this.initializedPages.add('home');
    }

    /**
     * 初始化設定頁面
     */
    initSettingsPage() {
        if (this.initializedPages.has('settings')) {
            // console.log("⚙️ 設定頁面已初始化，跳過重複初始化");
            return;
        }

        // console.log("⚙️ 初始化設定頁面");

        // 綁定設定頁面事件
        if (window.opuiApp && window.opuiApp.eventManager) {
            window.opuiApp.eventManager.bindSettingsPageEvents();
        }

        // 初始化設定頁面 UI
        this.setupSettingsPageUI();

        this.initializedPages.add('settings');
    }

    /**
     * 設定首頁 UI
     */
    setupHomePageUI() {
        // 檢查必要的 DOM 元素是否存在
        const requiredElements = [
            '.product-btn',
            '.num-btn',
            '[data-call-empty]',
            '[data-dispatch-full]',
            '.room-btn'
        ];

        const missingElements = [];
        requiredElements.forEach(selector => {
            if (!document.querySelector(selector)) {
                missingElements.push(selector);
            }
        });

        if (missingElements.length > 0) {
            // console.warn('⚠️ 首頁缺少必要的 DOM 元素:', missingElements);
        }

        // 設定初始狀態
        this.resetHomePageState();

        // console.log("✅ 首頁 UI 設定完成");
    }

    /**
     * 設定設定頁面 UI
     */
    setupSettingsPageUI() {
        // 檢查必要的 DOM 元素是否存在
        const requiredElements = [
            '.machine-btn',
            '.product-input',
            '#factory-restore'
        ];

        const missingElements = [];
        requiredElements.forEach(selector => {
            if (!document.querySelector(selector)) {
                missingElements.push(selector);
            }
        });

        if (missingElements.length > 0) {
            // console.warn('⚠️ 設定頁面缺少必要的 DOM 元素:', missingElements);
        }

        // 初始化產品輸入內容
        const currentState = window.appStore?.getState();
        if (currentState && window.opuiApp && window.opuiApp.uiManager) {
            window.opuiApp.uiManager.updateProductInputs(currentState.operation);
            window.opuiApp.uiManager.updateProductValidation(currentState.data.products);
            window.opuiApp.uiManager.updateParkingArea(currentState.data.parking);
        }

        // console.log("✅ 設定頁面 UI 設定完成");
    }

    /**
     * 重置首頁狀態
     */
    resetHomePageState() {
        // 重置產品按鈕狀態
        document.querySelectorAll('.product-btn').forEach(btn => {
            btn.classList.remove('is-loading', 'is-disabled');
        });

        // 重置數量按鈕狀態
        document.querySelectorAll('.num-btn').forEach(btn => {
            btn.classList.remove('is-selected', 'is-primary');
        });

        // 重置操作按鈕狀態
        document.querySelectorAll('[data-call-empty], [data-dispatch-full]').forEach(btn => {
            btn.classList.remove('is-danger', 'is-success', 'is-warning', 'is-loading');
            btn.disabled = false;
        });

        // 重置房號按鈕狀態
        document.querySelectorAll('.room-btn').forEach(btn => {
            btn.classList.remove('is-selected', 'is-primary');
        });
    }

    /**
     * 重置設定頁面狀態
     */
    resetSettingsPageState() {
        // 重置機台按鈕狀態
        document.querySelectorAll('.machine-btn').forEach(btn => {
            btn.classList.remove('is-selected', 'is-primary', 'is-loading');
        });

        // 重置產品輸入狀態
        document.querySelectorAll('.product-input').forEach(input => {
            input.classList.remove('is-danger', 'is-success');
        });

        // 重置驗證訊息
        document.querySelectorAll('.help').forEach(help => {
            help.textContent = '';
            help.classList.remove('is-danger', 'is-success');
        });
    }

    /**
     * 切換到指定頁面
     */
    switchToPage(pageName) {
        if (this.currentPage === pageName) {
            // console.log(`📄 已在 ${pageName} 頁面，無需切換`);
            return;
        }

        // console.log(`📄 切換頁面: ${this.currentPage} → ${pageName}`);

        // 清理當前頁面
        this.cleanupCurrentPage();

        // 設定新頁面
        this.currentPage = pageName;

        // 初始化新頁面
        if (pageName === 'home') {
            this.initHomePage();
        } else if (pageName === 'settings') {
            this.initSettingsPage();
        }

        // 更新狀態
        stateHelpers.setUser({ currentPage: this.currentPage });

        // 更新 UI 管理器的當前頁面
        if (window.opuiApp && window.opuiApp.uiManager) {
            window.opuiApp.uiManager.setCurrentPage(this.currentPage);
        }
    }

    /**
     * 清理當前頁面
     */
    cleanupCurrentPage() {
        if (!this.currentPage) return;

        // console.log(`🧹 清理 ${this.currentPage} 頁面`);

        // 移除頁面特定的事件監聽器
        // 注意：由於我們使用事件委派，大部分事件不需要手動移除

        // 重置頁面狀態
        if (this.currentPage === 'home') {
            this.resetHomePageState();
        } else if (this.currentPage === 'settings') {
            this.resetSettingsPageState();
        }
    }

    /**
     * 檢查頁面是否已初始化
     */
    isPageInitialized(pageName) {
        return this.initializedPages.has(pageName);
    }

    /**
     * 強制重新初始化頁面
     */
    forceReinitializePage(pageName) {
        // console.log(`🔄 強制重新初始化 ${pageName} 頁面`);

        // 移除初始化標記
        this.initializedPages.delete(pageName);

        // 重新初始化
        if (pageName === 'home') {
            this.initHomePage();
        } else if (pageName === 'settings') {
            this.initSettingsPage();
        }
    }

    /**
     * 獲取頁面資訊
     */
    getPageInfo() {
        return {
            currentPage: this.currentPage,
            initializedPages: Array.from(this.initializedPages),
            pathname: window.location.pathname
        };
    }

    /**
     * 檢查頁面 DOM 完整性
     */
    checkPageDOMIntegrity(pageName) {
        const requiredElements = {
            home: [
                '.product-btn',
                '.num-btn',
                '[data-call-empty]',
                '[data-dispatch-full]',
                '.room-btn',
                '.rack-selected'
            ],
            settings: [
                '.machine-btn',
                '.product-input',
                '#factory-restore',
                '.parking-area'
            ]
        };

        const elements = requiredElements[pageName] || [];
        const missingElements = [];

        elements.forEach(selector => {
            if (!document.querySelector(selector)) {
                missingElements.push(selector);
            }
        });

        if (missingElements.length > 0) {
            // console.warn(`⚠️ ${pageName} 頁面 DOM 完整性檢查失敗，缺少元素:`, missingElements);
            return false;
        }

        // console.debug(`✅ ${pageName} 頁面 DOM 完整性檢查通過`);
        return true;
    }

    /**
     * 修復頁面 DOM
     */
    repairPageDOM(pageName) {
        // console.log(`🔧 嘗試修復 ${pageName} 頁面 DOM`);

        // 這裡可以添加 DOM 修復邏輯
        // 例如：重新載入頁面、顯示錯誤訊息等

        if (!this.checkPageDOMIntegrity(pageName)) {
            console.error(`❌ ${pageName} 頁面 DOM 修復失敗`);
            return false;
        }

        // console.log(`✅ ${pageName} 頁面 DOM 修復成功`);
        return true;
    }

    /**
     * 重置所有頁面
     */
    resetAllPages() {
        // console.log('🔄 重置所有頁面');

        this.initializedPages.clear();
        this.currentPage = null;

        // 重新初始化當前頁面
        this.initCurrentPage();
    }
}
