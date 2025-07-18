/**
 * AGVC UI - 按需載入模組化架構
 * 根據當前頁面路徑動態載入所需的 JavaScript 模組
 * 優化載入效能和記憶體使用
 */

// ========== 全域 Store 和共用模組 - 所有頁面都需要 ==========
// 只引用所有頁面都需要的 Store
import {
    mapStore,    // 用於清除操作，所有頁面都需要
    userStore    // 用於用戶狀態管理，navbar 需要
} from './store/index.js';

// 共用模組 - 在所有頁面都需要載入
import { navbar } from './js/navbar.js';
import { notify } from './js/notify.js';
import socket from './js/socket.js';
import { deleteModal } from './js/deleteModal.js';

/**
 * 頁面路由配置 - 定義每個頁面需要載入的模組
 */
const PAGE_MODULES = {
    '/': {
        modules: ['./js/dashboardPage.js'],
        setup: (modules) => modules[0].dashboardPage.setup()
    },
    '/map': {
        modules: ['./js/mapPage.js'],
        setup: (modules) => modules[0].mapPage.setup()
    },
    '/signals': {
        modules: ['./js/signalsPage.js'],
        setup: (modules) => modules[0].signalsPage.setup()
    },
    '/tasks': {
        modules: ['./js/tasksPage.js'],
        setup: (modules) => modules[0].tasksPage.setup()
    },
    '/racks': {
        modules: ['./js/racksPage.js'],
        setup: (modules) => modules[0].racksPage.setup()
    },
    '/agvs': {
        modules: ['./js/agvsPage.js'],
        setup: (modules) => modules[0].agvsPage.setup()
    },
    '/products': {
        modules: ['./js/productsPage.js'],
        setup: (modules) => modules[0].productsPage.setup()
    },
    '/clients': {
        modules: ['./js/clientsPage.js'],
        setup: (modules) => modules[0].clientsPage.setup()
    },
    '/carriers': {
        modules: ['./js/carriersPage.js'],
        setup: (modules) => modules[0].carriersPage.setup()
    },
    '/rosout_logs': {
        modules: ['./js/rosoutLogsPage.js'],
        setup: (modules) => modules[0].rosoutLogsPage.setup()
    },
    '/runtime_logs': {
        modules: ['./js/runtimeLogsPage.js'],
        setup: (modules) => modules[0].runtimeLogsPage.setup()
    },
    '/login': {
        modules: ['./js/loginPage.js'],
        setup: (modules) => modules[0].loginPage.setup()
    },
    '/works': {
        modules: ['./js/worksPage.js'],
        setup: (modules) => modules[0].worksPage.setup()
    }
};

/**
 * 特殊路徑配置 - 處理動態路徑或複雜條件
 */
const SPECIAL_PATHS = {
    deviceForm: {
        condition: (path) => path === "/devices/create" || (path.startsWith("/devices/") && path.endsWith("/edit")),
        modules: ['./js/deviceFormPage.js'],
        setup: (modules) => modules[0].deviceFormPage.setup()
    }
};

/**
 * 模組載入狀態追蹤
 */
const moduleLoadState = {
    currentPath: null,
    loadedModules: null,
    config: null,
    loadError: null,
    isLoading: false,
    loadPromise: null
};

/**
 * 提前載入頁面特定模組（不執行 setup）
 */
async function preloadPageModules(currentPath) {
    console.log(`🔄 提前載入頁面模組: ${currentPath}`);

    moduleLoadState.currentPath = currentPath;
    moduleLoadState.isLoading = true;

    try {
        // 檢查標準頁面路徑
        if (PAGE_MODULES[currentPath]) {
            const config = PAGE_MODULES[currentPath];
            moduleLoadState.config = config;

            const modules = await Promise.all(
                config.modules.map(modulePath => import(modulePath))
            );

            moduleLoadState.loadedModules = modules;
            console.log(`✅ 模組載入成功: ${config.modules.join(', ')}`);
            return;
        }

        // 檢查特殊路徑
        for (const [name, config] of Object.entries(SPECIAL_PATHS)) {
            if (config.condition(currentPath)) {
                moduleLoadState.config = config;

                const modules = await Promise.all(
                    config.modules.map(modulePath => import(modulePath))
                );

                moduleLoadState.loadedModules = modules;
                console.log(`✅ 特殊路徑模組載入成功 (${name}): ${config.modules.join(', ')}`);
                return;
            }
        }

        console.log(`ℹ️ 頁面 ${currentPath} 不需要特定模組`);
    } catch (error) {
        moduleLoadState.loadError = error;
        console.error(`❌ 模組載入失敗 (${currentPath}):`, error);
    } finally {
        moduleLoadState.isLoading = false;
    }
}

/**
 * 初始化已載入的頁面模組
 */
function initializePageModules() {
    const { loadedModules, config, loadError, currentPath } = moduleLoadState;

    if (loadError) {
        console.error(`❌ 無法初始化頁面模組，載入時發生錯誤:`, loadError);
        return;
    }

    if (!loadedModules || !config) {
        console.log(`ℹ️ 頁面 ${currentPath} 沒有需要初始化的模組`);
        return;
    }

    try {
        console.log(`🚀 初始化頁面模組: ${currentPath}`);
        config.setup(loadedModules);
        console.log(`✅ 頁面模組初始化完成: ${currentPath}`);
    } catch (error) {
        console.error(`❌ 頁面模組初始化失敗 (${currentPath}):`, error);
    }
}

// ========== 提前載入頁面模組（在 DOM 準備之前） ==========
const currentPath = window.location.pathname;
const loadStartTime = performance.now();

console.log(`📍 當前頁面路徑: ${currentPath}`);
console.log('🔄 開始提前載入頁面模組...');

// 立即開始載入頁面特定模組，並保存 Promise
moduleLoadState.loadPromise = preloadPageModules(currentPath);

// ---------------- DOM 完成後初始化 ----------------
document.addEventListener("DOMContentLoaded", async () => {
    console.log('🚀 AGVC UI DOM 初始化開始');

    // 清除 mapStore（全域需要）
    mapStore.clear();

    // ========== 共用模組初始化 - 所有頁面都需要 ==========
    console.log('🔧 初始化共用模組...');

    // 初始化通用刪除模態框
    deleteModal.setup();

    // 初始化通用通知處理
    notify.setup();

    // 初始化 Navbar（依賴 userStore 狀態）
    navbar.setup();

    // 初始化 Socket.IO（所有頁面都需要）
    socket.setup();

    console.log('✅ 共用模組初始化完成');

    // ========== 初始化已載入的頁面特定模組 ==========
    console.log('🔧 初始化頁面特定模組...');

    // 等待模組載入完成
    if (moduleLoadState.loadPromise) {
        console.log('⏳ 等待模組載入完成...');
        await moduleLoadState.loadPromise;
    }

    // 初始化頁面模組
    initializePageModules();

    // 計算總載入時間
    const totalLoadTime = performance.now() - loadStartTime;
    console.log(`✅ AGVC UI 完整初始化完成 (總耗時: ${totalLoadTime.toFixed(2)}ms)`);
});