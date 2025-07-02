//console.debug = function () {};
import {
    mapStore,
    machinesStore,
    roomsStore,
    signalsStore,
    racksStore,
    carriersStore,
    tasksStore,
    userStore
} from './store/index.js';

//import { store } from './store.js';
import { navbar } from './js/navbar.js';
//import { initHomePage } from './home.js';
import { mapPage } from './js/mapPage.js';
import { signalsPage } from './js/signalsPage.js';
import { notify } from './js/notify.js';
import socket from './js/socket.js';// 在任何頁面引入 socket 模組
import { deleteModal } from './js/deleteModal.js';
import { rosoutLogsPage } from './js/rosoutLogsPage.js';
import { runtimeLogsPage } from './js/runtimeLogsPage.js';
import { deviceFormPage } from './js/deviceFormPage.js';
import { loginPage } from './js/loginPage.js';
import { productsPage } from './js/productsPage.js';
import { clientsPage } from './js/clientsPage.js';
import { carriersPage } from './js/carriersPage.js';

// ---------------- DOM 完成後初始化 ----------------
document.addEventListener("DOMContentLoaded", () => {
    mapStore.clear();

    // ========== 共用模組 - 所有頁面都需要 ==========
    // 初始化通用刪除模態框
    deleteModal.setup();

    // 初始化通用通知處理
    notify.setup();

    // 初始化 Navbar（依賴 userStore 狀態）
    navbar.setup();

    // ========== 頁面特定模組 - 只在對應頁面初始化 ==========
    const currentPath = window.location.pathname;

    //初始化 地圖頁面
    if (currentPath === "/map") {
        // 初始化地圖頁面 按鈕事件綁定 及設定預設值
        mapPage.setup();
    }

    //初始化 信號頁面
    if (currentPath === "/signals") {
        // 初始化信號頁面 即時更新功能
        signalsPage.setup();
    }

    // 初始化 Rosout Logs 頁面
    if (currentPath === "/rosout_logs") {
        rosoutLogsPage.setup();
    }

    // 初始化 Runtime Logs 頁面
    if (currentPath === "/runtime_logs") {
        runtimeLogsPage.setup();
    }

    // 初始化 Device Form 頁面
    if (currentPath === "/devices/create" || currentPath.startsWith("/devices/") && currentPath.endsWith("/edit")) {
        deviceFormPage.setup();
    }

    // 初始化 Product Form 頁面
    if (currentPath === "/products/create" || currentPath.startsWith("/products/") && currentPath.endsWith("/edit")) {
        // 產品表單使用簡單的表單，不需要特殊的 JavaScript 處理
        // 如果需要可以創建 productFormPage.js
    }

    // 初始化 Login 頁面
    if (currentPath === "/login") {
        loginPage.setup();
    }

    // 初始化 Products 頁面
    if (currentPath === "/products") {
        productsPage.setup();
    }

    // 初始化 Clients 頁面
    if (currentPath === "/clients") {
        clientsPage.setup();
    }

    // 初始化 Carriers 頁面
    if (currentPath === "/carriers") {
        carriersPage.setup();
    }

    // 初始化 Tasks 頁面
    if (currentPath === "/tasks") {
        // 任務頁面使用通用的刪除模態框，不需要特殊的 JavaScript 處理
    }

    // 初始化 Racks 頁面
    if (currentPath === "/racks") {
        // 貨架頁面使用通用的刪除模態框，不需要特殊的 JavaScript 處理
    }

    // 初始化 Devices 頁面
    if (currentPath === "/devices") {
        // 設備頁面使用通用的刪除模態框，不需要特殊的 JavaScript 處理
    }

    // 初始化 Users 頁面
    if (currentPath === "/users") {
        // 用戶頁面使用通用的刪除模態框，不需要特殊的 JavaScript 處理
    }

    // 初始化 User Create 頁面
    if (currentPath === "/users/create") {
        // notify.js 已經在共用模組中初始化，這裡不需要額外處理
    }
    //初始化 首頁
    //if (window.location.pathname === "/") {
    //    // 初始化首頁 按鈕事件綁定 及設定預設值
    //    homePage.setup();
    //}
    //
    ////初始化 設定頁面
    //if (window.location.pathname === "/setting") {
    //    // 初始化設定頁面 按鈕事件綁定 及設定預設值
    //    settingPage.setup();
    //}
    //

    // 初始化 Socket.IO
    socket.setup();
});