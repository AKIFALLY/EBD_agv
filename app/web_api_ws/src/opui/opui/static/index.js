//console.debug = function () {};
import { clientStore, productsStore } from './store/index.js';

//import { store } from './store.js';
//import { initSocket } from './socket.js';
import { navbar } from './js/navbar.js';
//import { initHomePage } from './home.js';
import { settingPage } from './js/settingPage.js';
import { homePage } from './js/homePage.js';
import { notify } from './js/notify.js';
import socket from './js/socket.js';// 在任何頁面引入 socket 模組

// ---------------- DOM 完成後初始化 ----------------
document.addEventListener("DOMContentLoaded", () => {

    //初始化 Socket.IO
    //initSocket();

    //初始化 Navbar
    navbar.setup();

    //初始化 首頁
    if (window.location.pathname === "/") {
        // 初始化首頁 按鈕事件綁定 及設定預設值
        homePage.setup();
    }

    //初始化 設定頁面
    if (window.location.pathname === "/setting") {
        // 初始化設定頁面 按鈕事件綁定 及設定預設值
        settingPage.setup();
    }

    //socket連線,取得初始資料等等
    socket.setup();
});