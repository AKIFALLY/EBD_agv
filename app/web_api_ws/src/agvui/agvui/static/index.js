//console.debug = function () {};
import { agvStore } from '/static/store/index.js';
import { navbar } from '/static/js/navbar.js';
import { agvPage } from '/static/js/agvPage.js';
import socket from '/static/js/socket.js';// 在任何頁面引入 socket 模組

// ---------------- DOM 完成後初始化 ----------------
document.addEventListener("DOMContentLoaded", () => {

    //初始化 Navbar
    navbar.setup();

    //初始化 首頁
    if (window.location.pathname === "/") {
        // 初始化首頁 按鈕事件綁定 及設定預設值
        agvPage.setup();
    }

    //socket連線,取得初始資料等等
    socket.setup();
});