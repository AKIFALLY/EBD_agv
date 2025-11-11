/**
 * 地圖資料同步管理器
 * 處理地圖操作與後端的即時同步，以及與其他頁面的資料更新
 * 
 * 注意：客戶端不需要定期同步，因為：
 * 1. Socket.IO 連接後服務器會立即發送完整資料
 * 2. 服務器會定期向前端推送更新
 * 3. 資料流向：Socket.IO → socket.js → Store → Map頁面監聽Store變化
 */

import { mapRackManager } from './mapRackManager.js';
import { mapTaskManager } from './mapTaskManager.js';

export const mapDataSync = (() => {
    let pendingOperations = [];
    let isOnline = true;

    // 初始化
    function init() {
        console.log('Initializing map data sync (Store-based)...');

        // 設置網路狀態監聽
        setupNetworkMonitoring();

        // 設置 localStorage 監聽（用於跨頁面通信）
        setupStorageSync();

        // 初始化管理器資料（從現有 Store 載入）
        initializeManagerData();
    }

    // 初始化管理器資料
    function initializeManagerData() {
        console.log('Initializing manager data from stores...');

        // 初始化貨架管理器資料
        if (window.mapRackManager) {
            window.mapRackManager.loadRackData();
        }

        // 初始化任務管理器資料
        if (window.mapTaskManager) {
            window.mapTaskManager.loadTaskData();
        }
    }

    // 設置網路狀態監聽
    function setupNetworkMonitoring() {
        window.addEventListener('online', () => {
            isOnline = true;
            console.log('Network back online');
            processPendingOperations();
        });

        window.addEventListener('offline', () => {
            isOnline = false;
            console.log('Network offline, operations will be queued');
        });
    }

    // 設置 localStorage 監聽（用於跨頁面通信）
    function setupStorageSync() {
        window.addEventListener('storage', (e) => {
            if (e.key && e.key.startsWith('map_data_')) {
                handleExternalDataChange(e.key, e.newValue);
            }
        });
    }

    // 處理外部資料變化
    function handleExternalDataChange(key, newValue) {
        console.log('External data change detected:', key);

        if (key === 'map_data_racks' && window.mapRackManager) {
            window.mapRackManager.loadRackData();
        } else if (key === 'map_data_tasks' && window.mapTaskManager) {
            window.mapTaskManager.loadTaskData();
        }
    }

    // 處理待處理操作
    function processPendingOperations() {
        if (pendingOperations.length === 0) return;

        console.log(`Processing ${pendingOperations.length} pending operations...`);

        // 這裡可以處理離線時累積的操作
        pendingOperations.forEach(operation => {
            console.log('Processing pending operation:', operation);
        });

        pendingOperations = [];
    }

    // 廣播資料更新（用於跨頁面通信）
    function broadcastDataUpdate(type, data) {
        const key = `map_data_${type}`;
        const value = JSON.stringify({
            timestamp: Date.now(),
            data: data
        });

        try {
            localStorage.setItem(key, value);
        } catch (error) {
            console.warn('Failed to broadcast data update:', error);
        }
    }

    // 公開方法
    return {
        init,
        broadcastDataUpdate,
        processPendingOperations
    };
})();

// 全域暴露
window.mapDataSync = mapDataSync;
