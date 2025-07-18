// roomCardGenerator.js - 可擴展的房間卡片生成器
// 提供通用的房間監控卡片創建和管理功能

export const roomCardGenerator = (() => {
    
    /**
     * 房間配置定義
     * 可以輕鬆添加新房間的配置
     */
    const ROOM_CONFIGS = {
        1: {
            name: '房間1監控',
            icon: 'mdi-home-variant-outline',
            entranceLocationIds: [100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110],
            exitLocationIds: [190, 191, 192, 193, 194, 195, 196, 197, 198, 199]
        },
        2: {
            name: '房間2監控',
            icon: 'mdi-home-variant',
            entranceLocationIds: [200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210],
            exitLocationIds: [290, 291, 292, 293, 294, 295, 296, 297, 298, 299]
        },
        3: {
            name: '房間3監控',
            icon: 'mdi-home-city-outline',
            entranceLocationIds: [300, 301, 302, 303, 304, 305, 306, 307, 308, 309, 310],
            exitLocationIds: [390, 391, 392, 393, 394, 395, 396, 397, 398, 399]
        },
        4: {
            name: '房間4監控',
            icon: 'mdi-home-city',
            entranceLocationIds: [400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410],
            exitLocationIds: [490, 491, 492, 493, 494, 495, 496, 497, 498, 499]
        },
        5: {
            name: '房間5監控',
            icon: 'mdi-home-modern',
            entranceLocationIds: [500, 501, 502, 503, 504, 505, 506, 507, 508, 509, 510],
            exitLocationIds: [590, 591, 592, 593, 594, 595, 596, 597, 598, 599]
        }
    };

    /**
     * 生成房間卡片的 HTML 結構
     * @param {number} roomId - 房間 ID
     * @returns {string} 房間卡片的 HTML 字符串
     */
    function generateRoomCardHTML(roomId) {
        const config = ROOM_CONFIGS[roomId];
        if (!config) {
            console.error(`房間 ${roomId} 的配置不存在`);
            return '';
        }

        return `
        <!-- 房間${roomId}監控卡片 -->
        <div class="dashboard-card" id="dashboard-card-room${roomId}">
            <div class="dashboard-card-header">
                <div class="dashboard-card-title">
                    <span class="dashboard-card-icon">
                        <i class="${config.icon}"></i>
                    </span>
                    ${config.name}
                </div>
            </div>
            
            <div class="dashboard-loading" style="display: flex;">
                <span class="icon">
                    <i class="mdi mdi-loading"></i>
                </span>
                載入中...
            </div>

            <div class="dashboard-metric">
                <div class="dashboard-metric-value" id="dashboard-metric-room${roomId}Carriers">0</div>
                <div class="dashboard-metric-label">處理中載具</div>
            </div>

            <div class="dashboard-status" id="dashboard-status-room${roomId}">
                <div class="dashboard-status-dot"></div>
                <div class="dashboard-status-text">等待資料...</div>
            </div>

            <!-- 生產資訊 -->
            <div class="dashboard-list" id="dashboard-room${roomId}-production">
                <div class="dashboard-list-item">
                    <span class="dashboard-list-item-text">產品：</span>
                    <span class="dashboard-list-item-value" id="dashboard-metric-room${roomId}Product">-</span>
                </div>
                <div class="dashboard-list-item">
                    <span class="dashboard-list-item-text">規格：</span>
                    <span class="dashboard-list-item-value" id="dashboard-metric-room${roomId}Size">-</span>
                </div>
                <div class="dashboard-list-item">
                    <span class="dashboard-list-item-text">泡藥次數：</span>
                    <span class="dashboard-list-item-value" id="dashboard-metric-room${roomId}SoakingTimes">-</span>
                </div>
            </div>

            <!-- 物流狀態 -->
            <div class="dashboard-trend">
                <span>入口：<span id="dashboard-metric-room${roomId}Entrance" class="status-info">無貨架</span></span>
                <span>出口：<span id="dashboard-metric-room${roomId}Exit" class="status-info">無貨架</span></span>
            </div>
        </div>`;
    }

    /**
     * 動態插入房間卡片到 Dashboard
     * @param {number} roomId - 房間 ID
     * @param {string} insertPosition - 插入位置 ('beforeend', 'afterbegin', 等)
     */
    function insertRoomCard(roomId, insertPosition = 'beforeend') {
        const dashboardGrid = document.querySelector('.dashboard-grid');
        if (!dashboardGrid) {
            console.error('找不到 dashboard-grid 容器');
            return false;
        }

        const cardHTML = generateRoomCardHTML(roomId);
        if (!cardHTML) {
            return false;
        }

        dashboardGrid.insertAdjacentHTML(insertPosition, cardHTML);
        console.log(`✅ 房間${roomId}卡片已插入到 Dashboard`);
        return true;
    }

    /**
     * 移除房間卡片
     * @param {number} roomId - 房間 ID
     */
    function removeRoomCard(roomId) {
        const card = document.getElementById(`dashboard-card-room${roomId}`);
        if (card) {
            card.remove();
            console.log(`✅ 房間${roomId}卡片已移除`);
            return true;
        }
        return false;
    }

    /**
     * 檢查房間卡片是否存在
     * @param {number} roomId - 房間 ID
     * @returns {boolean} 是否存在
     */
    function hasRoomCard(roomId) {
        return document.getElementById(`dashboard-card-room${roomId}`) !== null;
    }

    /**
     * 獲取房間配置
     * @param {number} roomId - 房間 ID
     * @returns {Object|null} 房間配置
     */
    function getRoomConfig(roomId) {
        return ROOM_CONFIGS[roomId] || null;
    }

    /**
     * 獲取所有可用的房間 ID
     * @returns {number[]} 房間 ID 列表
     */
    function getAvailableRoomIds() {
        return Object.keys(ROOM_CONFIGS).map(id => parseInt(id));
    }

    /**
     * 批量創建多個房間卡片
     * @param {number[]} roomIds - 房間 ID 列表
     */
    function createMultipleRoomCards(roomIds) {
        const results = [];
        roomIds.forEach(roomId => {
            if (!hasRoomCard(roomId)) {
                const success = insertRoomCard(roomId);
                results.push({ roomId, success });
            } else {
                console.log(`房間${roomId}卡片已存在，跳過創建`);
                results.push({ roomId, success: true, skipped: true });
            }
        });
        return results;
    }

    /**
     * 根據房間 ID 計算載具統計
     * @param {Array} carriers - 載具列表
     * @param {number} roomId - 房間 ID
     * @returns {Object} 載具統計資料
     */
    function calculateRoomCarrierStats(carriers, roomId) {
        const config = getRoomConfig(roomId);
        if (!config) return { carriersInProcess: 0 };

        // 根據 location_id 範圍判斷房間
        const roomLocationRange = [roomId * 100, (roomId + 1) * 100 - 1];
        
        const roomCarriers = carriers.filter(carrier => {
            // 如果載具在設備端口，檢查設備的 location_id
            if (carrier.port_id && carrier.location_id) {
                const locationId = carrier.location_id;
                return locationId >= roomLocationRange[0] && locationId <= roomLocationRange[1];
            }
            // 如果載具有 room_id，直接檢查
            return carrier.room_id === roomId;
        });

        return {
            carriersInProcess: roomCarriers.length
        };
    }

    /**
     * 檢查房間的貨架狀態
     * @param {Array} racks - 貨架列表
     * @param {number} roomId - 房間 ID
     * @returns {Object} 貨架狀態資料
     */
    function checkRoomRackStatus(racks, roomId) {
        const config = getRoomConfig(roomId);
        if (!config) {
            return {
                entranceHasRack: false,
                exitHasRack: false,
                entranceCount: 0,
                exitCount: 0
            };
        }

        // 檢查入口是否有貨架
        const entranceRacks = racks.filter(rack => 
            config.entranceLocationIds.includes(rack.location_id) && rack.count > 0
        );
        
        // 檢查出口是否有貨架
        const exitRacks = racks.filter(rack => 
            config.exitLocationIds.includes(rack.location_id) && rack.count > 0
        );

        return {
            entranceHasRack: entranceRacks.length > 0,
            exitHasRack: exitRacks.length > 0,
            entranceCount: entranceRacks.length,
            exitCount: exitRacks.length
        };
    }

    /**
     * 獲取房間相關的產品資訊
     * @param {Array} products - 產品列表
     * @param {number} roomId - 房間 ID
     * @returns {Object|null} 產品資訊
     */
    function getRoomProductInfo(products, roomId) {
        const roomProducts = products.filter(product => 
            product.room === roomId || product.room_id === roomId
        );
        
        if (roomProducts.length === 0) {
            return null;
        }

        // 取第一個產品作為當前處理的產品
        const currentProduct = roomProducts[0];
        
        return {
            name: currentProduct.name || '未知產品',
            size: currentProduct.size || '-',
            soakingTimes: currentProduct.soaking_times || 
                         currentProduct.process_settings?.soaking_times || 0
        };
    }

    // 返回公開的方法
    return {
        generateRoomCardHTML,
        insertRoomCard,
        removeRoomCard,
        hasRoomCard,
        getRoomConfig,
        getAvailableRoomIds,
        createMultipleRoomCards,
        calculateRoomCarrierStats,
        checkRoomRackStatus,
        getRoomProductInfo,
        ROOM_CONFIGS
    };
})();
