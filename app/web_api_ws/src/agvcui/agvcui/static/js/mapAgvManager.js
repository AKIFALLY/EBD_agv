// 地圖 AGV 管理器

export const mapAgvManager = (() => {
    let agvData = new Map();

    // 初始化
    function init() {
        loadAgvData();

        // 監聽 agvsStore 的變化
        if (window.agvsStore) {
            window.agvsStore.on('change', handleAgvsChange);
            console.log('mapAgvManager: 已訂閱 agvsStore 變化');
        }
    }

    // 處理 agvsStore 變化
    function handleAgvsChange(newState) {
        if (!newState?.agvs) return;

        const agvs = newState.agvs || [];
        console.log(`mapAgvManager: 收到 AGV 更新，共 ${agvs.length} 個 AGV`);

        // 更新本地資料
        agvData.clear();
        agvs.forEach(agv => {
            agvData.set(agv.id, agv);
        });

        // 如果側邊面板正在顯示 AGV 列表，只更新內容
        const agvsList = document.getElementById('agvs-list');
        if (agvsList && agvsList.children.length > 0) {
            console.log('mapAgvManager: 更新側邊面板 AGV 列表內容');
            updateAgvsListContent(agvsList, agvs);
        }
    }

    // 載入 AGV 資料
    async function loadAgvData() {
        try {
            // 使用現有的 store 資料而不是 API 調用
            if (window.agvsStore) {
                const agvsState = window.agvsStore.getState();
                const agvs = agvsState.agvs || [];

                agvData.clear();
                agvs.forEach(agv => {
                    agvData.set(agv.id, agv);
                });

                console.log(`Loaded ${agvs.length} AGVs from store`);
            } else {
                console.warn('agvsStore not available');
            }
        } catch (error) {
            console.error('Error loading AGV data:', error);
        }
    }

    // 獲取 AGV 資料
    function getAgvData(agvId) {
        return agvData.get(agvId);
    }

    // 獲取所有 AGV 資料
    function getAllAgvData() {
        return Array.from(agvData.values());
    }

    // 更新 AGV 列表內容（保持 DOM 結構）
    function updateAgvsListContent(agvsList, agvs) {
        // 保存當前滾動位置
        const scrollTop = agvsList.scrollTop;

        // 按 ID 排序 AGV
        const sortedAgvs = agvs.sort((a, b) => a.id - b.id);

        // 更新總數資訊
        let totalInfoElement = agvsList.querySelector('.total-info');
        if (!totalInfoElement) {
            totalInfoElement = document.createElement('div');
            totalInfoElement.className = 'total-info has-text-grey is-size-7 mb-2';
            agvsList.insertBefore(totalInfoElement, agvsList.firstChild);
        }
        totalInfoElement.textContent = `總計 ${agvs.length} 個 AGV`;

        // 獲取現有的 AGV 卡片
        const existingCards = Array.from(agvsList.querySelectorAll('.map-info-card'));

        // 處理新增和更新的 AGV
        sortedAgvs.forEach((agv, index) => {
            let cardElement = existingCards.find(card => {
                const onclick = card.getAttribute('onclick');
                const match = onclick?.match(/showAgvDetails\((\d+)\)/);
                return match && parseInt(match[1]) === agv.id;
            });

            if (!cardElement) {
                // 新增 AGV 卡片
                cardElement = document.createElement('div');
                cardElement.className = 'map-info-card';
                cardElement.style.cssText = 'margin-bottom: 0.5rem; cursor: pointer;';
                cardElement.setAttribute('onclick', `mapAgvManager.showAgvDetails(${agv.id})`);

                // 插入到正確位置
                const nextCard = existingCards[index];
                if (nextCard) {
                    agvsList.insertBefore(cardElement, nextCard);
                } else {
                    agvsList.appendChild(cardElement);
                }
                existingCards.splice(index, 0, cardElement);
            }

            // 更新卡片內容
            const statusInfo = getAgvStatusInfo(agv.status_id);
            const batteryInfo = getAgvBatteryInfo(agv.battery);

            cardElement.innerHTML = `
                <div class="map-info-card-header">
                    <span class="map-info-card-title">${agv.name}</span>
                    <span class="tag ${statusInfo.class}">${statusInfo.text}</span>
                </div>
                <div class="is-size-7 has-text-grey">
                    位置: (${agv.x.toFixed(1)}, ${agv.y.toFixed(1)}) | ${batteryInfo}
                </div>
            `;
        });

        // 移除已刪除的 AGV 卡片
        existingCards.forEach(card => {
            const onclick = card.getAttribute('onclick');
            const match = onclick?.match(/showAgvDetails\((\d+)\)/);
            const agvId = match ? parseInt(match[1]) : null;

            if (agvId && !sortedAgvs.find(agv => agv.id === agvId)) {
                card.remove();
            }
        });

        // 恢復滾動位置
        agvsList.scrollTop = scrollTop;
    }

    // 載入 AGV 列表到側邊面板
    function loadAgvsList() {
        const agvs = Array.from(agvData.values());
        const agvsList = document.getElementById('agvs-list');

        if (!agvsList) return;

        if (agvs.length === 0) {
            agvsList.innerHTML = '<p class="has-text-grey">目前沒有 AGV</p>';
            return;
        }

        // 如果列表已經有內容，使用更新方式
        if (agvsList.children.length > 0) {
            updateAgvsListContent(agvsList, agvs);
            return;
        }

        // 初次載入時使用完整重建
        const sortedAgvs = agvs.sort((a, b) => a.id - b.id);

        const agvsHtml = sortedAgvs.map(agv => {
            const statusInfo = getAgvStatusInfo(agv.status_id);
            const batteryInfo = getAgvBatteryInfo(agv.battery);

            return `
                <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapAgvManager.showAgvDetails(${agv.id})">
                    <div class="map-info-card-header">
                        <span class="map-info-card-title">${agv.name}</span>
                        <span class="tag ${statusInfo.class}">${statusInfo.text}</span>
                    </div>
                    <div class="is-size-7 has-text-grey">
                        位置: (${agv.x.toFixed(1)}, ${agv.y.toFixed(1)}) | ${batteryInfo}
                    </div>
                </div>
            `;
        }).join('');

        const totalInfo = `<div class="total-info has-text-grey is-size-7 mb-2">總計 ${agvs.length} 個 AGV</div>`;
        agvsList.innerHTML = totalInfo + agvsHtml;
    }

    // 獲取 AGV 狀態資訊
    // 導入統一的狀態管理（如果可用）
    function getAgvStatusInfo(status) {
        // 處理 null, undefined 或空值
        if (status === null || status === undefined) {
            return { class: 'is-light', text: '未設定' };
        }

        switch (status) {
            case 1:
                return { class: 'is-dark', text: '離場' };
            case 2:
                return { class: 'is-danger', text: '離線' };
            case 3:
                return { class: 'is-success', text: '空閒' };
            case 4:
                return { class: 'is-info', text: '任務中' };
            case 5:
                return { class: 'is-warning', text: '充電中' };
            case 6:
                return { class: 'is-link', text: '更新中' };
            case 7:
                return { class: 'is-danger', text: '異常' };
            case 8:
                return { class: 'is-warning', text: '維護中' };
            case 9:
                return { class: 'is-light', text: '待機' };
            case 10:
                return { class: 'is-info', text: '初始化' };
            default:
                return { class: 'is-light', text: `未知(${status})` };
        }
    }

    // 獲取 AGV 電量資訊
    function getAgvBatteryInfo(battery) {
        if (battery === null || battery === undefined) {
            return '電量: 未知';
        }
        return `電量: ${battery.toFixed(1)}%`;
    }

    // 顯示 AGV 詳細資訊
    function showAgvDetails(agvId) {
        const agv = getAgvData(agvId);
        if (!agv) {
            console.warn('AGV not found:', agvId);
            return;
        }

        console.log('顯示 AGV 詳細資訊:', agv);

        // 跳轉到 AGV 編輯頁面
        window.open(`/agvs/${agvId}/edit`, '_blank');
    }

    // 公開方法
    return {
        init,
        loadAgvData,
        getAgvData,
        getAllAgvData,
        loadAgvsList,
        showAgvDetails
    };
})();

// 全域暴露
window.mapAgvManager = mapAgvManager;
