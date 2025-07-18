/**
 * 地圖貨架管理模組
 * 處理地圖上貨架相關的操作和顯示
 */

import { mapPermissions } from './mapPermissions.js';
import { mapInteraction } from './mapInteraction.js';

export const mapRackManager = (() => {
    let rackData = new Map(); // 儲存貨架資料
    let carrierData = new Map(); // 儲存載具資料

    // 初始化
    function init() {
        loadRackData();
        loadCarrierData();

        // 監聽 racksStore 的變化
        if (window.racksStore) {
            window.racksStore.on('change', handleRacksChange);
            console.debug('mapRackManager: 已訂閱 racksStore 變化');
        }

        // 監聽 carriersStore 的變化
        if (window.carriersStore) {
            window.carriersStore.on('change', handleCarriersChange);
            console.debug('mapRackManager: 已訂閱 carriersStore 變化');
        }
    }

    // 處理 racksStore 變化
    function handleRacksChange(newState) {
        if (!newState?.racks) return;

        const racks = newState.racks || [];
        console.debug(`mapRackManager: 收到貨架更新，共 ${racks.length} 個貨架`);

        // 更新本地資料
        rackData.clear();
        racks.forEach(rack => {
            rackData.set(rack.id, rack);
        });

        // 如果側邊面板正在顯示貨架列表，只更新內容
        const racksList = document.getElementById('racks-list');
        if (racksList && racksList.children.length > 0) {
            console.debug('mapRackManager: 更新側邊面板貨架列表內容');
            updateRacksListContent(racksList, racks);
        }
    }

    // 處理 carriersStore 變化（用於更新載具資料）
    function handleCarriersChange(newState) {
        if (!newState?.carriers) return;

        const carriers = newState.carriers || [];
        console.debug(`mapRackManager: 收到載具更新，共 ${carriers.length} 個載具`);

        // 更新本地載具資料
        carrierData.clear();
        carriers.forEach(carrier => {
            if (carrier.rack_id) {
                if (!carrierData.has(carrier.rack_id)) {
                    carrierData.set(carrier.rack_id, []);
                }
                carrierData.get(carrier.rack_id).push(carrier);
            }
        });
    }

    // 載入貨架資料
    async function loadRackData() {
        try {
            // 使用現有的 store 資料而不是 API 調用
            if (window.racksStore) {
                const racksState = window.racksStore.getState();
                const racks = racksState.racks || [];

                rackData.clear();
                racks.forEach(rack => {
                    rackData.set(rack.id, rack);
                });

                console.debug(`Loaded ${racks.length} racks from store`);
            } else {
                console.warn('racksStore not available');
            }
        } catch (error) {
            console.error('Error loading rack data:', error);
        }
    }

    // 載入載具資料
    async function loadCarrierData() {
        try {
            // 使用現有的 store 資料而不是 API 調用
            if (window.carriersStore) {
                const carriersState = window.carriersStore.getState();
                const carriers = carriersState.carriers || [];

                carrierData.clear();
                carriers.forEach(carrier => {
                    if (carrier.rack_id) {
                        if (!carrierData.has(carrier.rack_id)) {
                            carrierData.set(carrier.rack_id, []);
                        }
                        carrierData.get(carrier.rack_id).push(carrier);
                    }
                });

                console.debug(`Loaded ${carriers.length} carriers from store for rack manager`);
            } else {
                console.warn('carriersStore not available for rack manager');
            }
        } catch (error) {
            console.error('Error loading carrier data for rack manager:', error);
        }
    }

    // 獲取貨架詳細資訊
    function getRackDetails(rackId) {
        const actualRackId = parseRackId(rackId);
        const rack = rackData.get(actualRackId);
        const carriers = carrierData.get(actualRackId) || [];

        if (!rack) {
            console.warn('Rack not found for ID:', rackId, 'Parsed as:', actualRackId);
            console.debug('Available rack IDs:', Array.from(rackData.keys()));

            // 返回預設值以避免錯誤
            return {
                rack: null,
                carriers: [],
                carrierCount: 0,
                maxCapacity: 32 // 預設為 S 產品容量
            };
        }

        console.debug('Found rack:', rack);
        console.debug('Rack product_id:', rack.product_id);

        return {
            rack,
            carriers,
            carrierCount: carriers.length,
            maxCapacity: getMaxCapacity(rack.product_id)
        };
    }

    // 獲取最大容量
    function getMaxCapacity(productId) {
        // 根據產品類型決定容量
        console.debug('getMaxCapacity called with:', productId, 'type:', typeof productId);

        if (productId) {
            // 轉換為字串進行檢查
            const productIdStr = String(productId);
            if (productIdStr.includes('L')) {
                console.debug('Product is L type, returning 16 slots');
                return 16; // L 產品 16 格
            }
        }

        console.debug('Product is S type or unknown, returning 32 slots');
        return 32; // S 產品 32 格
    }

    // 輔助函數：處理不同的 ID 格式
    function parseRackId(rackId) {
        if (typeof rackId === 'string' && rackId.startsWith('rack-info-')) {
            // 如果是 "rack-info-6" 格式，提取數字部分
            return parseInt(rackId.replace('rack-info-', ''));
        } else {
            // 如果是純數字或數字字串，直接轉換
            return parseInt(rackId);
        }
    }

    // 顯示貨架詳細資訊彈出視窗
    function showRackPopup(rackObject, latlng) {
        const details = getRackDetails(rackObject.id);
        const { rack, carriers, carrierCount, maxCapacity } = details;

        if (!rack) {
            console.warn('Rack not found:', rackObject.id);
            return;
        }

        const title = `貨架: ${rack.name || rack.id}`;

        const content = `
            <table class="table is-narrow is-fullwidth popup-table">
                <tbody>
                    <tr>
                        <td class="popup-label">貨架 ID</td>
                        <td><span class="tag is-primary">${rack.id}</span></td>
                    </tr>
                    <tr>
                        <td class="popup-label">名稱</td>
                        <td>${rack.name || 'N/A'}</td>
                    </tr>
                    <tr>
                        <td class="popup-label">位置</td>
                        <td>X: ${latlng.lng.toFixed(2)}, Y: ${latlng.lat.toFixed(2)}</td>
                    </tr>
                    <tr>
                        <td class="popup-label">狀態</td>
                        <td>
                            <span class="status-indicator">
                                <span class="status-dot ${getRackStatusColor(rack.status_id)}"></span>
                                ${getRackStatusName(rack.status_id)}
                            </span>
                        </td>
                    </tr>
                    <tr>
                        <td class="popup-label">產品類型</td>
                        <td><span class="tag is-info">${rack.product_id || 'N/A'}</span></td>
                    </tr>
                    <tr>
                        <td class="popup-label">載具數量</td>
                        <td>
                            <span class="tag ${carrierCount >= maxCapacity ? 'is-warning' : 'is-success'}">
                                ${carrierCount} / ${maxCapacity}
                            </span>
                        </td>
                    </tr>
                    ${carriers.length > 0 ? `
                    <tr>
                        <td class="popup-label">載具列表</td>
                        <td>
                            <div class="tags">
                                ${carriers.slice(0, 5).map(carrier =>
            `<span class="tag is-light">載具 ${carrier.id}</span>`
        ).join('')}
                                ${carriers.length > 5 ? `<span class="tag is-light">+${carriers.length - 5} 更多</span>` : ''}
                            </div>
                        </td>
                    </tr>
                    ` : ''}
                </tbody>
            </table>
        `;

        const actions = [
            {
                text: '查看載具',
                icon: 'mdi-package-variant',
                class: 'is-info',
                onclick: `mapRackManager.viewRackCarriers('${rack.id}')`,
                permission: 'view_carriers'
            },
            {
                text: '編輯貨架',
                icon: 'mdi-pencil',
                class: 'is-primary',
                onclick: `mapRackManager.editRack('${rack.id}')`,
                permission: 'edit_rack'
            },
            {
                text: '新增載具',
                icon: 'mdi-plus',
                class: 'is-success',
                onclick: `mapRackManager.addCarrierToRack('${rack.id}')`,
                permission: 'create_carrier'
            }
        ];

        mapInteraction.showPopup(latlng, title, content, actions);
    }

    // 顯示載具格位視覺化
    function showCarrierGrid(rackId) {
        const details = getRackDetails(rackId);
        const { rack, carriers, maxCapacity } = details;

        if (!rack) return;

        const isLProduct = rack.product_id && String(rack.product_id).includes('L');
        const gridLayout = isLProduct ?
            { rows: 2, cols: 8, sides: ['A面', 'B面'] } :
            { rows: 4, cols: 4, sides: ['A面', 'B面'] };

        let gridHtml = '<div class="carrier-grid">';

        gridLayout.sides.forEach((side, sideIndex) => {
            gridHtml += `<div class="grid-side">`;
            gridHtml += `<h6 class="title is-6">${side}</h6>`;
            gridHtml += `<div class="grid-container" style="display: grid; grid-template-columns: repeat(${gridLayout.cols}, 1fr); gap: 2px;">`;

            const startIndex = sideIndex * (maxCapacity / 2);
            for (let i = 0; i < maxCapacity / 2; i++) {
                const slotIndex = startIndex + i + 1;
                const carrier = carriers.find(c => c.rack_index === slotIndex);
                const isOccupied = !!carrier;

                gridHtml += `
                    <div class="grid-slot ${isOccupied ? 'is-occupied' : 'is-empty'}" 
                         data-slot="${slotIndex}" 
                         title="${isOccupied ? `載具 ${carrier.id}` : `空格 ${slotIndex}`}">
                        ${slotIndex}
                    </div>
                `;
            }

            gridHtml += `</div></div>`;
        });

        gridHtml += '</div>';

        return gridHtml;
    }

    // 輔助函數
    function getRackStatusColor(statusId) {
        switch (statusId) {
            case 1: return 'is-success';
            case 2: return 'is-warning';
            case 3: return 'is-danger';
            default: return 'is-light';
        }
    }

    function getRackStatusName(statusId) {
        switch (statusId) {
            case 1: return '正常';
            case 2: return '維護中';
            case 3: return '故障';
            default: return '未知';
        }
    }

    // 操作方法
    function viewRackCarriers(rackId) {
        mapPermissions.executeWithPermission('view_carriers', () => {
            const actualRackId = parseRackId(rackId);
            window.open(`/carriers?rack_id=${actualRackId}`, '_blank');
        });
    }

    function editRack(rackId) {
        mapPermissions.executeWithPermission('edit_rack', () => {
            const actualRackId = parseRackId(rackId);
            window.open(`/racks/${actualRackId}/edit`, '_blank');
        });
    }

    function addCarrierToRack(rackId) {
        mapPermissions.executeWithPermission('create_carrier', () => {
            const actualRackId = parseRackId(rackId);
            window.open(`/carriers/create?rack_id=${actualRackId}`, '_blank');
        });
    }

    // 更新貨架狀態
    function updateRackStatus(rackId, statusId) {
        mapPermissions.executeWithPermission('edit_rack', async () => {
            try {
                const actualRackId = parseRackId(rackId);

                const response = await fetch(`/api/racks/${actualRackId}`, {
                    method: 'PATCH',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ status_id: statusId })
                });

                if (response.ok) {
                    // 更新本地資料
                    const rack = rackData.get(actualRackId);
                    if (rack) {
                        rack.status_id = statusId;
                    }

                    console.debug(`Rack ${actualRackId} status updated to ${statusId}`);
                } else {
                    console.error('Failed to update rack status');
                }
            } catch (error) {
                console.error('Error updating rack status:', error);
            }
        });
    }

    // 更新貨架列表內容（保持 DOM 結構）
    function updateRacksListContent(racksList, racks) {
        // 保存當前滾動位置
        const scrollTop = racksList.scrollTop;

        // 按 ID 排序貨架
        const sortedRacks = racks.sort((a, b) => a.id - b.id);

        // 更新總數資訊
        let totalInfoElement = racksList.querySelector('.total-info');
        if (!totalInfoElement) {
            totalInfoElement = document.createElement('div');
            totalInfoElement.className = 'total-info has-text-grey is-size-7 mb-2';
            racksList.insertBefore(totalInfoElement, racksList.firstChild);
        }
        totalInfoElement.textContent = `總計 ${racks.length} 個貨架`;

        // 獲取現有的貨架卡片
        const existingCards = Array.from(racksList.querySelectorAll('.map-info-card'));

        // 處理新增和更新的貨架
        sortedRacks.forEach((rack, index) => {
            let cardElement = existingCards.find(card => {
                const onclick = card.getAttribute('onclick');
                const match = onclick?.match(/showRackDetails\((\d+)\)/);
                return match && parseInt(match[1]) === rack.id;
            });

            if (!cardElement) {
                // 新增貨架卡片
                cardElement = document.createElement('div');
                cardElement.className = 'map-info-card';
                cardElement.style.cssText = 'margin-bottom: 0.5rem; cursor: pointer;';
                cardElement.setAttribute('onclick', `mapRackManager.showRackDetails(${rack.id})`);

                // 插入到正確位置
                const nextCard = existingCards[index];
                if (nextCard) {
                    racksList.insertBefore(cardElement, nextCard);
                } else {
                    racksList.appendChild(cardElement);
                }
                existingCards.splice(index, 0, cardElement);
            }

            // 更新卡片內容
            cardElement.innerHTML = `
                <div class="map-info-card-header">
                    <span class="map-info-card-title">${rack.name || `貨架 ${rack.id}`}</span>
                    <span class="tag ${getRackStatusColor(rack.status_id)}">${getRackStatusName(rack.status_id)}</span>
                </div>
                <div class="is-size-7 has-text-grey">
                    位置: ${rack.location_id || 'N/A'} | 產品: ${rack.product_name || rack.product_id || 'N/A'}
                </div>
            `;
        });

        // 移除已刪除的貨架卡片
        existingCards.forEach(card => {
            const onclick = card.getAttribute('onclick');
            const match = onclick?.match(/showRackDetails\((\d+)\)/);
            const rackId = match ? parseInt(match[1]) : null;

            if (rackId && !sortedRacks.find(rack => rack.id === rackId)) {
                card.remove();
            }
        });

        // 恢復滾動位置
        racksList.scrollTop = scrollTop;
    }

    // 載入貨架列表到側邊面板
    function loadRacksList() {
        const racks = Array.from(rackData.values());
        const racksList = document.getElementById('racks-list');

        if (!racksList) return;

        if (racks.length === 0) {
            racksList.innerHTML = '<p class="has-text-grey">目前沒有貨架</p>';
            return;
        }

        // 如果列表已經有內容，使用更新方式
        if (racksList.children.length > 0) {
            updateRacksListContent(racksList, racks);
            return;
        }

        // 初次載入時使用完整重建
        const sortedRacks = racks.sort((a, b) => a.id - b.id);

        const racksHtml = sortedRacks.map(rack => `
            <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapRackManager.showRackDetails(${rack.id})">
                <div class="map-info-card-header">
                    <span class="map-info-card-title">${rack.name || `貨架 ${rack.id}`}</span>
                    <span class="tag ${getRackStatusColor(rack.status_id)}">${getRackStatusName(rack.status_id)}</span>
                </div>
                <div class="is-size-7 has-text-grey">
                    位置: ${rack.location_id || 'N/A'} | 產品: ${rack.product_name || rack.product_id || 'N/A'}
                </div>
            </div>
        `).join('');

        const totalInfo = `<div class="total-info has-text-grey is-size-7 mb-2">總計 ${racks.length} 個貨架</div>`;
        racksList.innerHTML = totalInfo + racksHtml;
    }

    // 顯示貨架詳情（用於列表點擊）
    function showRackDetails(rackId) {
        mapPermissions.executeWithPermission('view_racks', () => {
            window.open(`/racks/${rackId}/edit`, '_blank');
        });
    }

    // 公開方法
    return {
        init,
        loadRackData,
        loadCarrierData,
        getRackDetails,
        showRackPopup,
        showCarrierGrid,
        loadRacksList,
        showRackDetails,
        viewRackCarriers,
        editRack,
        addCarrierToRack,
        updateRackStatus
    };
})();

// 全域暴露
window.mapRackManager = mapRackManager;
