/**
 * 地圖載具管理模組
 * 處理地圖上載具相關的操作和顯示
 */

import { mapPermissions } from './mapPermissions.js';
import { mapInteraction } from './mapInteraction.js';

export const mapCarrierManager = (() => {
    let carrierData = new Map(); // 儲存載具資料
    let rackData = new Map(); // 儲存貨架資料
    let eqpPortData = new Map(); // 儲存設備端口資料

    // 初始化
    function init() {
        loadCarrierData();
        loadRackData();
        loadEqpPortData();

        // 監聽 carriersStore 的變化
        if (window.carriersStore) {
            window.carriersStore.on('change', handleCarriersChange);
            console.log('mapCarrierManager: 已訂閱 carriersStore 變化');
        }

        // 監聽 racksStore 的變化
        if (window.racksStore) {
            window.racksStore.on('change', handleRacksChange);
            console.log('mapCarrierManager: 已訂閱 racksStore 變化');
        }
    }

    // 處理 carriersStore 變化
    function handleCarriersChange(newState) {
        if (!newState?.carriers) return;

        const carriers = newState.carriers || [];
        console.log(`mapCarrierManager: 收到載具更新，共 ${carriers.length} 個載具`);

        // 更新本地資料
        carrierData.clear();
        carriers.forEach(carrier => {
            carrierData.set(carrier.id, carrier);
        });

        // 如果側邊面板正在顯示載具列表，只更新內容
        const carriersList = document.getElementById('carriers-list');
        if (carriersList && carriersList.children.length > 0) {
            console.log('mapCarrierManager: 更新側邊面板載具列表內容');
            updateCarriersListContent(carriersList, carriers);
        }
    }

    // 處理 racksStore 變化
    function handleRacksChange(newState) {
        if (!newState?.racks) return;

        const racks = newState.racks || [];
        console.log(`mapCarrierManager: 收到貨架更新，共 ${racks.length} 個貨架`);

        // 更新本地資料
        rackData.clear();
        racks.forEach(rack => {
            rackData.set(rack.id, rack);
        });
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
                    carrierData.set(carrier.id, carrier);
                });

                console.log(`Loaded ${carriers.length} carriers from store`);
            } else {
                console.warn('carriersStore not available');
            }
        } catch (error) {
            console.error('Error loading carrier data:', error);
        }
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

                console.log(`Loaded ${racks.length} racks from store for carrier manager`);
            } else {
                console.warn('racksStore not available for carrier manager');
            }
        } catch (error) {
            console.error('Error loading rack data:', error);
        }
    }

    // 載入設備端口資料
    async function loadEqpPortData() {
        try {
            // 暫時使用空資料，因為沒有對應的 store
            // 如果需要，可以從 machinesStore 或其他地方獲取
            eqpPortData.clear();
            console.log('EqpPort data loading skipped - no corresponding store available');
        } catch (error) {
            console.error('Error loading eqp port data:', error);
        }
    }

    // 獲取載具詳細資訊
    function getCarrierDetails(carrierId) {
        const carrier = carrierData.get(parseInt(carrierId));
        if (!carrier) return null;

        const rack = carrier.rack_id ? rackData.get(carrier.rack_id) : null;
        const eqpPort = carrier.port_id ? eqpPortData.get(carrier.port_id) : null;

        return {
            carrier,
            rack,
            eqpPort,
            location: getCarrierLocation(carrier, rack, eqpPort)
        };
    }

    // 獲取載具位置描述
    function getCarrierLocation(carrier, rack, eqpPort) {
        if (rack) {
            return {
                type: 'rack',
                description: `貨架 ${rack.name || rack.id}`,
                position: `位置 ${carrier.rack_index || 'N/A'}`
            };
        } else if (eqpPort) {
            return {
                type: 'equipment',
                description: `設備端口 ${eqpPort.eqp_id}`,
                position: `端口 ${eqpPort.port_name || eqpPort.id}`
            };
        } else {
            return {
                type: 'unknown',
                description: '未知位置',
                position: ''
            };
        }
    }

    // 顯示載具詳細資訊彈出視窗
    function showCarrierPopup(carrierId, latlng) {
        const details = getCarrierDetails(carrierId);
        if (!details) {
            console.warn('Carrier not found:', carrierId);
            return;
        }

        const { carrier, location } = details;
        const title = `載具: ${carrier.id}`;

        const content = `
            <div class="map-info-card">
                <div class="field">
                    <label class="label">載具 ID</label>
                    <div class="control">
                        <span class="tag is-primary">${carrier.id}</span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">狀態</label>
                    <div class="control">
                        <span class="status-indicator">
                            <span class="status-dot ${getCarrierStatusColor(carrier.status_id)}"></span>
                            ${getCarrierStatusName(carrier.status_id)}
                        </span>
                    </div>
                </div>
                <div class="field">
                    <label class="label">位置類型</label>
                    <div class="control">
                        <span class="tag ${location.type === 'rack' ? 'is-info' : location.type === 'equipment' ? 'is-warning' : 'is-light'}">
                            ${location.description}
                        </span>
                    </div>
                </div>
                ${location.position ? `
                    <div class="field">
                        <label class="label">具體位置</label>
                        <div class="control">
                            <span>${location.position}</span>
                        </div>
                    </div>
                ` : ''}
                ${carrier.product_id ? `
                    <div class="field">
                        <label class="label">產品 ID</label>
                        <div class="control">
                            <span class="tag is-light">${carrier.product_id}</span>
                        </div>
                    </div>
                ` : ''}
                <div class="field">
                    <label class="label">創建時間</label>
                    <div class="control">
                        <span class="is-size-7 has-text-grey">
                            ${carrier.created_at ? new Date(carrier.created_at).toLocaleString() : 'N/A'}
                        </span>
                    </div>
                </div>
            </div>
        `;

        const actions = [
            {
                text: '編輯載具',
                icon: 'mdi-pencil',
                class: 'is-primary',
                onclick: `mapCarrierManager.editCarrier('${carrier.id}')`,
                permission: 'edit_carrier'
            },
            {
                text: '移動載具',
                icon: 'mdi-arrow-all',
                class: 'is-info',
                onclick: `mapCarrierManager.moveCarrier('${carrier.id}')`,
                permission: 'edit_carrier'
            },
            {
                text: '刪除載具',
                icon: 'mdi-delete',
                class: 'is-danger',
                onclick: `mapCarrierManager.deleteCarrier('${carrier.id}')`,
                permission: 'delete_carrier'
            }
        ];

        mapInteraction.showPopup(latlng, title, content, actions);
    }

    // 顯示載具統計資訊
    function showCarrierStats() {
        const totalCarriers = carrierData.size;
        const statusCounts = {};
        const locationCounts = { rack: 0, equipment: 0, unknown: 0 };

        carrierData.forEach(carrier => {
            // 統計狀態
            const status = carrier.status_id || 0;
            statusCounts[status] = (statusCounts[status] || 0) + 1;

            // 統計位置類型
            if (carrier.rack_id) {
                locationCounts.rack++;
            } else if (carrier.port_id) {
                locationCounts.equipment++;
            } else {
                locationCounts.unknown++;
            }
        });

        return {
            total: totalCarriers,
            statusCounts,
            locationCounts
        };
    }

    // 輔助函數
    function getCarrierStatusColor(statusId) {
        switch (statusId) {
            case 1: return 'is-success';   // 正常
            case 2: return 'is-info';      // 使用中
            case 3: return 'is-warning';   // 維護中
            case 4: return 'is-danger';    // 故障
            default: return 'is-light';
        }
    }

    function getCarrierStatusName(statusId) {
        switch (statusId) {
            case 1: return '正常';
            case 2: return '使用中';
            case 3: return '維護中';
            case 4: return '故障';
            default: return '未知';
        }
    }

    // 操作方法
    function editCarrier(carrierId) {
        mapPermissions.executeWithPermission('edit_carrier', () => {
            window.open(`/carriers/${carrierId}/edit`, '_blank');
        });
    }

    function moveCarrier(carrierId) {
        mapPermissions.executeWithPermission('edit_carrier', () => {
            // 可以實作載具移動邏輯
            console.log('Move carrier:', carrierId);
            // 例如：顯示移動對話框或打開移動頁面
        });
    }

    function deleteCarrier(carrierId) {
        mapPermissions.executeWithPermission('delete_carrier', async () => {
            if (confirm('確定要刪除這個載具嗎？')) {
                try {
                    const response = await fetch(`/api/carriers/${carrierId}`, {
                        method: 'DELETE'
                    });

                    if (response.ok) {
                        carrierData.delete(parseInt(carrierId));
                        console.log(`Carrier ${carrierId} deleted`);

                        // 關閉彈出視窗
                        mapInteraction.closePopup();

                        // 可以觸發地圖更新
                        if (window.notify) {
                            window.notify.success('載具已刪除');
                        }
                    } else {
                        console.error('Failed to delete carrier');
                        if (window.notify) {
                            window.notify.error('刪除載具失敗');
                        }
                    }
                } catch (error) {
                    console.error('Error deleting carrier:', error);
                    if (window.notify) {
                        window.notify.error('刪除載具時發生錯誤');
                    }
                }
            }
        });
    }

    // 更新載具狀態
    function updateCarrierStatus(carrierId, statusId) {
        mapPermissions.executeWithPermission('edit_carrier', async () => {
            try {
                const response = await fetch(`/api/carriers/${carrierId}`, {
                    method: 'PATCH',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ status_id: statusId })
                });

                if (response.ok) {
                    // 更新本地資料
                    const carrier = carrierData.get(parseInt(carrierId));
                    if (carrier) {
                        carrier.status_id = statusId;
                    }

                    console.log(`Carrier ${carrierId} status updated to ${statusId}`);
                } else {
                    console.error('Failed to update carrier status');
                }
            } catch (error) {
                console.error('Error updating carrier status:', error);
            }
        });
    }

    // 批量操作載具
    function batchUpdateCarriers(carrierIds, updates) {
        mapPermissions.executeWithPermission('edit_carrier', async () => {
            try {
                const response = await fetch('/api/carriers/batch', {
                    method: 'PATCH',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        carrier_ids: carrierIds,
                        updates: updates
                    })
                });

                if (response.ok) {
                    console.log(`Batch updated ${carrierIds.length} carriers`);
                    // 重新載入資料
                    loadCarrierData();
                } else {
                    console.error('Failed to batch update carriers');
                }
            } catch (error) {
                console.error('Error batch updating carriers:', error);
            }
        });
    }

    // 更新載具列表內容（保持 DOM 結構）
    function updateCarriersListContent(carriersList, carriers) {
        // 保存當前滾動位置
        const scrollTop = carriersList.scrollTop;

        // 按 ID 排序載具
        const sortedCarriers = carriers.sort((a, b) => a.id - b.id);

        // 更新總數資訊
        let totalInfoElement = carriersList.querySelector('.total-info');
        if (!totalInfoElement) {
            totalInfoElement = document.createElement('div');
            totalInfoElement.className = 'total-info has-text-grey is-size-7 mb-2';
            carriersList.insertBefore(totalInfoElement, carriersList.firstChild);
        }
        totalInfoElement.textContent = `總計 ${carriers.length} 個載具`;

        // 獲取現有的載具卡片
        const existingCards = Array.from(carriersList.querySelectorAll('.map-info-card'));

        // 處理新增和更新的載具
        sortedCarriers.forEach((carrier, index) => {
            let cardElement = existingCards.find(card => {
                const onclick = card.getAttribute('onclick');
                const match = onclick?.match(/showCarrierDetails\((\d+)\)/);
                return match && parseInt(match[1]) === carrier.id;
            });

            if (!cardElement) {
                // 新增載具卡片
                cardElement = document.createElement('div');
                cardElement.className = 'map-info-card';
                cardElement.style.cssText = 'margin-bottom: 0.5rem; cursor: pointer;';
                cardElement.setAttribute('onclick', `mapCarrierManager.showCarrierDetails(${carrier.id})`);

                // 插入到正確位置
                const nextCard = existingCards[index];
                if (nextCard) {
                    carriersList.insertBefore(cardElement, nextCard);
                } else {
                    carriersList.appendChild(cardElement);
                }
                existingCards.splice(index, 0, cardElement);
            }

            // 更新卡片內容
            cardElement.innerHTML = `
                <div class="map-info-card-header">
                    <span class="map-info-card-title">載具 ${carrier.id}</span>
                    <span class="tag ${getCarrierStatusColor(carrier.status_id)}">${getCarrierStatusName(carrier.status_id)}</span>
                </div>
                <div class="is-size-7 has-text-grey">
                    貨架: ${carrier.rack_id || 'N/A'} | 位置: ${carrier.rack_index || 'N/A'}
                </div>
            `;
        });

        // 移除已刪除的載具卡片
        existingCards.forEach(card => {
            const onclick = card.getAttribute('onclick');
            const match = onclick?.match(/showCarrierDetails\((\d+)\)/);
            const carrierId = match ? parseInt(match[1]) : null;

            if (carrierId && !sortedCarriers.find(carrier => carrier.id === carrierId)) {
                card.remove();
            }
        });

        // 恢復滾動位置
        carriersList.scrollTop = scrollTop;
    }

    // 載入載具列表到側邊面板
    function loadCarriersList() {
        const carriers = Array.from(carrierData.values());
        const carriersList = document.getElementById('carriers-list');

        if (!carriersList) return;

        if (carriers.length === 0) {
            carriersList.innerHTML = '<p class="has-text-grey">目前沒有載具</p>';
            return;
        }

        // 如果列表已經有內容，使用更新方式
        if (carriersList.children.length > 0) {
            updateCarriersListContent(carriersList, carriers);
            return;
        }

        // 初次載入時使用完整重建
        const sortedCarriers = carriers.sort((a, b) => a.id - b.id);

        const carriersHtml = sortedCarriers.map(carrier => `
            <div class="map-info-card" style="margin-bottom: 0.5rem; cursor: pointer;" onclick="mapCarrierManager.showCarrierDetails(${carrier.id})">
                <div class="map-info-card-header">
                    <span class="map-info-card-title">載具 ${carrier.id}</span>
                    <span class="tag ${getCarrierStatusColor(carrier.status_id)}">${getCarrierStatusName(carrier.status_id)}</span>
                </div>
                <div class="is-size-7 has-text-grey">
                    貨架: ${carrier.rack_id || 'N/A'} | 位置: ${carrier.rack_index || 'N/A'}
                </div>
            </div>
        `).join('');

        const totalInfo = `<div class="total-info has-text-grey is-size-7 mb-2">總計 ${carriers.length} 個載具</div>`;
        carriersList.innerHTML = totalInfo + carriersHtml;
    }

    // 顯示載具詳情（用於列表點擊）
    function showCarrierDetails(carrierId) {
        mapPermissions.executeWithPermission('view_carriers', () => {
            window.open(`/carriers/${carrierId}/edit`, '_blank');
        });
    }

    // 公開方法
    return {
        init,
        loadCarrierData,
        loadRackData,
        loadEqpPortData,
        getCarrierDetails,
        showCarrierPopup,
        showCarrierStats,
        loadCarriersList,
        showCarrierDetails,
        editCarrier,
        moveCarrier,
        deleteCarrier,
        updateCarrierStatus,
        batchUpdateCarriers
    };
})();

// 全域暴露
window.mapCarrierManager = mapCarrierManager;
