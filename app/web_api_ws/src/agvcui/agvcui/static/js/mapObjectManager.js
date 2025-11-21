/**
 * 地圖物件管理器
 * 統一管理地圖上所有物件的互動功能
 */

import { mapInteraction } from './mapInteraction.js';
import { mapPermissions } from './mapPermissions.js';
import { mapRackManager } from './mapRackManager.js';
import { mapTaskManager } from './mapTaskManager.js';
import { mapAgvManager } from './mapAgvManager.js';
import { mapAuditLogger } from './mapAuditLogger.js';
import { getStatusInfo } from './agvStatus.js';

export const mapObjectManager = (() => {
    // 初始化
    function init() {
        // 初始化管理器
        mapRackManager.init();
        mapTaskManager.init();
        mapAgvManager.init();
        mapAuditLogger.init();
    }

    // 設置 AGV 物件互動 (改為 Hover 模式)
    function setupAgvInteraction(agvObject) {
        console.debug('Setting up AGV hover interaction for:', agvObject.id, 'name:', agvObject.name, 'className:', agvObject.el?.className);

        // 檢查 AGV 物件是否有必要的方法
        if (!agvObject.addHoverHandler) {
            console.error('AGV object missing addHoverHandler method:', agvObject);
            return;
        }

        agvObject.addHoverHandler({
            onEnter: (obj, e) => {
                // 取消任何待執行的關閉定時器
                mapInteraction.cancelPopupClose();

                console.debug('AGV hover enter:', obj.id, 'name:', obj.name, 'agvId:', obj.agvId);
                const agvData = obj.getData();
                const agvId = obj.agvId || obj.id; // 使用 agvId 或 fallback 到 id
                const title = `AGV: ${obj.name || 'Unknown'}`;

                // 取得 AGV 狀態資訊
                const statusInfo = getStatusInfo(agvData.status_id);

                const content = `
                    <table class="table is-narrow is-fullwidth popup-table">
                        <tbody>
                            <tr>
                                <td class="popup-label">AGV ID</td>
                                <td><span class="tag is-info">${obj.id || 'N/A'}</span></td>
                            </tr>
                            <tr>
                                <td class="popup-label">名稱</td>
                                <td>${obj.name || 'N/A'}</td>
                            </tr>
                            <tr>
                                <td class="popup-label">位置</td>
                                <td>X: ${obj.latlng.lng.toFixed(2)}, Y: ${obj.latlng.lat.toFixed(2)}</td>
                            </tr>
                            <tr>
                                <td class="popup-label">狀態</td>
                                <td>
                                    <span class="status-indicator">
                                        <span class="status-dot ${statusInfo.color}"></span>
                                        ${statusInfo.name}
                                    </span>
                                </td>
                            </tr>
                        </tbody>
                    </table>
                `;

                const actions = [];

                actions.push({
                    text: '查看任務',
                    icon: 'mdi-format-list-checks',
                    class: 'is-primary',
                    onclick: `mapObjectManager.viewAgvTasks('${agvId}')`,
                    permission: 'view_tasks'
                });

                actions.push({
                    text: '分配任務',
                    icon: 'mdi-plus',
                    class: 'is-success',
                    onclick: `mapObjectManager.assignTaskToAgv('${agvId}')`,
                    permission: 'create_task'
                });

                // KUKA AGV 專用：移動功能按鈕
                // 檢查是否為 KUKA AGV (根據 className 或其他識別方式)
                const isKukaAgv = obj.el?.className?.includes('kuka-agv') ||
                                  agvData?.type === 'kuka' ||
                                  agvData?.model?.toLowerCase().includes('kuka');

                if (isKukaAgv) {
                    actions.push({
                        text: '移動',
                        icon: 'mdi-map-marker-path',
                        class: 'is-link',
                        onclick: `mapObjectManager.createMoveTaskForKukaAgv('${agvId}')`,
                        permission: 'create_task'
                    });
                }

                mapInteraction.showPopup(obj.latlng, title, content, actions);

                // 記錄查看操作
                mapAuditLogger.logView(mapAuditLogger.RESOURCE_TYPES.AGV, obj.id, {
                    agvName: obj.name,
                    location: { x: obj.latlng.lng, y: obj.latlng.lat }
                });
            },
            onLeave: (obj, e) => {
                console.debug('AGV hover leave:', obj.id);
                // 延遲 300ms 關閉 popup，讓用戶有時間將鼠標移到 popup 上
                mapInteraction.schedulePopupClose(300);
            }
        });
    }

    // 設置設備物件互動
    function setupEquipmentInteraction(eqpObject) {
        eqpObject.addClickHandler((obj, e) => {
            const title = `設備: ${obj.eqp_id}`;

            const content = `
                <table class="table is-narrow is-fullwidth popup-table">
                    <tbody>
                        <tr>
                            <td class="popup-label">設備 ID</td>
                            <td><span class="tag is-primary">${obj.eqp_id}</span></td>
                        </tr>
                        <tr>
                            <td class="popup-label">類型</td>
                            <td>${getEquipmentTypeName(obj)}</td>
                        </tr>
                        <tr>
                            <td class="popup-label">位置</td>
                            <td>X: ${obj.latlng.lng.toFixed(2)}, Y: ${obj.latlng.lat.toFixed(2)}</td>
                        </tr>
                        <tr>
                            <td class="popup-label">端口狀態</td>
                            <td><div id="port-status-${obj.eqp_id}">載入中...</div></td>
                        </tr>
                    </tbody>
                </table>
            `;

            const actions = [];

            actions.push({
                text: '查看詳情',
                icon: 'mdi-information',
                class: 'is-info',
                onclick: `mapObjectManager.viewEquipmentDetails('${obj.eqp_id}')`,
                permission: 'view_equipment'
            });

            actions.push({
                text: '控制設備',
                icon: 'mdi-cog',
                class: 'is-warning',
                onclick: `mapObjectManager.controlEquipment('${obj.eqp_id}')`,
                permission: 'control_equipment'
            });

            mapInteraction.showPopup(obj.latlng, title, content, actions);

            // 載入端口狀態
            loadPortStatus(obj.eqp_id);
        });
    }

    // 設置貨架物件互動
    function setupRackInteraction(rackObject) {
        console.debug('Setting up Rack interaction for:', rackObject.id);
        rackObject.addClickHandler((obj, e) => {
            console.debug('Rack clicked:', obj.id);

            // 移除切換功能，現在由切換按鈕處理
            // obj.setMini(!obj.isMiniMode);

            // 只顯示貨架詳細資訊彈出視窗
            if (window.mapRackManager && typeof window.mapRackManager.showRackPopup === 'function') {
                window.mapRackManager.showRackPopup(obj, obj.latlng);
            } else {
                console.warn('mapRackManager.showRackPopup not available');
            }
        });
    }

    // 設置節點物件互動
    function setupNodeInteraction(nodeObject) {
        console.debug('Setting up Node interaction for:', nodeObject.id);
        nodeObject.addClickHandler((obj, e) => {
            console.debug('Node clicked:', obj.id);

            // 從 mapStore 獲取完整的節點資料
            let fullNodeData = null;
            if (window.mapStore) {
                const mapState = window.mapStore.getState();

                // 先在 kukaNodes 中查找
                fullNodeData = mapState.kukaNodes?.find(n => n.id === obj.nodeId);

                // 如果沒找到，再在 nodes 中查找
                if (!fullNodeData) {
                    fullNodeData = mapState.nodes?.find(n => n.id === obj.nodeId);
                }
            }

            const nodeData = fullNodeData || obj.getData();
            const title = `節點: ${nodeData.name || obj.nodeId}`;

            let content = `
                <table class="table is-narrow is-fullwidth popup-table">
                    <tbody>
                        <tr>
                            <td class="popup-label">節點 ID</td>
                            <td><span class="tag is-info">${obj.nodeId}</span></td>
                        </tr>
            `;

            // 如果有 UUID，顯示 UUID
            if (nodeData.uuid) {
                content += `
                        <tr>
                            <td class="popup-label">UUID</td>
                            <td><span class="tag is-light">${nodeData.uuid}</span></td>
                        </tr>
                `;
            }

            // 如果有名稱，顯示名稱
            if (nodeData.name) {
                content += `
                        <tr>
                            <td class="popup-label">名稱</td>
                            <td>${nodeData.name}</td>
                        </tr>
                `;
            }

            content += `
                <table class="table is-narrow is-fullwidth popup-table">
                    <tbody>
                        <tr>
                            <td class="popup-label">類型</td>
                            <td>${getNodeTypeName(nodeData.node_type_id)}</td>
                        </tr>
                        <tr>
                            <td class="popup-label">位置</td>
                            <td>X: ${obj.latlng.lng.toFixed(2)}, Y: ${obj.latlng.lat.toFixed(2)}</td>
                        </tr>
            `;

            // 如果有描述，顯示描述
            if (nodeData.description) {
                content += `
                        <tr>
                            <td class="popup-label">描述</td>
                            <td>${nodeData.description}</td>
                        </tr>
                `;
            }

            content += `
                    </tbody>
                </table>
            `;

            const actions = [];

            actions.push({
                text: '編輯節點',
                icon: 'mdi-pencil',
                class: 'is-info',
                onclick: `mapObjectManager.editNode(${obj.nodeId})`,
                permission: 'edit_node'
            });

            actions.push({
                text: '創建任務',
                icon: 'mdi-plus',
                class: 'is-primary',
                onclick: `mapObjectManager.createTaskAtNode('${obj.nodeId}')`,
                permission: 'create_task'
            });

            mapInteraction.showPopup(obj.latlng, title, content, actions);
        });
    }

    // 輔助函數
    function getEquipmentTypeName(eqpObject) {
        const className = eqpObject.el?.className || '';
        if (className.includes('transfer-box')) return '傳輸箱';
        if (className.includes('cleaner')) return '清洗設備';
        if (className.includes('soaking')) return '浸泡設備';
        if (className.includes('dryer')) return '烘乾設備';
        if (className.includes('oven')) return '烤箱設備';
        return '未知設備';
    }

    // 這些函數已移到 mapRackManager 中
    // function getRackStatusColor(statusId) { ... }
    // function getRackStatusName(statusId) { ... }

    function getNodeTypeName(nodeTypeId) {
        switch (nodeTypeId) {
            case 2: return '貨架點';
            case 4: return '充電入口點';
            case 6: return '避讓點';
            case 10: return '充電站';
            default: return '普通節點';
        }
    }

    // 載入端口狀態
    async function loadPortStatus(eqpId) {
        const element = document.getElementById(`port-status-${eqpId}`);
        if (!element) return;

        element.innerHTML = `<span class="tag is-light">載入中...</span>`;

        try {
            const response = await fetch(`/api/eqps/${eqpId}/ports`);
            if (!response.ok) throw new Error('API 請求失敗');
            const ports = await response.json();

            if (!Array.isArray(ports) || ports.length === 0) {
                element.innerHTML = `<span class="tag is-light">無端口資料</span>`;
                return;
            }

            // 根據每個 port 的 presence/load/unload 狀態渲染
            element.innerHTML = `
                <div class="tags" style="flex-wrap: wrap;">
                    ${ports.map(port => {
                let tags = [];
                // Presence 狀態
                if (port.presence !== undefined && port.presence !== null) {
                    let presenceClass = port.presence ? 'is-success' : 'is-light';
                    let presenceText = port.presence ? '有載具' : '無載具';
                    tags.push(`<span class="tag ${presenceClass}">${port.name}: ${presenceText}</span>`);
                }
                // Allow Load
                if (port.allow_load !== undefined && port.allow_load !== null) {
                    let loadClass = port.allow_load ? 'is-success' : 'is-danger';
                    let loadText = port.allow_load ? '允許載入' : '禁止載入';
                    tags.push(`<span class="tag ${loadClass}">${loadText}</span>`);
                }
                // Allow Unload
                if (port.allow_unload !== undefined && port.allow_unload !== null) {
                    let unloadClass = port.allow_unload ? 'is-success' : 'is-danger';
                    let unloadText = port.allow_unload ? '允許卸載' : '禁止卸載';
                    tags.push(`<span class="tag ${unloadClass}">${unloadText}</span>`);
                }
                return tags.join(' ');
            }).join('<br>')}
                </div>
            `;
        } catch (err) {
            element.innerHTML = `<span class="tag is-danger">端口狀態載入失敗</span>`;
        }
    }

    // 載具數量載入已移到 mapRackManager 中
    // function loadCarrierCount(rackId) { ... }

    // AGV 相關操作
    function viewAgvTasks(agvId) {
        mapPermissions.executeWithPermission('view_tasks', () => {
            console.debug('Opening tasks page for AGV:', agvId);

            // 記錄查看任務操作
            mapAuditLogger.logView(mapAuditLogger.RESOURCE_TYPES.TASK, null, {
                action: 'view_agv_tasks',
                agvId: agvId,
                source: 'map_popup'
            });

            // 直接打開任務頁面，並通過 URL 參數過濾 AGV
            window.open(`/tasks?agv_id=${agvId}`, '_blank');
        });
    }

    function assignTaskToAgv(agvId) {
        mapPermissions.executeWithPermission('create_task', () => {
            // 記錄分配任務操作
            mapAuditLogger.logAssign(mapAuditLogger.RESOURCE_TYPES.TASK, null, agvId, {
                action: 'assign_task_to_agv',
                agvId: agvId,
                source: 'map_popup'
            });

            window.open(`/tasks/create?agv_id=${agvId}`, '_blank');
        });
    }

    // 設備相關操作
    function viewEquipmentDetails(eqpId) {
        mapPermissions.executeWithPermission('view_equipment', () => {
            // 記錄查看設備詳情操作
            mapAuditLogger.logView(mapAuditLogger.RESOURCE_TYPES.EQUIPMENT, eqpId, {
                action: 'view_equipment_details',
                source: 'map_popup'
            });

            // 可以在彈出視窗顯示詳細資訊
            console.debug('View equipment details:', eqpId);
            // 或者打開設備管理頁面
            window.open(`/devices/${eqpId}`, '_blank');
        });
    }

    function controlEquipment(eqpId) {
        console.debug('Control equipment:', eqpId);
        alert('目前沒有控制設備功能')
        return;
        mapPermissions.executeWithPermission('control_equipment', () => {
            // 記錄控制設備操作
            mapAuditLogger.logControl(mapAuditLogger.RESOURCE_TYPES.EQUIPMENT, eqpId, 'manual_control', {
                action: 'control_equipment',
                source: 'map_popup'
            });

            // 顯示設備控制面板
            console.debug('Control equipment:', eqpId);
            // 可以實作設備控制邏輯
        });
    }

    // 貨架相關操作
    function editRack(rackId) {
        mapPermissions.executeWithPermission('edit_rack', () => {
            // 記錄編輯貨架操作
            mapAuditLogger.logUpdate(mapAuditLogger.RESOURCE_TYPES.RACK, rackId, {
                action: 'edit_rack',
                source: 'map_popup'
            });

            window.open(`/racks/${rackId}/edit`, '_blank');
        });
    }

    // 設置邊線物件互動
    function setupEdgeInteraction(edgeObject) {
        console.debug('Setting up Edge interaction for:', edgeObject.id);
        // 邊線物件的互動設定會在 LineObject 中直接處理
    }

    // 處理邊線點擊事件
    function handleEdgeClick(edgeObj, e) {
        console.debug('Edge clicked:', edgeObj.id);

        // 從 mapStore 獲取完整的邊線資料
        let fullEdgeData = null;
        let fromNodeData = null;
        let toNodeData = null;

        if (window.mapStore) {
            const mapState = window.mapStore.getState();

            // 先在 kukaEdges 中查找
            fullEdgeData = mapState.kukaEdges?.find(e => e.name === edgeObj.id);

            // 如果沒找到，再在 edges 中查找
            if (!fullEdgeData) {
                fullEdgeData = mapState.edges?.find(e => e.name === edgeObj.id);
            }

            // 如果找到邊線資料，獲取起點和終點節點資料
            if (fullEdgeData) {
                // 查找起點節點
                fromNodeData = mapState.kukaNodes?.find(n => n.id === fullEdgeData.from_id) ||
                    mapState.nodes?.find(n => n.id === fullEdgeData.from_id);

                // 查找終點節點
                toNodeData = mapState.kukaNodes?.find(n => n.id === fullEdgeData.to_id) ||
                    mapState.nodes?.find(n => n.id === fullEdgeData.to_id);
            }
        }

        const edgeData = fullEdgeData || edgeObj.getData();
        const title = `邊線: ${edgeData.name || edgeObj.id}`;

        let content = `
            <table class="table is-narrow is-fullwidth popup-table">
                <tbody>
                    <tr>
                        <td class="popup-label">邊線 ID</td>
                        <td><span class="tag is-info">${edgeData.name || edgeObj.id}</span></td>
                    </tr>
        `;

        // 如果有起點節點資料，顯示起點
        if (fromNodeData) {
            content += `
                    <tr>
                        <td class="popup-label">起點</td>
                        <td>${fromNodeData.name || fromNodeData.id}</td>
                    </tr>
            `;
        } else if (edgeData.from_id) {
            content += `
                    <tr>
                        <td class="popup-label">起點 ID</td>
                        <td>${edgeData.from_id}</td>
                    </tr>
            `;
        }

        // 如果有終點節點資料，顯示終點
        if (toNodeData) {
            content += `
                    <tr>
                        <td class="popup-label">終點</td>
                        <td>${toNodeData.name || toNodeData.id}</td>
                    </tr>
            `;
        } else if (edgeData.to_id) {
            content += `
                    <tr>
                        <td class="popup-label">終點 ID</td>
                        <td>${edgeData.to_id}</td>
                    </tr>
            `;
        }

        // 如果有權重，顯示權重
        if (edgeData.weight !== undefined && edgeData.weight !== null) {
            content += `
                    <tr>
                        <td class="popup-label">權重</td>
                        <td>${edgeData.weight}</td>
                    </tr>
            `;
        }

        // 如果有描述，顯示描述
        if (edgeData.description) {
            content += `
                    <tr>
                        <td class="popup-label">描述</td>
                        <td>${edgeData.description}</td>
                    </tr>
            `;
        }

        content += `
                </tbody>
            </table>
        `;

        const actions = [];

        // 可以添加邊線相關的操作按鈕
        // actions.push({
        //     text: '編輯邊線',
        //     icon: 'mdi-pencil',
        //     class: 'is-primary',
        //     onclick: `mapObjectManager.editEdge('${edgeObj.id}')`,
        //     permission: 'edit_edge'
        // });

        mapInteraction.showPopup(edgeObj.latlng, title, content, actions);
    }

    // 節點相關操作
    function createTaskAtNode(nodeId) {
        mapPermissions.executeWithPermission('create_task', () => {
            // 記錄在節點創建任務操作
            mapAuditLogger.logCreate(mapAuditLogger.RESOURCE_TYPES.TASK, null, {
                action: 'create_task_at_node',
                nodeId: nodeId,
                source: 'map_popup'
            });

            window.open(`/tasks/create?node_id=${nodeId}`, '_blank');
        });
    }

    function editNode(nodeId) {
        console.log('editNode called with ID:', nodeId);
        // 呼叫 mapInteraction 的編輯功能
        mapInteraction.editNode(nodeId);
    }

    // ==================== KUKA AGV 移動功能 ====================

    /**
     * 為 KUKA AGV 創建移動任務
     * @param {string} agvId - AGV ID
     */
    function createMoveTaskForKukaAgv(agvId) {
        mapPermissions.executeWithPermission('create_task', () => {
            console.log('Creating KUKA move task for AGV:', agvId);

            // 1. 獲取 AGV 數據
            const agvData = mapAgvManager.getAgvData(agvId);
            if (!agvData) {
                if (window.notify) {
                    window.notify.showErrorMessage(`找不到 AGV ${agvId}`);
                } else {
                    alert(`找不到 AGV ${agvId}`);
                }
                return;
            }

            // 2. 顯示節點選擇提示
            if (window.notify) {
                window.notify.showNotifyMessage('請在地圖上選擇目標 KUKA 節點', 'is-info');
            }

            // 3. 啟用節點選擇模式
            enableKukaNodeSelection(agvData, (selectedNode) => {
                // 4. 顯示確認對話框
                showMoveConfirmDialog(agvData, selectedNode);
            });
        });
    }

    /**
     * 啟用 KUKA 節點選擇模式
     * @param {object} agvData - AGV 數據
     * @param {function} callback - 選擇節點後的回調函數
     */
    function enableKukaNodeSelection(agvData, callback) {
        // 獲取 KUKA 節點列表
        const mapState = window.mapStore?.getState();
        const kukaNodes = mapState?.kukaNodes || [];

        if (kukaNodes.length === 0) {
            if (window.notify) {
                window.notify.showErrorMessage('地圖上沒有可用的 KUKA 節點');
            }
            return;
        }

        console.log(`Found ${kukaNodes.length} KUKA nodes for selection`);

        // 儲存原始的點擊處理器
        const originalHandlers = new Map();
        let isSelectionActive = true;

        // 為每個 KUKA 節點設置臨時點擊處理器
        kukaNodes.forEach(node => {
            const nodeObj = window.kukaNodeObjects?.get(node.id);
            if (nodeObj) {
                // 保存原始處理器
                if (nodeObj.clickHandler) {
                    originalHandlers.set(node.id, nodeObj.clickHandler);
                }

                // 高亮節點（如果支持）
                if (typeof nodeObj.highlight === 'function') {
                    nodeObj.highlight();
                }

                // 設置臨時點擊處理器
                nodeObj.addClickHandler((obj, e) => {
                    if (!isSelectionActive) return;

                    console.log('KUKA node selected:', node.id, node.name);

                    // 取消選擇模式
                    isSelectionActive = false;

                    // 恢復所有節點的原始處理器和樣式
                    restoreNodeHandlers(kukaNodes, originalHandlers);

                    // 執行回調
                    callback(node);

                    // 關閉提示
                    if (window.notify) {
                        window.notify.showNotifyMessage(`已選擇節點: ${node.name || node.id}`, 'is-success');
                    }
                });
            }
        });

        // 添加取消選擇的機制（ESC 鍵）
        const escHandler = (e) => {
            if (e.key === 'Escape' && isSelectionActive) {
                isSelectionActive = false;
                restoreNodeHandlers(kukaNodes, originalHandlers);
                if (window.notify) {
                    window.notify.showNotifyMessage('已取消節點選擇', 'is-warning');
                }
                document.removeEventListener('keydown', escHandler);
            }
        };
        document.addEventListener('keydown', escHandler);
    }

    /**
     * 恢復節點的原始處理器
     * @param {array} nodes - 節點列表
     * @param {Map} originalHandlers - 原始處理器映射
     */
    function restoreNodeHandlers(nodes, originalHandlers) {
        nodes.forEach(node => {
            const nodeObj = window.kukaNodeObjects?.get(node.id);
            if (nodeObj) {
                // 取消高亮（如果支持）
                if (typeof nodeObj.unhighlight === 'function') {
                    nodeObj.unhighlight();
                }

                // 恢復原始處理器
                const originalHandler = originalHandlers.get(node.id);
                if (originalHandler) {
                    nodeObj.clickHandler = originalHandler;
                } else {
                    // 重新設置默認節點互動
                    setupNodeInteraction(nodeObj);
                }
            }
        });
    }

    /**
     * 顯示移動任務確認對話框
     * @param {object} agvData - AGV 數據
     * @param {object} targetNode - 目標節點數據
     */
    function showMoveConfirmDialog(agvData, targetNode) {
        const modal = document.getElementById('kuka-move-confirm-modal');
        if (!modal) {
            console.error('KUKA move confirm modal not found');
            return;
        }

        // 更新對話框內容
        document.getElementById('move-confirm-agv-name').textContent = agvData.name || agvData.id;
        document.getElementById('move-confirm-agv-id').textContent = agvData.id;
        document.getElementById('move-confirm-target-node').textContent = targetNode.name || targetNode.id;
        document.getElementById('move-confirm-target-id').textContent = targetNode.id;

        // 顯示對話框
        modal.classList.add('is-active');

        // 綁定確認按鈕事件
        const confirmBtn = document.getElementById('move-confirm-submit');
        const cancelBtn = document.getElementById('move-confirm-cancel');
        const closeBtn = document.getElementById('move-confirm-close');

        const closeModal = () => {
            modal.classList.remove('is-active');
        };

        const submitTask = () => {
            const taskData = {
                name: `KUKA移動 - AGV${agvData.id} 至節點${targetNode.id}`,
                description: `從當前位置移動到節點 ${targetNode.name || targetNode.id}`,
                work_id: 210001,  // KUKA 移動任務
                agv_id: agvData.id,
                priority: 0,
                parameters: {
                    model: "KUKA400i",
                    nodes: [targetNode.id],  // 僅目標節點
                    agvId: agvData.id  // 🆕 在 parameters 中也添加 agvId，供 simple_kuka_manager 使用
                }
            };

            closeModal();
            submitKukaMoveTask(taskData);
        };

        // 移除舊的事件監聽器並添加新的
        const newConfirmBtn = confirmBtn.cloneNode(true);
        confirmBtn.parentNode.replaceChild(newConfirmBtn, confirmBtn);
        newConfirmBtn.addEventListener('click', submitTask);

        const newCancelBtn = cancelBtn.cloneNode(true);
        cancelBtn.parentNode.replaceChild(newCancelBtn, cancelBtn);
        newCancelBtn.addEventListener('click', closeModal);

        const newCloseBtn = closeBtn.cloneNode(true);
        closeBtn.parentNode.replaceChild(newCloseBtn, closeBtn);
        newCloseBtn.addEventListener('click', closeModal);

        // 記錄操作
        mapAuditLogger.logCreate(mapAuditLogger.RESOURCE_TYPES.TASK, null, {
            action: 'show_kuka_move_dialog',
            agvId: agvData.id,
            targetNodeId: targetNode.id,
            source: 'map_kuka_agv_popup'
        });
    }

    /**
     * 提交 KUKA 移動任務到後端
     * @param {object} taskData - 任務數據
     */
    async function submitKukaMoveTask(taskData) {
        try {
            console.log('Submitting KUKA move task:', taskData);

            const response = await fetch('/api/tasks/create', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(taskData)
            });

            if (response.ok) {
                const result = await response.json();
                console.log('Task created successfully:', result);

                if (window.notify) {
                    window.notify.showNotifyMessage(`任務創建成功: ${result.task_id || result.id}`, 'is-success');
                } else {
                    alert(`任務創建成功: ${result.task_id || result.id}`);
                }

                // 記錄成功操作
                mapAuditLogger.logCreate(mapAuditLogger.RESOURCE_TYPES.TASK, result.task_id || result.id, {
                    action: 'create_kuka_move_task',
                    agvId: taskData.agv_id,
                    targetNodeId: taskData.parameters.nodes[0],
                    source: 'map_kuka_move_function'
                });

                // 刷新任務列表（如果存在）
                if (window.tasksStore && typeof window.tasksStore.refresh === 'function') {
                    window.tasksStore.refresh();
                }
            } else {
                const errorData = await response.json().catch(() => ({ detail: '未知錯誤' }));
                throw new Error(errorData.detail || `HTTP ${response.status}`);
            }
        } catch (error) {
            console.error('Failed to create KUKA move task:', error);
            if (window.notify) {
                window.notify.showErrorMessage(`創建任務失敗: ${error.message}`);
            } else {
                alert(`創建任務失敗: ${error.message}`);
            }
        }
    }

    // 公開方法
    const publicMethods = {
        init,
        setupAgvInteraction,
        setupEquipmentInteraction,
        setupRackInteraction,
        setupNodeInteraction,
        setupEdgeInteraction,

        // AGV 相關操作
        viewAgvTasks,
        assignTaskToAgv,
        createMoveTaskForKukaAgv,  // KUKA AGV 移動功能

        // 設備相關操作
        viewEquipmentDetails,
        controlEquipment,

        // 貨架相關操作
        editRack,

        // 節點相關操作
        createTaskAtNode,
        editNode,

        // 邊線相關操作
        handleEdgeClick
    };

    return publicMethods;
})();

// 全域暴露以供 HTML onclick 使用
window.mapObjectManager = mapObjectManager;
