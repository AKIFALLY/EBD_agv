/**
 * Flows Page JavaScript
 * 實現流程和節點管理頁面的功能
 */

class FlowsPageManager {
    constructor() {
        this.currentView = 'flows';
        this.flowsData = [];
        this.nodesData = {};
        
        this.init();
    }
    
    init() {
        console.log('初始化 Flows Page Manager');
        this.bindEvents();
        this.loadFlowsData();
    }
    
    bindEvents() {
        // 分頁切換事件
        const flowsViewBtn = document.getElementById('flowsViewBtn');
        const nodesViewBtn = document.getElementById('nodesViewBtn');
        
        if (flowsViewBtn) {
            flowsViewBtn.addEventListener('click', () => this.switchView('flows'));
        }
        
        if (nodesViewBtn) {
            nodesViewBtn.addEventListener('click', () => this.switchView('nodes'));
        }
    }
    
    switchView(view) {
        console.log(`切換到 ${view} 視圖`);
        this.currentView = view;
        
        // 更新按鈕樣式
        const flowsViewBtn = document.getElementById('flowsViewBtn');
        const nodesViewBtn = document.getElementById('nodesViewBtn');
        const flowsView = document.getElementById('flowsView');
        const nodesView = document.getElementById('nodesView');
        
        if (view === 'flows') {
            flowsViewBtn.classList.add('is-primary');
            nodesViewBtn.classList.remove('is-primary');
            flowsView.style.display = 'block';
            nodesView.style.display = 'none';
            
            if (this.flowsData.length === 0) {
                this.loadFlowsData();
            }
        } else {
            flowsViewBtn.classList.remove('is-primary');
            nodesViewBtn.classList.add('is-primary');
            flowsView.style.display = 'none';
            nodesView.style.display = 'block';
            
            if (Object.keys(this.nodesData).length === 0) {
                this.loadNodesData();
            }
        }
    }
    
    async loadFlowsData() {
        console.log('載入流程數據');
        this.showFlowsLoading(true);
        
        try {
            const response = await fetch('/api/flows/list');
            const data = await response.json();
            
            if (response.ok && data.flows) {
                this.flowsData = data.flows;
                this.renderFlowsTable();
            } else {
                throw new Error('無法載入流程數據');
            }
        } catch (error) {
            console.error('載入流程數據失敗:', error);
            this.showFlowsError(error.message);
        } finally {
            this.showFlowsLoading(false);
        }
    }
    
    async loadNodesData() {
        console.log('載入節點數據');
        this.showNodesLoading(true);
        
        try {
            const response = await fetch('/api/nodes/list');
            const data = await response.json();
            
            if (response.ok && data.nodes) {
                this.nodesData = data.nodes;
                this.renderNodesTable();
            } else {
                throw new Error('無法載入節點數據');
            }
        } catch (error) {
            console.error('載入節點數據失敗:', error);
            this.showNodesError(error.message);
        } finally {
            this.showNodesLoading(false);
        }
    }
    
    showFlowsLoading(show) {
        const loading = document.getElementById('flows-loading');
        const content = document.getElementById('flows-content');
        const empty = document.getElementById('flows-empty');
        
        if (show) {
            loading.style.display = 'block';
            content.style.display = 'none';
            empty.style.display = 'none';
        } else {
            loading.style.display = 'none';
        }
    }
    
    showNodesLoading(show) {
        const loading = document.getElementById('nodes-loading');
        const content = document.getElementById('nodes-content');
        const empty = document.getElementById('nodes-empty');
        
        if (show) {
            loading.style.display = 'block';
            content.style.display = 'none';
            empty.style.display = 'none';
        } else {
            loading.style.display = 'none';
        }
    }
    
    showFlowsError(message) {
        const content = document.getElementById('flows-content');
        const empty = document.getElementById('flows-empty');
        
        content.style.display = 'none';
        empty.style.display = 'block';
        empty.innerHTML = `
            <span class="icon is-large has-text-danger">
                <i class="mdi mdi-alert-circle mdi-48px"></i>
            </span>
            <p class="has-text-danger">載入失敗: ${message}</p>
        `;
    }
    
    showNodesError(message) {
        const content = document.getElementById('nodes-content');
        const empty = document.getElementById('nodes-empty');
        
        content.style.display = 'none';
        empty.style.display = 'block';
        empty.innerHTML = `
            <span class="icon is-large has-text-danger">
                <i class="mdi mdi-alert-circle mdi-48px"></i>
            </span>
            <p class="has-text-danger">載入失敗: ${message}</p>
        `;
    }
    
    renderFlowsTable() {
        const tableBody = document.getElementById('flows-table-body');
        const content = document.getElementById('flows-content');
        const empty = document.getElementById('flows-empty');
        
        if (!this.flowsData || this.flowsData.length === 0) {
            content.style.display = 'none';
            empty.style.display = 'block';
            return;
        }
        
        // 檢查用戶權限（從模板傳入的全局變量）
        const hasEditPermission = window.currentUser && ['operator', 'admin'].includes(window.currentUser.role);
        const hasDeletePermission = window.currentUser && window.currentUser.role === 'admin';
        
        let html = '';
        this.flowsData.forEach(flow => {
            const statusClass = flow.enabled ? 'is-success' : 'is-danger';
            const statusText = flow.enabled ? '啟用' : '停用';
            const errorClass = flow.error ? 'error-row' : '';
            
            const locationsHtml = flow.applicable_locations && flow.applicable_locations.length > 0
                ? flow.applicable_locations.map(loc => `<span class="tag is-light">${loc}</span>`).join(' ')
                : '<span class="has-text-grey-light">無</span>';
                
            // 操作按鈕
            let actionsHtml = '';
            if (hasEditPermission || hasDeletePermission) {
                actionsHtml = '<td><div class="buttons are-small">';
                
                if (hasEditPermission) {
                    actionsHtml += `
                        <a href="/tafl/editor?flow=${encodeURIComponent(flow.filename.replace('.yaml', ''))}&mode=edit" class="button is-info is-small">
                            <span class="icon">
                                <i class="mdi mdi-pencil"></i>
                            </span>
                            <span>編輯</span>
                        </a>
                    `;
                }
                
                if (hasDeletePermission && !flow.error) {
                    actionsHtml += `
                        <button class="button is-danger is-small" 
                                onclick="confirmDelete('${flow.name.replace(/'/g, "\\'")}', '${flow.filename}')">
                            <span class="icon">
                                <i class="mdi mdi-delete"></i>
                            </span>
                            <span>刪除</span>
                        </button>
                    `;
                }
                
                actionsHtml += '</div></td>';
            }
            
            html += `
                <tr class="${errorClass}">
                    <td class="truncate" title="${flow.name}">${flow.name}</td>
                    <td class="truncate" title="${flow.description}">${flow.description}</td>
                    <td><span class="tag is-info">${flow.priority}</span></td>
                    <td><span class="tag is-primary">${flow.work_id || '無'}</span></td>
                    <td><span class="tag ${statusClass}">${statusText}</span></td>
                    <td><span class="tag is-warning">${flow.trigger_conditions_count}</span></td>
                    <td class="locations-list">${locationsHtml}</td>
                    ${actionsHtml}
                </tr>
            `;
        });
        
        tableBody.innerHTML = html;
        content.style.display = 'block';
        empty.style.display = 'none';
    }
    
    renderNodesTable() {
        const content = document.getElementById('nodes-content');
        const empty = document.getElementById('nodes-empty');
        
        if (!this.nodesData || Object.keys(this.nodesData).length === 0) {
            content.style.display = 'none';
            empty.style.display = 'block';
            return;
        }
        
        // 渲染動作節點
        this.renderNodeTypeTable('action-nodes-table-body', this.nodesData.action_nodes || []);
        
        // 渲染條件節點
        this.renderNodeTypeTable('condition-nodes-table-body', this.nodesData.condition_nodes || []);
        
        // 渲染邏輯節點
        this.renderNodeTypeTable('logic-nodes-table-body', this.nodesData.logic_nodes || []);
        
        content.style.display = 'block';
        empty.style.display = 'none';
    }
    
    renderNodeTypeTable(tableBodyId, nodes) {
        const tableBody = document.getElementById(tableBodyId);
        
        if (!nodes || nodes.length === 0) {
            tableBody.innerHTML = `
                <tr>
                    <td colspan="7" class="has-text-centered has-text-grey">
                        沒有找到節點
                    </td>
                </tr>
            `;
            return;
        }
        
        let html = '';
        nodes.forEach(node => {
            const errorClass = node.error ? 'error-row' : '';
            
            const inputsHtml = node.inputs && node.inputs.length > 0
                ? node.inputs.map(input => `<span class="tag is-input">${input}</span>`).join(' ')
                : '<span class="has-text-grey-light">無</span>';
                
            const outputsHtml = node.outputs && node.outputs.length > 0
                ? node.outputs.map(output => `<span class="tag is-output">${output}</span>`).join(' ')
                : '<span class="has-text-grey-light">無</span>';
            
            const iconHtml = node.icon ? `<span class="node-icon" style="background-color: ${node.color || '#ccc'}">${node.icon}</span>` : '';
            
            html += `
                <tr class="${errorClass}">
                    <td>
                        ${iconHtml}
                        <span class="truncate" title="${node.name}">${node.name}</span>
                    </td>
                    <td class="truncate" title="${node.description}">${node.description}</td>
                    <td><span class="tag is-light">${node.category}</span></td>
                    <td class="has-text-centered">${node.icon || '無'}</td>
                    <td class="io-tags">${inputsHtml}</td>
                    <td class="io-tags">${outputsHtml}</td>
                    <td class="has-text-centered">
                        <span class="tag is-info">${node.parameters ? node.parameters.length : 0}</span>
                    </td>
                </tr>
            `;
        });
        
        tableBody.innerHTML = html;
    }
}

// 當 DOM 載入完成後初始化
document.addEventListener('DOMContentLoaded', function() {
    console.log('DOM 載入完成，初始化 Flows Page');
    new FlowsPageManager();
});