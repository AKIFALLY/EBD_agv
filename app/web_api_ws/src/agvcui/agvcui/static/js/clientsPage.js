import { notify } from './notify.js';

export const clientsPage = (() => {

    // 切換客戶端詳情顯示
    function toggleClientDetails(clientId) {
        const detailsRow = document.getElementById(`details-${clientId}`);
        const toggleBtn = document.querySelector(`[data-toggle-client="${clientId}"]`);
        const icon = toggleBtn.querySelector('i');

        if (detailsRow.style.display === 'none' || !detailsRow.style.display) {
            detailsRow.style.display = 'table-row';
            icon.className = 'mdi mdi-chevron-up';
            toggleBtn.querySelector('span:last-child').textContent = '收起詳情';
            loadClientDetails(clientId);
        } else {
            detailsRow.style.display = 'none';
            icon.className = 'mdi mdi-chevron-down';
            toggleBtn.querySelector('span:last-child').textContent = '查看詳情';
        }
    }

    // 載入客戶端詳細資訊
    async function loadClientDetails(clientId) {
        try {
            const response = await fetch(`/clients/${clientId}/details`);
            if (!response.ok) {
                throw new Error('載入詳細資訊失敗');
            }

            const data = await response.json();
            renderClientDetails(clientId, data);
        } catch (error) {
            console.error('載入客戶端詳情失敗:', error);
            notify.showErrorMessage('載入詳細資訊失敗');
        }
    }

    // 渲染客戶端詳細資訊
    function renderClientDetails(clientId, data) {
        const detailsContainer = document.getElementById(`details-content-${clientId}`);
        if (!detailsContainer) return;

        const op = data.op || {};
        const leftOp = op.left || { productSelected: 0, product: [{}, {}] };
        const rightOp = op.right || { productSelected: 0, product: [{}, {}] };

        detailsContainer.innerHTML = `
            <div class="columns">
                <div class="column">
                    <h6 class="subtitle is-6 has-text-weight-bold">
                        <span class="icon"><i class="mdi mdi-account"></i></span>
                        OP1 (左側操作員)
                    </h6>
                    <div class="box">
                        <div class="field">
                            <label class="label is-small">當前選中產品</label>
                            <div class="control">
                                <span class="tag is-info">產品 ${leftOp.productSelected + 1}</span>
                            </div>
                        </div>
                        ${renderProductDetails('OP1', leftOp.product, leftOp.productSelected)}
                    </div>
                </div>
                <div class="column">
                    <h6 class="subtitle is-6 has-text-weight-bold">
                        <span class="icon"><i class="mdi mdi-account"></i></span>
                        OP2 (右側操作員)
                    </h6>
                    <div class="box">
                        <div class="field">
                            <label class="label is-small">當前選中產品</label>
                            <div class="control">
                                <span class="tag is-info">產品 ${rightOp.productSelected + 1}</span>
                            </div>
                        </div>
                        ${renderProductDetails('OP2', rightOp.product, rightOp.productSelected)}
                    </div>
                </div>
            </div>
            <div class="columns">
                <div class="column">
                    <div class="field is-grouped">
                        <div class="control">
                            <button class="button is-warning is-small" data-reset-client="${clientId}">
                                <span class="icon"><i class="mdi mdi-refresh"></i></span>
                                <span>重置設定</span>
                            </button>
                        </div>
                        <div class="control">
                            <button class="button is-info is-small" data-edit-client="${clientId}">
                                <span class="icon"><i class="mdi mdi-pencil"></i></span>
                                <span>編輯基本資訊</span>
                            </button>
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    // 渲染產品詳細資訊
    function renderProductDetails(opName, products, selectedIndex) {
        if (!Array.isArray(products)) return '<p class="has-text-grey">無產品資訊</p>';

        return products.map((product, index) => {
            const isSelected = index === selectedIndex;
            const cardClass = isSelected ? 'card has-background-primary-light' : 'card';

            return `
                <div class="${cardClass} mb-2">
                    <div class="card-content is-small">
                        <div class="media">
                            <div class="media-left">
                                <span class="tag ${isSelected ? 'is-primary' : 'is-light'}">
                                    產品 ${index + 1} ${isSelected ? '(當前)' : ''}
                                </span>
                            </div>
                        </div>
                        <div class="content is-small">
                            <div class="columns is-mobile">
                                <div class="column">
                                    <strong>產品名稱:</strong> ${product.name || '未設定'}
                                </div>
                                <div class="column">
                                    <strong>尺寸:</strong> 
                                    <span class="tag ${product.size === 'S' ? 'is-success' : 'is-warning'} is-small">
                                        ${product.size || 'S'}
                                    </span>
                                </div>
                            </div>
                            <div class="columns is-mobile">
                                <div class="column">
                                    <strong>數量:</strong> 
                                    <span class="tag is-info is-small">${product.count || 32}</span>
                                </div>
                                <div class="column">
                                    <strong>房號:</strong> 
                                    <span class="tag is-link is-small">${product.room || 2}</span>
                                </div>
                            </div>
                            <div class="columns is-mobile">
                                <div class="column">
                                    <strong>料架ID:</strong> 
                                    ${product.rackId ?
                    `<span class="tag is-primary is-small">${product.rackId}</span>` :
                    '<span class="tag is-light is-small">未設定</span>'
                }
                                </div>
                                <div class="column">
                                    <strong>產品ID:</strong> 
                                    ${product.id ?
                    `<span class="tag is-dark is-small">${product.id}</span>` :
                    '<span class="tag is-light is-small">未設定</span>'
                }
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            `;
        }).join('');
    }

    // 重置客戶端設定
    async function resetClientSettings(clientId) {
        if (!confirm('確定要重置此客戶端的所有 OP 設定嗎？此操作無法撤銷！')) {
            return;
        }

        try {
            const response = await fetch(`/clients/${clientId}/reset`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            });

            if (!response.ok) {
                throw new Error('重置設定失敗');
            }

            notify.showErrorMessage('客戶端設定已重置', 'is-success');
            // 重新載入詳細資訊
            loadClientDetails(clientId);
        } catch (error) {
            console.error('重置客戶端設定失敗:', error);
            notify.showErrorMessage('重置設定失敗');
        }
    }

    // 編輯客戶端基本資訊
    function editClient(clientId) {
        window.location.href = `/clients/${clientId}/edit`;
    }

    // 格式化時間差顯示
    function formatTimeDifference(updatedAt) {
        if (!updatedAt) return '';

        const now = new Date();
        const updated = new Date(updatedAt);
        const diffSeconds = Math.floor((now - updated) / 1000);

        if (diffSeconds < 300) { // 5分鐘內
            return '<span class="tag is-success is-small">5分鐘內</span>';
        } else if (diffSeconds < 1800) { // 30分鐘內
            return '<span class="tag is-warning is-small">30分鐘內</span>';
        } else {
            return '<span class="tag is-danger is-small">超過30分鐘</span>';
        }
    }

    // 綁定事件
    function bindEvents() {
        // 綁定詳情切換按鈕
        document.querySelectorAll('[data-toggle-client]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                const clientId = btn.getAttribute('data-toggle-client');
                toggleClientDetails(clientId);
            });
        });

        // 使用事件委託處理動態生成的按鈕
        document.addEventListener('click', (e) => {
            // 重置客戶端設定按鈕
            if (e.target.closest('[data-reset-client]')) {
                e.preventDefault();
                const clientId = e.target.closest('[data-reset-client]').getAttribute('data-reset-client');
                resetClientSettings(clientId);
            }

            // 編輯客戶端按鈕
            if (e.target.closest('[data-edit-client]')) {
                e.preventDefault();
                const clientId = e.target.closest('[data-edit-client]').getAttribute('data-edit-client');
                editClient(clientId);
            }
        });
    }

    // 初始化
    function setup() {
        console.log('ClientsPage: 開始初始化');
        bindEvents();

        // 將函數掛載到 window 供模板調用
        if (typeof window !== 'undefined') {
            window.clientsPage = {
                toggleClientDetails,
                resetClientSettings,
                editClient
            };
        }
    }

    return {
        setup,
        toggleClientDetails,
        resetClientSettings,
        editClient
    };
})();
