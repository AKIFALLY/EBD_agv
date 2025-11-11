/**
 * Rack Bitmap 管理器
 * 处理货架格位的可视化和操作
 */

// 全局状态
let currentRackId = null;
let currentRackData = null;

/**
 * 初始化页面
 */
document.addEventListener('DOMContentLoaded', () => {
    // 加载所有 rack 的 bitmap 状态
    loadAllRackBitmapStatus();

    // 绑定"管理格位"按钮事件
    document.querySelectorAll('.rack-bitmap-manage-btn').forEach(btn => {
        btn.addEventListener('click', handleManageBitmapClick);
    });

    // 绑定模态框关闭事件
    const modal = document.getElementById('rack-bitmap-modal');
    const closeBtn = document.getElementById('close-bitmap-modal');
    const modalBg = modal.querySelector('.modal-background');

    closeBtn.addEventListener('click', closeModal);
    modalBg.addEventListener('click', closeModal);

    // 绑定批量操作按钮
    document.getElementById('btn-clear-all').addEventListener('click', handleClearAll);
    document.getElementById('btn-enable-all').addEventListener('click', handleEnableAll);
    document.getElementById('btn-disable-all').addEventListener('click', handleDisableAll);
});

/**
 * 加载所有 rack 的 bitmap 状态（表格中显示）
 */
async function loadAllRackBitmapStatus() {
    const rows = document.querySelectorAll('[data-rack-id]');

    for (const row of rows) {
        const rackId = row.dataset.rackId;
        const statusDiv = document.getElementById(`rack-bitmap-status-${rackId}`);

        if (!statusDiv) continue;

        try {
            const response = await fetch(`/api/racks/${rackId}/bitmap-status`);
            const result = await response.json();

            if (result.success) {
                const data = result.data;
                const stats = data.stats;

                // 显示统计信息
                statusDiv.innerHTML = `
                    <span class="tag is-success">
                        <strong>${stats.occupied_count}</strong> / ${data.max_slots}
                    </span>
                    <span class="tag is-light is-small">
                        可用: ${stats.empty_enabled_count}
                    </span>
                `;
            } else {
                statusDiv.innerHTML = '<span class="tag is-danger">載入失敗</span>';
            }
        } catch (error) {
            console.error(`Failed to load bitmap status for rack ${rackId}:`, error);
            statusDiv.innerHTML = '<span class="tag is-danger">錯誤</span>';
        }
    }
}

/**
 * 处理"管理格位"按钮点击
 */
async function handleManageBitmapClick(event) {
    const btn = event.currentTarget;
    const rackId = btn.dataset.rackId;
    const rackName = btn.dataset.rackName;

    currentRackId = rackId;

    // 加载 rack bitmap 数据
    await loadRackBitmapData(rackId);

    // 更新模态框标题
    document.getElementById('modal-rack-title').textContent = `貨架 ${rackName} 格位管理`;

    // 打开模态框
    document.getElementById('rack-bitmap-modal').classList.add('is-active');
}

/**
 * 加载 rack bitmap 数据并渲染
 */
async function loadRackBitmapData(rackId) {
    try {
        const response = await fetch(`/api/racks/${rackId}/bitmap-status`);
        const result = await response.json();

        if (!result.success) {
            alert('載入格位數據失敗');
            return;
        }

        currentRackData = result.data;

        // 更新统计信息
        updateModalStats(currentRackData);

        // 渲染格位网格
        renderSlotGrid(currentRackData);

    } catch (error) {
        console.error('Failed to load rack bitmap data:', error);
        alert('載入格位數據時發生錯誤');
    }
}

/**
 * 更新模态框统计信息
 */
function updateModalStats(data) {
    document.getElementById('modal-occupied-count').textContent = data.stats.occupied_count;
    document.getElementById('modal-enabled-count').textContent = data.stats.enabled_count;
    document.getElementById('modal-empty-enabled-count').textContent = data.stats.empty_enabled_count;
    document.getElementById('modal-total-slots').textContent = data.max_slots;
}

/**
 * 渲染格位网格
 */
function renderSlotGrid(data) {
    const gridA = document.getElementById('rack-grid-a');
    const gridB = document.getElementById('rack-grid-b');

    // 清空现有内容
    gridA.innerHTML = '';
    gridB.innerHTML = '';

    // A 面：格位 1-16
    for (let i = 1; i <= 16; i++) {
        const status = data.bitmap_status[String(i)]; // JSON 键是字符串
        const slot = createSlotButton(i, status);
        gridA.appendChild(slot);
    }

    // B 面：格位 17-32
    for (let i = 17; i <= 32; i++) {
        const status = data.bitmap_status[String(i)]; // JSON 键是字符串
        const slot = createSlotButton(i, status);
        gridB.appendChild(slot);
    }
}

/**
 * 创建格位按钮
 */
function createSlotButton(slotIndex, status) {
    const button = document.createElement('button');
    button.className = 'button is-fullwidth rack-slot';
    button.dataset.slotIndex = slotIndex;

    // 设置样式
    if (!status.enabled) {
        button.classList.add('disabled');
    }

    if (status.occupied) {
        button.classList.add('occupied');
    } else {
        button.classList.add('empty');
    }

    // 显示格位编号和状态
    const statusText = status.occupied ? '✓ 有貨' : '○ 空';
    const enabledText = status.enabled ? '' : '<span class="slot-badge tag is-warning is-light">禁用</span>';

    button.innerHTML = `
        <div>
            <div><strong>格位 ${slotIndex}</strong></div>
            <div class="is-size-7">${statusText}</div>
            ${enabledText}
        </div>
    `;

    // 绑定点击事件（左键切换有货/空）
    button.addEventListener('click', () => handleSlotClick(slotIndex, status));

    // 绑定右键菜单（切换启用/禁用）
    button.addEventListener('contextmenu', (e) => {
        e.preventDefault();
        handleSlotRightClick(slotIndex, status);
    });

    return button;
}

/**
 * 处理格位左键点击（切换有货/空）
 */
async function handleSlotClick(slotIndex, status) {
    if (!currentRackId) return;

    // 如果格位禁用，不允许操作
    if (!status.enabled) {
        alert('此格位已禁用，請先右鍵啟用');
        return;
    }

    try {
        const action = status.occupied ? 'empty' : 'occupy';
        const response = await fetch(`/racks/${currentRackId}/slot/${slotIndex}/${action}`, {
            method: 'POST'
        });

        const result = await response.json();

        if (result.success) {
            // 重新加载数据
            await loadRackBitmapData(currentRackId);

            // 刷新表格中的统计
            refreshTableStatus(currentRackId);
        } else {
            alert('操作失敗: ' + (result.message || '未知錯誤'));
        }
    } catch (error) {
        console.error('Slot operation failed:', error);
        alert('操作時發生錯誤');
    }
}

/**
 * 处理格位右键点击（切换启用/禁用）
 */
async function handleSlotRightClick(slotIndex, status) {
    if (!currentRackId) return;

    const action = status.enabled ? '禁用' : '啟用';
    const confirmed = confirm(`確定要${action}格位 ${slotIndex} 嗎？`);

    if (!confirmed) return;

    try {
        const response = await fetch(`/racks/${currentRackId}/slot/${slotIndex}/toggle-enable`, {
            method: 'POST'
        });

        const result = await response.json();

        if (result.success) {
            // 重新加载数据
            await loadRackBitmapData(currentRackId);

            // 刷新表格中的统计
            refreshTableStatus(currentRackId);
        } else {
            alert('操作失敗: ' + (result.message || '未知錯誤'));
        }
    } catch (error) {
        console.error('Slot toggle failed:', error);
        alert('操作時發生錯誤');
    }
}

/**
 * 处理全部清空
 */
async function handleClearAll() {
    if (!currentRackId) return;

    const confirmed = confirm('確定要清空所有格位嗎？這將設置所有格位為無貨狀態。');
    if (!confirmed) return;

    try {
        const response = await fetch(`/racks/${currentRackId}/slots/clear-all`, {
            method: 'POST'
        });

        const result = await response.json();

        if (result.success) {
            await loadRackBitmapData(currentRackId);
            refreshTableStatus(currentRackId);
        } else {
            alert('操作失敗: ' + (result.message || '未知錯誤'));
        }
    } catch (error) {
        console.error('Clear all failed:', error);
        alert('操作時發生錯誤');
    }
}

/**
 * 处理全部启用
 */
async function handleEnableAll() {
    if (!currentRackId || !currentRackData) return;

    const productSize = currentRackData.product_size || 'S';
    const confirmed = confirm(`確定要啟用所有格位嗎？（產品尺寸: ${productSize}）`);
    if (!confirmed) return;

    try {
        const response = await fetch(`/racks/${currentRackId}/slots/enable-all?product_size=${productSize}`, {
            method: 'POST'
        });

        const result = await response.json();

        if (result.success) {
            await loadRackBitmapData(currentRackId);
            refreshTableStatus(currentRackId);
        } else {
            alert('操作失敗: ' + (result.message || '未知錯誤'));
        }
    } catch (error) {
        console.error('Enable all failed:', error);
        alert('操作時發生錯誤');
    }
}

/**
 * 处理全部禁用
 */
async function handleDisableAll() {
    if (!currentRackId) return;

    const confirmed = confirm('確定要禁用所有格位嗎？這將阻止對所有格位的操作。');
    if (!confirmed) return;

    try {
        const response = await fetch(`/racks/${currentRackId}/slots/disable-all`, {
            method: 'POST'
        });

        const result = await response.json();

        if (result.success) {
            await loadRackBitmapData(currentRackId);
            refreshTableStatus(currentRackId);
        } else {
            alert('操作失敗: ' + (result.message || '未知錯誤'));
        }
    } catch (error) {
        console.error('Disable all failed:', error);
        alert('操作時發生錯誤');
    }
}

/**
 * 刷新表格中的 rack 状态显示
 */
async function refreshTableStatus(rackId) {
    const statusDiv = document.getElementById(`rack-bitmap-status-${rackId}`);
    if (!statusDiv) return;

    try {
        const response = await fetch(`/api/racks/${rackId}/bitmap-status`);
        const result = await response.json();

        if (result.success) {
            const data = result.data;
            const stats = data.stats;

            statusDiv.innerHTML = `
                <span class="tag is-success">
                    <strong>${stats.occupied_count}</strong> / ${data.max_slots}
                </span>
                <span class="tag is-light is-small">
                    可用: ${stats.empty_enabled_count}
                </span>
            `;
        }
    } catch (error) {
        console.error('Failed to refresh table status:', error);
    }
}

/**
 * 关闭模态框
 */
function closeModal() {
    document.getElementById('rack-bitmap-modal').classList.remove('is-active');
    currentRackId = null;
    currentRackData = null;
}
