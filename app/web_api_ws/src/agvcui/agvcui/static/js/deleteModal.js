/**
 * 通用刪除確認模態框功能
 * 用於處理各種實體的刪除確認操作
 */

function closeModal(modal) {
    if (modal) {
        modal.classList.remove('is-active');
    }
}

function showModal(modal, form, id, name, entityType) {
    if (!modal || !form) {
        console.error('DeleteModal: 模態框未正確初始化');
        return;
    }

    // 設置實體名稱
    setEntityName(modal, name, entityType);

    // 設置表單 action
    form.action = `/${entityType}/${id}/delete`;

    // 顯示模態框
    modal.classList.add('is-active');

    // 聚焦到取消按鈕（更安全的默認選項）
    const cancelButton = modal.querySelector('button[type="button"]');
    if (cancelButton) {
        cancelButton.focus();
    }
}

function setEntityName(modal, name, entityType) {
    // 嘗試多種可能的元素 ID
    const possibleIds = [
        `delete${capitalize(entityType.slice(0, -1))}Name`, // deleteCarrierName
        `delete${capitalize(entityType)}Name`,              // deleteCarriersName
        'deleteEntityName',                                 // 通用名稱
        'deleteItemName'                                    // 備用名稱
    ];

    let nameElement = null;
    for (const id of possibleIds) {
        nameElement = document.getElementById(id);
        if (nameElement) break;
    }

    if (nameElement) {
        nameElement.textContent = name;
    } else {
        console.warn(`DeleteModal: 找不到名稱顯示元素，嘗試的 ID: ${possibleIds.join(', ')}`);
    }
}

function capitalize(str) {
    return str.charAt(0).toUpperCase() + str.slice(1);
}

function setupCloseEvents(modal) {
    // 關閉按鈕
    const closeButton = modal.querySelector('.delete');
    if (closeButton) {
        closeButton.addEventListener('click', () => closeModal(modal));
    }

    // 取消按鈕
    const cancelButton = modal.querySelector('button[type="button"]');
    if (cancelButton) {
        cancelButton.addEventListener('click', () => closeModal(modal));
    }

    // 點擊背景關閉
    const modalBackground = modal.querySelector('.modal-background');
    if (modalBackground) {
        modalBackground.addEventListener('click', () => closeModal(modal));
    }

    // ESC 鍵關閉
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && modal.classList.contains('is-active')) {
            closeModal(modal);
        }
    });
}

function bindDeleteButtons(modal, form) {
    // 使用事件委託，綁定到 document 上
    document.addEventListener('click', (e) => {
        const button = e.target.closest('[data-delete-action]');
        if (!button) return;

        e.preventDefault();
        
        const action = button.dataset.deleteAction;
        const id = button.dataset.deleteId;
        const name = button.dataset.deleteName;
        
        if (!action || !id || !name) {
            console.warn('DeleteModal: 刪除按鈕缺少必要的 data 屬性', { action, id, name });
            return;
        }

        showModal(modal, form, id, name, action);
    });
}

export const deleteModal = (() => {
    function setup() {
        // 查找刪除模態框元素
        const modal = document.getElementById('deleteModal');
        const form = document.getElementById('deleteForm');
        
        if (!modal || !form) {
            console.warn('DeleteModal: 找不到必要的模態框元素');
            return;
        }

        // 設置關閉按鈕事件
        setupCloseEvents(modal);
        
        // 綁定所有刪除按鈕的點擊事件
        bindDeleteButtons(modal, form);

        // 將實例掛載到 window 供調試使用（可選）
        if (typeof window !== 'undefined') {
            window.deleteModal = { modal, form, closeModal: () => closeModal(modal) };
        }
    }

    return {
        setup,
    };
})();
