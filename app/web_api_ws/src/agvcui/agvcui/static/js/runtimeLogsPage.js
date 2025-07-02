/**
 * Runtime Logs 頁面功能
 * 處理日誌篩選、分頁和自動提交功能
 */

function clearFiltersWithoutSubmit() {
    document.getElementById('levelFilter').value = '';
    document.getElementById('nodeNameFilter').value = '';
    document.getElementById('startTimeFilter').value = '';
    document.getElementById('endTimeFilter').value = '';
    document.getElementById('messageFilter').value = '';
}

function clearFilters() {
    clearFiltersWithoutSubmit();
    document.getElementById('filterForm').submit();
}

function formatDateTimeLocal(date) {
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hours = String(date.getHours()).padStart(2, '0');
    const minutes = String(date.getMinutes()).padStart(2, '0');

    return `${year}-${month}-${day}T${hours}:${minutes}`;
}

function setQuickFilter(type) {
    const now = new Date();

    // 清除現有篩選
    clearFiltersWithoutSubmit();

    switch (type) {
        case 'last_hour':
            const oneHourAgo = new Date(now.getTime() - 60 * 60 * 1000);
            document.getElementById('startTimeFilter').value = formatDateTimeLocal(oneHourAgo);
            document.getElementById('endTimeFilter').value = formatDateTimeLocal(now);
            break;

        case 'today':
            const today = new Date();
            today.setHours(0, 0, 0, 0);
            document.getElementById('startTimeFilter').value = formatDateTimeLocal(today);
            document.getElementById('endTimeFilter').value = formatDateTimeLocal(now);
            break;

        case 'errors_only':
            document.getElementById('levelFilter').value = '40'; // ERROR level
            break;
    }

    document.getElementById('filterForm').submit();
}

function buildPaginationUrl(page) {
    const params = new URLSearchParams();
    params.set('page', page);

    const level = document.getElementById('levelFilter').value;
    const nodeName = document.getElementById('nodeNameFilter').value;
    const startTime = document.getElementById('startTimeFilter').value;
    const endTime = document.getElementById('endTimeFilter').value;
    const messageFilter = document.getElementById('messageFilter').value;

    if (level) params.set('level', level);
    if (nodeName) params.set('node_name', nodeName);
    if (startTime) params.set('start_time', startTime);
    if (endTime) params.set('end_time', endTime);
    if (messageFilter) params.set('message_filter', messageFilter);

    return '/runtime_logs?' + params.toString();
}

function setupAutoSubmit() {
    // 為選擇框添加自動提交
    const levelFilter = document.getElementById('levelFilter');
    if (levelFilter) {
        levelFilter.addEventListener('change', function () {
            document.getElementById('filterForm').submit();
        });
    }

    const nodeNameFilter = document.getElementById('nodeNameFilter');
    if (nodeNameFilter) {
        nodeNameFilter.addEventListener('change', function () {
            document.getElementById('filterForm').submit();
        });
    }

    // 為時間輸入添加自動提交（延遲提交避免頻繁觸發）
    let timeoutId;
    function delayedSubmit() {
        clearTimeout(timeoutId);
        timeoutId = setTimeout(() => {
            document.getElementById('filterForm').submit();
        }, 1000);
    }

    const startTimeFilter = document.getElementById('startTimeFilter');
    if (startTimeFilter) {
        startTimeFilter.addEventListener('change', delayedSubmit);
    }

    const endTimeFilter = document.getElementById('endTimeFilter');
    if (endTimeFilter) {
        endTimeFilter.addEventListener('change', delayedSubmit);
    }

    // 為消息篩選添加 Enter 鍵提交
    const messageFilter = document.getElementById('messageFilter');
    if (messageFilter) {
        messageFilter.addEventListener('keypress', function (e) {
            if (e.key === 'Enter') {
                document.getElementById('filterForm').submit();
            }
        });
    }
}

function bindQuickFilterButtons() {
    // 綁定快速篩選按鈕
    document.addEventListener('click', (e) => {
        const button = e.target.closest('[data-quick-filter]');
        if (!button) return;

        e.preventDefault();
        const filterType = button.dataset.quickFilter;
        setQuickFilter(filterType);
    });

    // 綁定清除篩選按鈕
    document.addEventListener('click', (e) => {
        const button = e.target.closest('[data-clear-filters]');
        if (!button) return;

        e.preventDefault();
        clearFilters();
    });
}

export const runtimeLogsPage = (() => {
    function setup() {
        // 設置自動提交功能
        setupAutoSubmit();

        // 綁定快速篩選按鈕
        bindQuickFilterButtons();

        // 將函數掛載到 window 供模板中的按鈕使用（向後兼容）
        if (typeof window !== 'undefined') {
            window.clearFilters = clearFilters;
            window.setQuickFilter = setQuickFilter;
            window.buildPaginationUrl = buildPaginationUrl;
        }
    }

    return {
        setup,
    };
})();
