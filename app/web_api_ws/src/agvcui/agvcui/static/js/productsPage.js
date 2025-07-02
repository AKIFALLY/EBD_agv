/**
 * Products 頁面功能
 * 處理產品搜尋、篩選和排序功能
 */

function clearFilters() {
    // 清除所有篩選條件
    document.querySelector('input[name="search"]').value = '';
    document.querySelector('select[name="size_filter"]').value = '';
    document.querySelector('select[name="sort_by"]').value = 'id';

    // 提交表單
    document.getElementById('filterForm').submit();
}

function setupAutoSubmit() {
    console.log('ProductsPage: 設置自動提交功能');

    // 為選擇框添加自動提交
    const sizeFilter = document.querySelector('select[name="size_filter"]');
    if (sizeFilter) {
        console.log('ProductsPage: 找到尺寸篩選器，添加事件監聽器');
        sizeFilter.addEventListener('change', function () {
            console.log('ProductsPage: 尺寸篩選器變更，提交表單');
            document.getElementById('filterForm').submit();
        });
    } else {
        console.warn('ProductsPage: 找不到尺寸篩選器');
    }

    const sortBy = document.querySelector('select[name="sort_by"]');
    if (sortBy) {
        console.log('ProductsPage: 找到排序選擇器，添加事件監聽器');
        sortBy.addEventListener('change', function () {
            console.log('ProductsPage: 排序選擇器變更，提交表單');
            document.getElementById('filterForm').submit();
        });
    } else {
        console.warn('ProductsPage: 找不到排序選擇器');
    }

    // 為搜尋輸入框添加 Enter 鍵提交
    const searchInput = document.querySelector('input[name="search"]');
    if (searchInput) {
        searchInput.addEventListener('keypress', function (e) {
            if (e.key === 'Enter') {
                document.getElementById('filterForm').submit();
            }
        });
    }
}

function bindClearButton() {
    // 綁定清除篩選按鈕
    document.addEventListener('click', (e) => {
        const button = e.target.closest('[data-clear-filters]');
        if (!button) return;

        e.preventDefault();
        clearFilters();
    });
}

function setupTableSorting() {
    // 為表格標題添加排序功能
    const headers = document.querySelectorAll('th');
    headers.forEach(header => {
        const text = header.textContent.trim();
        let sortField = '';

        switch (text) {
            case 'ID':
                sortField = 'id';
                break;
            case '產品名稱':
                sortField = 'name';
                break;
            case '創建時間':
                sortField = 'created_at';
                break;
            case '更新時間':
                sortField = 'updated_at';
                break;
        }

        if (sortField) {
            header.style.cursor = 'pointer';
            header.title = `點擊按 ${text} 排序`;

            header.addEventListener('click', () => {
                const currentSort = document.querySelector('select[name="sort_by"]').value;
                if (currentSort !== sortField) {
                    document.querySelector('select[name="sort_by"]').value = sortField;
                    document.getElementById('filterForm').submit();
                }
            });
        }
    });
}

function highlightSearchTerms() {
    // 高亮搜尋關鍵字
    const searchTerm = document.querySelector('input[name="search"]').value.trim();
    if (!searchTerm) return;

    const productNames = document.querySelectorAll('td strong');
    productNames.forEach(nameElement => {
        const text = nameElement.textContent;
        if (text.toLowerCase().includes(searchTerm.toLowerCase())) {
            const regex = new RegExp(`(${searchTerm})`, 'gi');
            nameElement.innerHTML = text.replace(regex, '<mark>$1</mark>');
        }
    });
}

function addRowHoverEffects() {
    // 添加表格行的 hover 效果
    const rows = document.querySelectorAll('tbody tr');
    rows.forEach(row => {
        row.addEventListener('mouseenter', function () {
            this.style.backgroundColor = '#f5f5f5';
        });

        row.addEventListener('mouseleave', function () {
            this.style.backgroundColor = '';
        });
    });
}

function updateResultsInfo() {
    // 更新搜尋結果信息
    const searchTerm = document.querySelector('input[name="search"]').value.trim();
    const sizeFilter = document.querySelector('select[name="size_filter"]').value;

    if (searchTerm || sizeFilter) {
        const infoElement = document.querySelector('.has-text-grey.is-size-7');
        if (infoElement) {
            let filterInfo = '';
            if (searchTerm) {
                filterInfo += `搜尋: "${searchTerm}"`;
            }
            if (sizeFilter) {
                if (filterInfo) filterInfo += ', ';
                filterInfo += `尺寸: ${sizeFilter}`;
            }

            const originalText = infoElement.textContent;
            infoElement.innerHTML = `${originalText} <span class="tag is-info is-small">${filterInfo}</span>`;
        }
    }
}

export const productsPage = (() => {
    function setup() {
        console.log('ProductsPage: 開始初始化');

        // 設置自動提交功能
        setupAutoSubmit();

        // 綁定清除按鈕
        bindClearButton();

        // 設置表格排序
        setupTableSorting();

        // 高亮搜尋關鍵字
        highlightSearchTerms();

        // 添加行 hover 效果
        addRowHoverEffects();

        // 更新結果信息
        updateResultsInfo();

        // 將函數掛載到 window 供調試使用（可選）
        if (typeof window !== 'undefined') {
            window.clearFilters = clearFilters;
        }
    }

    return {
        setup,
    };
})();
