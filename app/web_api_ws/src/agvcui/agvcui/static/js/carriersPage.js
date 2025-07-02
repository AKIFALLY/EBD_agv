export const carriersPage = (() => {

    // 切換區段顯示/隱藏
    function toggleSection(sectionId) {
        const content = document.getElementById(sectionId + '-content');
        const icon = document.getElementById(sectionId + '-icon');

        if (!content || !icon) return;

        const isCollapsed = content.classList.contains('collapsed');

        if (isCollapsed) {
            // 展開
            content.classList.remove('collapsed');
            icon.classList.remove('rotated');
            const chevron = icon.querySelector('i');
            if (chevron) {
                chevron.classList.remove('mdi-chevron-up');
                chevron.classList.add('mdi-chevron-down');
            }
        } else {
            // 收合
            content.classList.add('collapsed');
            icon.classList.add('rotated');
            const chevron = icon.querySelector('i');
            if (chevron) {
                chevron.classList.remove('mdi-chevron-down');
                chevron.classList.add('mdi-chevron-up');
            }
        }
    }

    // 切換檢視模式
    function toggleView(mode) {
        const groupedView = document.getElementById('grouped-view');
        const listView = document.getElementById('list-view');

        if (mode === 'list') {
            // 切換到列表檢視
            groupedView.style.display = 'none';
            listView.style.display = 'block';
        } else {
            // 切換到分組檢視
            groupedView.style.display = 'block';
            listView.style.display = 'none';
        }
    }

    // 格位點擊處理
    function handleSlotClick(event) {
        const slot = event.currentTarget;
        const slotIndex = slot.getAttribute('data-slot');
        const carrierId = slot.getAttribute('data-carrier-id');
        const rackId = slot.getAttribute('data-rack-id');

        if (carrierId) {
            // 有載具的格位 - 跳轉到編輯載具頁面
            showCarrierDetails(carrierId);
        } else {
            // 空格位 - 跳轉到新增載具頁面，並自動選好 rack 和 rack_index
            createNewCarrierInSlot(rackId, slotIndex);
        }
    }

    // 顯示載具詳情
    function showCarrierDetails(carrierId) {
        // 這裡可以實現載具詳情彈窗或跳轉到編輯頁面
        console.log('顯示載具詳情:', carrierId);
        // 暫時跳轉到編輯頁面
        window.location.href = `/carriers/${carrierId}/edit`;
    }

    // 在指定格位新增載具
    function createNewCarrierInSlot(rackId, slotIndex) {
        // 跳轉到新增載具頁面，並帶上 rack_id 和 rack_index 參數
        const url = `/carriers/create?rack_id=${rackId}&rack_index=${slotIndex}`;
        window.location.href = url;
    }

    // 載具狀態更新
    function updateCarrierStatus(carrierId, newStatus) {
        // 實現載具狀態更新邏輯
        console.log('更新載具狀態:', carrierId, newStatus);
    }

    // 綁定事件
    function bindEvents() {
        // 綁定區段切換按鈕
        document.querySelectorAll('[data-toggle-section]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                const sectionId = btn.getAttribute('data-toggle-section');
                toggleSection(sectionId);
            });
        });

        // 綁定檢視模式下拉選單
        const viewModeSelect = document.getElementById('view-mode');
        if (viewModeSelect) {
            viewModeSelect.addEventListener('change', (e) => {
                const selectedMode = e.target.value;
                toggleView(selectedMode);
            });
        }

        // 綁定格位點擊事件
        document.querySelectorAll('.rack-slot').forEach(slot => {
            slot.addEventListener('click', handleSlotClick);
        });

        // 綁定載具卡片點擊事件
        document.querySelectorAll('.carrier-card').forEach(card => {
            card.addEventListener('click', (e) => {
                if (e.target.closest('button') || e.target.closest('a')) {
                    // 如果點擊的是按鈕或連結，不處理卡片點擊
                    return;
                }
                const carrierId = card.getAttribute('data-carrier-id');
                if (carrierId) {
                    showCarrierDetails(carrierId);
                }
            });
        });
    }

    // 初始化頁面狀態
    function initializePageState() {
        // 檢查 URL 參數
        const urlParams = new URLSearchParams(window.location.search);
        const rackId = urlParams.get('rack_id');

        if (rackId) {
            // 如果有 rack_id 參數，只展開對應的貨架區段
            handleRackFilter(rackId);
        } else {
            // 確保所有分組都是展開狀態
            const collapsibleContents = document.querySelectorAll('.collapsible-content');
            collapsibleContents.forEach(content => {
                content.classList.remove('collapsed');
            });

            const toggleIcons = document.querySelectorAll('.toggle-icon');
            toggleIcons.forEach(icon => {
                icon.classList.remove('rotated');
                const chevron = icon.querySelector('i');
                if (chevron) {
                    chevron.classList.remove('mdi-chevron-up');
                    chevron.classList.add('mdi-chevron-down');
                }
            });
        }
    }

    // 處理貨架篩選
    function handleRackFilter(rackId) {
        console.log('篩選貨架:', rackId);

        // 先收合所有區段
        const collapsibleContents = document.querySelectorAll('.collapsible-content');
        collapsibleContents.forEach(content => {
            content.classList.add('collapsed');
        });

        const toggleIcons = document.querySelectorAll('.toggle-icon');
        toggleIcons.forEach(icon => {
            icon.classList.add('rotated');
            const chevron = icon.querySelector('i');
            if (chevron) {
                chevron.classList.remove('mdi-chevron-down');
                chevron.classList.add('mdi-chevron-up');
            }
        });

        // 展開貨架主區段
        const rackCarriersContent = document.getElementById('rack-carriers-content');
        const rackCarriersIcon = document.getElementById('rack-carriers-icon');
        if (rackCarriersContent && rackCarriersIcon) {
            rackCarriersContent.classList.remove('collapsed');
            rackCarriersIcon.classList.remove('rotated');
            const chevron = rackCarriersIcon.querySelector('i');
            if (chevron) {
                chevron.classList.remove('mdi-chevron-up');
                chevron.classList.add('mdi-chevron-down');
            }
        }

        // 展開特定貨架的區段
        const specificRackContent = document.getElementById(`rack-${rackId}-content`);
        const specificRackIcon = document.getElementById(`rack-${rackId}-icon`);
        if (specificRackContent && specificRackIcon) {
            specificRackContent.classList.remove('collapsed');
            specificRackIcon.classList.remove('rotated');
            const chevron = specificRackIcon.querySelector('i');
            if (chevron) {
                chevron.classList.remove('mdi-chevron-up');
                chevron.classList.add('mdi-chevron-down');
            }
        }

        // 滾動到對應的貨架區段
        const rackSection = document.querySelector(`[data-toggle-section="rack-${rackId}"]`);
        if (rackSection) {
            setTimeout(() => {
                rackSection.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }, 100);
        }
    }

    // 初始化
    function setup() {
        console.log('CarriersPage: 開始初始化');
        bindEvents();
        initializePageState();

        // 將函數掛載到 window 供模板調用（如果需要）
        if (typeof window !== 'undefined') {
            window.carriersPage = {
                toggleSection,
                toggleView,
                showCarrierDetails,
                updateCarrierStatus
            };
        }
    }

    return {
        setup,
        toggleSection,
        toggleView,
        showCarrierDetails,
        updateCarrierStatus
    };
})();
