// worksPage.js - Work 頁面 JavaScript 模組
// 實作 DOM 優化方法論和 miniStore 整合

import { worksStore } from './worksStore.js';

export const worksPage = (() => {
    let currentWorks = []; // 當前顯示的工作類型列表
    let currentSearch = null; // 當前搜尋條件

    /**
     * 處理 worksStore 變化事件
     * @param {Object} newState - 新的工作類型狀態
     */
    function handleWorksChange(newState) {
        if (!newState?.works) return;

        const allWorks = newState.works || [];
        console.debug('收到工作類型更新:', allWorks.length, '個工作類型');

        // 根據當前搜尋條件更新工作類型列表
        updateWorksDisplay(allWorks);
    }

    /**
     * 更新工作類型顯示
     * @param {Array} allWorks - 所有工作類型數據
     */
    function updateWorksDisplay(allWorks) {
        // 根據當前搜尋條件篩選工作類型
        let filteredWorks = allWorks;
        if (currentSearch) {
            filteredWorks = allWorks.filter(work =>
                work.name.toLowerCase().includes(currentSearch.toLowerCase()) ||
                (work.description && work.description.toLowerCase().includes(currentSearch.toLowerCase()))
            );
        }

        currentWorks = filteredWorks;

        // 更新頁面上的工作類型值
        updateWorkValues(filteredWorks);

        // 更新統計信息
        updateWorkStats(filteredWorks, allWorks);
    }

    /**
     * 更新單個工作類型的顯示值（帶變化檢測和動畫）
     * @param {Array} works - 工作類型數據陣列
     */
    function updateWorkValues(works) {
        works.forEach(work => {
            updateSingleWork(work);
        });
    }

    /**
     * 更新單個工作類型的所有欄位（帶精確變化檢測）
     * @param {Object} newWork - 新的工作類型資料
     */
    function updateSingleWork(newWork) {
        const workId = newWork.id;
        let hasChanges = false;

        // 更新工作類型名稱（帶變化檢測）
        const nameElement = document.getElementById(`work-name-${workId}`);
        if (nameElement) {
            const oldName = nameElement.textContent.trim();
            const newName = newWork.name;

            if (hasChanged(oldName, newName)) {
                nameElement.textContent = newName;
                // 統一動畫目標：應用到 td 元素
                const nameTdElement = nameElement.closest('td');
                addUpdateAnimation(nameTdElement);
                hasChanges = true;
                console.debug(`工作類型 ${workId} 名稱更新: "${oldName}" → "${newName}"`);
            }
        }

        // 更新描述（帶變化檢測）
        const descriptionElement = document.getElementById(`work-description-${workId}`);
        if (descriptionElement) {
            const oldDescription = extractDescriptionFromElement(descriptionElement);
            const newDescription = newWork.description || null;

            if (hasChanged(oldDescription, newDescription)) {
                updateWorkDescriptionTag(descriptionElement, newDescription);
                // 統一動畫目標：應用到 td 元素
                const descriptionTdElement = descriptionElement.closest('td');
                addUpdateAnimation(descriptionTdElement);
                hasChanges = true;
                console.debug(`工作類型 ${workId} 描述更新: "${oldDescription}" → "${newDescription}"`);
            }
        }

        // 更新參數（帶變化檢測）
        const parametersElement = document.getElementById(`work-parameters-${workId}`);
        if (parametersElement) {
            const oldParameters = extractParametersFromElement(parametersElement);
            const newParameters = newWork.parameters;

            if (hasChanged(JSON.stringify(oldParameters), JSON.stringify(newParameters))) {
                updateWorkParametersTag(parametersElement, newParameters);
                // 統一動畫目標：應用到 td 元素
                const parametersTdElement = parametersElement.closest('td');
                addUpdateAnimation(parametersTdElement);
                hasChanges = true;
                console.debug(`工作類型 ${workId} 參數更新`);
            }
        }

        // 更新相關任務數（帶變化檢測）
        const taskCountElement = document.getElementById(`work-task-count-${workId}`);
        if (taskCountElement) {
            const oldCount = extractTaskCountFromTag(taskCountElement);
            const newCount = newWork.tasks ? newWork.tasks.length : 0;

            if (hasChanged(oldCount, newCount)) {
                updateWorkTaskCountTag(taskCountElement, newCount);
                // 統一動畫目標：應用到 td 元素
                const taskCountTdElement = taskCountElement.closest('td');
                addUpdateAnimation(taskCountTdElement);
                hasChanges = true;
                console.debug(`工作類型 ${workId} 任務數更新: ${oldCount} → ${newCount}`);
            }
        }

        // 記錄變化但不添加整行動畫
        if (hasChanges) {
            console.debug(`工作類型 ${newWork.name || workId} 資料已更新`);
        }
    }

    /**
     * 精確的變化檢測函數
     * @param {any} oldValue - 舊值
     * @param {any} newValue - 新值
     * @returns {boolean} 是否有變化
     */
    function hasChanged(oldValue, newValue) {
        return oldValue !== newValue;
    }

    /**
     * 添加更新動畫效果（帶防重疊機制）
     * @param {Element} element - 要添加動畫的元素
     */
    function addUpdateAnimation(element) {
        if (!element) return;

        // 檢查是否已經在播放動畫
        if (element.classList.contains('work-updated')) {
            console.debug('Work 動畫進行中，跳過重複添加');
            return;
        }

        element.classList.add('work-updated');
        setTimeout(() => {
            element.classList.remove('work-updated');
        }, 1000); // 與 CSS 動畫持續時間一致
    }

    /**
     * 從描述元素中提取描述文字
     * @param {Element} element - 描述元素
     * @returns {string|null} 提取的描述
     */
    function extractDescriptionFromElement(element) {
        const text = element.textContent.trim();
        return text === '無描述' ? null : text;
    }

    /**
     * 從參數元素中提取參數對象
     * @param {Element} element - 參數元素
     * @returns {Object|null} 提取的參數對象
     */
    function extractParametersFromElement(element) {
        const preElement = element.querySelector('pre code');
        if (preElement) {
            try {
                return JSON.parse(preElement.textContent);
            } catch (e) {
                return null;
            }
        }
        return null;
    }

    /**
     * 從任務數標籤中提取數量
     * @param {Element} tagElement - 標籤元素
     * @returns {number} 提取的任務數量
     */
    function extractTaskCountFromTag(tagElement) {
        const text = tagElement.textContent.trim();
        const match = text.match(/(\d+)\s*個任務/);
        return match ? parseInt(match[1]) : 0;
    }

    /**
     * 更新工作類型描述標籤
     * @param {Element} element - 描述元素
     * @param {string|null} description - 新的描述
     */
    function updateWorkDescriptionTag(element, description) {
        if (description) {
            element.innerHTML = description;
        } else {
            element.innerHTML = '<span class="has-text-grey-light">無描述</span>';
        }
    }

    /**
     * 更新工作類型參數標籤
     * @param {Element} element - 參數元素
     * @param {Object|null} parameters - 新的參數對象
     */
    function updateWorkParametersTag(element, parameters) {
        if (parameters) {
            // 重新生成參數下拉選單
            const workId = element.id.replace('work-parameters-', '');
            element.innerHTML = `
                <div class="dropdown is-hoverable work-parameters-dropdown">
                    <div class="dropdown-trigger">
                        <button class="button is-small is-info" aria-haspopup="true"
                                aria-controls="dropdown-menu-${workId}">
                            <span class="icon">
                                <i class="mdi mdi-code-json"></i>
                            </span>
                            <span>JSON</span>
                            <span class="icon is-small">
                                <i class="mdi mdi-chevron-down"></i>
                            </span>
                        </button>
                    </div>
                    <div class="dropdown-menu" id="dropdown-menu-${workId}" role="menu">
                        <div class="dropdown-content">
                            <div class="dropdown-item">
                                <pre>${JSON.stringify(parameters, null, 2)}</pre>
                            </div>
                        </div>
                    </div>
                </div>
            `;
        } else {
            element.innerHTML = '<span class="has-text-grey-light">無參數</span>';
        }
    }

    /**
     * 更新工作類型任務數標籤
     * @param {Element} tagElement - 標籤元素
     * @param {number} count - 新的任務數量
     */
    function updateWorkTaskCountTag(tagElement, count) {
        tagElement.textContent = `${count} 個任務`;
    }



    /**
     * 更新統計信息
     * @param {Array} filteredWorks - 篩選後的工作類型
     * @param {Array} allWorks - 所有工作類型
     */
    function updateWorkStats(filteredWorks, allWorks) {
        // 這裡可以添加統計信息更新邏輯
        console.debug(`統計更新: 顯示 ${filteredWorks.length} / ${allWorks.length} 個工作類型`);
    }

    /**
     * 初始化頁面
     */
    function setup() {
        console.log('🔧 初始化 Works 頁面');

        // 獲取當前搜尋條件（從URL參數）
        const urlParams = new URLSearchParams(window.location.search);
        currentSearch = urlParams.get('search');

        // 監聽 worksStore 變化
        worksStore.on('change', handleWorksChange);

        // 初始化時獲取當前狀態並顯示
        const currentState = worksStore.getState();
        if (currentState.works) {
            handleWorksChange(currentState);
        }

        // 使用 is-hoverable 類別，不需要手動綁定事件

        console.log('✅ Works 頁面初始化完成');
    }

    /**
     * 清理資源
     */
    function cleanup() {
        worksStore.off('change', handleWorksChange);
        console.log('🧹 Works 頁面資源已清理');
    }

    // 返回公開的方法
    return {
        setup,
        cleanup
    };
})();
