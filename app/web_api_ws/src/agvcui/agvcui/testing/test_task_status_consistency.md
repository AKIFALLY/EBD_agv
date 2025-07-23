# 任務狀態一致性測試指南

## 🎯 測試目標
驗證 AGVCUI 任務管理系統中任務狀態更新邏輯的重構是否成功，確保前後端狀態完全一致。

## 📋 測試範圍
- 前後端狀態映射一致性
- Socket.IO 資料驗證機制
- DOM 更新優化效果
- 狀態變更動畫效果
- 錯誤處理和降級機制

## 🧪 測試步驟

### 1. 前後端狀態映射一致性測試

#### 1.1 狀態常數檢查
```javascript
// 在瀏覽器控制台執行
import('./static/js/taskStatus.js').then(({ TASK_STATUS, TASK_STATUS_ALIASES }) => {
    console.log('前端狀態定義:', TASK_STATUS);
    console.log('向後相容別名:', TASK_STATUS_ALIASES);
    
    // 檢查是否包含 READY_TO_EXECUTE
    console.assert(TASK_STATUS.READY_TO_EXECUTE, 'READY_TO_EXECUTE 狀態應該存在');
    console.assert(TASK_STATUS.READY_TO_EXECUTE.id === 2, 'READY_TO_EXECUTE 狀態 ID 應該是 2');
    
    // 檢查向後相容性
    console.assert(TASK_STATUS_ALIASES.READY === TASK_STATUS.READY_TO_EXECUTE, '向後相容別名應該正確');
});
```

#### 1.2 後端狀態對比
```bash
# 檢查後端狀態定義
curl -X GET "http://localhost/api/task-statuses" | jq '.'
```

### 2. Socket.IO 資料驗證測試

#### 2.1 監控 Socket.IO 事件
```javascript
// 在瀏覽器控制台執行
const originalConsoleWarn = console.warn;
const statusWarnings = [];

console.warn = function(...args) {
    if (args[0] && args[0].includes('狀態不一致')) {
        statusWarnings.push(args);
    }
    originalConsoleWarn.apply(console, args);
};

// 等待 Socket.IO 事件
setTimeout(() => {
    console.log('狀態不一致警告:', statusWarnings);
    console.warn = originalConsoleWarn;
}, 10000);
```

#### 2.2 模擬無效狀態資料
```javascript
// 模擬接收無效狀態的任務資料
const mockInvalidTask = {
    id: 999,
    name: '測試任務',
    status_id: 999, // 無效狀態 ID
    agv_id: null
};

// 檢查驗證函數
import('./static/js/taskStatus.js').then(({ validateTaskStatus }) => {
    const validation = validateTaskStatus(999);
    console.log('無效狀態驗證結果:', validation);
    console.assert(!validation.isValid, '應該檢測到無效狀態');
    console.assert(validation.fallbackStatus === 0, '應該降級為 REQUESTING 狀態');
});
```

### 3. DOM 更新優化測試

#### 3.1 檢查 DOM 結構
```javascript
// 檢查任務行是否有正確的屬性
const taskRows = document.querySelectorAll('tr[data-task-id]');
console.log(`找到 ${taskRows.length} 個任務行`);

taskRows.forEach(row => {
    const taskId = row.dataset.taskId;
    const statusCell = row.querySelector('.task-status-cell');
    
    console.assert(statusCell, `任務 ${taskId} 應該有狀態單元格`);
    console.assert(statusCell.dataset.statusId !== undefined, `任務 ${taskId} 狀態單元格應該有 data-status-id 屬性`);
});
```

#### 3.2 測試精確選擇器
```javascript
// 測試精確的任務選擇器
function testTaskSelector(taskId) {
    const taskRow = document.querySelector(`tr[data-task-id="${taskId}"]`);
    const statusCell = taskRow?.querySelector('.task-status-cell');
    
    console.log(`任務 ${taskId}:`, {
        taskRow: !!taskRow,
        statusCell: !!statusCell,
        statusId: statusCell?.dataset.statusId
    });
    
    return taskRow && statusCell;
}

// 測試前幾個任務
const firstTaskId = document.querySelector('tr[data-task-id]')?.dataset.taskId;
if (firstTaskId) {
    testTaskSelector(firstTaskId);
}
```

### 4. 狀態變更動畫測試

#### 4.1 手動觸發狀態變更（表格視圖）
```javascript
// 手動觸發狀態變更動畫
function testStatusAnimation(taskId, newStatusId) {
    const taskRow = document.querySelector(`tr[data-task-id="${taskId}"]`);
    const statusCell = taskRow?.querySelector('.task-status-cell');

    if (statusCell) {
        // 模擬狀態變更
        import('./static/js/tasksPage.js').then(({ tasksPage }) => {
            // 直接調用更新函數（如果暴露的話）
            console.log('觸發狀態變更動畫');
            statusCell.classList.add('status-updating');

            setTimeout(() => {
                statusCell.classList.remove('status-updating');
                statusCell.classList.add('status-updated');

                setTimeout(() => {
                    statusCell.classList.remove('status-updated');
                }, 300);
            }, 100);
        });
    }
}
```

#### 4.2 手動觸發狀態變更（階層視圖）
```javascript
// 測試階層視圖中的狀態變更動畫
function testHierarchyStatusAnimation(taskId, newStatusId) {
    const taskNode = document.querySelector(`.task-node[data-task-id="${taskId}"]`);
    const statusCell = taskNode?.querySelector('.task-status-cell');

    if (statusCell && window.updateTaskStatusCellOptimized) {
        console.log('觸發階層視圖狀態變更動畫');
        window.updateTaskStatusCellOptimized(statusCell, newStatusId, false);
    }
}
```

### 5. 階層視圖增量更新測試

#### 5.1 檢查階層視圖 DOM 結構
```javascript
// 檢查階層視圖節點是否有正確的屬性
function testHierarchyStructure() {
    const hierarchyView = document.getElementById('hierarchyView');
    if (!hierarchyView || hierarchyView.classList.contains('is-hidden')) {
        console.log('階層視圖未顯示，請先切換到階層視圖');
        return false;
    }

    const taskNodes = hierarchyView.querySelectorAll('.task-node[data-task-id]');
    console.log(`找到 ${taskNodes.length} 個階層任務節點`);

    taskNodes.forEach(node => {
        const taskId = node.dataset.taskId;
        const statusCell = node.querySelector('.task-status-cell');
        const statusTag = node.querySelector('.task-status-tag');

        console.assert(statusCell, `階層任務 ${taskId} 應該有狀態單元格`);
        console.assert(statusTag, `階層任務 ${taskId} 應該有狀態標籤`);
        console.assert(statusCell.dataset.statusId !== undefined, `階層任務 ${taskId} 狀態單元格應該有 data-status-id 屬性`);
    });

    return taskNodes.length > 0;
}
```

#### 5.2 測試增量更新效能
```javascript
// 測試階層視圖增量更新效能
function testHierarchyUpdatePerformance() {
    if (!window.hierarchyPerformanceMetrics) {
        console.error('階層效能監控不可用');
        return;
    }

    const metrics = window.hierarchyPerformanceMetrics.getMetrics();
    console.log('階層視圖效能指標:', metrics);

    // 觸發更新並測量時間
    const startTime = performance.now();

    if (window.generateHierarchy) {
        window.generateHierarchy();

        setTimeout(() => {
            const endTime = performance.now();
            const duration = endTime - startTime;

            console.log(`手動觸發更新耗時: ${duration.toFixed(2)}ms`);
            console.assert(duration < 100, `更新時間應小於 100ms，實際: ${duration.toFixed(2)}ms`);
        }, 100);
    }
}
```

#### 5.3 測試節點映射一致性
```javascript
// 測試階層視圖節點映射一致性
function testHierarchyNodeMapping() {
    if (!window.validateHierarchyConsistency) {
        console.error('階層一致性驗證不可用');
        return;
    }

    const isConsistent = window.validateHierarchyConsistency();
    console.log('階層一致性檢查結果:', isConsistent ? '通過' : '失敗');

    return isConsistent;
}
```

#### 5.4 測試防抖機制
```javascript
// 測試階層視圖防抖更新機制
function testHierarchyDebounce() {
    if (!window.generateHierarchyDebounced) {
        console.error('防抖更新函數不可用');
        return;
    }

    console.log('測試防抖機制：快速觸發多次更新');

    // 快速觸發多次更新
    for (let i = 0; i < 5; i++) {
        setTimeout(() => {
            console.log(`觸發更新 ${i + 1}`);
            window.generateHierarchyDebounced(50);
        }, i * 10);
    }

    // 檢查實際執行次數
    setTimeout(() => {
        const metrics = window.hierarchyPerformanceMetrics?.getMetrics();
        console.log('防抖測試完成，檢查效能指標:', metrics);
    }, 1000);
}
```

#### 4.2 檢查 CSS 動畫樣式
```javascript
// 檢查動畫樣式是否正確載入
const animationStyles = Array.from(document.styleSheets)
    .flatMap(sheet => {
        try {
            return Array.from(sheet.cssRules);
        } catch (e) {
            return [];
        }
    })
    .filter(rule => rule.selectorText && rule.selectorText.includes('status-updating'));

console.log('找到的動畫樣式:', animationStyles.length);
animationStyles.forEach(rule => {
    console.log(rule.selectorText, rule.style.cssText);
});
```

## ✅ 驗收標準

### 功能驗收
- [ ] 前端 `TASK_STATUS.READY_TO_EXECUTE` 與後端 `TaskStatus.READY_TO_EXECUTE` 一致
- [ ] Socket.IO 接收無效狀態時正確降級處理
- [ ] DOM 更新只影響目標任務，不誤更新其他元素
- [ ] 狀態變更時顯示平滑的動畫效果

### 階層視圖增量更新驗收
- [ ] 階層視圖只更新變更的任務節點，不重建整個結構
- [ ] 新增任務顯示淡入動畫
- [ ] 刪除任務顯示淡出動畫
- [ ] 狀態變更動畫與表格視圖保持一致
- [ ] 節點映射一致性檢查通過

### 效能驗收
- [ ] 表格視圖 DOM 更新操作在 100ms 內完成
- [ ] 階層視圖增量更新操作在 50ms 內完成
- [ ] 防抖機制有效減少重複更新
- [ ] 動畫效果不影響頁面滾動和互動
- [ ] 記憶體使用無明顯增長
- [ ] DOM 操作次數減少 80% 以上（相比完全重建）

### 錯誤處理驗收
- [ ] 無效狀態 ID 被正確檢測和記錄
- [ ] 降級機制正常工作（表格視圖和階層視圖）
- [ ] 控制台有清晰的錯誤訊息
- [ ] 階層一致性驗證能檢測並報告問題

## 🐛 常見問題排除

### 問題 1: 狀態不一致警告
**症狀**: 控制台出現「任務 X 狀態不一致」警告
**解決**: 檢查後端是否返回了無效的狀態 ID，確認資料庫 task_status 表的完整性

### 問題 2: 動畫效果不顯示
**症狀**: 狀態變更時沒有動畫效果
**解決**: 檢查 updateAnimations.css 是否正確載入，確認瀏覽器支援 CSS 動畫

### 問題 3: DOM 選擇器失效
**症狀**: 任務狀態無法更新
**解決**: 檢查 HTML 模板中的 data-task-id 屬性是否正確設置

## 📊 測試報告模板

```
測試日期: ____
測試環境: ____
瀏覽器版本: ____

功能測試結果:
- 狀態映射一致性: ✅/❌
- Socket.IO 驗證: ✅/❌  
- DOM 更新優化: ✅/❌
- 動畫效果: ✅/❌

效能測試結果:
- DOM 更新時間: ___ms
- 動畫流暢度: ✅/❌
- 記憶體使用: ___MB

發現問題:
1. ____
2. ____

建議改善:
1. ____
2. ____
```
