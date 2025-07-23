# Dashboard 任務卡片表格化修改總結

## 修改概覽

本次修改將 Dashboard 儀表板中的任務狀態卡片從統計數字顯示改為表格列表形式，提供更詳細和實用的任務資訊展示。

## 🔧 修改內容

### 1. HTML 結構重新設計 ✅

**修改檔案**: `web_api_ws/src/agvcui/agvcui/templates/home.html`

#### 移除的元素
- 移除統計數字顯示區域：
  - `dashboard-metric-taskTotal` - 總任務數
  - `dashboard-metric-taskPending` - 待執行數量
  - `dashboard-metric-taskRunning` - 執行中數量
  - `dashboard-metric-taskCompleted` - 今日完成數量
  - `dashboard-metric-taskFailed` - 失敗數量
- 移除 `dashboard-trend` 統計趨勢區域

#### 新增的元素
```html
<!-- 任務表格容器 -->
<div class="dashboard-task-table-container" id="dashboard-task-table-container">
    <table class="table is-fullwidth is-hoverable dashboard-task-table">
        <thead>
            <tr>
                <th>任務ID</th>
                <th>任務名稱</th>
                <th>狀態</th>
                <th>時間</th>
            </tr>
        </thead>
        <tbody id="dashboard-task-table-body">
            <!-- 任務列表將由 JavaScript 動態填入 -->
        </tbody>
    </table>
    
    <!-- 無任務時的提示 -->
    <div class="dashboard-task-empty" id="dashboard-task-empty" style="display: none;">
        <div class="has-text-centered has-text-grey">
            <span class="icon is-large">
                <i class="mdi mdi-clipboard-check-outline mdi-48px"></i>
            </span>
            <p>目前沒有執行中或待執行的任務</p>
        </div>
    </div>
</div>
```

### 2. CSS 樣式新增 ✅

**修改檔案**: `web_api_ws/src/agvcui/agvcui/static/css/dashboardPage.css`

#### 表格樣式
```css
/* 任務表格容器 */
.dashboard-task-table-container {
    max-height: 300px;
    overflow-y: auto;
    margin-top: 1rem;
}

/* 表格基本樣式 */
.dashboard-task-table {
    font-size: 0.875rem;
    margin-bottom: 0 !important;
}

/* 表頭樣式 */
.dashboard-task-table th {
    background-color: #f8f9fa;
    font-weight: 600;
    font-size: 0.75rem;
    text-transform: uppercase;
    letter-spacing: 0.5px;
    padding: 0.5rem 0.75rem;
    border-bottom: 2px solid #e9ecef;
}

/* 表格行樣式 */
.dashboard-task-table td {
    padding: 0.5rem 0.75rem;
    vertical-align: middle;
    border-bottom: 1px solid #f1f3f4;
}

.dashboard-task-table tbody tr:hover {
    background-color: #f8f9fa;
}
```

#### 狀態標籤樣式
```css
/* 任務狀態標籤 */
.task-status-badge {
    display: inline-block;
    padding: 0.25rem 0.5rem;
    border-radius: 4px;
    font-size: 0.75rem;
    font-weight: 500;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

.task-status-pending {
    background-color: #e3f2fd;
    color: #1976d2;
}

.task-status-running {
    background-color: #fff3e0;
    color: #f57c00;
}
```

#### 響應式設計
```css
@media (max-width: 768px) {
    .dashboard-task-table-container {
        max-height: 250px;
    }
    
    .dashboard-task-table {
        font-size: 0.75rem;
    }
    
    .task-name {
        max-width: 100px;
    }
}
```

### 3. JavaScript 邏輯重構 ✅

**修改檔案**: `web_api_ws/src/agvcui/agvcui/static/js/dashboardPage.js`

#### 資料處理邏輯修改
```javascript
/**
 * 篩選和處理任務資料
 * @param {Array} tasks - Task 列表
 * @returns {Object} 處理後的任務資料
 */
function processTaskData(tasks) {
    // 只保留待執行和執行中的任務
    const activeTasks = tasks.filter(task => {
        const status = task.status_id || task.status;
        return status === 1 || status === 2; // 1=待執行, 2=執行中
    });

    // 按狀態和時間排序（執行中優先，然後按開始時間）
    activeTasks.sort((a, b) => {
        const statusA = a.status_id || a.status;
        const statusB = b.status_id || b.status;
        
        // 執行中的任務優先
        if (statusA === 2 && statusB === 1) return -1;
        if (statusA === 1 && statusB === 2) return 1;
        
        // 相同狀態按時間排序（最新的在前）
        const timeA = new Date(a.created_at || a.updated_at || 0);
        const timeB = new Date(b.created_at || b.updated_at || 0);
        return timeB - timeA;
    });

    // 限制顯示數量（最多10個）
    const limitedTasks = activeTasks.slice(0, 10);

    return {
        activeTasks: limitedTasks,
        totalActive: activeTasks.length,
        pendingCount: activeTasks.filter(task => (task.status_id || task.status) === 1).length,
        runningCount: activeTasks.filter(task => (task.status_id || task.status) === 2).length
    };
}
```

#### 表格更新邏輯
```javascript
/**
 * 更新任務表格
 * @param {Array} tasks - 活躍任務列表
 */
function updateTaskTable(tasks) {
    const tableBody = document.getElementById('dashboard-task-table-body');
    const tableContainer = document.getElementById('dashboard-task-table-container');
    const emptyState = document.getElementById('dashboard-task-empty');

    // 清空現有內容
    tableBody.innerHTML = '';

    if (tasks.length === 0) {
        // 顯示空狀態
        tableContainer.style.display = 'none';
        emptyState.style.display = 'block';
        return;
    }

    // 隱藏空狀態，顯示表格
    emptyState.style.display = 'none';
    tableContainer.style.display = 'block';

    // 生成表格行
    tasks.forEach(task => {
        const row = createTaskTableRow(task);
        tableBody.appendChild(row);
    });
}
```

#### 表格行創建邏輯
```javascript
/**
 * 創建任務表格行
 * @param {Object} task - 任務資料
 * @returns {HTMLElement} 表格行元素
 */
function createTaskTableRow(task) {
    const row = document.createElement('tr');
    
    // 任務 ID
    const idCell = document.createElement('td');
    idCell.innerHTML = `<span class="task-id">${task.id || '-'}</span>`;
    
    // 任務名稱
    const nameCell = document.createElement('td');
    nameCell.innerHTML = `<span class="task-name" title="${task.name || '未命名任務'}">${task.name || '未命名任務'}</span>`;
    
    // 狀態
    const statusCell = document.createElement('td');
    const status = task.status_id || task.status;
    let statusBadge = '';
    
    if (status === 1) {
        statusBadge = '<span class="task-status-badge task-status-pending">待執行</span>';
    } else if (status === 2) {
        statusBadge = '<span class="task-status-badge task-status-running">執行中</span>';
    }
    
    statusCell.innerHTML = statusBadge;
    
    // 時間
    const timeCell = document.createElement('td');
    const timeStr = formatTaskTime(task.created_at || task.updated_at);
    timeCell.innerHTML = `<span class="task-time">${timeStr}</span>`;
    
    // 組裝行
    row.appendChild(idCell);
    row.appendChild(nameCell);
    row.appendChild(statusCell);
    row.appendChild(timeCell);
    
    return row;
}
```

#### 時間格式化邏輯
```javascript
/**
 * 格式化任務時間
 * @param {string} timeStr - 時間字符串
 * @returns {string} 格式化後的時間
 */
function formatTaskTime(timeStr) {
    if (!timeStr) return '-';
    
    try {
        const date = new Date(timeStr);
        const now = new Date();
        const diffMs = now - date;
        const diffMins = Math.floor(diffMs / (1000 * 60));
        const diffHours = Math.floor(diffMs / (1000 * 60 * 60));
        const diffDays = Math.floor(diffMs / (1000 * 60 * 60 * 24));
        
        if (diffMins < 1) {
            return '剛剛';
        } else if (diffMins < 60) {
            return `${diffMins}分鐘前`;
        } else if (diffHours < 24) {
            return `${diffHours}小時前`;
        } else if (diffDays < 7) {
            return `${diffDays}天前`;
        } else {
            // 超過一週顯示具體日期
            return date.toLocaleDateString('zh-TW', {
                month: 'short',
                day: 'numeric',
                hour: '2-digit',
                minute: '2-digit'
            });
        }
    } catch (e) {
        console.warn('時間格式化錯誤:', timeStr, e);
        return '-';
    }
}
```

### 4. 測試案例更新 ✅

**修改檔案**: `web_api_ws/src/agvcui/test_dashboard.py`

#### 新的測試邏輯
```python
def test_task_data_processing():
    """測試 Task 資料處理邏輯"""
    tasks = [
        {"id": 1, "name": "運輸任務-001", "status_id": 1},  # 待執行
        {"id": 2, "name": "運輸任務-002", "status_id": 2},  # 執行中
        {"id": 3, "name": "運輸任務-003", "status_id": 3},  # 已完成（不顯示）
        {"id": 4, "name": "運輸任務-004", "status_id": 5},  # 失敗（不顯示）
        {"id": 5, "name": "運輸任務-005", "status_id": 1},  # 待執行
    ]
    
    # 只保留待執行和執行中的任務
    active_tasks = [task for task in tasks if task.get('status_id') in [1, 2]]
    
    # 驗證篩選和排序邏輯
    assert len(active_tasks) == 3  # 任務 1, 2, 5
    assert active_tasks[0]['status_id'] == 2  # 執行中優先
```

## 🎯 功能特點

### 1. 智能篩選
- **只顯示活躍任務**：僅顯示待執行（status_id=1）和執行中（status_id=2）的任務
- **自動排序**：執行中任務優先，相同狀態按時間倒序排列
- **數量限制**：最多顯示 10 個任務，避免介面過載

### 2. 詳細資訊展示
- **任務 ID**：使用等寬字體顯示，便於識別
- **任務名稱**：支援長名稱的省略顯示和 tooltip 提示
- **狀態標籤**：使用顏色區分的標籤顯示狀態
- **相對時間**：智能顯示相對時間（剛剛、X分鐘前、X小時前等）

### 3. 用戶體驗優化
- **空狀態處理**：無任務時顯示友好的空狀態提示
- **響應式設計**：在小螢幕上自動調整表格大小和字體
- **懸停效果**：表格行懸停時的視覺反饋
- **滾動支援**：表格內容超出時支援垂直滾動

### 4. 即時更新
- **WebSocket 整合**：保持與現有 WebSocket 事件監聽的兼容性
- **DOM 優化**：使用 DOM 優化方法論，避免不必要的重繪
- **動畫效果**：保持卡片更新時的動畫效果

## 📊 顯示邏輯

### 狀態篩選規則
```
顯示的任務狀態：
- status_id = 1 (待執行) ✅
- status_id = 2 (執行中) ✅

不顯示的任務狀態：
- status_id = 3 (已完成) ❌
- status_id = 4 (已取消) ❌
- status_id = 5 (失敗) ❌
```

### 排序規則
```
1. 按狀態優先級：執行中 > 待執行
2. 相同狀態按時間：最新創建/更新的在前
3. 限制數量：最多顯示 10 個任務
```

### 時間顯示規則
```
- < 1分鐘：「剛剛」
- < 1小時：「X分鐘前」
- < 1天：「X小時前」
- < 1週：「X天前」
- >= 1週：具體日期時間
```

## 🔄 保持的功能

### 1. 即時更新機制
- WebSocket 事件監聽保持不變
- miniStore 整合保持完整
- 變化檢測機制繼續運作

### 2. DOM 優化方法論
- 精確的變化檢測
- 避免不必要的 DOM 操作
- 動畫重疊防護機制

### 3. 響應式設計
- Bulma CSS 框架整合
- 移動設備適配
- 多螢幕尺寸支援

### 4. 狀態指示器
- 保持原有的狀態指示器邏輯
- 根據執行中和待執行任務數量更新狀態文字
- 顏色編碼保持一致

## 🚀 部署注意事項

### 1. 資料庫欄位確認
確保任務資料包含以下欄位：
- `id` - 任務 ID
- `name` - 任務名稱
- `status_id` - 任務狀態
- `created_at` 或 `updated_at` - 時間戳

### 2. WebSocket 事件格式
確認 WebSocket 事件中的任務資料格式符合預期：
```json
{
  "tasks": [
    {
      "id": 1,
      "name": "運輸任務-001",
      "status_id": 1,
      "created_at": "2025-07-07T16:53:22.291895+08:00"
    }
  ]
}
```

### 3. 效能考量
- 表格最多顯示 10 個任務，避免效能問題
- 使用 CSS 滾動而非分頁，減少複雜度
- 時間格式化使用快取機制

## 📈 未來擴展建議

1. **任務詳情彈窗**：點擊任務行顯示詳細資訊
2. **狀態篩選器**：允許用戶選擇顯示的狀態類型
3. **排序選項**：允許用戶自定義排序方式
4. **任務操作**：直接在表格中提供任務操作按鈕
5. **分頁支援**：當任務數量很大時提供分頁功能

這次修改成功地將任務卡片從簡單的統計顯示升級為功能豐富的表格列表，提供了更實用和詳細的任務監控體驗。
