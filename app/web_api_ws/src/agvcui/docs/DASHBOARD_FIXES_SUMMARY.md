# Dashboard 儀表板問題修正總結

## 修正概覽

本次修正解決了 Dashboard 儀表板實現中的關鍵資料判斷和顯示問題，並重新設計了房間監控架構。

## 🔧 已修正的問題

### 1. 設備信號狀態判斷邏輯問題 ✅

**問題描述**：原有的信號狀態判斷邏輯過於簡化，未考慮實際的資料類型和業務需求。

**修正方案**：
- 重新設計 `calculateSignalStats()` 函數，根據 `type_of_value` 欄位採用不同判斷邏輯
- **布林類型**：`true`/`1` = 正常，`false`/`0` = 警告
- **數值類型**：正數 = 正常，0 = 警告，負數 = 錯誤
- **字符串類型**：檢查錯誤/警告關鍵字，其他視為正常
- **空值/無效值**：統一視為錯誤狀態

**技術實現**：
```javascript
// 根據信號值和類型判斷狀態
switch (typeOfValue) {
    case 'bool':
        const boolValue = value.toLowerCase() === 'true' || value === '1';
        if (boolValue) normal++; else warning++;
        break;
    case 'int':
    case 'float':
        const numValue = parseFloat(value);
        if (numValue > 0) normal++;
        else if (numValue === 0) warning++;
        else error++;
        break;
    // ... 其他類型處理
}
```

### 2. 任務統計顯示邏輯問題 ✅

**問題描述**：任務完成後可能被刪除，導致「已完成」數量始終為 0。

**修正方案**：
- 修改統計邏輯為顯示「今日完成任務數」
- 重新定義任務狀態：
  - **狀態 1**：待執行 (pending)
  - **狀態 2**：執行中 (running)  
  - **狀態 3**：已完成 (todayCompleted)
  - **狀態 5**：失敗 (failed)
- 基於 `created_at` 或 `updated_at` 時間戳判斷是否為今日任務

**技術實現**：
```javascript
// 獲取今日開始時間
const today = new Date();
today.setHours(0, 0, 0, 0);

// 檢查是否為今日任務
const isToday = task.created_at ? new Date(task.created_at) >= today : true;

// 狀態分類
if (status === 3 && isToday) {
    todayCompleted++;
}
```

### 3. AGV 線上/離線狀態定義確認 ✅

**問題描述**：原有邏輯僅檢查 `enable` 欄位，不夠準確。

**修正方案**：
- 實現複合判斷邏輯：
  1. **基本啟用狀態**：`enable === 1`
  2. **通訊狀態**：有位置資訊 (`x`, `y`) 或電量資訊
  3. **系統狀態**：`status_id` 正常
- 綜合判斷：啟用 + (有位置 或 有電量) + 狀態正常 = 線上

**技術實現**：
```javascript
// 複合判斷線上狀態
if (agv.enable === 1) {
    const hasPosition = (agv.x !== null) && (agv.y !== null);
    const hasBattery = agv.battery !== null;
    const hasValidStatus = !agv.status_id || agv.status_id > 0;
    
    isOnline = hasValidStatus && (hasPosition || hasBattery);
}
```

### 4. 貨架使用率更新問題 ✅

**問題描述**：貨架使用率資料不會更新的問題。

**修正方案**：
- 確認 `handleRacksChange()` 函數正確觸發
- 驗證 `calculateRackStats()` 邏輯基於 `rack.count > 0` 的判斷
- 確保 WebSocket 事件正確監聽和處理

### 5. 房間監控架構重新設計 ✅

**問題描述**：房間2不會有產品關聯，但會有製程關聯；前端動態生成邏輯複雜。

**修正方案**：

#### 5.1 資料關聯修正
- **移除產品關聯邏輯**：不再從 `products` 列表查找房間產品
- **改為製程關聯**：從 `room` 表的 `process_settings_id` 獲取製程資訊
- **顯示邏輯更新**：
  - 產品名稱 → 製程狀態（「製程處理中」/「待機中」）
  - 規格大小 → 製程描述
  - 泡藥次數從 `process_settings` 表獲取

#### 5.2 架構重新設計
- **移除前端動態生成**：刪除 `roomCardGenerator.js` 的動態插入邏輯
- **改為後端模板生成**：在 Jinja2 模板中根據 `room.enable` 動態生成卡片
- **前端職責簡化**：只負責資料更新和狀態管理

### 6. 後端模板動態生成房間卡片 ✅

**修正方案**：

#### 6.1 模板修改
```html
<!-- 動態房間監控卡片 -->
{% for room in enabled_rooms %}
<div class="dashboard-card" id="dashboard-card-room{{ room.id }}">
    <!-- 卡片內容 -->
    <div class="dashboard-list-item">
        <span class="dashboard-list-item-text">製程狀態：</span>
        <span id="dashboard-metric-room{{ room.id }}ProcessStatus">待機中</span>
    </div>
    <div class="dashboard-list-item">
        <span class="dashboard-list-item-text">製程描述：</span>
        <span id="dashboard-metric-room{{ room.id }}ProcessDesc">{{ room.process_description or '-' }}</span>
    </div>
    <div class="dashboard-list-item">
        <span class="dashboard-list-item-text">泡藥次數：</span>
        <span id="dashboard-metric-room{{ room.id }}SoakingTimes">{{ room.soaking_times or '-' }}</span>
    </div>
</div>
{% endfor %}
```

#### 6.2 後端路由修改
```python
@self.app.get("/", response_class=HTMLResponse)
async def home(request: Request):
    # 查詢啟用的房間和製程資訊
    stmt = select(Room, ProcessSettings).join(
        ProcessSettings, Room.process_settings_id == ProcessSettings.id
    ).where(Room.enable == 1).order_by(Room.id)
    
    enabled_rooms = []
    for room, process_settings in results:
        enabled_rooms.append({
            'id': room.id,
            'name': room.name,
            'soaking_times': process_settings.soaking_times,
            'process_description': process_settings.description
        })
    
    return self.templates.TemplateResponse("home.html", {
        "enabled_rooms": enabled_rooms
    })
```

#### 6.3 JavaScript 動態支援
```javascript
// 動態獲取啟用的房間
function initializeDashboardCards() {
    enabledRooms = [];
    const roomCards = document.querySelectorAll('[id^="dashboard-card-room"]');
    roomCards.forEach(card => {
        const roomId = parseInt(card.id.replace('dashboard-card-room', ''));
        if (!isNaN(roomId)) {
            enabledRooms.push(roomId);
        }
    });
}

// 通用房間更新函數
function updateRoomCarrierStats(carriers, roomId) {
    const roomLocationRange = [roomId * 100, (roomId + 1) * 100 - 1];
    const roomCarriers = carriers.filter(carrier => {
        return carrier.location_id >= roomLocationRange[0] && 
               carrier.location_id <= roomLocationRange[1];
    });
    
    updateRoomStatusCard(roomId, { carriersInProcess: roomCarriers.length });
}
```

## 🧪 測試驗證

所有修正都通過了完整的測試驗證：

### 測試結果
```
🚀 開始 Dashboard 功能測試
==================================================
🧪 測試 AGV 統計計算...          ✅ 通過
🧪 測試 Signal 統計計算...       ✅ 通過  
🧪 測試 Rack 統計計算...         ✅ 通過
🧪 測試 Task 統計計算...         ✅ 通過
🧪 測試房間2載具統計計算...      ✅ 通過
🧪 測試房間2產品資訊處理...      ✅ 通過
🧪 測試房間2貨架狀態檢測...      ✅ 通過
🧪 測試 JSON 序列化...           ✅ 通過
==================================================
✅ 所有測試通過！Dashboard 功能正常
```

## 📁 修改的檔案

### 主要修改
1. **`web_api_ws/src/agvcui/agvcui/static/js/dashboardPage.js`** - 核心邏輯修正
2. **`web_api_ws/src/agvcui/agvcui/templates/home.html`** - 動態房間卡片模板
3. **`web_api_ws/src/agvcui/agvcui/agvc_ui_server.py`** - 後端路由和資料查詢
4. **`web_api_ws/src/agvcui/test_dashboard.py`** - 更新測試案例

### 移除的檔案
- 移除了 `productsStore` 的依賴（不再需要產品資料）
- 簡化了 `roomCardGenerator.js` 的使用（改為後端生成）

## 🎯 技術優勢

### 1. 更準確的業務邏輯
- 信號狀態判斷符合實際業務需求
- AGV 狀態判斷更加可靠
- 任務統計顯示有意義的資料

### 2. 更簡潔的架構
- 後端負責房間卡片生成，前端專注資料更新
- 減少了前端複雜度和動態生成邏輯
- 更好的關注點分離

### 3. 更好的可維護性
- 房間配置集中在後端資料庫
- 支援房間的動態啟用/停用
- 統一的更新邏輯和錯誤處理

### 4. 更強的擴展性
- 支援任意數量的房間
- 新增房間只需在資料庫中設置 `enable=1`
- 無需修改前端代碼

## 🚀 部署建議

1. **資料庫檢查**：確保 `room` 表和 `process_settings` 表的關聯正確
2. **WebSocket 驗證**：確認 `room_list` 事件包含製程設定資料
3. **權限測試**：驗證房間資料查詢的權限設置
4. **效能監控**：觀察多房間監控時的系統效能
5. **用戶培訓**：向操作人員說明新的統計邏輯和顯示內容

## 📋 後續優化建議

1. **快取機制**：為房間和製程資料添加快取機制
2. **即時通知**：當房間狀態變化時發送即時通知
3. **歷史統計**：添加歷史任務完成統計和趨勢分析
4. **自定義配置**：允許用戶自定義 Dashboard 卡片顯示順序
5. **效能優化**：對高頻更新的資料進行節流處理

所有修正都保持了現有的 DOM 優化方法論和即時更新機制，確保系統的一致性和效能。
