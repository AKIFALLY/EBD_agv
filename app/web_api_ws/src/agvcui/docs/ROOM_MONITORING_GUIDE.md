# 房間監控卡片擴展指南

## 概述

本指南說明如何在 Dashboard 儀表板中添加新的房間監控卡片。系統已經實現了可擴展的房間監控架構，支援輕鬆添加房間1、3、4、5的監控功能。

## 已實現功能

### 房間2監控卡片（已完成）
- ✅ 載具統計：顯示房間2內處理中的載具數量
- ✅ 生產資訊：產品名稱、規格大小、泡藥次數
- ✅ 物流狀態：入口和出口貨架狀態
- ✅ 即時更新：整合 WebSocket 事件系統
- ✅ 響應式設計：適配不同螢幕尺寸

### 可擴展架構
- ✅ 通用房間卡片生成器 (`roomCardGenerator.js`)
- ✅ 統一的 DOM 優化方法論
- ✅ 預定義的房間配置（房間1-5）
- ✅ 動態卡片插入和移除功能

## 如何添加新房間監控

### 方法一：使用預定義配置（推薦）

房間1、3、4、5的配置已經預定義在 `roomCardGenerator.js` 中，只需要簡單的步驟即可啟用：

#### 1. 在 HTML 模板中添加房間卡片

編輯 `web_api_ws/src/agvcui/agvcui/templates/home.html`：

```html
<!-- 在現有房間2卡片後添加 -->

<!-- 房間1監控卡片 -->
<div class="dashboard-card" id="dashboard-card-room1">
    <!-- 卡片內容會由 roomCardGenerator 自動生成 -->
</div>

<!-- 房間3監控卡片 -->
<div class="dashboard-card" id="dashboard-card-room3">
    <!-- 卡片內容會由 roomCardGenerator 自動生成 -->
</div>
```

或者使用動態插入方式（在 JavaScript 中）：

```javascript
// 在 dashboardPage.js 的 setup() 函數中添加
import { roomCardGenerator } from './roomCardGenerator.js';

// 動態創建房間1和房間3的卡片
roomCardGenerator.insertRoomCard(1);
roomCardGenerator.insertRoomCard(3);

// 或批量創建多個房間
roomCardGenerator.createMultipleRoomCards([1, 3, 4, 5]);
```

#### 2. 在 dashboardPage.js 中添加資料處理

```javascript
// 在 handleCarriersChange 函數中添加
function handleCarriersChange(newState) {
    // ... 現有代碼 ...
    
    // 添加其他房間的載具統計
    updateRoom1CarrierStats(carriers);
    updateRoom3CarrierStats(carriers);
    // ... 其他房間
}

// 添加對應的更新函數
function updateRoom1CarrierStats(carriers) {
    const room1Stats = roomCardGenerator.calculateRoomCarrierStats(carriers, 1);
    updateRoomStatusCard(1, room1Stats);
    checkRoomRackStatus(1);
}

function updateRoom3CarrierStats(carriers) {
    const room3Stats = roomCardGenerator.calculateRoomCarrierStats(carriers, 3);
    updateRoomStatusCard(3, room3Stats);
    checkRoomRackStatus(3);
}
```

#### 3. 實現通用的房間狀態更新函數

```javascript
// 通用房間狀態卡片更新函數
function updateRoomStatusCard(roomId, roomStats) {
    const cardId = `dashboard-card-room${roomId}`;
    if (!hasCardChanged(cardId, roomStats)) return;

    // 更新載具數量
    updateMetricValue(`dashboard-metric-room${roomId}Carriers`, roomStats.carriersInProcess || 0);

    // 更新狀態指示器
    let statusType = 'info';
    let statusText = '待機中';
    
    if (roomStats.carriersInProcess > 0) {
        statusType = 'warning';
        statusText = `處理中 (${roomStats.carriersInProcess} 個載具)`;
    }
    
    updateStatusIndicator(`dashboard-status-room${roomId}`, statusType, statusText);

    // 添加動畫效果
    const card = document.getElementById(cardId);
    if (card) {
        card.dataset.currentData = JSON.stringify(roomStats);
        addUpdateAnimation(card);
    }

    console.debug(`房間${roomId}狀態卡片已更新:`, roomStats);
}

// 通用房間貨架狀態檢查函數
function checkRoomRackStatus(roomId) {
    const racksState = racksStore.getState();
    const racks = racksState?.racks || [];
    
    const rackStatus = roomCardGenerator.checkRoomRackStatus(racks, roomId);
    
    // 更新入口和出口狀態
    const entranceStatus = rackStatus.entranceHasRack ? '有貨架' : '無貨架';
    const exitStatus = rackStatus.exitHasRack ? '有貨架' : '無貨架';
    
    updateMetricValue(`dashboard-metric-room${roomId}Entrance`, entranceStatus);
    updateMetricValue(`dashboard-metric-room${roomId}Exit`, exitStatus);
    
    // 更新CSS類別
    const entranceElement = document.getElementById(`dashboard-metric-room${roomId}Entrance`);
    const exitElement = document.getElementById(`dashboard-metric-room${roomId}Exit`);
    
    if (entranceElement) {
        entranceElement.className = rackStatus.entranceHasRack ? 'status-warning' : 'status-info';
    }
    
    if (exitElement) {
        exitElement.className = rackStatus.exitHasRack ? 'status-success' : 'status-info';
    }
}
```

### 方法二：自定義房間配置

如果需要添加房間6或更多房間，或修改現有配置：

#### 1. 修改房間配置

編輯 `web_api_ws/src/agvcui/agvcui/static/js/roomCardGenerator.js`：

```javascript
const ROOM_CONFIGS = {
    // ... 現有配置 ...
    6: {
        name: '房間6監控',
        icon: 'mdi-home-automation',
        entranceLocationIds: [600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610],
        exitLocationIds: [690, 691, 692, 693, 694, 695, 696, 697, 698, 699]
    }
};
```

#### 2. 按照方法一的步驟添加對應的處理邏輯

## 預定義房間配置

| 房間ID | 名稱 | 圖標 | 入口位置ID範圍 | 出口位置ID範圍 |
|--------|------|------|---------------|---------------|
| 1 | 房間1監控 | `mdi-home-variant-outline` | 100-110 | 190-199 |
| 2 | 房間2監控 | `mdi-home-variant` | 200-210 | 290-299 |
| 3 | 房間3監控 | `mdi-home-city-outline` | 300-310 | 390-399 |
| 4 | 房間4監控 | `mdi-home-city` | 400-410 | 490-499 |
| 5 | 房間5監控 | `mdi-home-modern` | 500-510 | 590-599 |

## 資料流程

```
WebSocket 事件 → Store 更新 → Dashboard 處理函數 → 房間卡片更新
     ↓              ↓              ↓                ↓
carrier_list → carriersStore → updateRoomXCarrierStats → updateRoomStatusCard
rack_list    → racksStore    → checkRoomRackStatus     → 更新入口/出口狀態
product_list → productsStore → updateRoomXProductInfo  → 更新產品資訊
```

## 測試驗證

添加新房間後，建議進行以下測試：

1. **功能測試**：
   ```bash
   cd web_api_ws/src/agvcui
   python3 test_dashboard.py
   ```

2. **視覺測試**：
   - 啟動系統，檢查新房間卡片是否正確顯示
   - 測試響應式佈局在不同螢幕尺寸下的表現
   - 驗證即時資料更新功能

3. **效能測試**：
   - 監控多個房間卡片同時更新時的效能
   - 檢查動畫效果是否流暢

## 注意事項

1. **位置ID配置**：確保每個房間的入口和出口位置ID不重疊
2. **資料一致性**：確保後端資料中的 `room_id` 和 `location_id` 與前端配置一致
3. **效能考量**：避免同時啟用過多房間監控，建議根據實際需求選擇性啟用
4. **錯誤處理**：添加適當的錯誤處理邏輯，處理資料缺失或異常情況

## 故障排除

### 常見問題

1. **房間卡片不顯示**：
   - 檢查 HTML 模板中是否正確添加了卡片容器
   - 確認 JavaScript 中是否調用了 `insertRoomCard()` 函數

2. **資料不更新**：
   - 檢查 Store 監聽器是否正確設置
   - 確認房間ID和位置ID配置是否正確

3. **樣式問題**：
   - 確認 CSS 選擇器 `[id^="dashboard-card-room"]` 能正確匹配新房間
   - 檢查響應式斷點設置

### 調試工具

```javascript
// 在瀏覽器控制台中使用
console.log('可用房間ID:', roomCardGenerator.getAvailableRoomIds());
console.log('房間2配置:', roomCardGenerator.getRoomConfig(2));
console.log('房間1卡片是否存在:', roomCardGenerator.hasRoomCard(1));
```

## 總結

通過使用預建的可擴展架構，添加新房間監控功能變得非常簡單。只需要幾行代碼就可以啟用房間1、3、4、5的監控功能，並且保持與房間2相同的功能完整性和視覺一致性。
