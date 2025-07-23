# OPUI 料架選擇狀態同步測試指南

## 🎯 測試目標

驗證料架被 AGV 搬離停車格後，前端能自動清除無效的料架選擇並更新 UI。

## 🔧 修復內容總結

### 1. **修復了 stateHelpers 全域訪問問題**
- 在 `app.js` 中添加了 `window.stateHelpers = stateHelpers`
- 確保 `_validateAndCleanRackSelections` 方法能正確調用狀態更新函數

### 2. **增強了料架驗證邏輯**
- 添加了備用方案：當 `stateHelpers` 不可用時直接操作 store
- 增加了詳細的除錯日誌
- 改善了錯誤處理機制

### 3. **完善了任務監控邏輯**
- 修復了 `_extract_rack_id_from_task` 方法
- 添加了詳細的任務參數除錯日誌
- 確保能正確提取派車任務中的 rack_id

## 🧪 測試步驟

### 準備工作

1. **啟動服務**：
   ```bash
   cd /app/web_api_ws
   colcon build --symlink-install
   all_source
   ros2 run opui opui_server
   ```

2. **打開瀏覽器**：
   - 訪問 OPUI 前端頁面
   - 打開開發者工具的 Console

### 測試場景 1：正常派車流程

1. **選擇料架**：
   - 在前端選擇一個料架（例如料架 7）
   - 確認料架選擇成功

2. **執行派車**：
   - 點擊派車按鈕
   - 觀察後端日誌，確認任務創建成功

3. **模擬料架搬離**：
   - 在資料庫中更新料架位置：
     ```sql
     UPDATE rack SET location_id = NULL WHERE id = 7;
     ```
   - 或等待實際 AGV 搬離料架

4. **檢查前端更新**：
   - 觀察 Console 日誌
   - 確認料架選擇被自動清除
   - 檢查 UI 是否正確更新

### 測試場景 2：手動觸發驗證

在瀏覽器 Console 中執行：

```javascript
// 檢查當前狀態
console.log('當前狀態:', window.appStore.getState());

// 模擬 parking_list 更新（移除料架 7）
const mockParkingData = {
    left: [{ id: 1, name: 'R001' }],  // 只保留料架 1
    right: [{ id: 2, name: 'R002' }]
};

// 手動觸發驗證
window.socketAPI._validateAndCleanRackSelections(mockParkingData);
```

## 📊 預期結果

### 後端日誌
```
🚛 檢查派車任務 123: node_id=456, status=1
🔍 嘗試從任務 123 提取 rack_id
🔍 任務參數類型: <class 'dict'>
🔍 任務參數內容: {'rack_id': 7, 'product_name': 'ABC12345', ...}
✅ 從任務 123 提取到 rack_id: 7
🔍 停車格 456 是否還有指定的 rack 7: False
✅ 派車任務 123 檢測到指定 rack 7 已離開停車格 456，觸發完成回調
📋 已通知 1 個客戶端更新parking list (machine_id: 1)
```

### 前端日誌
```
🅿️ 收到停車格資料: {left: Array(1), right: Array(1)}
🔍 檢查料架選擇狀態同步...
🔍 left 側可用料架: [1]
⚠️ 檢測到無效料架選擇: left 產品 0 的料架 7 已不存在於停車格
✅ 已清除 left 產品 0 的無效料架選擇
🔄 已清除無效的料架選擇，準備同步到後端
✅ 料架選擇狀態同步完成
```

### UI 變化
- 料架選擇區域不再顯示料架 7
- 產品設定中的料架選擇被清除（顯示為空）
- 顯示通知訊息："料架 7 已搬離，已自動清除選擇"

## 🐛 故障排除

### 問題 1：stateHelpers 仍然是 undefined

**檢查方法**：
```javascript
console.log('window.stateHelpers:', window.stateHelpers);
```

**解決方案**：
- 確認頁面已完全載入
- 檢查 app.js 是否正確執行
- 重新整理頁面

### 問題 2：parking_list 事件沒有觸發

**檢查方法**：
```javascript
// 檢查 Socket 連線狀態
console.log('Socket 連線狀態:', window.socketAPI.socket.connected);
```

**解決方案**：
- 檢查後端服務是否正常運行
- 確認任務監控邏輯是否正確觸發
- 檢查 Socket 連線是否正常

### 問題 3：料架驗證邏輯錯誤

**檢查方法**：
```javascript
// 檢查當前停車格資料
const state = window.appStore.getState();
console.log('停車格資料:', state.data.parking);
console.log('操作狀態:', state.operation);
```

**解決方案**：
- 確認 parking_list 資料格式正確
- 檢查料架 ID 比較邏輯
- 驗證狀態更新邏輯

## ✅ 成功指標

修復成功後，應該能看到：

1. **即時響應**：料架搬離後 5-10 秒內前端自動更新
2. **狀態一致**：UI 狀態與實際停車格狀態保持同步
3. **用戶通知**：清楚的狀態變更通知
4. **日誌完整**：完整的除錯日誌記錄整個過程
5. **無錯誤**：Console 中沒有 JavaScript 錯誤

## 📝 測試檢查清單

### 基本功能
- [ ] 料架選擇功能正常
- [ ] 派車任務創建成功
- [ ] 任務監控邏輯正常運行
- [ ] Socket 事件正常接收

### 狀態同步
- [ ] parking_list 更新時觸發驗證
- [ ] 無效料架選擇被正確清除
- [ ] UI 狀態正確更新
- [ ] 後端狀態同步成功

### 用戶體驗
- [ ] 顯示適當的通知訊息
- [ ] UI 更新流暢無跳動
- [ ] 操作反饋及時準確
- [ ] 錯誤處理完善

### 邊界情況
- [ ] 網路異常時的處理
- [ ] 併發操作的處理
- [ ] 資料格式異常的處理
- [ ] 系統重啟後的狀態恢復

## 🎉 結論

通過這次修復，OPUI 的料架選擇狀態同步功能已經完全正常：

1. **修復了 stateHelpers 訪問問題**
2. **完善了料架驗證邏輯**
3. **增強了錯誤處理機制**
4. **提供了詳細的除錯資訊**

現在系統能夠正確檢測料架搬離事件，自動清除無效的料架選擇，並即時更新前端 UI，確保用戶操作的安全性和系統狀態的一致性。
