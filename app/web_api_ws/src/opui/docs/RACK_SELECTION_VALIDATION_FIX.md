# OPUI 前端料架選擇邏輯修復

## 🐛 問題描述

### 原始問題
目前前端允許用戶選擇任意料架編號，但應該限制只能選擇當前停車格中實際存在的料架。

### 問題分析

#### 1. **缺少料架驗證機制**
```javascript
// ❌ 修復前：沒有驗證料架是否存在於停車格
bindDynamicRackSelection() {
    document.addEventListener('click', (e) => {
        if (e.target.classList.contains('rack-btn')) {
            const rackId = parseInt(e.target.getAttribute('data-rackid'));
            // 直接更新狀態，沒有驗證
            stateHelpers.updateProduct(side, productIndex, { rackId });
        }
    });
}
```

#### 2. **UI 與資料不同步**
- UI 顯示的料架選項來自 `parking_list`
- 但用戶選擇時沒有驗證是否在有效範圍內
- 可能出現選擇了已不存在的料架的情況

#### 3. **狀態一致性問題**
- `parking_list` 更新時，已選擇的料架可能變為無效
- 沒有自動清除無效的料架選擇

## ✅ 修復方案

### 修復原則
1. **驗證料架存在性**：用戶選擇料架時，必須驗證該料架是否存在於當前機台對應側邊的 parking_list 中
2. **UI 限制**：料架選擇界面應該只顯示 parking_list 中可用的料架選項
3. **狀態同步**：確保 parking_list 更新時，料架選擇選項也同步更新

### 修復實施

#### 1. **新增料架驗證方法**

```javascript
/**
 * 驗證料架選擇是否有效
 */
validateRackSelection(rackId, side, currentState) {
    try {
        // 獲取當前停車格的料架列表
        const parkingList = currentState.data.parking || {};
        const availableRacks = parkingList[side] || [];

        // 檢查選擇的料架是否存在於可用列表中
        const isValidRack = availableRacks.some(rack => rack.id === rackId);

        if (!isValidRack) {
            console.warn(`⚠️ 料架驗證失敗: rack ${rackId} 不存在於 ${side} 側停車格`);
            console.log(`可用料架:`, availableRacks.map(r => `${r.id}(${r.name})`).join(', '));
            return false;
        }

        console.log(`✅ 料架驗證通過: rack ${rackId} 存在於 ${side} 側停車格`);
        return true;

    } catch (error) {
        console.error(`❌ 料架驗證過程發生錯誤:`, error);
        return false; // 發生錯誤時拒絕選擇，保守處理
    }
}
```

#### 2. **修復動態料架選擇事件**

**修復前**：
```javascript
bindDynamicRackSelection() {
    document.addEventListener('click', (e) => {
        if (e.target.classList.contains('rack-btn')) {
            const rackId = parseInt(e.target.getAttribute('data-rackid'));
            const side = e.target.getAttribute('data-side');

            // ❌ 沒有驗證，直接更新狀態
            stateHelpers.updateProduct(side, productIndex, { rackId });
        }
    });
}
```

**修復後**：
```javascript
bindDynamicRackSelection() {
    document.addEventListener('click', (e) => {
        if (e.target.classList.contains('rack-btn')) {
            const rackId = parseInt(e.target.getAttribute('data-rackid'));
            const side = e.target.getAttribute('data-side');

            // ✅ 驗證料架是否存在於當前停車格的 parking_list 中
            if (!this.validateRackSelection(rackId, side, currentState)) {
                notify.showErrorMessage(`料架 ${rackId} 不存在於當前停車格，請重新選擇`);
                return;
            }

            // 驗證通過後才更新狀態
            stateHelpers.updateProduct(side, productIndex, { rackId });
        }
    });
}
```

#### 3. **修復靜態料架選擇事件**

```javascript
bindRackSelection() {
    document.querySelectorAll('.rack-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            const rackId = parseInt(btn.getAttribute('data-rackid'));
            const side = btn.getAttribute('data-side');

            // ✅ 添加料架驗證
            if (!this.validateRackSelection(rackId, side, currentState)) {
                notify.showErrorMessage(`料架 ${rackId} 不存在於當前停車格，請重新選擇`);
                return;
            }

            // 驗證通過後更新狀態並同步到後端
            stateHelpers.updateProduct(side, productIndex, { rackId });
            socketAPI.updateClient(updatedState);
        });
    });
}
```

#### 4. **增強 UI 更新邏輯**

**修復前**：
```javascript
updateRackSelection(parking, operation) {
    // 只是單純顯示可用料架，沒有檢查當前選擇是否有效
    racks.forEach(rack => {
        const rackElement = document.createElement('button');
        // ... 創建按鈕
    });
}
```

**修復後**：
```javascript
updateRackSelection(parking, operation) {
    // ✅ 檢查當前選擇的料架是否仍然有效
    if (selectedRackId && !racks.some(rack => rack.id === selectedRackId)) {
        console.warn(`⚠️ 當前選擇的料架 ${selectedRackId} 不在 ${side} 側停車格中，將清除選擇`);
        // 清除無效的料架選擇
        window.stateHelpers.updateProduct(side, productIndex, { rackId: null });
    }

    racks.forEach(rack => {
        // 創建按鈕邏輯
    });

    // ✅ 重新綁定料架選擇事件（確保新生成的按鈕有事件處理）
    if (window.opuiApp && window.opuiApp.eventManager) {
        window.opuiApp.eventManager.bindRackSelection();
    }
}
```

## 📊 修復效果

### 1. **料架選擇安全性**
- ✅ **驗證機制**：用戶只能選擇實際存在於停車格的料架
- ✅ **錯誤提示**：選擇無效料架時顯示明確的錯誤訊息
- ✅ **狀態保護**：防止無效的料架選擇進入系統狀態

### 2. **UI 與資料同步**
- ✅ **即時驗證**：料架選擇時立即驗證有效性
- ✅ **自動清理**：parking_list 更新時自動清除無效選擇
- ✅ **狀態一致**：確保 UI 顯示與實際可用料架保持同步

### 3. **用戶體驗改善**
- ✅ **明確反饋**：無效選擇時提供清楚的錯誤訊息
- ✅ **操作安全**：防止用戶選擇不存在的料架
- ✅ **狀態透明**：顯示可用料架數量和狀態

### 4. **系統穩定性**
- ✅ **錯誤處理**：完善的異常處理機制
- ✅ **日誌記錄**：詳細的驗證過程日誌
- ✅ **容錯機制**：驗證失敗時的保守處理

## 🔍 測試驗證方法

### 1. **基本功能測試**
- [ ] **有效料架選擇**：
  - 選擇 parking_list 中存在的料架
  - 應該成功更新狀態並同步到後端

- [ ] **無效料架選擇**：
  - 嘗試選擇不存在的料架（如果可能）
  - 應該顯示錯誤訊息並拒絕選擇

### 2. **狀態同步測試**
- [ ] **parking_list 更新**：
  - 料架從停車格移除後
  - 如果該料架正被選擇，應該自動清除選擇

- [ ] **機台切換**：
  - 切換到不同機台
  - 料架選擇應該重置並顯示新機台的可用料架

### 3. **邊界情況測試**
- [ ] **空停車格**：
  - 停車格沒有料架時
  - 應該顯示空的料架選擇區域

- [ ] **網路異常**：
  - 料架選擇同步失敗時
  - 應該顯示適當的錯誤訊息

### 4. **日誌檢查**
```bash
# 檢查料架驗證日誌
# 應該看到類似的日誌：
# 🔄 料架選擇: left 產品 0 料架 123
# ✅ 料架驗證通過: rack 123 存在於 left 側停車格
# ✅ 料架選擇同步完成: left 產品 0 料架 123

# 或者驗證失敗的日誌：
# ⚠️ 料架驗證失敗: rack 999 不存在於 left 側停車格
# 可用料架: 123(R001), 124(R002)
```

## 📝 後續建議

### 1. **功能擴展**
- 考慮添加料架搜尋功能
- 支援料架批量操作
- 增加料架狀態顯示（如佔用、空閒等）

### 2. **性能優化**
- 快取料架驗證結果
- 減少重複的 DOM 操作
- 優化事件綁定機制

### 3. **用戶體驗提升**
- 添加料架選擇的視覺反饋
- 改善錯誤訊息的用戶友好性
- 考慮添加料架預覽功能

## 🎉 結論

通過這次修復，OPUI 的料架選擇邏輯已經完全符合安全性要求：

1. **驗證機制完善**：用戶只能選擇實際存在的料架
2. **狀態同步正確**：UI 與資料保持一致
3. **錯誤處理完整**：提供明確的錯誤反饋
4. **系統穩定性提升**：防止無效資料進入系統

現在用戶的料架選擇操作更加安全可靠，確保了系統資料的一致性和正確性。
