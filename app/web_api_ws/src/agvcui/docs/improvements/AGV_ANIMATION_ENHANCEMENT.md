# AGV 動畫系統增強

## 🎯 **改進概述**

針對 `RotatingMovingObject.js` 進行了重大改進，解決了地圖縮放影響動畫的問題，並新增了靈活的動畫模式控制。最新版本新增了**多點平滑模式**，可以處理多個連續的位置更新，提供最平滑的動畫效果。

**重要修正**：多點模式只對位置使用多點平滑，角度仍使用傳統插值，避免角度快速正反轉的問題。

## 🔧 **主要修改**

### **1. RotatingMovingObject.js 改進**
- **移除速度限制**：移除 `speedRotate` 和 `speedMove` 屬性
- **解決縮放問題**：改用地理座標系統計算，不受地圖縮放影響
- **新增動畫模式**：支援 `instant` 和 `smooth` 兩種模式
- **新增 `setAnimationMode()` 方法**：動態控制動畫行為

### **2. mapPage.js 整合**
- **新增配置區域**：`AGV_ANIMATION_CONFIG` 統一管理動畫設定
- **自動應用模式**：新創建的 AGV 自動應用配置的動畫模式
- **全域控制函數**：`setAllAgvAnimationMode()` 批量更新所有 AGV

## 🎮 **使用方式**

### **配置設定**
```javascript
const AGV_ANIMATION_CONFIG = {
    mode: 'queue',        // 'instant', 'smooth', 'ultra-smooth', 'queue'
    speed: 8.0,           // 動畫速度係數
    queueSize: 5,         // 佇列大小 (僅 queue 模式使用)
    testAgvSpeed: 6.0     // 測試 AGV 的速度係數
};
```

### **瀏覽器控制台命令**
```javascript
// 推薦：使用佇列模式（最平滑）
setQueueMode(8.0, 5);  // 速度 8.0，佇列大小 5

// 完整設定
setAllAgvAnimationMode('queue', 8.0, 5);

// 其他模式
setAllAgvAnimationMode('instant');
setAllAgvAnimationMode('smooth', 6.0);
setAllAgvAnimationMode('ultra-smooth', 6.0);

// 查看佇列狀態
showAgvQueueStatus();
```

### **單個 AGV 控制**
```javascript
// 針對特定 AGV
agvObject.setAnimationMode('smooth', 4.0);
agvObject.setAnimationMode('instant');
```

## ⚙️ **技術細節**

### **地理座標插值**
```javascript
// 在地理座標系統中進行插值，不受縮放影響
const latDiff = targetLat - currentLat;
const lngDiff = targetLng - currentLng;
const newLat = currentLat + latDiff * this.smoothSpeed * delta;
const newLng = currentLng + lngDiff * this.smoothSpeed * delta;
```

### **角度插值**
```javascript
// 處理 360° 循環，選擇最短旋轉路徑
const angleDiff = ((this.targetAngle - this.angle + 540) % 360) - 180;
this.angle += angleDiff * this.smoothSpeed * delta;
```

## 🎯 **解決的問題**

1. **✅ 地圖縮放影響**：現在使用地理座標系統，在任何縮放級別下移動距離都一致
2. **✅ 速度限制移除**：即時模式下完全移除速度限制
3. **✅ 動畫控制靈活性**：可動態切換動畫模式和調整速度
4. **✅ 效能優化**：保持 CSS transform 的 GPU 加速優勢

## 📊 **動畫模式比較**

| 模式 | 響應性 | 視覺效果 | 效能 | 適用場景 |
|------|--------|----------|------|----------|
| **instant** | 立即 | 跳躍式 | 最佳 | 生產監控 |
| **smooth** | 延遲 | 流暢 | 良好 | 展示演示 |

## 🔍 **測試建議**

1. **基本功能測試**：
   - 在不同縮放級別下測試 AGV 移動
   - 驗證動畫模式切換功能
   - 測試速度係數調整效果

2. **效能測試**：
   - 多個 AGV 同時移動的效能
   - 長時間運行的穩定性
   - 記憶體使用情況

3. **用戶體驗測試**：
   - 不同動畫速度的視覺效果
   - 地圖互動的流暢度
   - 附加物件（貨架）的跟隨效果

## 📝 **注意事項**

- **平滑模式會消耗更多 CPU 資源**，大量 AGV 時請注意效能
- **速度係數建議範圍**：1.0-10.0
- **生產環境建議**：使用 instant 模式以獲得最佳效能
- **展示環境建議**：使用 smooth 模式以獲得最佳視覺效果

## 🎛️ **參數調整指南**

### **速度係數建議值**
- **3.0-5.0**：慢速，適合觀察細節和調試
- **6.0-8.0**：中速，平衡流暢度和響應性（**推薦**）
- **9.0-12.0**：快速，適合快速更新場景
- **13.0-15.0**：極快，接近即時模式

### **自動測試功能**
```javascript
// 測試不同速度
testAgvSpeeds();

// 測試不同動畫模式（推薦）
testAgvModes();
```

### **解決縮放時移動距離不夠的問題**
1. **使用指數衰減模型**：`1 - Math.exp(-speed * delta)`
2. **設定最大插值係數**：防止過度移動
3. **距離閾值檢查**：接近目標時直接到達

### **改進的插值算法**
```javascript
// 新的插值計算，解決縮放問題
const interpolationFactor = Math.min(this.maxSpeed, 1 - Math.exp(-this.smoothSpeed * delta));
```

## 🔧 **故障排除**

### **問題：縮放時 AGV 移動距離不夠**
**原因**：舊的線性插值在高頻更新時會累積誤差
**解決**：使用指數衰減模型，確保最終會到達目標

### **問題：動畫太慢或太快**
**解決**：使用 `testAgvSpeeds()` 函數找到最適合的參數

### **問題：AGV 抖動或超調**
**解決**：降低速度係數或檢查 `maxSpeed` 設定

### **問題：動畫有頓挫感**
**原因**：插值算法不夠平滑或幀率不穩定
**解決**：
1. 使用 `multi-point` 模式：`setMultiPointMode(10.0, 5)`
2. 調整緩衝區大小和速度參數
3. 使用 `testAgvModes()` 找到最適合的模式

### **問題：角度快速正反轉**
**原因**：角度使用多點平均計算會造成 360° 循環問題
**解決**：已修正為只對位置使用多點平滑，角度使用傳統插值

## 🚀 **未來改進方向**

1. **自適應動畫**：根據 AGV 數量自動調整動畫模式
2. **路徑預測**：基於歷史軌跡優化動畫路徑
3. **碰撞檢測**：在動畫中加入簡單的碰撞避免
4. **效能監控**：自動監控並調整動畫參數
