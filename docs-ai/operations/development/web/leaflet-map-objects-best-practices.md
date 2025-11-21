# Leaflet 地圖物件開發最佳實踐

## 📋 適用範圍
- **適用層級**：🔧 操作層 - Web 開發實踐
- **適用系統**：AGVCUI 地圖視覺化系統
- **技術棧**：Leaflet.js + CSS Flexbox/Grid

---

## 🎯 核心問題：地圖物件縮放時位置偏移

### 問題描述
當使用 Leaflet DivIcon 創建自定義地圖標記時，如果 CSS 配置不當，在地圖縮放時會出現視覺偏移現象：
- 物件看起來「漂移」到錯誤位置
- 縮放級別越高，偏移越明顯
- 實際地理座標沒變，但視覺位置錯誤

---

## 🔍 根本原因分析

### 1. iconAnchor 是基於 iconSize 的幾何中心

**Leaflet 工作原理**：
```javascript
const icon = L.divIcon({
    iconSize: [96, 64],      // 圖標尺寸（寬x高）
    iconAnchor: [48, 32]     // 錨點位置（應該是正中心）
});
```

- `iconAnchor` 定義圖標的錨定點相對於**圖標左上角**的偏移
- Leaflet 使用這個錨點將圖標定位在地圖座標上
- **錨點應該設在幾何中心**：`[width/2, height/2]`

### 2. padding 會導致視覺中心 ≠ 幾何中心

**問題示意**：
```
iconSize: [96, 64]，iconAnchor: [48, 32]

❌ 有 padding (0.5rem = 8px):
┌────────────────────────┐ 96x64 (幾何邊界)
│ padding: 8px           │
│  ┌──────────────────┐  │
│  │   實際內容區域    │  │ 80x48 (視覺區域)
│  │     視覺中心      │  │ ← 視覺中心在 [48, 36]
│  └──────────────────┘  │     不等於 iconAnchor [48, 32]
│         padding: 8px    │
└────────────────────────┘
         ↑
    iconAnchor [48, 32] 指向這裡（幾何中心）

縮放時：視覺中心偏移 = (48-48, 36-32) = (0, 4px)
放大 4倍後：視覺偏移 = (0, 16px) ← 明顯偏移！
```

**數學公式**：
```
視覺中心偏移 = (視覺區域中心) - (iconAnchor)
             = ((width-2*paddingX)/2, (height-2*paddingY)/2 + paddingY) - (width/2, height/2)
             = (-paddingX, paddingY)

縮放後視覺偏移 = 視覺中心偏移 × scale
```

### 3. 必須使用 flexbox/grid 顯式居中內容

**錯誤方式（預設 block 布局）**：
```css
.map-object {
    padding: 0.5rem;
    /* ❌ 無布局聲明，內容預設靠左上角 */
}
```

**正確方式（Flexbox 居中）**：
```css
.map-object {
    display: flex;              /* ✅ 啟用 flexbox */
    flex-direction: column;     /* ✅ 垂直排列 */
    align-items: center;        /* ✅ 水平居中 */
    justify-content: center;    /* ✅ 垂直居中 */
    padding: 0;                 /* ✅ 移除 padding */
}
```

### 4. 保持對稱性：無 padding 或對稱 padding

**優先順序**：
1. **最佳**：完全移除 padding (`padding: 0`)
2. **次佳**：使用對稱 padding (`padding: 0.25rem 0` 或 `padding: 0.25rem`)
3. **避免**：不對稱 padding (`padding-top: 0.5rem; padding-bottom: 0.25rem;`)

---

## ✅ 正確實作模式

### 模式 1：Flexbox 完整居中（推薦）

**適用場景**：需要垂直/水平排列內容的資訊面板

**CSS 範例**（參考 `DoorStatusObject`）：
```css
.info-panel {
    /* 布局 */
    display: flex;
    flex-direction: column;

    /* 居中對齊 */
    align-items: center;      /* 水平居中 */
    justify-content: center;  /* 垂直居中 */

    /* 移除 padding */
    padding: 0;

    /* 其他樣式 */
    background-color: rgba(240, 240, 240, 0.95);
    border-radius: 0.5rem;
}

.info-panel-title {
    text-align: center;
    padding: 0.25rem 0;  /* 對稱 padding */
}

.info-panel-content {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    flex: 1;  /* 占滿剩餘空間 */
}
```

**JavaScript 範例**：
```javascript
export class InfoPanelObject extends BaseObject {
    constructor(map, latlng, id) {
        const html = `
            <div class="info-panel">
                <div class="info-panel-title">標題</div>
                <div class="info-panel-content">
                    <span>內容1</span>
                    <span>內容2</span>
                </div>
            </div>
        `;

        // iconAnchor 必須是 iconSize 的正中心
        super(map, latlng, html, [96, 64], [48, 32], 2000);
    }
}
```

### 模式 2：Grid 布局（替代方案）

**適用場景**：需要網格排列的多單元格內容

**CSS 範例**（參考 `TransferBoxObject`）：
```css
.grid-panel {
    /* Grid 布局 */
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    grid-template-rows: repeat(2, 1fr);

    /* 移除 padding */
    padding: 0;

    /* 縮放中心點 */
    transform-origin: center center;

    /* 其他樣式 */
    background-color: rgba(240, 240, 240, 0.95);
    border-radius: 0.5rem;
}

.grid-cell {
    /* Grid 自動居中內容 */
    display: flex;
    align-items: center;
    justify-content: center;
    text-align: center;
}
```

---

## 🔧 除錯步驟

### 步驟 1：驗證 iconAnchor 配置
```javascript
// ✅ 正確：iconAnchor 是 iconSize 的一半
super(map, latlng, html, [96, 64], [48, 32], 2000);
//                        ^^^^^^^^  ^^^^^^^^
//                        iconSize  iconAnchor (中心點)

// ❌ 錯誤：iconAnchor 不在中心
super(map, latlng, html, [96, 64], [50, 30], 2000);
```

### 步驟 2：檢查 CSS padding
```css
/* ❌ 問題配置 */
.map-object {
    padding: 0.5rem;  /* 有 padding */
}

/* ✅ 正確配置 */
.map-object {
    padding: 0;  /* 無 padding */
}
```

### 步驟 3：檢查布局聲明
```css
/* ❌ 問題配置 */
.map-object {
    /* 無 display 聲明 */
}

/* ✅ 正確配置 */
.map-object {
    display: flex;              /* 或 grid */
    align-items: center;
    justify-content: center;
}
```

### 步驟 4：瀏覽器測試
```javascript
// 在瀏覽器 Console 執行
const obj = workspaceObjects.get('1_op1');
console.log('iconSize:', obj.iconSize);
console.log('iconAnchor:', obj.iconAnchor);
console.log('Position:', obj.marker.getLatLng());

// 測試縮放
map.setZoom(map.getZoom() + 1);
console.log('After zoom:', obj.marker.getLatLng());  // 應該不變
```

---

## 📊 實戰案例對照表

| 物件類型 | iconSize | iconAnchor | 布局 | Padding | 內容對齊 | 縮放效果 | 參考檔案 |
|---------|----------|------------|------|---------|----------|----------|---------|
| **TransferBox** ✅ | [120, 80] | [60, 40] | Grid | 無 | Grid 自動 | 完美 | `TransferBoxObject.js` |
| **DoorStatus** ✅ | [100, 60] | [50, 30] | Flexbox | 無 | 顯式居中 | 完美 | `DoorStatusObject.js` |
| **DockedRackInfo (修改前)** ❌ | [96, 64] | [48, 32] | 無 | 0.5rem | 不完整 | 偏移 | - |
| **DockedRackInfo (修改後)** ✅ | [96, 64] | [48, 32] | Flexbox | 無 | 完整居中 | 完美 | `DockedRackInfoObject.js` |

---

## 🎓 關鍵教訓總結

### 四大核心原則

1. **iconAnchor 是基於 iconSize 的幾何中心**
   - 永遠設為 `[width/2, height/2]`
   - 不要偏移錨點來「修正」視覺問題

2. **padding 會導致視覺中心 ≠ 幾何中心 → 縮放偏移**
   - 優先移除所有 padding
   - 如需 padding，使用對稱值且越小越好

3. **必須使用 flexbox/grid 顯式居中內容**
   - 不能依賴預設 block 布局
   - 明確使用 `display: flex` 或 `display: grid`
   - 配合 `align-items` 和 `justify-content`

4. **保持對稱性：無 padding 或對稱 padding**
   - 最佳：`padding: 0`
   - 次佳：`padding: 0.25rem` 或 `padding: 0.25rem 0`
   - 避免：不對稱的 padding/margin

### 快速檢查清單

當地圖物件出現縮放偏移時：

- [ ] iconAnchor 是否為 `[iconSize[0]/2, iconSize[1]/2]`
- [ ] CSS 是否有 `display: flex` 或 `display: grid`
- [ ] padding 是否為 0 或完全對稱
- [ ] 內容是否有 `align-items: center` 和 `justify-content: center`
- [ ] 是否參考了成功案例（TransferBox/Door）的實作方式

---

## 📂 相關檔案位置

### 基礎類別
- **BaseObject**: `app/web_api_ws/src/agvcui/agvcui/static/objects/BaseObject.js`
  - 提供基礎的 DivIcon 創建和縮放處理機制

### 成功範例（推薦參考）
- **TransferBoxObject**: `app/web_api_ws/src/agvcui/agvcui/static/objects/TransferBoxObject.js`
  - 使用 Grid 布局，無 padding，完美居中
  - CSS: `.transfer-box-static`

- **DoorStatusObject**: `app/web_api_ws/src/agvcui/agvcui/static/objects/DoorStatusObject.js`
  - 使用 Flexbox 完整居中，無 padding
  - CSS: `.door-status-container`, `.door-status-value`

### CSS 樣式
- **mapPage.css**: `app/web_api_ws/src/agvcui/agvcui/static/css/mapPage.css`
  - 所有地圖物件的樣式定義

---

## 🔗 相關文檔

- **Web 開發指南**: `docs-ai/operations/development/web/web-development.md`
- **AGVCUI 監控系統**: `docs-ai/knowledge/system/agvui-monitoring-system.md`
- **前端開發規範**: `docs-ai/operations/development/web/frontend-standards.md`

---

## 📝 修改記錄

| 日期 | 版本 | 說明 | 作者 |
|------|------|------|------|
| 2025-11-13 | 1.0 | 初始版本，記錄 Leaflet 地圖物件縮放對齊最佳實踐 | Claude |

---

**最後更新**: 2025-11-13
**維護者**: RosAGV 開發團隊
**適用版本**: RosAGV AGVCUI v1.0+
