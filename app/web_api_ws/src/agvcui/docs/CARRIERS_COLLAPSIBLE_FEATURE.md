# 📦 Carriers 頁面收合展開功能實現

## 🎯 功能概述

為 Carriers 頁面的分組區塊添加收合展開功能，讓用戶可以根據需要隱藏或顯示特定分組的詳細內容，提升頁面的可用性和視覺整潔度。

## ✅ 實現的收合展開功能

### 1. 🏠 房間內載具分組
- **標題可點擊**: 點擊 "房間內載具" 標題可收合/展開整個分組
- **視覺指示**: 右側箭頭圖示顯示當前狀態
- **動畫效果**: 平滑的收合展開動畫

### 2. 📦 貨架上載具分組
- **兩層收合**:
  - **第一層**: 整個 "貨架上載具" 分組可收合
  - **第二層**: 每個貨架子分組也可獨立收合
- **獨立控制**: 每個貨架可以獨立收合，不影響其他貨架
- **格位視覺化**: 收合時隱藏格位圖和載具列表

### 3. 🔌 設備端口載具分組
- **標題可點擊**: 點擊標題收合/展開設備端口載具
- **卡片佈局**: 收合時隱藏所有載具卡片

### 4. ❓ 未分配載具分組
- **標題可點擊**: 點擊標題收合/展開未分配載具
- **卡片佈局**: 收合時隱藏所有載具卡片

## 🎨 視覺設計

### 標題樣式
```css
.is-clickable {
    cursor: pointer;
    user-select: none;
}

.is-clickable:hover {
    background-color: rgba(0, 0, 0, 0.05);
    border-radius: 4px;
}
```

### 箭頭圖示
- **展開狀態**: `mdi-chevron-down` (向下箭頭)
- **收合狀態**: `mdi-chevron-up` (向上箭頭)
- **旋轉動畫**: 180度旋轉效果

### 內容動畫
```css
.collapsible-content {
    transition: all 0.3s ease;
    overflow: hidden;
}

.collapsible-content.collapsed {
    max-height: 0;
    opacity: 0;
    margin: 0;
    padding: 0;
}
```

## ⚙️ 技術實現

### HTML 結構
```html
<!-- 分組標題 -->
<h2 class="subtitle is-4 is-clickable" onclick="toggleSection('room-carriers')">
    <span class="icon has-text-info">
        <i class="mdi mdi-home"></i>
    </span>
    房間內載具 ({{ count }})
    <span class="icon is-pulled-right toggle-icon" id="room-carriers-icon">
        <i class="mdi mdi-chevron-down"></i>
    </span>
</h2>

<!-- 可收合內容 -->
<div class="collapsible-content" id="room-carriers-content">
    <!-- 分組內容 -->
</div>
```

### JavaScript 功能
```javascript
function toggleSection(sectionId) {
    const content = document.getElementById(sectionId + '-content');
    const icon = document.getElementById(sectionId + '-icon');
    
    if (content && icon) {
        const isCollapsed = content.classList.contains('collapsed');
        
        if (isCollapsed) {
            // 展開
            content.classList.remove('collapsed');
            icon.classList.remove('rotated');
            icon.querySelector('i').classList.remove('mdi-chevron-up');
            icon.querySelector('i').classList.add('mdi-chevron-down');
        } else {
            // 收合
            content.classList.add('collapsed');
            icon.classList.add('rotated');
            icon.querySelector('i').classList.remove('mdi-chevron-down');
            icon.querySelector('i').classList.add('mdi-chevron-up');
        }
    }
}
```

## 🔧 分組 ID 命名規則

### 主要分組
- `room-carriers`: 房間內載具
- `rack-carriers`: 貨架上載具
- `port-carriers`: 設備端口載具
- `unassigned-carriers`: 未分配載具

### 貨架子分組
- `rack-{rack_id}`: 特定貨架分組
- 例如: `rack-123` 表示貨架 ID 為 123 的分組

## 🎯 用戶體驗

### 預設狀態
- **全部展開**: 頁面載入時所有分組預設為展開狀態
- **直觀操作**: 標題有 hover 效果，提示可點擊
- **視覺回饋**: 箭頭圖示清楚顯示當前狀態

### 互動體驗
- **平滑動畫**: 0.3秒的收合展開動畫
- **獨立控制**: 每個分組可獨立操作
- **狀態保持**: 在同一頁面會話中保持收合狀態

### 實用場景
1. **專注特定分組**: 收合不需要的分組，專注查看特定類型載具
2. **節省螢幕空間**: 在小螢幕上提供更好的瀏覽體驗
3. **快速概覽**: 收合詳細內容，快速查看各分組的載具數量

## 📊 功能對比

| 功能 | 實現前 | 實現後 |
|------|--------|--------|
| 頁面長度 | 固定長度，所有內容展開 | 可調整，按需顯示 |
| 視覺整潔度 | 內容較多，可能雜亂 | 可收合，更整潔 |
| 導航效率 | 需要滾動查看 | 快速跳轉到關注區域 |
| 行動裝置體驗 | 需要大量滾動 | 更適合小螢幕 |
| 用戶控制 | 被動瀏覽 | 主動控制顯示內容 |

## 🚀 使用指南

### 收合分組
1. 點擊分組標題（如 "房間內載具"）
2. 觀察箭頭圖示變為向上
3. 內容平滑收合隱藏

### 展開分組
1. 點擊已收合的分組標題
2. 觀察箭頭圖示變為向下
3. 內容平滑展開顯示

### 貨架子分組
1. 先展開 "貨架上載具" 主分組
2. 點擊特定貨架標題（如 "貨架 123"）
3. 該貨架的格位圖和載具列表會收合/展開

## 🔮 未來擴展

### 可能的增強功能
1. **記憶狀態**: 使用 localStorage 記住用戶的收合偏好
2. **全部收合/展開**: 添加一鍵收合或展開所有分組的按鈕
3. **鍵盤快捷鍵**: 支援鍵盤操作收合展開
4. **動畫選項**: 允許用戶選擇動畫速度或關閉動畫

### 技術優化
1. **效能優化**: 對大量載具的情況優化渲染效能
2. **無障礙支援**: 添加 ARIA 標籤支援螢幕閱讀器
3. **觸控優化**: 優化觸控裝置的操作體驗

---

**🎉 收合展開功能已成功實現！**

現在用戶可以更靈活地控制 Carriers 頁面的顯示內容，提升了頁面的可用性和用戶體驗。無論是在桌面還是行動裝置上，都能提供更好的瀏覽體驗。
