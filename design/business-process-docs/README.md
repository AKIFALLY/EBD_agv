# RosAGV 業務流程文件中心

## 📁 文件結構

```
business-process-docs/
├── index.html                                    # 📋 主導航頁面
├── assets/                                       # 🎨 共用資源
│   ├── css/
│   │   └── rosagv-common.css                    # 統一樣式表
│   └── js/
│       └── rosagv-common.js                     # 統一 JavaScript 功能庫
├── rosagv_business_process_2025.html           # 🏠 房間內製程操作指南 (藍色主題)
├── rosagv_outdoor_process_2025.html            # 🌿 房間外製程操作指南 (綠色主題)
├── rosagv_architecture_2025.html               # 🏗️ 系統架構技術文件 (紫色主題)
├── rosagv_complete_business_system_2025.html   # 🏢 完整業務系統指南 (橘色主題)
└── README.md                                    # 📖 本說明文件
```

## 🎯 使用指南

### 快速開始
1. 開啟 `index.html` 作為導航入口
2. 根據需求點選對應的文件類型
3. 每個文件都採用統一的視覺設計和互動功能

### 文件分類

#### 🏠 房間內製程操作指南
- **檔案**: `rosagv_business_process_2025.html`
- **主題色**: 藍色
- **適用對象**: 製程工程師、AGV操作員
- **主要內容**: Loader AGV、Unloader AGV協同作業流程

#### 🌿 房間外製程操作指南  
- **檔案**: `rosagv_outdoor_process_2025.html`
- **主題色**: 綠色
- **適用對象**: 射出工程師、KUKA操作員、OPUI使用者
- **主要內容**: KUKA Fleet整合、架台管理、OPUI操作流程

#### 🏗️ 系統架構技術文件
- **檔案**: `rosagv_architecture_2025.html`  
- **主題色**: 紫色
- **適用對象**: 系統架構師、後端工程師、DevOps工程師
- **主要內容**: 技術架構、模組設計、API接口

#### 🏢 完整業務系統指南
- **檔案**: `rosagv_complete_business_system_2025.html`
- **主題色**: 橘色  
- **適用對象**: 專案經理、業務分析師、系統整合工程師
- **主要內容**: 企業級整合方案、業務流程設計

## 🛠️ 技術特色

### 統一設計系統
- **響應式設計**: 支援桌面、平板、手機
- **主題色彩**: 每個文件有專屬主題色，易於識別
- **統一元件**: 卡片、時間軸、統計數據、導航等統一設計
- **動畫效果**: 平滑滾動、淡入動畫、懸停效果

### 互動功能
- **平滑導航**: 點擊導航自動滾動到對應章節
- **動態統計**: 數字計數動畫效果
- **懸停回饋**: 卡片和按鈕的互動回饋
- **主題切換**: 支援 URL 參數切換主題

### 統一樣式系統
```css
/* 主要 CSS 變數 */
--primary-blue: #3498db;      /* 房間內製程 */
--primary-green: #27ae60;     /* 房間外製程 */  
--primary-purple: #9b59b6;    /* 系統架構 */
--primary-orange: #e67e22;    /* 完整業務系統 */
```

## 📱 使用方式

### 本地開發
```bash
# 在文件目錄下啟動簡單 HTTP 伺服器
python3 -m http.server 8000
# 或
npx serve .

# 然後開啟瀏覽器
http://localhost:8000
```

### 主題切換
可以透過 URL 參數切換文件主題：
```
rosagv_business_process_2025.html?theme=blue
rosagv_outdoor_process_2025.html?theme=green  
rosagv_architecture_2025.html?theme=purple
rosagv_complete_business_system_2025.html?theme=orange
```

## 🔧 維護說明

### 更新內容
- 內容更新只需修改對應的 HTML 文件
- 樣式修改統一在 `assets/css/rosagv-common.css`
- 功能增強統一在 `assets/js/rosagv-common.js`

### 新增文件
1. 複製現有 HTML 文件作為模板
2. 修改標題和內容
3. 調整主題色彩變數
4. 在 `index.html` 中添加導航連結

### 樣式自定義
每個文件都支援頁面特定的樣式覆蓋：
```html
<style>
    /* 頁面特定樣式 */
    body { background: custom-gradient; }
</style>
```

## 📊 系統需求

- **瀏覽器**: Chrome 60+, Firefox 60+, Safari 12+, Edge 79+
- **解析度**: 支援 320px - 2560px 寬度
- **JavaScript**: 需要啟用 JavaScript 以獲得完整互動功能

## 🚀 更新歷史

### 2025-01-30
- ✅ 建立統一的 CSS 和 JS 資源文件
- ✅ 實施統一的設計系統和主題色彩
- ✅ 建立 index.html 導航中心  
- ✅ 整合所有現有文件並統一樣式
- ✅ 實作響應式設計和互動功能
- ✅ 完成 KUKA Fleet 和架台系統整合文件

## 📞 技術支援

如有技術問題或需要協助，請參考：
- RosAGV 系統診斷: `r agvc-check`
- 容器狀態檢查: `r containers-status`  
- 網路診斷: `r network-check`
- 快速診斷: `r quick-diag`

---

**🤖 RosAGV Project - Intelligent Manufacturing Execution System**  
*Powered by ROS 2 Jazzy + Zenoh RMW + Docker Compose V2*