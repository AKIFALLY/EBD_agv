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
├── simple_wcs_system_2025.html                 # ⚡ Simple WCS 決策引擎指南 (紅色主題)
└── README.md                                    # 📖 本說明文件
```

## 🎯 使用指南

### ⚠️ 系統工具使用前提條件
**使用 RosAGV `r` 工具集進行系統診斷和管理之前，必須將 RosAGV 目錄加入 PATH 環境變數**

在 `~/.bashrc` 中添加以下設定：
```bash
# RosAGV 工具路徑配置
export PATH="/home/ct/RosAGV:$PATH"
```

設定完成後，重新載入環境：
```bash
source ~/.bashrc
```

驗證配置是否正確：
```bash
which r                    # 應該顯示 /home/ct/RosAGV/r
r menu                     # 顯示工具選單
r agvc-check              # 執行 AGVC 健康檢查
```

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

#### ⚡ Simple WCS 決策引擎指南
- **檔案**: `simple_wcs_system_2025.html`
- **主題色**: 紅色
- **適用對象**: AI工程師、配置管理員、WCS開發者
- **主要內容**: 極簡化配置驅動的WCS決策引擎，YAML配置系統，AI Agent自動化管理

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
--primary-red: #e74c3c;       /* Simple WCS 決策引擎 */
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
simple_wcs_system_2025.html?theme=red
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

### 2025-07-31
- ✅ 更新系統架構文檔新增 Web API Launch 服務管理
- ✅ 新增 launch_ws 服務編排工作空間說明
- ✅ 統一服務管理 API 架構更新 (manage_web_api_launch)
- ✅ ROS 2 Launch 系統整合到核心技術棧
- ✅ 統一工具系統服務管理功能擴展
- ✅ **新增配置管理工具到 r 工具集**
  - 新增 `r zenoh-config` - Zenoh Router 配置管理
  - 新增 `r hardware-config` - 硬體映射配置管理
  - 整合現有 scripts/config-tools/ 專業工具
  - 更新統一工具文檔和使用指南

### 2025-07-30
- ✅ 新增 Simple WCS 決策引擎系統文檔
- ✅ 完整的 YAML 配置驅動系統設計說明
- ✅ AI Agent 整合和 yq 工具使用指導
- ✅ 紅色主題設計，與現有文檔體系整合
- ✅ 更新主導航頁面，新增 Simple WCS 入口

### 2025-01-30
- ✅ 建立統一的 CSS 和 JS 資源文件
- ✅ 實施統一的設計系統和主題色彩
- ✅ 建立 index.html 導航中心  
- ✅ 整合所有現有文件並統一樣式
- ✅ 實作響應式設計和互動功能
- ✅ 完成 KUKA Fleet 和架台系統整合文件

## 📞 技術支援

如有技術問題或需要協助，請參考：

### 🔍 系統診斷工具
- RosAGV 系統診斷: `r agvc-check`
- 容器狀態檢查: `r containers-status`  
- 網路診斷: `r network-check`
- Zenoh 連接檢查: `r zenoh-check`
- 快速診斷: `r quick-diag`

### ⚙️ 配置管理工具 (新增)
- Zenoh Router 配置管理: `r zenoh-config`
- 硬體映射配置管理: `r hardware-config`
- 配置檔案驗證和編輯功能
- 端點連接性測試和診斷

---

**🤖 RosAGV Project - Intelligent Manufacturing Execution System**  
*Powered by ROS 2 Jazzy + Zenoh RMW + Docker Compose V2*