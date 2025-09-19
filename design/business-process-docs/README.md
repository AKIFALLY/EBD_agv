# RosAGV 業務流程文件中心

## 📁 文件結構

```
business-process-docs/
├── index.html                                    # 📋 主導航頁面（動態 Markdown 載入系統）
├── content/                                      # 📝 Markdown 內容檔案
│   ├── getting-started/                         # 快速入門指南
│   ├── business-processes/                      # 業務流程說明
│   ├── agv-vehicles/                            # AGV 車輛相關
│   ├── system-architecture/                     # 系統架構
│   ├── operations/                              # 操作指南
│   └── technical-details/                       # 技術細節
├── css/                                          # 🎨 樣式資源
│   └── vendors/                                 # 第三方樣式
├── js/                                           # 📜 JavaScript 資源
│   ├── content-loader.js                        # Markdown 內容載入器
│   ├── navigation.js                            # 導航功能
│   └── vendors/                                 # 第三方程式庫
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
2. 系統會動態載入 `content/` 目錄下的 Markdown 檔案
3. 透過左側導航選單瀏覽不同主題的文檔

### 文檔內容分類

#### 🎯 快速入門 (getting-started/)
- 什麼是 RosAGV？
- 系統整體概覽
- 核心概念解釋
- 快速上手指導

#### 🏭 業務流程 (business-processes/)
- 眼鏡生產流程
- 室內物料搬運

#### 🚗 AGV 車輛 (agv-vehicles/)
- 車型介紹
- Cargo Mover
- Loader AGV
- Unloader AGV

#### 🏗️ 系統架構 (system-architecture/)
- 雙環境架構
- 技術棧架構

#### 🔧 操作指南 (operations/)
- 部署指南
- 開發環境
- 維護操作
- 故障排除
- 系統診斷
- 統一工具系統
- 服務管理
- 效能調優

#### 💻 技術細節 (technical-details/)
- TAFL 系統概覽
- TAFL 語言規格
- TAFL 開發指南

## 🛠️ 技術特色

### 統一設計系統
- **響應式設計**: 支援桌面、平板、手機
- **動態內容載入**: Markdown 檔案即時渲染
- **語法高亮**: 使用 Prism.js 提供程式碼語法高亮
- **平滑導航**: 左側導航選單，點擊即可切換內容

### 技術特點
- **Markdown 支援**: 使用 marked.js 解析 Markdown
- **單頁應用**: 無需重新載入頁面即可切換內容
- **本地化資源**: 所有資源都本地化，無需網路連接

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

### 訪問方式
- **Web 訪問**: `http://agvc.ui/docs/index.html`
- **本地開發**: 使用 Python HTTP 伺服器或 Node.js serve
- **注意**: `http://agvc.ui/docs` 是 FastAPI 文檔，不是業務流程文檔

## 🔧 維護說明

### 更新內容
- **內容更新**: 直接修改 `content/` 目錄下對應的 Markdown 檔案
- **新增頁面**: 在適當的子目錄下創建新的 `.md` 檔案
- **導航更新**: 修改 `index.html` 中的導航選單項目

### Markdown 檔案格式
```markdown
# 頁面標題

## 章節標題

內容說明...

\`\`\`javascript
// 程式碼區塊會自動語法高亮
const example = "code";
\`\`\`
```

## 📊 系統需求

- **瀏覽器**: Chrome 60+, Firefox 60+, Safari 12+, Edge 79+
- **解析度**: 支援 320px - 2560px 寬度
- **JavaScript**: 需要啟用 JavaScript 以獲得完整互動功能

## 🚀 更新歷史

### 2025-09-19
- ✅ 移除舊版 2025.html 檔案，改用動態 Markdown 載入系統
- ✅ 整理文檔結構，使用 `content/` 目錄組織 Markdown 內容
- ✅ 更新為單頁應用架構，提升使用體驗

### 2025-08-04
- ✅ 建立新的動態文檔系統
- ✅ 整合 Markdown 解析和語法高亮功能
- ✅ 實作左側導航選單和內容動態載入

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