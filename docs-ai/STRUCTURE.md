# docs-ai 文檔結構

## 📁 目錄結構總覽

```
docs-ai/ (68個 .md 檔案)
├── README.md                   # AI Agent 記憶系統說明
├── STRUCTURE.md               # 本檔案 - 文檔結構導航
├── LAYERED-IMPORT-GUIDE.md   # AI Agent 三層載入架構指南
├── context/                    # 系統上下文和架構 (7個檔案)
│   ├── structure/             # 模組結構文檔 (1個檔案)
│   ├── system/                # 系統架構文檔 (4個檔案)
│   └── workspaces/            # 工作空間文檔 (2個檔案)
├── knowledge/                  # 知識庫 (30個檔案)
│   ├── agv-domain/            # AGV 領域知識 (9個檔案)
│   ├── business/              # 業務流程知識 (1個檔案)
│   ├── protocols/             # 通訊協議 (7個檔案)
│   └── system/                # 系統知識 (13個檔案)
│       └── tafl/              # TAFL 語言相關文檔 (7個檔案)
└── operations/                 # 操作指南 (28個檔案)
    ├── deployment/             # 部署相關 (4個檔案)
    ├── development/            # 開發指南 (16個檔案)
    │   ├── core/              # 核心開發原則 (3個檔案)
    │   ├── ros2/              # ROS 2 開發 (3個檔案)
    │   ├── testing/           # 測試相關 (5個檔案)
    │   └── web/               # Web 開發 (2個檔案)
    ├── guides/                # 操作手冊和故障排除 (7個檔案)
    └── tools/                 # 工具文檔 (1個檔案)
```

## 📚 核心文檔分類

### 1. Context（系統上下文）- 7個檔案
- **system/** (4個): 系統架構、雙環境設計、技術棧、語言配置
- **workspaces/** (2個): AGV 和 AGVC 工作空間詳細說明
- **structure/** (1個): 模組索引和結構說明

### 2. Knowledge（知識庫）- 30個檔案
- **agv-domain/** (9個): AGV 車型、狀態機、資料庫設計、工作流程、授權表設計
- **protocols/** (7個): PLC、ROS 2、Zenoh、KUKA Fleet 協議
- **system/** (13個): 系統設計知識、監控、rack管理
  - **tafl/** (7個): TAFL 語言規格、API、編輯器、實作專案
- **business/** (1個): 眼鏡生產流程（含系統現狀）

### 3. Operations（操作指南）- 28個檔案
- **deployment/** (4個): Docker、Nginx、容器管理、套件清單
- **development/** (16個):
  - **core/** (3個): 核心原則、Linus Torvalds 思維、文檔標準
  - **ros2/** (3個): ROS 2 開發、容器指令、PLC 通訊
  - **web/** (2個): Web 開發、API Launch 管理
  - **testing/** (5個): 測試標準、程序、Pytest、檔案管理
- **guides/** (7個): 操作手冊、故障排除、系統診斷、日誌分析、設備授權
- **tools/** (1個): 統一工具系統

## 🎯 文檔定位指南

### 新文檔應該放在哪裡？

**開發相關**：
- 核心原則和理念 → `operations/development/core/`
- ROS 2 開發技術 → `operations/development/ros2/`
- Web 開發技術 → `operations/development/web/`
- 測試相關 → `operations/development/testing/`
- Docker 開發 → `operations/development/`

**系統知識**：
- AGV 車輛相關 → `knowledge/agv-domain/`
- 通訊協議 → `knowledge/protocols/`
- TAFL 語言 → `knowledge/system/tafl/`
- 系統設計 → `knowledge/system/`
- 業務流程 → `knowledge/business/`

**操作指導**：
- 部署配置 → `operations/deployment/`
- 工具使用 → `operations/tools/`
- 故障排除 → `operations/guides/`
- 操作手冊 → `operations/guides/`

**架構說明**：
- 系統架構 → `context/system/`
- 工作空間 → `context/workspaces/`
- 模組結構 → `context/structure/`

## 📊 文檔統計（2025-09-19 更新）

### 各分類檔案數量
| 分類 | 子分類 | 檔案數 |
|------|--------|--------|
| **Context** | | 8 |
| | system | 4 |
| | workspaces | 2 |
| | structure | 1 |
| | (根目錄) | 1 |
| **Knowledge** | | 30 |
| | agv-domain | 9 |
| | protocols | 7 |
| | system (含 tafl) | 13 |
| | business | 1 |
| **Operations** | | 28 |
| | deployment | 4 |
| | development | 16 |
| | guides | 7 |
| | tools | 1 |
| **總計** | | **68** |

## 🔄 最近重組變更（2025-09-19）

### 最新新增文檔（2025-09-19）
1. **LAYERED-IMPORT-GUIDE.md**: AI Agent 三層載入架構指南
2. **knowledge/agv-domain/license-table-design.md**: 設備授權管理表設計
3. **operations/guides/device-authorization-guide.md**: 設備授權操作指南

### 最新清理工作
1. **刪除空目錄**:
   - `context/business/` (空目錄)
   - `knowledge/automation/` (空目錄)

2. **刪除重複文檔**:
   - `knowledge/system/current-system-status.md` (內容併入 eyewear-production-process.md)

3. **保留文檔結構**:
   - Rack management 文檔保持分離（操作手冊 vs 技術文檔）
   - STRUCTURE.md 保留作為人類參考

### 已移動的文檔
1. **核心開發原則** → `operations/development/core/`
2. **ROS 2 開發** → `operations/development/ros2/`
3. **Web 開發** → `operations/development/web/`
4. **測試文檔** → `operations/development/testing/`
5. **TAFL 文檔** → `knowledge/system/tafl/`
6. **故障排除和手冊** → `operations/guides/`

### 已更新的引用
- CLAUDE.md 中的所有 docs-ai/ 引用已更新
- README.md 中的引用已更新
- 所有文檔內的交叉引用已更新至新路徑

## 📝 維護指南

### 文檔管理原則
1. **保持結構清晰**：每個目錄都有明確的職責範圍
2. **避免重複**：相同主題的文檔應集中管理
3. **正確分類**：根據文檔性質選擇合適的目錄
4. **更新引用**：移動文檔後必須更新所有交叉引用
5. **定期清理**：刪除過時或重複的文檔

### 相關文檔
- **📚 LAYERED-IMPORT-GUIDE.md**：AI Agent 三層載入架構指南
  - 定義文檔的邏輯分層（通用層、工作空間層、專業層）
  - 提供引用統計和權重評分
  - 與本文檔互補：本文檔關注物理結構，LAYERED-IMPORT-GUIDE 關注邏輯載入

### Claude AI 使用說明
- **STRUCTURE.md 角色**：人類維護者的導航地圖（物理結構視角）
- **LAYERED-IMPORT-GUIDE.md 角色**：AI Agent 載入策略指南（邏輯分層視角）
- **docs-ai/ 引用**：Claude 透過 CLAUDE.md 中的 docs-ai/ 引用載入文檔
- **文檔維護**：定期檢查引用完整性，確保 Claude 能存取必要文檔

### 檔案命名規範
- 使用 kebab-case：`example-document.md`
- 描述性名稱：清楚表達文檔內容
- 避免過長：保持簡潔但資訊充足

---
*最後更新：2025-09-19*
*文檔總數：68個 Markdown 檔案*