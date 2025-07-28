# 語言配置

## 🎯 適用場景
- AI Agent 與用戶的互動語言設定
- 代碼註釋和文檔的語言規範
- 系統介面和錯誤訊息的語言統一

## 📋 語言配置規範

### CLI 互動語言
- **主要語言**: 繁體中文
- **適用範圍**: 
  - AI Agent 回應和說明
  - 系統狀態報告
  - 錯誤訊息和警告
  - 操作指導和建議

### 代碼註釋語言
- **註釋語言**: 繁體中文
- **適用範圍**:
  - Python 代碼註釋
  - ROS 2 節點說明
  - 配置檔案註釋
  - 腳本工具說明

### 文檔語言
- **技術文檔**: 繁體中文
- **API 文檔**: 英文 + 繁體中文註解
- **README**: 繁體中文為主
- **CLAUDE.md**: 繁體中文

### 變數和函數命名
- **變數名稱**: 英文 (snake_case)
- **函數名稱**: 英文 (snake_case) 
- **類別名稱**: 英文 (PascalCase)
- **檔案名稱**: 英文 (snake_case.py/.md)

## 🔧 實施指導

### AI Agent 互動
- 使用繁體中文進行技術說明
- 保持專業術語的一致性
- 錯誤訊息提供中文說明和英文參考

### 代碼開發
- 代碼邏輯註釋使用繁體中文
- 保留英文的技術關鍵字和 API 名稱
- 文檔字串 (docstring) 使用繁體中文

### 文檔撰寫
- 技術概念使用繁體中文解釋
- 保留原始英文術語並提供中文對照
- 範例代碼註釋使用繁體中文

## 📋 術語對照

### 常用技術術語
- **Container** → 容器
- **Workspace** → 工作空間  
- **Node** → 節點
- **Topic** → 主題
- **Service** → 服務
- **AGV** → 自動導引車
- **AGVC** → 車隊控制系統
- **State Machine** → 狀態機
- **Robot** → 機器人/機械臂

### ROS 2 專用術語
- **Publisher** → 發布者
- **Subscriber** → 訂閱者
- **Launch** → 啟動
- **Package** → 套件
- **Build** → 建置
- **Colcon** → Colcon (保留原名)

## 🔗 交叉引用
- 系統概覽: @docs-ai/context/system/rosagv-overview.md
- 開發規範: @docs-ai/operations/development/core-principles.md
- 文檔標準: @docs-ai/context/structure/module-index.md