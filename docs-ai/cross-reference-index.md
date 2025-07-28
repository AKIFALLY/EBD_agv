# Prompts 交叉引用索引

## 🎯 用途
提供 prompts 文件間的交叉引用索引，確保引用的一致性和準確性，方便 AI Agent 快速定位相關資訊。

## 📋 引用索引

### Context 文件系列

#### 系統層級 Context
- @docs-ai/context/system/rosagv-overview.md
  - **被引用於**: 根目錄 CLAUDE.md, 新手導入場景
  - **引用其他**: dual-environment.md, technology-stack.md
  - **關鍵詞**: 專案概覽, 系統架構, 業務價值

- @docs-ai/context/system/dual-environment.md
  - **被引用於**: 跨環境開發, 部署相關任務
  - **引用其他**: technology-stack.md, agv-workspaces.md, agvc-workspaces.md
  - **關鍵詞**: AGV車載, AGVC管理, 容器架構

- @docs-ai/context/system/technology-stack.md
  - **被引用於**: 技術決策, 架構設計, 相容性問題
  - **引用其他**: dual-environment.md, ros2-development.md
  - **關鍵詞**: ROS 2 Jazzy, Zenoh RMW, Docker, Python 3.12

#### 工作空間層級 Context
- @docs-ai/context/workspaces/agv-workspaces.md
  - **被引用於**: AGV 相關開發任務
  - **引用其他**: agv-control-logic.md, vehicle-types.md, ros2-development.md
  - **關鍵詞**: 車載系統, 狀態機, 9個工作空間

- @docs-ai/context/workspaces/agvc-workspaces.md
  - **被引用於**: AGVC 相關開發任務
  - **引用其他**: fleet-management.md, web-development.md, database-operations.md
  - **關鍵詞**: 管理系統, Web API, 11個工作空間

#### 業務邏輯 Context
- @docs-ai/context/business/agv-control-logic.md
  - **被引用於**: AGV 控制開發, 狀態機設計
  - **引用其他**: vehicle-types.md, agv-workspaces.md
  - **關鍵詞**: 3層狀態機, 車型特化, 事件驅動

- @docs-ai/context/business/fleet-management.md
  - **被引用於**: 車隊管理功能開發
  - **引用其他**: agvc-workspaces.md, external-integration.md
  - **關鍵詞**: 任務調度, 車隊協調, 資源分配

### Operations 文件系列

#### 開發操作
- @docs-ai/operations/development/ros2-development.md
  - **被引用於**: ROS 2 節點開發, 工作空間管理
  - **引用其他**: agv-workspaces.md, agvc-workspaces.md, testing-procedures.md
  - **關鍵詞**: 節點開發, 建置管理, 最佳實踐

- @docs-ai/operations/development/docker-development.md
  - **被引用於**: 容器內開發, 環境配置
  - **引用其他**: dual-environment.md, container-management.md
  - **關鍵詞**: 容器開發, 環境載入, 工具使用

- @docs-ai/operations/development/web-development.md
  - **被引用於**: Web API 開發, 前端整合
  - **引用其他**: agvc-workspaces.md, database-operations.md
  - **關鍵詞**: FastAPI, Socket.IO, AGVCUI, OPUI

#### 維護操作
- @docs-ai/operations/maintenance/system-diagnostics.md
  - **被引用於**: 系統健康檢查, 故障排除
  - **引用其他**: troubleshooting.md, container-management.md, zenoh-rmw.md
  - **關鍵詞**: 統一工具, 診斷流程, 效能監控

- @docs-ai/operations/maintenance/troubleshooting.md
  - **被引用於**: 故障排除, 問題診斷
  - **引用其他**: system-diagnostics.md, log-analysis.md
  - **關鍵詞**: 故障流程, 決策樹, 解決方案

#### 部署操作
- @docs-ai/operations/deployment/container-management.md
  - **被引用於**: 容器管理, 部署操作
  - **引用其他**: dual-environment.md, system-diagnostics.md
  - **關鍵詞**: Docker Compose, 容器生命週期, 網路配置

### Knowledge 文件系列

#### AGV 領域知識
- @docs-ai/knowledge/agv-domain/vehicle-types.md
  - **被引用於**: 車型開發, 應用場景選擇
  - **引用其他**: agv-control-logic.md, navigation-systems.md
  - **關鍵詞**: Cargo Mover, Loader, Unloader, 機械臂

- @docs-ai/knowledge/agv-domain/navigation-systems.md
  - **被引用於**: 導航開發, 路徑規劃
  - **引用其他**: vehicle-types.md, safety-protocols.md
  - **關鍵詞**: A*演算法, 避障, 定位精度

#### 自動化知識
- @docs-ai/knowledge/automation/fleet-coordination.md
  - **被引用於**: 車隊協調, 多車管理
  - **引用其他**: fleet-management.md, control-systems.md
  - **關鍵詞**: 車隊協調, 任務分配, 衝突解決

#### 協定知識
- @docs-ai/knowledge/protocols/zenoh-rmw.md
  - **被引用於**: 通訊問題, 跨環境整合
  - **引用其他**: dual-environment.md, ros2-development.md
  - **關鍵詞**: Zenoh Router, 跨容器通訊, QoS配置

## 🔄 引用關係圖

### 核心引用路徑
```
根目錄 CLAUDE.md
├── Context Loading
│   ├── rosagv-overview.md
│   ├── dual-environment.md
│   └── technology-stack.md
├── 開發指導
│   ├── ros2-development.md
│   └── docker-development.md
├── 維護支援
│   ├── system-diagnostics.md
│   └── troubleshooting.md
└── 領域知識
    ├── vehicle-types.md
    └── zenoh-rmw.md
```

### 工作空間引用路徑
```
AGV 工作空間 CLAUDE.md
├── Context Loading
│   ├── agv-workspaces.md
│   ├── agv-control-logic.md
│   └── vehicle-types.md
├── 開發指導
│   ├── ros2-development.md
│   └── testing-procedures.md
└── 維護支援
    └── system-diagnostics.md

AGVC 工作空間 CLAUDE.md
├── Context Loading
│   ├── agvc-workspaces.md
│   ├── fleet-management.md
│   └── fleet-coordination.md
├── 開發指導
│   ├── web-development.md
│   └── database-operations.md
└── 維護支援
    └── system-diagnostics.md
```

## 📋 引用驗證清單

### 新增文件時
- [ ] 在此索引中註冊新文件
- [ ] 確定被引用的場景
- [ ] 列出引用的其他文件
- [ ] 定義關鍵詞標籤
- [ ] 更新相關的引用關係

### 更新文件時
- [ ] 檢查引用路徑是否仍然有效
- [ ] 更新引用關係描述
- [ ] 驗證交叉引用的準確性
- [ ] 確保關鍵詞標籤最新
- [ ] 測試引用的可達性

### 刪除文件時
- [ ] 檢查所有引用此文件的位置
- [ ] 更新或移除相關引用
- [ ] 從索引中移除條目
- [ ] 通知相關文件維護者
- [ ] 提供替代引用建議

## 🔧 引用最佳實踐

### 引用選擇原則
1. **相關性**: 引用與當前內容高度相關的文件
2. **層次性**: 遵循 Context → Operations → Knowledge 的層次
3. **完整性**: 提供足夠的背景知識和操作指導
4. **避免循環**: 防止文件間的循環引用

### 引用格式規範
```markdown
# 正確的引用格式
@docs-ai/category/subcategory/filename.md

# 引用說明格式
- 詳細說明: @docs-ai/path/to/file.md
- 相關參考: @docs-ai/path/to/related.md
```

### 引用維護策略
1. **定期檢查**: 每月檢查引用的有效性
2. **自動化驗證**: 使用腳本驗證引用路徑
3. **版本控制**: 將引用變更納入版本控制
4. **文檔同步**: 確保引用與實際文件同步

## 🔍 快速查找

### 按功能領域查找
- **AGV 開發**: agv-workspaces.md, vehicle-types.md, ros2-development.md
- **AGVC 開發**: agvc-workspaces.md, web-development.md, database-operations.md
- **系統維護**: system-diagnostics.md, troubleshooting.md, container-management.md
- **通訊整合**: zenoh-rmw.md, dual-environment.md, external-integration.md

### 按問題類型查找
- **環境問題**: dual-environment.md, container-management.md
- **開發問題**: ros2-development.md, testing-procedures.md
- **效能問題**: system-diagnostics.md, performance-monitoring.md
- **整合問題**: external-integration.md, api-integration.md

### 按技術棧查找
- **ROS 2**: ros2-development.md, ros2-interfaces.md
- **Docker**: container-management.md, docker-development.md
- **Web**: web-development.md, web-protocols.md
- **資料庫**: database-operations.md, database-schemas.md

## 🔗 維護聯絡
如發現引用錯誤或需要更新索引，請：
1. 檢查引用的實際可達性
2. 驗證引用內容的相關性
3. 更新此索引文件
4. 通知相關文件維護者
5. 測試更新後的引用效果
