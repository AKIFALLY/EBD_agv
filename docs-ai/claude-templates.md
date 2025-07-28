# CLAUDE.md 引用模板

## 🎯 模板用途
為不同層級的 CLAUDE.md 提供標準引用模板，確保 AI Agent 記憶系統的一致性和完整性。

## 📋 模板分類

### 根目錄 CLAUDE.md 模板
適用於專案根目錄的主要 CLAUDE.md 文件。

```markdown
# [專案名稱] CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md

## 系統概述
簡潔的專案描述，突出核心價值和主要特性...

## 核心架構
關鍵架構要點和設計理念...

## 開發指導
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/deployment/environment-setup.md

## 維護支援
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md

## 領域知識
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/knowledge/automation/fleet-coordination.md

## AI 開發助手指導
核心開發原則和最佳實踐...

## 快速開始
基本操作指令和入門指導...

## 模組文檔索引
當涉及以下功能領域時，請讀取對應的詳細 CLAUDE.md：
- **功能領域 1**: `path/to/module1/CLAUDE.md`
- **功能領域 2**: `path/to/module2/CLAUDE.md`
```

### 工作空間層級 CLAUDE.md 模板
適用於各個 ROS 2 工作空間的 CLAUDE.md 文件。

```markdown
# [工作空間名稱] CLAUDE.md

## 📚 Context Loading
@docs-ai/context/workspaces/[agv|agvc]-workspaces.md
@docs-ai/context/business/[相關業務邏輯].md
@docs-ai/knowledge/[相關領域]/[相關知識].md

## 工作空間概述
工作空間的功能、職責和在整體系統中的角色...

## 核心功能
- 主要功能點 1
- 主要功能點 2
- 主要功能點 3

## 開發指導
@docs-ai/operations/development/[相關技術]-development.md
@docs-ai/operations/development/testing-procedures.md

## 維護支援
@docs-ai/operations/maintenance/[相關維護].md

## 快速開始
```bash
# 環境載入
all_source              # 智能載入工作空間

# 基本操作
colcon build --packages-select [package_name]
ros2 launch [package_name] [launch_file]
```

## 套件結構
```
[workspace_name]/src/
├── package1/           # 套件1功能描述
├── package2/           # 套件2功能描述
└── package3/           # 套件3功能描述
```

## 依賴關係
- **上游依賴**: 依賴的其他工作空間或套件
- **下游依賴**: 依賴此工作空間的其他模組
- **外部依賴**: 第三方庫和系統依賴

## 配置管理
關鍵配置檔案和參數說明...

## 故障排除
常見問題和解決方案...

## 相關文檔
- 詳細技術文檔連結
- API 參考文檔
- 使用範例和教學
```

### 套件層級 CLAUDE.md 模板
適用於具體 ROS 2 套件的 CLAUDE.md 文件。

```markdown
# [套件名稱] CLAUDE.md

## 📚 Context Loading
@docs-ai/context/workspaces/[相關工作空間].md
@docs-ai/knowledge/[相關領域]/[相關知識].md

## 套件概述
套件的具體功能、用途和設計目標...

## 核心組件
- **節點 1**: 功能描述
- **節點 2**: 功能描述
- **服務**: 提供的服務介面
- **主題**: 發布和訂閱的主題

## 開發指導
具體的開發指導和程式碼範例...

## API 參考
### 節點介面
```python
# 主要節點類別
class MainNode(Node):
    def __init__(self):
        # 初始化邏輯
        pass
```

### 訊息定義
```
# 自定義訊息格式
CustomMessage.msg
├── field1: type1
├── field2: type2
└── field3: type3
```

### 服務定義
```
# 服務介面
CustomService.srv
├── Request: 請求格式
└── Response: 回應格式
```

## 配置參數
| 參數名稱 | 類型 | 預設值 | 描述 |
|---------|------|--------|------|
| param1 | string | "default" | 參數1描述 |
| param2 | int | 10 | 參數2描述 |

## 使用範例
```bash
# 啟動節點
ros2 run [package_name] [node_name]

# 設定參數
ros2 param set /[node_name] [param_name] [value]

# 呼叫服務
ros2 service call /[service_name] [service_type] "[request_data]"
```

## 故障排除
### 常見問題
1. **問題描述**: 解決方案
2. **問題描述**: 解決方案

### 除錯指令
```bash
# 檢查節點狀態
ros2 node info /[node_name]

# 查看主題資料
ros2 topic echo /[topic_name]

# 檢查服務可用性
ros2 service list | grep [service_name]
```

## 測試
```bash
# 執行單元測試
colcon test --packages-select [package_name]

# 執行整合測試
ros2 launch [package_name] test_launch.py
```

## 相關套件
- **上游套件**: 依賴的其他套件
- **下游套件**: 依賴此套件的其他套件
- **相關套件**: 功能相關的套件
```

## 🔧 模板使用指導

### 選擇適當模板
1. **根目錄**: 使用根目錄模板，提供整體系統概覽
2. **工作空間**: 使用工作空間模板，專注於特定功能領域
3. **套件**: 使用套件模板，提供具體實作細節

### 自定義原則
1. **保持引用結構**: 始終保持 @docs-ai/ 引用的結構
2. **適應具體需求**: 根據實際功能調整內容
3. **保持簡潔**: CLAUDE.md 應該簡潔，詳細內容透過引用載入
4. **定期更新**: 隨著功能變更更新引用和內容

### 引用選擇指導
```
根據功能領域選擇引用：

AGV 車載開發 →
├── @docs-ai/context/workspaces/agv-workspaces.md
├── @docs-ai/knowledge/agv-domain/vehicle-types.md
└── @docs-ai/operations/development/ros2-development.md

AGVC 管理開發 →
├── @docs-ai/context/workspaces/agvc-workspaces.md
├── @docs-ai/knowledge/automation/fleet-coordination.md
└── @docs-ai/operations/development/web-development.md

系統維護 →
├── @docs-ai/context/system/dual-environment.md
├── @docs-ai/operations/maintenance/system-diagnostics.md
└── @docs-ai/operations/maintenance/troubleshooting.md
```

## 📋 檢查清單

### 建立新 CLAUDE.md 時
- [ ] 選擇適當的模板
- [ ] 載入相關的 Context 文件
- [ ] 引用適當的 Operations 指導
- [ ] 包含相關的 Knowledge 文件
- [ ] 提供簡潔的概述
- [ ] 包含快速開始指導

### 更新現有 CLAUDE.md 時
- [ ] 檢查引用的有效性
- [ ] 更新過時的內容
- [ ] 確保與實際程式碼同步
- [ ] 驗證交叉引用的正確性
- [ ] 測試引用的可達性

### 品質檢查
- [ ] 引用路徑正確
- [ ] 內容簡潔明瞭
- [ ] 結構清晰一致
- [ ] 避免內容重複
- [ ] 與其他文件協調

## 🔗 相關文檔
- Prompts 系統概覽: `docs-ai/README.md`
- 使用指南: `docs-ai/USAGE_GUIDE.md`
- AI 學習指南: `docs-ai/AI_LEARNING_GUIDE.md`
- 快速參考: `docs-ai/QUICK_REFERENCE.md`
