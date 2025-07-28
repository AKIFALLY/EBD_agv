# agv_base - AGV基礎核心框架

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md

## 專案概述
agv_base是RosAGV系統的核心基礎框架，提供AGV狀態機的抽象基類和通用邏輯。實現3層架構的基礎層(Base層)，為所有AGV車型提供統一的狀態管理、事件處理和硬體控制介面。

## 🏗️ 3層狀態機架構
@docs-ai/context/workspaces/agv-workspaces.md

### 關鍵檔案位置
```
app/agv_ws/src/agv_base/
├─ agv_base/agv_node_base.py      # AGV節點基類 (50ms主循環)
├─ agv_base/base_context.py       # 狀態機上下文管理
├─ agv_base/agv_states/           # Base層狀態定義
├─ agv_base/context_abc.py        # Context抽象基類
├─ agv_base/event.py              # 事件處理機制
└─ agv_base/robot.py              # 機器人硬體抽象層
```

## 🔧 開發環境
@docs-ai/operations/development/docker-development.md

## 📋 開發指導
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/knowledge/agv-domain/robot-pgno-rules.md

### 測試執行
```bash
# 進入AGV容器並執行測試
agv_enter
cd /app/agv_ws/src/agv_base
python3 -m pytest test/
```

## 📊 配置文件
- `/app/config/agv/base_config.yaml` - AGV基礎參數配置
- `/app/config/hardware_mapping.yaml` - 硬體設備映射

## 🔗 整合點
- **車型專案** (`cargo_mover_agv`, `loader_agv`, `unloader_agv`): 繼承BaseContext和AgvNodebase
- **plc_proxy_ws**: 透過PlcClient進行PLC通訊
- **agv_interfaces**: 使用AgvStatus和AgvStateChange訊息

## 🚨 故障排除
@docs-ai/operations/maintenance/system-diagnostics.md