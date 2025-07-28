# agv_ws - AGV 核心控制系統

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/development/docker-development.md

## 📋 模組概述
AGV核心控制系統，採用3層狀態機架構：Base層(通用邏輯) → AGV層(車型特定) → Robot層(機械臂任務)

## 🏗️ 專案結構
```
src/
├── agv_base/           # 通用狀態機與核心邏輯
├── agv_interfaces/     # ROS 2訊息與服務介面定義
├── cargo_mover_agv/    # Cargo Mover AGV實作(Hokuyo 8bit光通訊模組)
├── loader_agv/         # Loader AGV實作(完整測試套件)
└── unloader_agv/       # Unloader AGV實作
```

### 3層狀態機設計
@docs-ai/context/workspaces/agv-workspaces.md

## 🔧 開發環境
@docs-ai/operations/development/docker-development.md

## 📋 車型開發
@docs-ai/knowledge/agv-domain/vehicle-types.md

## ⚠️ 重要開發規範

### Robot PGNO 參數順序規則
@docs-ai/knowledge/agv-domain/robot-pgno-rules.md

### 狀態轉換開發
```python
# 狀態轉換必須包含完整驗證和日誌
def transition_to_executing(self):
    if self.validate_preconditions():
        self.log_state_change("WAITING", "EXECUTING")
        return ExecutingState()
    return self
```

## 🚀 快速測試

### 基本測試
```bash
# 進入AGV容器並執行測試
agv_enter
cd /app/agv_ws

# 構建和測試
build_ws agv_ws
colcon test --packages-select agv_base loader_agv cargo_mover_agv unloader_agv
```

### 車型特定測試
```bash
# Loader AGV測試
cd src/loader_agv && python3 -m pytest test/ -v

# Cargo Mover AGV測試  
cd src/cargo_mover_agv && python3 -m pytest test/ -v

# Unloader AGV測試
cd src/unloader_agv && python3 -m pytest test/ -v
```

## 🚨 故障排除
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md

## 💡 開發最佳實踐  
@docs-ai/operations/development/core-principles.md
@docs-ai/operations/tools/unified-tools.md