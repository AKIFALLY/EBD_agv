# agv_ws - AGV 核心控制系統工作空間

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄通用層知識（系統架構、核心原則、通用工具）

## 🔧 工作空間層文檔（第二層）
# AGV 領域專業知識
@docs-ai/knowledge/agv-domain/agv-state-machine.md  # AGV 狀態機設計
@docs-ai/knowledge/agv-domain/vehicle-types.md      # 車型定義與規格
@docs-ai/knowledge/system/manual-rack-management.md # Rack 管理技術
@docs-ai/operations/guides/rack-management-guide.md # Rack 操作指南

# 工作空間通用文檔
@docs-ai/context/workspaces/agv-workspaces.md      # AGV 工作空間架構
@docs-ai/knowledge/protocols/ros2-interfaces.md     # ROS2 介面規範
@docs-ai/knowledge/protocols/zenoh-rmw.md          # Zenoh 通訊協議
@docs-ai/operations/development/ros2/ros2-development.md # ROS2 開發流程

## 📋 工作空間概述

**AGV 核心控制系統工作空間** 專注於實現 3層狀態機架構的 AGV 控制邏輯：Base層(通用邏輯) → AGV層(車型特定) → Robot層(機械臂任務)。

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

## 🚀 AGV 核心系統專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

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

## 🚨 AGV 核心系統專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

## 🔗 交叉引用

### 車型實作
- **Cargo Mover AGV**: `src/cargo_mover_agv/CLAUDE.md`
- **Loader AGV**: `src/loader_agv/CLAUDE.md`
- **Unloader AGV**: `src/unloader_agv/CLAUDE.md`

### 專業指導
- **車型特性**: @docs-ai/knowledge/agv-domain/vehicle-types.md

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節