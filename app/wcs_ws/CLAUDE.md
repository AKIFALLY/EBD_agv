# wcs_ws - WCS 系統工作空間

## 📚 Context Loading
../CLAUDE.md  # 引用根目錄通用層知識（系統架構、核心原則、通用工具）

## 🔧 工作空間層文檔（第二層）
@docs-ai/context/workspaces/agvc-workspaces.md   # AGVC 工作空間架構
@docs-ai/knowledge/protocols/ros2-interfaces.md    # ROS2 介面規範
@docs-ai/knowledge/protocols/zenoh-rmw.md         # Zenoh 通訊協議
@docs-ai/operations/development/ros2/ros2-development.md # ROS2 開發流程

## 📋 工作空間概述

**WCS 系統工作空間** 提供倉儲控制系統（Warehouse Control System）的核心功能，包括 PLC 監控、自動任務建立等功能。

## 🏗️ 專案結構
```
src/
└── alan_room_task_build/   # PLC DM 監控與自動任務建立
```

## 📦 套件說明

### alan_room_task_build
**功能**：監控 PLC DM2500-2509 範圍，根據讀取的 work_id 自動建立 Task

**特性**：
- 每 1 秒監控 PLC DM2500-2509（10 words）
- Loader AGV: DM2500-2501（32-bit work_id）
- Unloader AGV: DM2502-2503（32-bit work_id）
- 自動從 Work 資料表建立 Task
- 避免重複建立（檢查未完成的 Task）
- 建立完成後清除對應的 DM

**資料庫操作**：
- 使用 SQLModel + ConnectionPoolManager + BaseCRUD
- 不依賴 TAFL 系統
- 直接操作 PostgreSQL

## 🚀 快速開始

### 建置套件
```bash
# 進入 AGVC 容器
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 建置
cd /app/wcs_ws
colcon build --packages-select alan_room_task_build

# 載入環境
source install/setup.bash
```

### 執行節點
```bash
# 啟動房間任務建立節點
ros2 run alan_room_task_build room_task_build_node
```

### 測試
```bash
# 執行測試
colcon test --packages-select alan_room_task_build
colcon test-result --verbose
```

## 🔗 相關文檔
- **PLC 通訊**: ../plc_proxy_ws/CLAUDE.md
- **資料庫操作**: ../db_proxy_ws/CLAUDE.md
- **通用指導**: ../CLAUDE.md
