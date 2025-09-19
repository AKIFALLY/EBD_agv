# ecs_ws - 設備控制系統工作空間

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档

# 設備控制領域知識（工作空間層）
@docs-ai/knowledge/business/eyewear-production-process.md  # 生產流程
@docs-ai/knowledge/protocols/keyence-plc-protocol.md       # PLC 協議

# 通用協議與介面
@docs-ai/knowledge/protocols/ros2-interfaces.md            # ROS2 介面
@docs-ai/knowledge/protocols/zenoh-rmw.md                  # Zenoh 通訊

## 📋 工作空間概述

**設備控制系統工作空間** 專注於PLC數據收集、門控制和設備信號管理，為AGVC系統提供核心的工業設備控制功能。

### ECS 工作空間特有功能
- **🔄 PLC數據管理**: 0.1秒週期讀取PLC數據，智能信號變更檢測
- **🚪 門控制系統**: 支援同步/異步門控制，批次操作
- **📡 MQTT整合**: 與外部系統(KUKA ECS)無縫整合
- **💾 資料庫同步**: 實時更新設備信號值到PostgreSQL

## 🚀 ECS 專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

### ECS 特定啟動
```bash
# 【推薦方式】透過根目錄統一工具
# 參考: ../../CLAUDE.md 開發指導

# 【直接啟動】ECS 服務
cd /app/ecs_ws
build_ws ecs_ws
ros2 run ecs ecs_core                     # 啟動PLC數據採集
ros2 run ecs door_controller_node_mqtt    # 啟動MQTT門控制器
```

## 🚨 ECS 專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### ECS 特有問題檢查
```bash
# 門控制異常檢查
curl http://localhost:8000/door/status/1    # 測試門狀態API
ros2 topic echo /ecs/door_status            # 檢查門狀態主題
ros2 service list | grep door               # 檢查門控制服務

# PLC 數據採集檢查
ros2 topic echo /ecs/plc_data              # 檢查PLC數據主題
ros2 node info /ecs_core                   # 檢查ECS核心節點狀態
```

## 🔗 交叉引用

### 業務流程整合
- **眼鏡生產流程**: @docs-ai/knowledge/business/eyewear-production-process.md

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節