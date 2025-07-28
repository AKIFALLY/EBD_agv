# ECS 設備控制系統 CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agvc-workspaces.md
@docs-ai/knowledge/protocols/zenoh-rmw.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/log-analysis.md

## 概述
設備控制系統(Equipment Control System)，專注於PLC數據收集、門控制和設備信號管理，為AGVC系統提供核心的工業設備控制功能

## 關鍵特色
- **PLC數據管理**: 0.1秒週期讀取PLC數據，智能信號變更檢測
- **門控制系統**: 支援同步/異步門控制，批次操作
- **MQTT整合**: 與外部系統(KUKA ECS)無縫整合
- **資料庫同步**: 實時更新設備信號值到PostgreSQL

## 快速開始

詳細容器開發指導請參考: @docs-ai/operations/development/docker-development.md

```bash
# 進入AGVC容器並啟動ECS
agvc_enter                          # 自動載入環境
ros2 run ecs ecs_core              # 啟動PLC數據採集
ros2 run ecs door_controller_node_mqtt  # 啟動MQTT門控制器
```

## 詳細指導
架構設計和開發指導請參考: @docs-ai/context/workspaces/agvc-workspaces.md
PLC通訊實現請參考: @docs-ai/knowledge/protocols/zenoh-rmw.md

## 故障排除

詳細故障排除指導請參考: 
- @docs-ai/operations/maintenance/troubleshooting.md - 故障排除流程
- @docs-ai/operations/maintenance/system-diagnostics.md - 系統診斷工具
- @docs-ai/operations/tools/unified-tools.md - 統一工具系統
- @docs-ai/operations/maintenance/log-analysis.md - 日誌分析方法

### ECS 特定問題檢查
```bash
# 門控制異常檢查
curl http://localhost:8000/door/status/1    # 測試門狀態API
ros2 topic echo /ecs/door_status            # 檢查門狀態主題
ros2 service list | grep door               # 檢查門控制服務
```