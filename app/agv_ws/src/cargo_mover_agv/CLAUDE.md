# cargo_mover_agv - 貨物搬運車AGV控制系統

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md

## 📋 專案概述
cargo_mover_agv 實現 Cargo Mover AGV 的完整控制邏輯，支援 Hokuyo 8bit 光通訊模組管理、架台搬運操作、入口/出口流程控制等。

詳細 Cargo Mover 開發指導請參考: @docs-ai/knowledge/agv-domain/vehicle-types.md

## 📂 關鍵檔案位置

### 核心控制
```
cargo_mover_agv/
├─ agv_core_node.py          # Cargo AGV 核心控制節點
├─ cargo_context.py          # Cargo 狀態管理上下文 (AGV層)
├─ robot_context.py          # 機器人狀態控制 (Robot層)
└─ launch/launch.py          # ROS 2 啟動配置
```

### 完整測試套件
```
test/
├─ FINAL_TEST_REPORT.md                          # 完整測試報告
├─ async_update_task_analysis_report.md          # 非同步任務分析報告
├─ test_idle_state_hokuyo.py                     # Idle狀態Hokuyo測試
├─ test_complete_state_delayed_reset.py          # 延遲重置測試
├─ test_hokuyo_busy_states.py                    # Hokuyo忙碌狀態測試
└─ test_wait_rotation_async_update_task.py       # 等待旋轉非同步測試
```

## 🔧 開發環境
@docs-ai/operations/development/docker-development.md

## 🚀 測試執行

### 完整測試套件
```bash
# 進入 cargo_mover_agv 目錄
cd /app/agv_ws/src/cargo_mover_agv

# 執行所有測試
python3 -m pytest test/ -v

# 查看測試報告
cat test/FINAL_TEST_REPORT.md
cat test/async_update_task_analysis_report.md
```

### 專項測試
```bash
# Hokuyo 設備測試
python3 -m pytest test/test_idle_state_hokuyo.py -v              # 初始化測試
python3 -m pytest test/test_hokuyo_busy_states.py -v             # 忙碌狀態測試

# 非同步任務測試
python3 -m pytest test/test_wait_rotation_async_update_task.py -v # 非同步任務測試

# 延遲重置測試
python3 -m pytest test/test_complete_state_delayed_reset.py -v   # 延遲重置測試
```

## 📊 配置設定

### AGV 配置檔案
- `/app/config/agv/cargo01_config.yaml` - Cargo01 配置
- `/app/config/agv/cargo02_config.yaml` - Cargo02 配置

### 關鍵配置參數
```yaml
agv_id: "cargo01"
agv_type: "cargo"

hokuyo_devices:
  hokuyo_1: {ip: "192.168.1.101", port: 8000}
  hokuyo_2: {ip: "192.168.1.102", port: 8000}

rotation_params:
  wait_timeout_ms: 30000      # 等待旋轉逾時
  async_task_interval_ms: 500 # 非同步任務間隔
```

## 🔗 系統整合

### ROS 2 通訊
```bash
# 發布話題
/<agv_id>/status              # Cargo AGV 狀態
/<agv_id>/hokuyo_status       # Hokuyo 設備狀態
/<agv_id>/vision_result       # 視覺定位結果

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/rack_status           # 架台狀態
```

### 外部整合
- **agv_base**: 繼承 3層狀態機架構，使用 Hokuyo DMS 8-bit 光通訊模組
- **plc_proxy_ws**: 架台和傳送箱 PLC 控制
- **sensorpart_ws**: 視覺定位和 Hokuyo 8bit 光通訊模組整合

## 🚨 故障排除
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/knowledge/agv-domain/vehicle-types.md