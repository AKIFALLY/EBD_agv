# loader_agv - 裝載車AGV控制系統

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md

## 📋 專案概述
loader_agv 實現 Loader AGV 的完整控制邏輯，支援從傳送箱取料、多工位操作（清潔機、浸潤機、預乾燥機）、AGV端口管理等複雜載料流程。

詳細 Loader AGV 開發指導請參考: @docs-ai/knowledge/agv-domain/vehicle-types.md

## 📂 關鍵檔案位置

### 核心控制
```
loader_agv/
├─ agv_core_node.py          # Loader AGV 核心控制節點
├─ loader_context.py         # Loader 狀態管理上下文 (AGV層)
├─ robot_context.py          # 機器人狀態控制 (Robot層)
└─ launch/launch.py          # ROS 2 啟動配置
```

### 完整測試套件
```
test/
├─ TEST_REPORT.md                          # 詳細測試報告
├─ conftest.py                             # 測試配置和fixtures
├─ run_tests.py                            # 測試運行器
├─ test_take_transfer_integration.py       # Take Transfer完整流程測試
├─ test_agv_port_check_empty_state.py      # AGV端口檢查測試
├─ test_transfer_check_have_state.py       # 傳送箱載具檢查測試
├─ test_transfer_vision_position_state.py  # 傳送箱視覺定位測試
├─ test_take_transfer_state.py             # 取料轉移測試
└─ test_put_agv_state.py                   # AGV放料測試
```

## 🔧 開發環境
@docs-ai/operations/development/docker-development.md

## 🚀 測試執行

### 完整測試套件
```bash
# 進入 loader_agv 目錄
cd /app/agv_ws/src/loader_agv

# 執行完整測試套件
python3 test/run_tests.py

# 執行特定功能測試
python3 -m pytest test/test_take_transfer_integration.py -v      # 完整載料流程
python3 -m pytest test/test_agv_port_check_empty_state.py -v     # AGV端口檢查
python3 -m pytest test/test_transfer_vision_position_state.py -v # 視覺定位

# 查看測試報告
cat test/TEST_REPORT.md
```

### Demo 測試
```bash
# 快速Demo測試
python3 test/test_demo.py

# 簡化測試運行器
python3 test/simple_test_runner.py
```

## 📊 配置設定

### AGV 配置檔案
- `/app/config/agv/loader01_config.yaml` - Loader01 配置
- `/app/config/agv/loader02_config.yaml` - Loader02 配置

### 關鍵配置參數
```yaml
agv_id: "loader01"
agv_type: "loader"
robot_arm_enabled: true
vision_system_enabled: true

# 端口配置
agv_ports:
  port_1: {x: 0.5, y: 0.3, z: 0.1}
  port_2: {x: 0.5, y: -0.3, z: 0.1}

# 工位配置
stations:
  transfer_box: {x: 5.0, y: 2.0, approach_angle: 0.0}
  cleaner: {x: 8.0, y: 1.5, approach_angle: 1.57}
  soaker: {x: 11.0, y: 1.5, approach_angle: 1.57}
  pre_dryer: {x: 14.0, y: 1.5, approach_angle: 1.57}
```

## 🔗 系統整合

### ROS 2 通訊
```bash
# 發布話題
/<agv_id>/status              # Loader AGV 狀態
/<agv_id>/robot_state         # 機器人狀態
/<agv_id>/vision_result       # 視覺定位結果
/<agv_id>/loading_status      # 載料過程狀態

# 訂閱話題
/<agv_id>/cmd                 # 任務指令
/system/stations_status       # 工位狀態
```

### 外部整合
- **agv_base**: 繼承 3層狀態機架構
- **plc_proxy_ws**: 機器人手臂 PLC 控制
- **sensorpart_ws**: 視覺定位和感測器整合

## 🚨 故障排除
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/knowledge/agv-domain/vehicle-types.md