# unloader_agv - 卸載車AGV控制系統

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md

## 📋 專案概述
unloader_agv 實現 Unloader AGV 的完整控制邏輯，支援從烘箱取料、預乾燥機操作、出料傳送帶操作、數量計算等複雜卸載流程。

詳細 Unloader AGV 開發指導請參考: @docs-ai/knowledge/agv-domain/vehicle-types.md

## 📂 關鍵檔案位置

### 核心控制
```
unloader_agv/
├─ agv_core_node.py          # Unloader AGV 核心控制節點
├─ unloader_context.py       # Unloader 狀態管理上下文 (AGV層)
├─ robot_context.py          # 機器人狀態控制 (Robot層)
└─ launch/launch.py          # ROS 2 啟動配置
```

### 測試套件
```
test/
├─ README.md                           # 測試說明文檔
├─ test_pre_dryer_calculation.py       # 預乾燥機計算邏輯測試
└─ test_take_quantity.py               # 取料數量計算測試
```

## 🔧 開發環境
@docs-ai/operations/development/docker-development.md

## 🚀 測試執行

### 基礎測試套件
```bash
# 進入 unloader_agv 目錄
cd /app/agv_ws/src/unloader_agv

# 執行所有測試
python3 -m pytest test/ -v

# 執行特定功能測試
python3 -m pytest test/test_pre_dryer_calculation.py -v    # 預乾燥機計算測試
python3 -m pytest test/test_take_quantity.py -v           # 取料數量測試

# 或使用直接執行方式
python3 test/test_pre_dryer_calculation.py
python3 test/test_take_quantity.py
```

### 測試範圍
```bash
# Pre Dryer Port 計算邏輯
# - Port 1-4 → row=1, column=0
# - Port 5-8 → row=2, column=0

# Take Quantity 功能邏輯
# - Carrier 查詢結果處理
# - 參數類型轉換和整合
# - 邊界條件和錯誤處理
```

## 📊 配置設定

### AGV 配置檔案
- `/app/config/agv/unloader01_config.yaml` - Unloader01 配置
- `/app/config/agv/unloader02_config.yaml` - Unloader02 配置

### 關鍵配置參數
```yaml
agv_id: "unloader01"
agv_type: "unloader"
robot_arm_enabled: true
vision_system_enabled: true

# 基礎工位配置
stations:
  oven: {approach_angle: 0.0}
  pre_dryer: {approach_angle: 1.57}
  boxout_transfer: {approach_angle: 3.14}

# 測試參數
test_params:
  pre_dryer_port_calculation: true   # Pre Dryer Port 計算邏輯
  take_quantity_logic: true          # Take Quantity 功能邏輯
```

## 🔗 系統整合

### ROS 2 通訊
```bash
# 發布話題
/<agv_id>/status           # Unloader AGV 狀態
/<agv_id>/robot_state      # 機器人狀態

# 訂閱話題
/<agv_id>/cmd              # 任務指令
/system/station_status     # 工位狀態
```

### 外部整合
- **agv_base**: 繼承 3層狀態機架構 (Base → AGV → Robot)
- **plc_proxy_ws**: 工位設備 PLC 控制
- **sensorpart_ws**: 視覺定位和感測器整合

## 🚨 故障排除
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/knowledge/agv-domain/vehicle-types.md