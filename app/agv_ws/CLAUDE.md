# agv_ws CLAUDE.md

## 模組概述
AGV核心控制系統，採用3層狀態機架構：Base層(通用邏輯) → AGV層(車型特定) → Robot層(機械臂任務)

## 專案結構
```
src/
├── agv_base/           # 通用狀態機與核心邏輯
├── agv_interfaces/     # ROS 2訊息與服務介面定義
├── cargo_mover_agv/    # Cargo Mover AGV實作(雙Hokuyo)
├── loader_agv/         # Loader AGV實作(完整測試套件)
└── unloader_agv/       # Unloader AGV實作
```

## 核心架構

### 3層狀態機設計
- **Base層**: `agv_base/agv_states/` - 通用狀態邏輯
- **AGV層**: `*_agv/src/states/` - 車型特定狀態實現
- **Robot層**: `*_agv/robot_context.py` - 機械臂控制邏輯

### 關鍵檔案
- `agv_base/agv_base/agv_node_base.py:23` - AGV節點基礎類別
- `agv_base/agv_states/` - 狀態機基礎實現
- `*/robot_context.py` - 機械臂上下文管理

## 🔧 開發工具指南

### 宿主機操作 (Docker 容器管理)

#### AGV 容器管理工具
```bash
# 載入 Docker 工具集
source scripts/docker-tools/docker-tools.sh

# AGV 容器基本操作
agv_start                    # 啟動 AGV 容器
agv_stop                     # 停止 AGV 容器  
agv_restart                  # 重啟 AGV 容器
agv_status                   # 查看 AGV 容器狀態
agv_logs                     # 查看 AGV 容器日誌
agv_health                   # AGV 容器健康檢查

# 快速進入 AGV 開發環境
agv_enter                    # 進入 AGV 容器 (自動載入 agv_source)

# 快速執行 AGV 容器內指令
quick_agv "build_all"        # 在 AGV 容器內執行建置
quick_agv "ros2 node list"   # 在 AGV 容器內執行 ROS 2 指令
```

#### 系統診斷工具 (宿主機執行)
```bash
# 系統健康檢查
scripts/system-tools/health-check.sh --quick     # 快速健康檢查
scripts/system-tools/service-monitor.sh status   # 服務狀態監控

# AGV 專項診斷
scripts/docker-tools/container-status.sh agv     # AGV 容器狀態詳情
scripts/log-tools/log-analyzer.sh agv --stats    # AGV 日誌分析

# 網路和通訊診斷
scripts/network-tools/zenoh-network.sh agv-check # AGV Zenoh 通訊檢查
scripts/network-tools/port-check.sh --verbose    # 端口連接檢查
```

#### 開發工作流工具 (宿主機執行)
```bash
# 載入開發工具集
source scripts/dev-tools/dev-tools.sh

# AGV 工作空間開發
dev_build --workspace agv_ws          # 建置 AGV 工作空間
dev_test --workspace agv_ws           # 測試 AGV 工作空間
dev_check --workspace agv_ws --severity warning  # 代碼品質檢查

# 完整開發流程
scripts/dev-tools/build-helper.sh fast --workspace agv_ws    # 快速建置
scripts/dev-tools/test-runner.sh unit --workspace agv_ws     # 單元測試
scripts/dev-tools/code-analyzer.sh style --workspace agv_ws  # 代碼風格檢查
```

### 容器內操作 (ROS 2 開發)

#### 環境設定 (容器內)
```bash
source /app/setup.bash
all_source  # 載入所有工作空間 (或使用 agv_source 載入 AGV 專用)
cd /app/agv_ws
```

#### 構建與測試 (容器內執行)
```bash
# 【方法1: 透過宿主機工具進入】(推薦)
# 在宿主機執行：
source scripts/docker-tools/docker-tools.sh
agv_enter  # 自動進入 AGV 容器並載入環境

# 【方法2: 手動進入容器】
docker compose -f docker-compose.yml exec rosagv bash  # AGV容器
source /app/setup.bash && all_source

# 構建工作空間 (setup.bash驗證可用)
build_ws agv_ws                    # 使用setup.bash中的函數

# 測試特定套件 (需在容器內)
cd /app/agv_ws
colcon test --packages-select agv_base loader_agv

# 測試檔案位置驗證: 實際在src/和build/目錄中
python3 src/agv_base/agv_base/test_agv_node.py
```

#### 啟動服務 (容器內執行，launch檔案驗證存在)
```bash
# 【方法1: 透過宿主機工具】(推薦)
# 在宿主機執行：
source scripts/docker-tools/docker-tools.sh
quick_agv "ros2 launch loader_agv launch.py"      # 啟動 Loader AGV
quick_agv "ros2 launch cargo_mover_agv launch.py" # 啟動 Cargo Mover AGV
quick_agv "ros2 launch unloader_agv launch.py"    # 啟動 Unloader AGV

# 【方法2: 手動進入容器】
# 必須在AGV容器內執行
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# Loader AGV (驗證存在: src/loader_agv/launch/launch.py)
ros2 launch loader_agv launch.py

# Cargo Mover AGV (驗證存在: src/cargo_mover_agv/launch/launch.py)
ros2 launch cargo_mover_agv launch.py

# Unloader AGV (驗證存在: src/unloader_agv/launch/launch.py)
ros2 launch unloader_agv launch.py
```

## 狀態開發指南

### 新增AGV狀態
1. 在`agv_base/agv_states/`創建基礎狀態類別
2. 在對應`*_agv/src/states/`實現車型特定邏輯
3. 更新狀態常數與轉換邏輯
4. 添加日誌記錄與錯誤處理

### ⚠️ 重要：Robot PGNO 參數順序規則
**根據 cargo_mover_agv 標準模式，ACTION_FROM 和 ACTION_TO 的參數順序不同：**

#### ACTION_FROM (取料動作)
```python
# 從源位置取料 - 源位置在前
TAKE_XXX_PGNO = context.robot.ACTION_FROM + \
    context.robot.SOURCE_POSITION + context.robot.NONE_POSITION

# 範例
TAKE_RACK_PGNO = context.robot.ACTION_FROM + \
    context.robot.RACK_IN_POSITION + context.robot.NONE_POSITION

TAKE_LOADER_AGV_PGNO = context.robot.ACTION_FROM + \
    context.robot.AGV_POSITION + context.robot.NONE_POSITION
```

#### ACTION_TO (放料動作)
```python
# 放到目標位置 - NONE_POSITION在前，目標位置在後
PUT_XXX_PGNO = context.robot.ACTION_TO + \
    context.robot.NONE_POSITION + context.robot.TARGET_POSITION

# 範例
PUT_RACK_PGNO = context.robot.ACTION_TO + \
    context.robot.NONE_POSITION + context.robot.RACK_OUT_POSITION

PUT_LOADER_AGV_PGNO = context.robot.ACTION_TO + \
    context.robot.NONE_POSITION + context.robot.AGV_POSITION
```

#### 常用位置常數
```python
# 來源：agv_base/robot.py
NONE_POSITION = "00"           # 填充參數
RACK_IN_POSITION = "01"        # Rack入口位置
RACK_OUT_POSITION = "02"       # Rack出口位置
BOX_IN_POSITION = "03"         # 傳送箱入口位置
BOX_OUT_POSITION = "04"        # 傳送箱出口位置
AGV_POSITION = "05"            # AGV位置(前端)
AGV_POSITION_SIDE = "55"       # AGV位置(側邊)
CLEANER_POSITION = "06"        # 清潔機位置
SOAKER_POSITION = "07"         # 浸潤機位置
PRE_DRYER_POSITION = "08"      # 預乾燥機位置
OVEN_POSITION = "09"           # 烤箱位置
```

### 狀態轉換規則
```python
# 範例：從等待到執行狀態轉換
def transition_to_executing(self):
    if self.validate_preconditions():
        self.log_state_change("WAITING", "EXECUTING")
        return ExecutingState()
    return self
```

## 測試架構

### Loader AGV測試 (驗證實際檔案位置)
- 測試檔案：`src/loader_agv/loader_agv/test_agv_core_node.py` (實際存在)
- 構建後位置：`build/loader_agv/build/lib/loader_agv/test_agv_core_node.py`
- 執行方式：`python3 src/loader_agv/loader_agv/test_agv_core_node.py`

### 調試工具

#### 宿主機調試工具 (推薦)
```bash
# AGV 系統狀態檢查
source scripts/docker-tools/docker-tools.sh
agv_health                          # AGV 容器健康檢查
agv_status                          # AGV 容器詳細狀態

# AGV 日誌分析
scripts/log-tools/log-analyzer.sh agv --stats      # AGV 日誌統計分析
scripts/log-tools/log-analyzer.sh agv --timeline   # AGV 錯誤時間軸

# ROS 2 節點和主題檢查
quick_agv "ros2 node list"          # 查看運行中的節點
quick_agv "ros2 topic list"         # 查看可用主題
quick_agv "ros2 topic echo /agv/status"  # 監控 AGV 狀態主題

# 網路和通訊診斷
scripts/network-tools/zenoh-network.sh agv-check   # AGV Zenoh 通訊檢查
```

#### 容器內調試工具
```bash
# 在AGV容器內，載入環境後執行
source /app/setup.bash && all_source

# 注意：主題名稱需依實際節點設定而定
ros2 topic list                    # 先查看可用主題
ros2 node list                     # 查看運行中的節點
ros2 run rqt_graph rqt_graph       # 節點關係圖
```

## 車型特定配置

### Cargo Mover
- 雙Hokuyo感測器配置
- 物料搬運邏輯
- 配置檔：`/app/config/agv/cargo_mover.yaml`

### Loader/Unloader  
- 機械臂控制整合
- 載入/卸載序列
- 安全檢查機制

## 🛠️ 故障排除

### 系統診斷工作流程

#### 第一步：快速系統檢查 (宿主機執行)
```bash
# 完整系統健康檢查
scripts/system-tools/health-check.sh --quick

# AGV 容器狀態檢查
source scripts/docker-tools/docker-tools.sh
agv_health                           # AGV 容器健康檢查
agv_status                           # AGV 容器詳細狀態
```

#### 第二步：專項診斷 (宿主機執行)
```bash
# AGV 日誌分析
scripts/log-tools/log-analyzer.sh agv --stats       # 日誌統計分析
scripts/log-tools/log-analyzer.sh agv --timeline    # 錯誤時間軸
scripts/log-tools/log-analyzer.sh agv --suggestions # 解決建議

# 網路和通訊診斷
scripts/network-tools/zenoh-network.sh agv-check    # AGV Zenoh 通訊
scripts/network-tools/port-check.sh --verbose       # 端口連接檢查

# 開發環境診斷
scripts/dev-tools/build-helper.sh check --workspace agv_ws  # 建置環境檢查
```

### 常見問題及解決方案

#### 1. **AGV 容器無法啟動**
```bash
# 宿主機診斷步驟
agv_status                           # 查看容器狀態
agv_logs                            # 查看啟動日誌
scripts/docker-tools/container-status.sh agv  # 詳細診斷報告
```

#### 2. **狀態機卡死**
```bash
# 宿主機檢查
quick_agv "ros2 topic echo /agv/status"  # 監控狀態
scripts/log-tools/log-analyzer.sh agv | grep -i "state"  # 狀態轉換日誌

# 容器內檢查
agv_enter                           # 進入容器
ros2 topic list | grep agv          # 檢查 AGV 相關主題
```

#### 3. **機械臂異常**
```bash
# PLC 連接檢查
quick_agv "ros2 service list | grep plc"  # 檢查 PLC 服務
scripts/network-tools/connectivity-test.sh performance --target <PLC_IP>
```

#### 4. **感測器失效**
```bash
# 設備映射檢查
scripts/config-tools/hardware-mapping.sh show <agv_id>  # 查看設備映射
agv_logs | grep -i sensor           # 查看感測器相關日誌
```

#### 5. **⚠️ PGNO參數錯誤**
檢查ACTION_FROM/ACTION_TO參數順序是否正確 (參考上方規則)

### 日誌位置和分析
```bash
# 宿主機日誌分析 (推薦)
scripts/log-tools/log-analyzer.sh agv --stats      # 統計分析
scripts/log-tools/log-analyzer.sh agv --severity 3  # 嚴重錯誤

# 容器內日誌位置
# - AGV日誌：/tmp/agv.log
# - ROS日誌：~/.ros/log/
# - 狀態轉換：查看節點輸出
```

## 💡 重要提醒

### 開發環境使用原則
- **🖥️ 宿主機**: 使用 `scripts/` 工具進行容器管理、系統診斷、開發工作流
- **🐳 容器內**: 執行 ROS 2 相關指令、代碼開發、測試執行
- **📡 推薦方式**: 使用 `agv_enter` 進入容器，使用 `quick_agv` 執行容器內指令

### 技術規範提醒
- **PGNO參數順序**: 嚴格遵守ACTION_FROM/ACTION_TO的參數順序規則
- **狀態轉換**: 必須包含完整驗證和日誌記錄
- **機械臂操作**: 需要安全檢查和錯誤處理
- **感測器數據**: 要實現容錯處理機制

### 故障排除最佳實踐
1. **優先使用宿主機工具**: 快速診斷和系統檢查
2. **日誌分析為主**: 使用 `scripts/log-tools/` 進行智能分析
3. **環境隔離**: 明確區分宿主機操作和容器內操作
4. **工具組合**: 結合多個診斷工具獲得完整視圖