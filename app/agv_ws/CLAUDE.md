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

## 開發指令

### 環境設定 (容器內)
```bash
source /app/setup.bash
all_source  # 載入所有工作空間
cd /app/agv_ws
```

### 構建與測試 (容器內執行)
```bash
# 必須先進入容器並載入環境
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

### 啟動服務 (容器內執行，launch檔案驗證存在)
```bash
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

### 調試工具 (容器內執行)
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

## 故障排除

### 常見問題
1. **狀態卡死**: 檢查狀態轉換條件與日誌
2. **機械臂異常**: 確認PLC連接與安全訊號
3. **感測器失效**: 驗證設備映射與權限
4. **⚠️ PGNO參數錯誤**: 檢查ACTION_FROM/ACTION_TO參數順序是否正確 (參考上方規則)

### 日誌位置
- AGV日誌：`/tmp/agv.log`
- ROS日誌：`~/.ros/log/`
- 狀態轉換：查看節點輸出

## 重要提醒
- **PGNO參數順序**: 嚴格遵守ACTION_FROM/ACTION_TO的參數順序規則
- 狀態轉換必須包含完整驗證
- 機械臂操作需安全檢查
- 感測器數據要容錯處理
- 所有操作需Docker容器內執行