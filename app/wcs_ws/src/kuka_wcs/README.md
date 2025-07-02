# KUKA WCS (Warehouse Control System)

KUKA AGV 車隊的倉庫控制系統，提供智能任務判斷和自動化任務分配功能。

## 功能特點

- 🤖 **智能任務判斷**: 基於機器人狀態、電量、位置等因素進行任務分配
- 📊 **實時監控**: 持續監控 KUKA AGV 和容器狀態
- 🔄 **自動化流程**: 自動接收任務請求並分配給最適合的機器人
- 💾 **資料庫整合**: 與 AGVC 資料庫系統無縫整合
- 📈 **優先級管理**: 支援多級任務優先級和動態調整
- 🔧 **靈活配置**: 豐富的配置選項適應不同場景需求

## 系統架構

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   KUKA Fleet    │    │   WCS Decision   │    │   Database      │
│   Adapter       │◄──►│   Engine         │◄──►│   Client        │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                        │                        │
         ▼                        ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   KUKA API      │    │   Task Queue     │    │   AGVC DB       │
│   (HTTP REST)   │    │   Management     │    │   (PostgreSQL)  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 快速開始

### 1. 編譯工作空間

```bash
cd /app/wcs_ws
colcon build --packages-select kuka_wcs
source install/setup.bash
```

### 2. 啟動系統

#### 方式一：使用專用啟動腳本
```bash
cd /app
./startup.wcs.bash
```

#### 方式二：使用 ROS 2 Launch
```bash
# 完整啟動
ros2 launch kuka_wcs kuka_wcs_launch.py

# 測試模式啟動
ros2 launch kuka_wcs test_launch.py log_level:=DEBUG
```

#### 方式三：直接運行節點
```bash
ros2 run kuka_wcs kuka_wcs_node
```

### 3. 監控系統狀態

```bash
# 查看系統狀態
ros2 topic echo /kuka_wcs/system_status

# 查看任務決策
ros2 topic echo /kuka_wcs/task_decisions

# 查看節點信息
ros2 node info /kuka_wcs/kuka_wcs_node
```

## 配置說明

主要配置文件位於 `config/kuka_wcs_config.yaml`：

```yaml
kuka_wcs_node:
  ros__parameters:
    # KUKA API 設定
    api_base_url: "http://192.168.11.206:10870"
    api_username: "admin"
    api_password: "Admin"
    query_cycle_time: 5.0
    
    # 決策設定
    decision_cycle_time: 10.0
    enable_auto_task_assignment: true
    max_pending_tasks: 50
    min_robot_battery_level: 20
```

## API 介面

### ROS Topics

#### Publishers
- `/kuka_wcs/system_status` (std_msgs/String): 系統狀態信息
- `/kuka_wcs/task_decisions` (std_msgs/String): 任務分配決策

#### Subscribers
- `/kuka_wcs/task_request` (std_msgs/String): 接收任務請求

### 任務請求格式

```json
{
  "task_id": "TASK_001",
  "task_type": "TRANSPORT",
  "priority": 10,
  "source_location": "STATION_A",
  "target_location": "STATION_B",
  "container_code": "CONTAINER_001",
  "robot_requirements": {
    "min_battery": 30,
    "robot_type": "LOADER"
  }
}
```

## 開發指南

### 添加新的任務類型

1. 在 `task_decision_engine.py` 中定義新的任務類型
2. 實現對應的決策邏輯
3. 更新配置文件

### 自定義機器人選擇策略

```python
def custom_robot_selection(self, task: TaskRequest) -> Optional[RobotInfo]:
    # 實現自定義選擇邏輯
    pass
```

### 擴展決策引擎

繼承 `TaskDecisionEngine` 類並覆寫相關方法：

```python
class CustomDecisionEngine(TaskDecisionEngine):
    def calculate_task_priority_score(self, task: TaskRequest) -> float:
        # 自定義優先級計算
        return super().calculate_task_priority_score(task)
```

## 故障排除

### 常見問題

1. **KUKA API 連接失敗**
   - 檢查網路連接和 API 地址
   - 確認用戶名和密碼正確

2. **資料庫連接問題**
   - 確保 db_proxy 服務正在運行
   - 檢查資料庫服務狀態

3. **節點啟動失敗**
   - 檢查依賴包是否正確安裝
   - 查看日誌文件獲取詳細錯誤信息

### 日誌查看

```bash
# 查看 WCS 日誌
tail -f /tmp/kuka_wcs.log

# 查看 ROS 日誌
ros2 log view kuka_wcs_node
```

## 貢獻

歡迎提交 Issue 和 Pull Request 來改進這個項目。

## 授權

Apache 2.0 License
