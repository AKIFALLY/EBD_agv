# kuka_wcs - KUKA 倉庫控制系統

## 專案概述
KUKA AGV 車隊的倉庫控制系統，提供智能任務判斷和自動化任務分配功能。實現基於機器人狀態、電量、位置等因素的智能任務分配，與 AGVC 資料庫系統無縫整合。

## 核心模組

### 主要類別
- **KukaWcsNode** (`kuka_wcs_node.py`): KUKA WCS 主節點，任務決策引擎
- **TaskDecisionEngine** (`task_decision_engine.py`): 智能任務判斷核心
- **FleetAdapter** (`fleet_adapter.py`): KUKA Fleet API 適配器
- **DatabaseClient** (`database_client.py`): AGVC 資料庫客戶端
- **TaskQueueManager** (`task_queue_manager.py`): 任務佇列管理

### 系統架構
```
KUKA Fleet ◄──► WCS Decision ◄──► Database
Adapter         Engine          Client
    │               │               │
    ▼               ▼               ▼
KUKA API        Task Queue      AGVC DB
(HTTP REST)     Management      (PostgreSQL)
```

## 關鍵檔案

### 核心檔案
- `/kuka_wcs/kuka_wcs_node.py` - WCS 主節點，任務調度核心
- `/kuka_wcs/task_decision_engine.py` - 智能任務判斷邏輯
- `/kuka_wcs/fleet_adapter.py` - KUKA Fleet API 整合
- `/kuka_wcs/database_client.py` - 資料庫連線和查詢

### 配置檔案
- `/config/kuka_wcs_config.yaml` - WCS 系統配置
- `/config/task_priority_config.yaml` - 任務優先級配置
- `/config/fleet_config.yaml` - KUKA Fleet 連線配置

### 啟動檔案
- `/launch/kuka_wcs_launch.py` - 完整系統啟動
- `/launch/decision_engine_launch.py` - 決策引擎啟動
- `/launch/fleet_adapter_launch.py` - Fleet 適配器啟動

## 開發指令

### 基本構建
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 構建 wcs_ws
build_ws wcs_ws

# 單獨構建 kuka_wcs
cd /app/wcs_ws
colcon build --packages-select kuka_wcs
```

### 服務啟動
```bash
# 完整啟動
ros2 launch kuka_wcs kuka_wcs_launch.py

# 單獨啟動 WCS 節點
ros2 run kuka_wcs kuka_wcs_node

# 啟動決策引擎
ros2 run kuka_wcs task_decision_engine

# 啟動 Fleet 適配器
ros2 run kuka_wcs fleet_adapter
```

## 配置設定

### WCS 系統配置
```yaml
# kuka_wcs_config.yaml
decision_engine:
  task_timeout: 300
  priority_levels: 5
  max_concurrent_tasks: 10

fleet_adapter:
  kuka_api_url: "http://kuka-fleet:8080"
  polling_interval: 5.0
  connection_timeout: 30

database:
  connection_pool_size: 10
  query_timeout: 15
```

### 任務優先級配置
```yaml
# task_priority_config.yaml
priority_rules:
  emergency: 1
  production: 2
  maintenance: 3
  logistics: 4
  default: 5
```

## 整合點

### 與其他專案整合
- **db_proxy_ws**: 使用 DatabaseClient 進行資料庫操作
- **kuka_fleet_ws**: 透過 FleetAdapter 與 KUKA Fleet Manager 通訊
- **rcs_ws**: 接收任務請求，回傳任務分配結果
- **web_api_ws**: 提供 WCS 狀態和統計資訊 API

### ROS 2 話題
```bash
# 發布話題
/wcs/task_assignment        # 任務分配結果
/wcs/fleet_status          # 車隊狀態資訊
/wcs/decision_metrics      # 決策引擎指標

# 訂閱話題
/rcs/task_request          # 任務請求
/fleet/robot_status        # 機器人狀態
/database/status_update    # 資料庫狀態更新
```

## 測試方法

### 單元測試
```bash
# 執行所有測試
python3 -m pytest test/

# 測試決策引擎
python3 test/test_decision_engine.py

# 測試 Fleet 適配器
python3 test/test_fleet_adapter.py

# 測試資料庫客戶端
python3 test/test_database_client.py
```

### 整合測試
```bash
# 完整系統測試
python3 test/test_integration.py

# 任務分配測試
python3 test/test_task_assignment.py

# 效能測試
python3 test/test_performance.py
```

## 故障排除

### 常見問題

#### KUKA Fleet 連線失敗
```bash
# 檢查 Fleet API 狀態
curl http://kuka-fleet:8080/api/status

# 檢查網路連通性
ping kuka-fleet

# 查看 Fleet 適配器日誌
ros2 topic echo /wcs/fleet_adapter/logs
```

#### 任務分配異常
```bash
# 檢查決策引擎狀態
ros2 topic echo /wcs/decision_metrics

# 查看任務佇列
ros2 service call /wcs/get_task_queue

# 重置決策引擎
ros2 service call /wcs/reset_decision_engine
```

#### 資料庫連線問題
```bash
# 檢查資料庫連線
ros2 service call /db_proxy/test_connection

# 查看連線池狀態
ros2 topic echo /wcs/database/connection_status

# 重新初始化資料庫客戶端
ros2 service call /wcs/reinit_database_client
```

### 除錯技巧
- 使用 `self.get_logger().info()` 記錄決策過程
- 監控 `/wcs/decision_metrics` 話題掌握系統效能
- 檢查任務優先級配置是否正確
- 確認 KUKA Fleet API 回應格式

### 效能監控
- 任務分配延遲應保持在 5 秒內
- Fleet API 回應時間監控
- 資料庫查詢效能分析
- 記憶體使用情況檢查
