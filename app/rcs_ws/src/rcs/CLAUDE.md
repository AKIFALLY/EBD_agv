# rcs - 車隊控制系統

## 專案概述
RCS (Robot Control System) 是 RosAGV 系統的車隊控制核心，負責智能任務分派、AGV 車隊管理和任務狀態協調。支援多種車型（Cargo、Loader、Unloader、KUKA）的混合車隊管理，提供基於房間和車型的智能分派機制。

## 核心模組

### 主要類別
- **RcsCore** (`rcs_core.py`): RCS 核心節點，系統協調中心
- **CtManager** (`ct_manager.py`): CT 車隊管理器，智能任務分派核心
- **KukaManager** (`kuka_manager.py`): KUKA 車隊管理器
- **KukaDispatcher** (`kuka_dispatcher.py`): KUKA 任務派發器
- **KukaRobot** (`kuka_robot.py`): KUKA 機器人控制
- **KukaContainer** (`kuka_container.py`): KUKA 容器管理

### 車隊管理架構
```
RCS Core
    ├── CT Manager (Cargo/Loader/Unloader)
    │   ├── 房內任務: Loader{房間:02d}, Unloader{房間:02d}
    │   └── 房外任務: Cargo02
    └── KUKA Manager (KUKA400i)
        ├── KUKA Dispatcher
        ├── KUKA Robot Control
        └── KUKA Container Management
```

## 關鍵檔案

### 核心檔案
- `/rcs/rcs_core.py` - RCS 核心節點，系統協調中心
- `/rcs/ct_manager.py` - CT 車隊管理器，智能分派核心
- `/rcs/kuka_manager.py` - KUKA 車隊管理器
- `/rcs/kuka_dispatcher.py` - KUKA 任務派發器

### 車型控制
- `/rcs/kuka_robot.py` - KUKA 機器人控制邏輯
- `/rcs/kuka_container.py` - KUKA 容器管理
- `/rcs/task_status_simulator.py` - 任務狀態模擬器

### 測試檔案
- `/test_ct_dispatch.py` - CT 任務派發測試腳本
- `/docs/testing/` - 測試相關文件目錄

## 開發指令

### 基本構建
```bash
# 進入 AGV 容器
docker compose -f docker-compose.yml exec rosagv bash
source /app/setup.bash && all_source

# 構建 rcs_ws
build_ws rcs_ws

# 單獨構建 rcs
cd /app/rcs_ws
colcon build --packages-select rcs
```

### 服務啟動
```bash
# 啟動 RCS 核心
ros2 run rcs rcs_core

# 啟動 CT 管理器
ros2 run rcs ct_manager

# 啟動 KUKA 管理器
ros2 run rcs kuka_manager

# 啟動任務狀態模擬器
ros2 run rcs task_status_simulator
```

### 測試指令
```bash
# CT 任務派發測試
cd /app/rcs_ws/src/rcs
python3 test_ct_dispatch.py

# 單元測試
python3 -m pytest test/

# 整合測試
python3 test/test_integration.py
```

## 配置設定

### CT 車隊配置
```python
# ct_manager.py
CT_FLEET_CONFIG = {
    "supported_types": ["Cargo", "Loader", "Unloader"],
    "room_based_dispatch": True,
    "default_cargo_agv": "Cargo02",
    "loader_pattern": "Loader{room:02d}",
    "unloader_pattern": "Unloader{room:02d}"
}
```

### KUKA 車隊配置
```python
# kuka_manager.py
KUKA_FLEET_CONFIG = {
    "fleet_type": "KUKA400i",
    "api_endpoint": "http://kuka-fleet:8080",
    "max_concurrent_tasks": 5,
    "task_timeout": 300
}
```

### 任務分派規則
```python
# 分派邏輯配置
DISPATCH_RULES = {
    "room_inside_tasks": {
        "loader": "Loader{room:02d}",
        "unloader": "Unloader{room:02d}"
    },
    "room_outside_tasks": {
        "cargo": "Cargo02"
    },
    "kuka_tasks": {
        "fleet_managed": True
    }
}
```

## 整合點

### 與其他專案整合
- **db_proxy_ws**: 使用資料庫服務查詢 AGV 狀態和任務資訊
- **wcs_ws**: 接收 WCS 的任務分配請求，回傳執行結果
- **agv_ws**: 透過 ROS 2 話題與 AGV 基礎系統通訊
- **kuka_fleet_ws**: 整合 KUKA Fleet Manager 進行 KUKA 車隊控制

### ROS 2 話題
```bash
# 發布話題
/rcs/task_assignment        # 任務分配結果
/rcs/agv_status            # AGV 狀態更新
/rcs/fleet_metrics         # 車隊效能指標

# 訂閱話題
/wcs/task_request          # WCS 任務請求
/agv/status_update         # AGV 狀態更新
/database/agv_info         # 資料庫 AGV 資訊
```

### ROS 2 服務
```bash
# 提供的服務
/rcs/assign_task           # 任務分配服務
/rcs/get_fleet_status      # 獲取車隊狀態
/rcs/cancel_task           # 取消任務
/rcs/get_agv_by_type       # 根據車型獲取 AGV

# 調用的服務
/db_proxy/get_agv_status   # 查詢 AGV 狀態
/db_proxy/update_task      # 更新任務狀態
```

## 測試方法

### CT 管理器測試
```bash
# CT 任務派發測試
python3 test_ct_dispatch.py

# 房間分派邏輯測試
python3 test/test_room_dispatch.py

# 車型選擇測試
python3 test/test_vehicle_selection.py
```

### KUKA 管理器測試
```bash
# KUKA 派發測試
python3 test/test_kuka_dispatch.py

# KUKA Fleet 整合測試
python3 test/test_kuka_integration.py

# 容器管理測試
python3 test/test_container_management.py
```

### 整合測試
```bash
# 完整車隊測試
python3 test/test_full_fleet.py

# 混合車型測試
python3 test/test_mixed_fleet.py

# 效能測試
python3 test/test_performance.py
```

## 故障排除

### 常見問題

#### CT 任務分派失敗
```bash
# 檢查 CT 管理器狀態
ros2 topic echo /rcs/ct_manager/status

# 檢查可用 AGV
ros2 service call /rcs/get_agv_by_type rcs_interfaces/srv/GetAgvByType "{vehicle_type: 'Cargo'}"

# 查看分派日誌
ros2 topic echo /rcs/dispatch_logs
```

#### KUKA 車隊連線問題
```bash
# 檢查 KUKA Fleet API
curl http://kuka-fleet:8080/api/status

# 檢查 KUKA 管理器
ros2 topic echo /rcs/kuka_manager/status

# 重啟 KUKA 連線
ros2 service call /rcs/restart_kuka_connection
```

#### 任務狀態同步異常
```bash
# 檢查任務狀態
ros2 topic echo /rcs/task_status

# 檢查資料庫同步
ros2 service call /db_proxy/test_connection

# 重新同步任務狀態
ros2 service call /rcs/resync_task_status
```

### 除錯技巧
- 使用 `self.get_logger().info()` 記錄分派決策過程
- 監控 `/rcs/fleet_metrics` 話題掌握車隊效能
- 檢查車型配置是否正確
- 確認房間編號格式符合預期

### 效能監控
- 任務分派延遲應保持在 3 秒內
- AGV 利用率監控
- 任務完成率統計
- 車隊負載平衡分析

## 智能分派邏輯

### CT 車隊分派規則
```python
# 房內任務分派
if task.location.is_inside_room():
    if task.type == "loading":
        agv = f"Loader{task.room_id:02d}"
    elif task.type == "unloading":
        agv = f"Unloader{task.room_id:02d}"

# 房外任務分派
else:
    agv = "Cargo02"
```

### KUKA 車隊分派
- 透過 KUKA Fleet Manager API 進行智能分派
- 考慮機器人位置、電量、任務優先級
- 支援動態負載平衡
