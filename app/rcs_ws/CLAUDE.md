# rcs_ws CLAUDE.md

## 模組概述

企業級車隊控制系統(Robot Control System)，負責 AGV 車隊的智能任務分派、車隊管理和交通協調。支援多種車型（Cargo、Loader、Unloader、KUKA400i）的混合車隊管理，提供基於房間和車型的智能分派機制。

### 🚀 最新版本特色
- **統一配置管理**: 整合 KUKA 和 CT AGV 的完整配置系統
- **增強版派發器**: KukaDispatcherV2 具備 API 重試和錯誤恢復機制
- **完整測試框架**: 單元測試、整合測試和離線測試環境
- **企業級監控**: 完整的系統監控、告警和維護工具
- **生產就緒**: 具備自動恢復、配置管理和部署指南

## 專案結構

```
src/
├── rcs/                          # RCS 車隊控制系統核心
│   ├── rcs_core.py              # RCS 核心節點 - 系統協調中心
│   ├── ct_manager.py            # CT 車隊管理器 - 智能任務分派
│   ├── kuka_manager.py          # KUKA 車隊管理器 (1518行完整實現)
│   ├── kuka_dispatcher_v2.py    # 增強版 KUKA 派發器 (507行)
│   ├── kuka_robot.py            # KUKA 機器人控制
│   ├── kuka_container.py        # KUKA 容器管理
│   ├── kuka_config_manager.py   # 統一配置管理系統
│   ├── kuka_config_cli.py       # 配置管理 CLI 工具
│   └── task_status_simulator.py # 任務狀態模擬器
├── test/                        # 完整測試套件
│   ├── conftest.py             # pytest 配置和 fixtures
│   ├── test_kuka_manager.py    # KukaManager 單元測試
│   ├── test_kuka_integration.py # KUKA Fleet API 整合測試
│   ├── offline_test_server.py  # 離線測試伺服器
│   └── test_environment_manager.py # 測試環境管理
├── docs/                        # 完整文檔系統
│   └── README.md               # 企業級完整文檔 (3000+行)
├── rcs_interfaces/              # RCS 介面定義
├── traffic_manager/             # 交通管理模組
│   └── traffic_controller.py   # 交通區域控制器
├── test_config_manager.py      # 配置管理器測試腳本
└── test_ct_dispatch.py         # CT 派發測試腳本
```

## 核心功能

### 車隊管理系統
- **CT 車隊**: Cargo/Loader/Unloader 車型的智能任務分派
- **KUKA 車隊**: KUKA400i 機器人車隊管理 (1518行完整實現)
- **增強派發器**: KukaDispatcherV2 具備 API 重試和指數退避機制
- **房間分派**: 基於房間和車型的智能分派機制
- **狀態監控**: AGV 狀態變更監控和資料庫同步
- **配置管理**: 統一的 KUKA 和 CT AGV 配置管理系統

### 測試與監控
- **完整測試**: 單元測試、整合測試、離線測試環境
- **系統監控**: 完整的性能監控和告警機制
- **故障診斷**: 智能錯誤檢測和自動恢復
- **配置備份**: 自動配置備份和版本控制

### 交通管理
- **交通區域**: 交通區域控制權管理
- **衝突預防**: AGV 路徑衝突預防機制
- **區域鎖定**: 根據 AGV 需求鎖定/釋放交通區域
- **安全協調**: 多車同時運行的安全協調

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/rcs_ws
```

### 服務啟動
```bash
# 啟動 RCS 核心節點 (包含 CT 和 KUKA 管理器)
ros2 run rcs rcs_core

# 單獨啟動 CT 管理器 (如需要)
ros2 run rcs ct_manager

# 單獨啟動 KUKA 管理器 (如需要)
ros2 run rcs kuka_manager

# 啟動交通控制器
ros2 run traffic_manager traffic_controller
```

### 配置管理
```bash
# 統一配置管理 CLI 工具
cd /app/rcs_ws/src/rcs

# 列出所有 AGV
python3 kuka_config_cli.py list

# 驗證配置
python3 kuka_config_cli.py validate

# 與資料庫同步
python3 kuka_config_cli.py sync

# 新增 KUKA AGV
python3 kuka_config_cli.py add KUKA004 8506999 KUKA400i \
  --description "新增的 KUKA AGV" \
  --x 1000.0 --y 2000.0 \
  --robot-id "8506999"

# 匯出配置備份
python3 kuka_config_cli.py export --format yaml --output backup.yaml
```

### 構建與測試
```bash
# 構建 rcs_ws
build_ws rcs_ws

# 執行完整測試套件
python3 -m pytest test/ -v

# 執行單元測試
python3 -m pytest test/test_kuka_manager.py -v

# 執行整合測試
python3 -m pytest test/test_kuka_integration.py -v

# 啟動離線測試伺服器
python3 test/offline_test_server.py

# 測試 CT 任務分派
cd /app/rcs_ws/src/rcs
python3 test_ct_dispatch.py

# 測試配置管理器
python3 test_config_manager.py

# 生成測試覆蓋率報告
python3 -m pytest test/ --cov=rcs --cov-report=html --cov-report=term

# 測試交通控制器
cd /app/rcs_ws/src/traffic_manager
python3 -m pytest test/
```

## 核心模組說明

### RcsCore - 系統協調中心
```python
# rcs/rcs_core.py
class RcsCore(Node):
    def __init__(self):
        # 初始化資料庫連線池
        self.db_pool = ConnectionPoolManager(
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
        
        # 初始化車隊管理器
        self.kuka_manager = KukaManager(self)  # KUKA 車隊
        self.ct_manager = CtManager(self)      # CT 車隊
        
        # 任務狀態模擬器
        self.task_status_simulator = TaskStatusSimulator(self.db_pool, self.get_logger())
        
        # 1秒定時器 - 主迴圈
        self.timer_1s = self.create_timer(1.0, self.main_loop)
```

### CtManager - CT 車隊管理器
```python
# rcs/ct_manager.py
class CtManager:
    def dispatch(self):
        """智能任務分派邏輯"""
        # 1. 查詢待執行的 CT 任務
        ct_tasks = session.exec(
            select(Task).where(
                Task.status_id == 1,  # 待執行
                (Task.parameters["model"].as_string() == "Cargo") |
                (Task.parameters["model"].as_string() == "Loader") |
                (Task.parameters["model"].as_string() == "Unloader")
            )
        )
        
        # 2. 根據房間和車型選擇合適的 AGV
        target_agv = self._select_agv_for_task(session, task)
```

### TrafficController - 交通管理
```python
# traffic_manager/traffic_controller.py
class TrafficController:
    def acquire_traffic_zone(self, traffic_id, agv_id):
        """取得交通區域控制權"""
        zone = session.exec(
            select(TrafficZone).where(TrafficZone.id == traffic_id)
        ).first()
        if zone and zone.status == "free":
            zone.status = "controlled"
            zone.owner_agv_id = agv_id
            return True
        return False
```

## 車隊管理架構

### CT 車隊分派邏輯
```python
# CT 車隊分派規則 (ct_manager.py)
def _determine_target_agv_name(self, model, room_id):
    if room_id is None:
        # 房外任務 - Cargo 車輛
        if model == 'Cargo':
            return 'Cargo02'  # 負責房間2的房外任務
    else:
        # 房內任務 - Loader 或 Unloader
        if model == 'Loader':
            return f'Loader{room_id:02d}'  # 如 Loader02
        elif model == 'Unloader':
            return f'Unloader{room_id:02d}'  # 如 Unloader01
```

### 交通協調機制
- **區域鎖定**: AGV 進入特定區域時自動鎖定
- **衝突避免**: 防止多台 AGV 同時使用相同路徑
- **動態釋放**: AGV 離開後自動釋放交通區域
- **狀態監控**: 即時監控所有交通區域狀態

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

### 統一配置管理系統
```python
# kuka_config_manager.py
@dataclass
class UnifiedFleetConfig:
    """統一車隊配置"""
    kuka_api: KukaAPIConfig = field(default_factory=KukaAPIConfig)
    kuka_fleet: KukaFleetConfig = field(default_factory=KukaFleetConfig)
    ct_fleet: CTFleetConfig = field(default_factory=CTFleetConfig)
    agvs: Dict[str, AGVConfig] = field(default_factory=dict)
    system: Dict[str, Any] = field(default_factory=lambda: {
        'update_interval': 5.0,
        'heartbeat_timeout': 30.0,
        'task_dispatch_interval': 1.0,
        'log_level': 'INFO'
    })
```

### KUKA API 配置
```python
# KUKA Fleet API 配置
kuka_api:
  base_url: "http://192.168.10.3:10870"
  username: "admin"
  password: "Admin"
  timeout: 30.0
  max_retries: 3
  retry_delay: 1.0
  endpoints:
    login: "/api/login"
    robot_query: "/api/amr/robotQuery"
    container_query: "/api/amr/containerQuery"
    submit_mission: "/api/amr/submitMission"
    mission_cancel: "/api/amr/missionCancel"
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
/agv/state_change          # AGV 狀態變更
/agv/status                # AGV 狀態更新
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

### 完整測試套件

#### 單元測試
```bash
# KukaManager 單元測試
python3 -m pytest test/test_kuka_manager.py -v

# 測試任務派發邏輯
python3 -m pytest test/test_kuka_manager.py::TestKukaManager::test_dispatch_with_idle_agvs -v

# 測試 API 重試機制
python3 -m pytest test/test_kuka_dispatcher_v2.py::test_api_retry_mechanism -v
```

#### 整合測試
```bash
# KUKA Fleet API 整合測試
python3 -m pytest test/test_kuka_integration.py -v

# 需要真實 API 的測試 (需要 KUKA Fleet 可用)
python3 -m pytest test/test_kuka_integration.py -m "integration" -v
```

#### 離線測試環境
```bash
# 啟動離線測試伺服器
python3 test/offline_test_server.py

# 伺服器將在 http://localhost:10870 啟動
# 模擬 KUKA Fleet API 的所有端點

# 測試登入 API
curl -X POST http://localhost:10870/api/login \
  -H "Content-Type: application/json" \
  -d '{"username": "admin", "password": "Admin"}'

# 查詢機器人狀態
curl -X POST http://localhost:10870/api/amr/robotQuery \
  -H "Content-Type: application/json" \
  -d '{"token": "test_token"}'
```

#### 配置管理測試
```bash
# 執行配置管理器完整測試
python3 test_config_manager.py

# 測試內容包括:
# - 配置管理器基本功能
# - AGV 管理功能（新增、更新、移除）
# - 配置持久化（儲存、匯出）
# - KUKA 特定功能
# - CT AGV 特定功能
# - API 配置更新
```

#### CT 車隊測試
```bash
# CT 任務派發測試
cd /app/rcs_ws/src/rcs
python3 test_ct_dispatch.py

# 房間分派邏輯測試
python3 test/test_room_dispatch.py

# 車型選擇測試
python3 test/test_vehicle_selection.py
```

## 故障排除

### 常見問題

#### CT 任務分派失敗
```bash
# 檢查 RCS 核心節點狀態
ros2 node list | grep rcs
ros2 node info /rcs_core

# 檢查 CT 管理器狀態
ros2 topic echo /rcs/fleet_status --once

# 查看分派日誌
ros2 topic echo /rcs/dispatch_logs

# 檢查資料庫中的任務狀態
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select
from db_proxy.db_proxy.models import Task

with ConnectionPoolManager().get_session() as session:
    tasks = session.exec(select(Task).where(Task.status_id == 1).limit(10)).all()
    for task in tasks:
        print(f'Task {task.id}: {task.parameters}')
"

# 檢查 AGV 狀態
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select
from db_proxy.db_proxy.models import AGV

with ConnectionPoolManager().get_session() as session:
    agvs = session.exec(select(AGV).where(AGV.model.in_(['Cargo', 'Loader', 'Unloader']))).all()
    for agv in agvs:
        print(f'{agv.name}: status={agv.status_id}, enable={agv.enable}')
"
```

#### KUKA 車隊連線問題
```bash
# 檢查 KUKA Fleet API 可達性
curl -X GET http://192.168.10.3:10870/api/status

# 測試 API 登入
curl -X POST http://192.168.10.3:10870/api/login \
  -H "Content-Type: application/json" \
  -d '{"username": "admin", "password": "Admin"}'

# 檢查網路連通性
ping 192.168.10.3
telnet 192.168.10.3 10870

# 檢查 KUKA Manager 日誌
ros2 topic echo /rcs/system_logs | grep -i kuka

# 檢查配置檔案
python3 -c "
from rcs.kuka_config_manager import get_config_manager
config = get_config_manager()
print('KUKA API Config:')
print(f'  Base URL: {config.config.kuka_api.base_url}')
print(f'  Username: {config.config.kuka_api.username}')
print(f'  Timeout: {config.config.kuka_api.timeout}')
print(f'  Max Retries: {config.config.kuka_api.max_retries}')
"

# 使用離線測試環境進行調試
cd /app/rcs_ws/src/rcs
python3 test/offline_test_server.py &

# 修改配置指向本地測試伺服器
python3 -c "
from rcs.kuka_config_manager import get_config_manager
config_manager = get_config_manager()
api_updates = {'base_url': 'http://localhost:10870'}
config_manager.update_kuka_api_config(api_updates)
print('已切換到離線測試環境')
"
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
- **新工具**: 使用離線測試伺服器進行 API 調試
- **配置工具**: 使用 `kuka_config_cli.py` 進行配置管理
- **測試覆蓋**: 執行完整測試套件驗證功能

### 效能監控
- 任務分派延遲應保持在 3 秒內
- AGV 利用率監控 (目標 > 70%)
- 任務完成率統計 (目標 > 95%)
- 車隊負載平衡分析
- **API 監控**: KUKA Fleet API 成功率監控
- **資料庫監控**: 連線池使用率和查詢性能
- **系統資源**: CPU、記憶體使用率監控

### 新增功能驗證
```bash
# 驗證統一配置管理
python3 kuka_config_cli.py validate

# 驗證增強版派發器
python3 -m pytest test/test_kuka_dispatcher_v2.py -v

# 驗證離線測試環境
python3 test/offline_test_server.py &
curl http://localhost:10870/api/login

# 驗證配置同步
python3 kuka_config_cli.py sync
```

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

## 系統監控與維護

### 關鍵性能指標 (KPI)
1. **任務分派延遲**: < 3 秒
2. **AGV 利用率**: > 70%
3. **任務完成率**: > 95%
4. **系統可用時間**: > 99%
5. **API 成功率**: > 98%

### 監控命令
```bash
# 即時監控腳本
echo "RCS 系統監控 - $(date)"
echo "============================"

# 檢查節點狀態
echo "ROS 2 節點狀態:"
ros2 node list | grep rcs

# 檢查系統指標
echo -e "\n系統指標:"
ros2 topic echo /rcs/system_metrics --once 2>/dev/null | \
  grep -E "(cpu_usage|memory_usage|dispatch_latency|tasks_dispatched)"

# 檢查車隊狀態
echo -e "\n車隊狀態:"
ros2 topic echo /rcs/fleet_status --once 2>/dev/null | \
  grep -E "(total_agvs|idle_agvs|busy_agvs|error_agvs)"

# 檢查資料庫連線
echo -e "\n資料庫連線:"
ros2 service call /db_proxy/test_connection 2>/dev/null | \
  grep -E "(success|connection_count)"
```

### 完整文檔連結
詳細的系統文檔位於: `docs/README.md` (3000+行企業級文檔)
- 完整 ROS 2 API 文檔
- 配置管理指南
- 故障排除手冊
- 開發指南
- 部署指南
- 監控與維護指南

## 重要提醒
- RCS 系統為車隊管理不是機械臂控制
- 所有任務分派必須包含完整驗證和日誌
- CT 車隊分派需根據房間和車型正確分配
- 必須在 AGVC 容器內運行並與 db_proxy_ws 協調
- **新功能**: 支援統一配置管理、增強版派發器、完整測試框架
- **生產就緒**: 具備企業級監控、告警和自動恢復機制