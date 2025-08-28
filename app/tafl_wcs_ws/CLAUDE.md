# TAFL WCS Workspace CLAUDE.md

## 📚 Context Loading
@docs-ai/knowledge/system/tafl-language-spec.md
@docs-ai/operations/development/ros2-workspace-test-structure.md
@docs-ai/operations/development/testing-standards.md

## 🎯 Module Overview
**TAFL WCS** (Task Automation Flow Language - Warehouse Control System) 是基於 TAFL v1.1 語言的倉庫控制系統實作，作為 Linear Flow v2 的替代方案，提供更結構化和標準化的流程定義和執行能力。

## 🔧 Core Features
- **TAFL v1.1 執行引擎**: 完整支援 6 區塊結構（metadata, initialization, preload, variables, parameters, flow）
- **4 階段執行模型**: Initialization → Preload → Variables → Flow
- **資料庫整合**: 使用 db_proxy 的 ConnectionPoolManager 直接連接 PostgreSQL
- **ROS 2 服務**: 提供標準 ROS 2 服務介面，支援流程觸發和管理
- **異步執行**: 使用 ThreadPoolExecutor 支援並行流程執行
- **生命週期管理**: 實現 ROS 2 節點生命週期，支援優雅關閉

## 📁 Project Structure
```
tafl_wcs_ws/
├── src/tafl_wcs/
│   ├── tafl_wcs/                    # 主程式碼目錄
│   │   ├── __init__.py
│   │   ├── tafl_db_bridge.py        # 資料庫橋接模組
│   │   ├── tafl_executor_wrapper.py # TAFL 執行器封裝
│   │   ├── tafl_functions.py        # TAFL 函數庫擴展
│   │   ├── tafl_wcs_manager.py      # 流程管理器
│   │   └── tafl_wcs_node.py         # ROS 2 主節點
│   ├── test/                        # 測試目錄（標準結構）
│   │   ├── __init__.py
│   │   ├── README.md
│   │   ├── test_tafl_system.py      # 系統整合測試
│   │   ├── test_simple_db.py        # 簡單資料庫測試
│   │   ├── test_check_all_data.py   # 完整資料查詢測試
│   │   ├── test_copyright.py        # ROS 2 標準測試
│   │   ├── test_flake8.py
│   │   └── test_pep257.py
│   ├── launch/
│   │   └── tafl_wcs.launch.py       # ROS 2 Launch 檔案
│   ├── config/                      # 配置目錄（待建立）
│   ├── package.xml                  # ROS 2 套件描述
│   └── setup.py                     # Python 套件設定
├── pytest.ini                        # pytest 配置
└── run_tests.sh                     # 測試執行腳本
```

## 🔍 Key Technical Details

### Database Integration
```python
# 使用 db_proxy 的 ConnectionPoolManager
DATABASE_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"
self.db_bridge = TAFLDatabaseBridge(DATABASE_URL)

# 支援的查詢操作
- query_locations()
- query_racks()
- query_tasks()
- query_works()
- create_task()
- update_task_status()
```

### TAFL Executor Integration
```python
# 4 階段執行流程
async def execute_tafl(self, tafl_content: dict):
    # 1. Initialization Phase
    await self._execute_initialization(tafl_content.get('initialization', {}))
    
    # 2. Preload Phase - 資料庫查詢
    await self._execute_preload(tafl_content.get('preload', {}))
    
    # 3. Variables Phase - 變數處理
    self._process_variables(tafl_content.get('variables', {}))
    
    # 4. Flow Phase - 主要邏輯執行
    await self._execute_flow(tafl_content.get('flow', []))
```

### ROS 2 Service Interface
```python
# 發布者
self.status_publisher = self.create_publisher(String, '/tafl_wcs/status', 10)

# 訂閱者
self.trigger_subscriber = self.create_subscription(
    String, '/tafl_wcs/trigger', self._handle_trigger, 10)

# 服務（待實作）
self.execute_service = self.create_service(
    ExecuteTAFL, '/tafl_wcs/execute', self._handle_execute_service)
```

## 🚀 Development Workflow

### Building
```bash
cd /app/tafl_wcs_ws
colcon build --packages-select tafl_wcs
source install/setup.bash
```

### Running
```bash
# 方式 1: ROS 2 節點
ros2 run tafl_wcs tafl_wcs_node

# 方式 2: Launch 檔案
ros2 launch tafl_wcs tafl_wcs.launch.py
```

### Testing
```bash
# 執行所有測試
./run_tests.sh all

# 執行特定類型測試
./run_tests.sh unit       # 單元測試
./run_tests.sh db         # 資料庫測試
./run_tests.sh integration # 整合測試

# 直接執行 pytest
python3 -m pytest src/tafl_wcs/test/ -v
```

## 🚨 Common Issues and Solutions

### Issue: Database Column Name Errors
**問題**: `Location.location_id` doesn't exist
**解決**: 使用正確的欄位名稱
- `Location.id` (不是 location_id)
- `Rack.status_id` (不是 rack_status_id)
- `Task.status_id` (不是 task_status_id)

### Issue: RMW Implementation Error
**問題**: Failed to load shared library 'librmw_zenoh_cpp.so'
**解決**: 確保正確載入 ROS 2 環境
```bash
source /app/setup.bash
source /app/tafl_wcs_ws/install/setup.bash
```

### Issue: Permission Denied
**問題**: 無法寫入檔案
**解決**: 在容器內執行操作
```bash
docker compose -f docker-compose.agvc.yml exec agvc_server bash
```

## 📊 Database Schema Reference

### 主要資料表
- **locations**: 位置資訊（90 筆）
- **racks**: 料架資訊（8 筆）  
- **agvc_task**: 任務資訊（21 筆）
- **work**: 工作定義（48 筆）
- **location_status**: 位置狀態
- **rack_status**: 料架狀態

### 重要欄位對應
```sql
-- 正確的欄位名稱
SELECT l.id, l.node_id FROM locations l;
SELECT r.id, r.status_id FROM racks r;
SELECT t.id, t.status_id FROM agvc_task t;
```

## 🔗 Related Documentation
- TAFL 語言規格: @docs-ai/knowledge/system/tafl-language-spec.md
- Flow WCS 系統: `app/flow_wcs_ws/CLAUDE.md`
- 資料庫代理: `app/db_proxy_ws/CLAUDE.md`
- ROS 2 工作空間測試結構: @docs-ai/operations/development/ros2-workspace-test-structure.md

## 📅 Development Timeline
- **2025-08-22**: 初始建立，實現基本 TAFL v1.1 執行框架
- **待實作**: TAFL 流程載入器、ROS 2 服務介面、流程管理 UI

## 💡 Design Decisions
1. **獨立工作空間**: 避免影響現有 flow_wcs_ws，便於平行開發和測試
2. **直接資料庫連接**: 使用 db_proxy 的 ConnectionPoolManager，避免重複造輪子
3. **標準測試結構**: 遵循 ROS 2 工作空間測試規範，測試檔案放在 `src/tafl_wcs/test/`
4. **pytest 框架**: 遵循最新測試標準，使用 pytest 而非 unittest