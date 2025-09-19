# TAFL WCS Workspace CLAUDE.md

## 📚 Context Loading
@docs-ai/knowledge/system/tafl/tafl-language-specification.md
@docs-ai/knowledge/system/tafl/tafl-api-reference.md
@docs-ai/knowledge/system/tafl/tafl-troubleshooting-guide.md
@docs-ai/knowledge/agv-domain/wcs-system-design.md
@docs-ai/knowledge/agv-domain/wcs-workid-system.md
@docs-ai/knowledge/agv-domain/wcs-database-design.md
@docs-ai/operations/development/testing/ros2-workspace-test-structure.md
@docs-ai/operations/development/testing/testing-standards.md

## 🎯 Module Overview
**TAFL WCS** (Task Automation Flow Language - Warehouse Control System) 是基於 TAFL v1.1 語言的倉庫控制系統實作，作為 Linear Flow v2 的替代方案，提供更結構化和標準化的流程定義和執行能力。

## 🔧 Core Features
- **TAFL v1.1 執行引擎**: 完整支援 6 段式結構（metadata, settings, preload, rules, variables, flow）
- **4 階段執行模型**: Settings → Preload → Rules → Variables → Flow execution
- **5-Level 變數作用域**: Rules Scope → Preload Scope → Global Scope → Flow Scope → Loop Scope
- **資料庫整合**: 使用 db_proxy 的 ConnectionPoolManager 直接連接 PostgreSQL
- **ROS 2 服務**: 提供標準 ROS 2 服務介面，支援流程觸發和管理
- **同步執行**: 使用同步執行模式，避免 asyncio.run() 記憶體問題（參考 RCS 實作）
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

# 支援的資料庫操作
# 查詢操作
- query_locations()      # 查詢位置資訊
- query_racks()          # 查詢料架資訊
- query_tasks()          # 查詢任務資訊
- query_works()          # 查詢工作定義

# 建立操作
- create_task()          # 建立新任務
- create_rack()          # 建立新料架

# 更新操作
- update_task_status()   # 更新任務狀態
- update_rack()          # 更新料架資訊
- update_rack_side_completed()  # 更新料架面完成狀態
- update_location_status()      # 更新位置狀態
```

### TAFL Executor Integration
```python
# TAFL v1.1 4-Phase 執行流程（已實作）
async def execute_flow(self, flow_content: str):
    # Phase 1: Settings - 執行設定
    await self._execute_settings(flow_data.get('settings', {}))
    
    # Phase 2: Preload - 資料預載與快取
    await self._execute_preload(flow_data.get('preload', {}))
    
    # Phase 3: Rules - 規則定義（唯讀）
    self._process_rules(flow_data.get('rules', {}))
    
    # Phase 4: Variables - 變數初始化
    self._process_variables(flow_data.get('variables', {}))
    
    # Execute Flow - 執行主流程
    await self._execute_flow(flow_data.get('flow', []))

# 5-Level 變數作用域解析（已實作）
def _resolve_variable(self, var_ref: str):
    # 從最具體到最廣泛的作用域搜尋
    var_name = var_ref.strip('${}')  # 移除變數標記
    scope_order = ['loop', 'flow', 'global', 'preload', 'rules']
    for scope in scope_order:
        if var_name in self.scopes[scope]:
            return self.scopes[scope][var_name]
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
- **location**: 位置資訊（90 筆）
- **rack**: 料架資訊（8 筆）  
- **task**: 任務資訊（21 筆）
- **work**: 工作定義（48 筆）
- **location_status**: 位置狀態
- **rack_status**: 料架狀態

### 重要欄位對應
```sql
-- 正確的欄位名稱
SELECT l.id, l.node_id FROM location l;
SELECT r.id, r.status_id FROM rack r;
SELECT t.id, t.status_id FROM task t;
```

### 命名對照表
**⚠️ 重要：避免混淆不同層級的命名**

| 實際資料表名 | Python 檔案名 | Python 類別名 | TAFL 查詢目標 | 查詢函數名 |
|------------|-------------|-------------|--------------|-----------|
| `location` | `agvc_location.py` | `Location` | `locations` | `query_locations()` |
| `rack` | `rack.py` | `Rack` | `racks` | `query_racks()` |
| `task` | `agvc_task.py` | `Task` | `tasks` | `query_tasks()` |
| `work` | `agvc_task.py` | `Work` | `works` | `query_works()` |

**說明**：
- **實際資料表名**：PostgreSQL 資料庫中的實際表名（單數形式）
- **Python 檔案名**：db_proxy 模組中的檔案名稱
- **Python 類別名**：SQLModel 定義的類別名稱
- **TAFL 查詢目標**：在 TAFL 流程中 `query:` 動詞的 `target:` 參數值（**統一使用複數形式**）
- **查詢函數名**：TAFLDatabaseBridge 中的查詢方法名稱

### 命名規範（2025-09-12 統一）
**🎯 統一原則：TAFL 和 Python 函數使用複數，資料庫保持單數**

```
TAFL Query Target: 複數 (racks, tasks, locations, works)
        ↓
Python Functions: 複數 (query_racks(), query_tasks(), etc.)
        ↓
Database Tables: 單數 (rack, task, location, work)
```

**優點**：
- ✅ 語義清晰：`query_racks()` 明確表示查詢多個 rack（集合）
- ✅ 符合 REST 慣例：`/api/tasks`, `/api/racks`（集合用複數）
- ✅ 保持資料庫不變：不需要修改現有資料表結構
- ✅ 只需一次轉換：只在 SQL 查詢時轉換（複數→單數）

## 🔗 Related Documentation
- TAFL 語言規格: @docs-ai/knowledge/system/tafl/tafl-language-specification.md
- TAFL 實作計畫: @docs-ai/knowledge/system/tafl/tafl-implementation-plan.md
- TAFL 快速入門: @docs-ai/knowledge/system/tafl/tafl-quick-start-guide.md
- Flow WCS 系統: `app/flow_wcs_ws/CLAUDE.md`
- 資料庫代理: `app/db_proxy_ws/CLAUDE.md`
- ROS 2 工作空間測試結構: @docs-ai/operations/development/testing/ros2-workspace-test-structure.md
- 測試程序: @docs-ai/operations/development/testing/testing-procedures.md

## 📅 Development Timeline
- **2024-12-22**: 初始建立，實現基本 TAFL v1.1 執行框架
- **2025-01-11**: 更新以完全符合 TAFL v1.1 規格書
  - 實作完整 6 段式結構驗證（metadata, settings, preload, rules, variables, flow）
  - 統一術語使用（settings 取代 initialization）
  - 實作 5-Level 變數作用域管理
  - 增強執行器支援 v1.1 動詞格式（switch 範圍條件、set 多格式）
  - 新增 TAFL v1.1 合規性測試套件
- **2025-01-11**: 切換為同步執行模式，修正資料表名稱文檔
  - 從 asyncio 改為同步執行（避免記憶體問題）
  - 修正文檔中的資料表名稱錯誤（task 而非 agvc_task）
  - 新增命名對照表避免混淆
- **待實作**: ROS 2 服務介面完整實作、流程管理 UI

## 💡 Design Decisions
1. **獨立工作空間**: 避免影響現有 flow_wcs_ws，便於平行開發和測試
2. **直接資料庫連接**: 使用 db_proxy 的 ConnectionPoolManager，避免重複造輪子
3. **標準測試結構**: 遵循 ROS 2 工作空間測試規範，測試檔案放在 `src/tafl_wcs/test/`
4. **pytest 框架**: 遵循最新測試標準，使用 pytest 而非 unittest