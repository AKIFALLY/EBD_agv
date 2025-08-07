# Simple WCS 系統設計文檔

## 🎯 系統定位說明
**Simple WCS** 是 RosAGV 系統中的**主要 WCS 核心系統**，基於配置檔案驅動的決策引擎，與 Flow Designer 可視化介面整合，提供靈活且易於維護的倉庫控制解決方案。

## 🎯 適用場景
- **生產環境**: 主要的 WCS 決策引擎，處理所有倉庫控制邏輯
- **配置驅動**: 基於 YAML/JSON 檔案的靈活業務流程配置
- **可視化管理**: 透過 Flow Designer 進行流程設計和管理
- **易於維護**: 無需修改程式碼即可調整業務邏輯

## 📋 系統概述

### 🎯 主要系統定位
**Simple WCS** 是 RosAGV 的主要 WCS (Warehouse Control System) 決策引擎，基於 ROS 2 Jazzy 和 Zenoh RMW 構建，採用配置檔案驅動系統，專注於透過 YAML/JSON flow 檔案實現靈活的業務流程自動化決策。

### 🆚 與 AI WCS 的關係
```
RosAGV WCS 系統架構:
├── 🎯 simple_wcs_ws (主要生產系統)
│   ├── 生產環境使用
│   ├── 配置驅動決策引擎
│   ├── Flow 檔案驅動業務流程
│   └── Flow Designer 可視化配置整合
└── 🔬 ai_wcs_ws (實驗性系統)
    ├── 研究和對比測試使用
    ├── Python 原生決策邏輯
    ├── 實驗性演算法驗證
    └── 非主要生產系統
```

### 🎯 主要系統特色
- **配置驅動**: Flow 檔案 (YAML/JSON) 定義業務邏輯，靈活且易於維護
- **Flow Designer 整合**: 可視化設計介面產生 flow 檔案
- **ROS 2 + Zenoh 整合**: 原生 ROS 2 節點實作，使用 Zenoh RMW 支援分散式部署
- **資料庫整合**: 直接整合現有 db_proxy 資料庫系統和連接池
- **Python 邏輯執行**: 根據 flow 檔案配置執行對應的 Python 程式邏輯

### 🎯 生產價值
- **易於維護**: 透過修改 flow 檔案即可調整業務邏輯，無需修改程式碼
- **可視化管理**: Flow Designer 提供直觀的流程設計和管理介面
- **靈活配置**: 支援動態優先度、條件判斷、自定義動作等靈活配置
- **擴展性**: 新的業務流程可透過新增 flow 檔案輕鬆實現

### ⚠️ 使用建議
- **生產環境**: **建議使用 simple_wcs_ws** 作為主要 WCS 系統
- **Flow Designer**: 使用 AGVCUI 中的 Flow Designer 進行流程設計
- **配置管理**: 透過 flow 檔案進行業務邏輯配置和調整
- **實驗對比**: ai_wcs_ws 可用於演算法研究和對比測試

## 🏗️ 系統架構

### 核心組件架構
```
Simple WCS 系統架構 (ROS 2 Workspace: simple_wcs_ws)
├── SimpleWCSEngine (ROS 2 節點)
│   ├── 決策循環 (每5秒)
│   ├── 業務流程執行器 (按優先級排序)
│   ├── ROS 2 發布者 (/simple_wcs/task_decisions, /simple_wcs/system_status)
│   └── 任務衝突檢查和防護機制
├── FlowParser (YAML 解析器)
│   ├── 多檔案目錄解析 (/app/config/wcs/flows/)
│   ├── 業務流程驗證和完整性檢查
│   ├── TriggerCondition 和 FlowAction 結構化定義
│   └── 支援房間和位置的動態適用性配置
├── DatabaseClient (資料庫客戶端)
│   ├── 直接整合 db_proxy_ws 連接池管理器
│   ├── 基於 0-360度 rack.direction 的旋轉邏輯
│   ├── 智能人工收料區位置分配
│   └── 完整的業務邏輯查詢方法
└── LocationManager (位置配置管理)
    ├── 靜態位置配置 (locations.yaml)
    ├── 房間入口/出口停靠點管理
    └── 旋轉中間點動態計算
```

### 實際檔案結構
```
simple_wcs_ws/
├── src/simple_wcs/
│   ├── package.xml                      # ROS 2 套件配置
│   ├── setup.py                        # Python 套件設置
│   ├── launch/
│   │   └── simple_wcs_launch.py        # ROS 2 Launch 檔案
│   └── simple_wcs/
│       ├── wcs_engine.py               # 核心決策引擎 (ROS 2 節點)
│       ├── flow_parser.py              # 多檔案 YAML 解析器
│       └── database_client.py          # SQLModel 資料庫客戶端
├── build/                              # 建置產出
├── install/                            # 安裝產出
└── log/                               # 建置日誌

配置檔案位置:
/app/config/wcs/
├── flows/                              # 業務流程目錄
│   ├── rack_rotation_inlet.yaml        # Rack旋轉-入口 (優先級100)
│   ├── rack_rotation_exit.yaml         # Rack旋轉-出口 (優先級90)
│   └── full_rack_to_manual_area.yaml   # 滿料架運輸 (優先級80)
└── locations.yaml                      # 靜態房間位置配置
```

## 🔧 技術實作詳解

### ROS 2 節點設計 (Zenoh RMW)
```python
class SimpleWCSEngine(Node):
    """Simple WCS 決策引擎 - ROS 2 節點"""
    
    def __init__(self):
        super().__init__('simple_wcs_engine')
        self.logger = self.get_logger()
        
        # 初始化組件
        self._init_components()
        
        # 設定決策循環定時器 (5秒一次)
        self.decision_timer = self.create_timer(5.0, self.decision_cycle_callback)
        
        # ROS 2 發布者
        self.task_publisher = self.create_publisher(String, '/simple_wcs/task_decisions', 10)
        self.status_publisher = self.create_publisher(String, '/simple_wcs/system_status', 10)
    
    def _init_components(self):
        """初始化系統組件"""
        # 使用統一配置目錄
        config_dir = Path('/app/config/wcs')
        
        # 初始化資料庫客戶端
        self.db = DatabaseClient()
        
        # 初始化位置管理器
        locations_path = config_dir / 'locations.yaml'
        self.locations = LocationManager(str(locations_path))
        
        # 初始化流程解析器 - 支援多檔案目錄
        flows_path = config_dir / 'flows'
        self.flow_parser = FlowParser(str(flows_path))
        self.business_flows = self.flow_parser.parse()
        
        # 驗證配置
        validation = self.flow_parser.validate_flows(self.business_flows)
        if validation['errors']:
            self.logger.error(f"配置錯誤: {validation['errors']}")
```

### 業務流程 YAML 格式
```yaml
# 業務流程檔案格式 (rack_rotation_inlet.yaml)
name: "Rack旋轉檢查-房間入口"
description: "當 Rack A面完成後，檢查是否需要旋轉處理 B面"
priority: 100
work_id: "220001"
enabled: true

# 觸發條件 - 所有條件必須為 true 才執行
trigger_conditions:
  - condition: "rack_at_location_exists"
    description: "房間入口位置有 Rack"
    parameters:
      location_type: "room_inlet"
      
  - condition: "rack_side_completed" 
    description: "Rack A面已完成"
    parameters:
      side: "A"
      
  - condition: "rack_has_b_side_work"
    description: "Rack B面有待處理工作"
    parameters:
      side: "B"
      
  - condition: "rack_needs_rotation_for_b_side"
    description: "Rack 需要旋轉以處理 B面"
    parameters:
      location_type: "room_inlet"
      
  - condition: "no_active_task"
    description: "該位置無衝突任務"
    parameters:
      work_id: "220001"

# 執行動作
action:
  type: "create_task"
  task_type: "rack_rotation"
  function: "rack_move"
  model: "KUKA400i"
  api: "submit_mission"
  mission_type: "RACK_MOVE"
  
  # 路徑配置
  path:
    type: "inlet_rotation"
    
# 適用房間
applicable_rooms: [1, 2, 3, 4, 5]

# 調試選項
debug:
  enabled: false
  log_conditions: true
  dry_run: false
```

### 多檔案解析器設計
```python
class FlowParser:
    """業務流程解析器 - 多檔案 YAML 格式 (flows/ 目錄)"""
    
    def parse(self, flows_dir: str = None) -> List[BusinessFlow]:
        """解析 flows/ 目錄中的所有業務流程配置"""
        # 掃描目錄中所有 .yaml 檔案
        # 每個檔案解析為一個 BusinessFlow 物件
        # 支援配置驗證和錯誤處理
```

## 🚀 業務流程執行邏輯

### 決策循環流程
```
決策循環 (每5秒執行)
├── 1. 發布系統狀態到 ROS 2 主題
├── 2. 執行所有業務流程檢查 (_run_business_flows)
│   ├── 按優先級排序執行 (高到低)
│   ├── 檢查每個流程的觸發條件
│   ├── 產生任務決策 (TaskDecision)
│   └── 一次只處理一個同類型任務 (防衝突)
├── 3. 處理決策結果
│   ├── 根據任務類型選擇建立方法
│   ├── Rack 旋轉任務 → create_rack_rotation_task
│   ├── Rack 運輸任務 → create_rack_transport_task
│   └── 發布任務決策到 ROS 2 主題
└── 4. 記錄詳細決策日誌和原因
```

### 實際支援的業務流程
1. **Rack旋轉檢查-房間入口** (優先級: 100)
   - 檢查條件: A面完成 + B面有工作 + 需要旋轉 + 無衝突任務
   - 執行動作: 0° → 180° 旋轉 (RACK_MOVE 任務)

2. **Rack旋轉檢查-房間出口** (優先級: 90)
   - 檢查條件: B面完成 + A面有工作 + 需要旋轉 + 無衝突任務
   - 執行動作: 180° → 0° 旋轉 (RACK_MOVE 任務)

3. **滿料架到人工收料區-傳送箱出口** (優先級: 80)
   - 檢查條件: 傳送箱出口有滿料架 + 人工收料區可用 + 無衝突任務
   - 執行動作: 動態分配目標位置並建立運輸任務

### 觸發條件評估
```python
# 觸發條件評估邏輯
def _evaluate_trigger_condition(self, trigger, rack_id, room_id, location):
    """評估單一觸發條件"""
    condition = trigger.condition
    params = trigger.parameters
    
    if condition == "rack_at_location_exists":
        return self.db.rack_at_location_exists(location)
    elif condition == "rack_side_completed":
        side = params.get('side', 'A')
        return self.db.rack_side_completed(rack_id, side)
    # ... 其他條件邏輯
```

### 任務決策生成
```python
# 任務決策資料結構
@dataclass
class TaskDecision:
    name: str                    # 任務名稱
    work_id: str                # 工作 ID
    priority: int               # 優先級
    room_id: int               # 房間 ID
    rack_id: int               # Rack ID
    nodes: List[int]           # 路徑節點
    parameters: Dict[str, Any]  # 任務參數
    reason: str                # 決策原因
```

## 🛠️ 資料庫整合

### SQLModel 直接整合
```python
class DatabaseClient:
    """直接使用 SQLModel 的資料庫客戶端"""
    
    def __init__(self):
        # 直接使用 db_proxy_ws 的連接池管理器
        db_url = "postgresql://agvc:password@postgres_container:5432/agvc"
        self.pool_manager = ConnectionPoolManager(db_url)
    
    @contextmanager
    def get_session(self):
        """獲取資料庫 session 的上下文管理器"""
        session = self.pool_manager.get_session()
        try:
            yield session
        except Exception as e:
            session.rollback()
            self.logger.error(f"資料庫操作失敗: {e}")
            raise
        finally:
            session.close()
```

### 核心業務邏輯查詢方法

#### Rack 旋轉相關 (基於 0-360度 rack.direction)
- `rack_at_location_exists(location_id)` - 檢查指定位置是否有 Rack
- `get_rack_at_location(location_id)` - 獲取指定位置的 Rack 資訊
- `rack_side_completed(rack_id, side)` - 檢查 Rack A/B 面完成狀態
- `rack_has_b_side_work(rack_id)` - 檢查 B面是否有待處理工作
- `rack_has_a_side_work(rack_id)` - 檢查 A面是否有待處理工作
- `rack_needs_rotation_for_b_side(rack_id, location_type)` - 判斷是否需要旋轉處理B面
- `rack_needs_rotation_for_a_side(rack_id, location_type)` - 判斷是否需要旋轉處理A面

#### Rack 運輸相關
- `transfer_exit_has_full_rack(location_id)` - 檢查傳送箱出口是否有滿料架
- `rack_is_full(rack_id)` - 檢查 Rack 是否滿載 (32個carrier)
- `find_available_manual_location()` - 智能尋找可用的人工收料區位置
- `manual_collection_area_available()` - 檢查人工收料區是否有可用空間

#### 任務衝突檢查
- `no_active_task(work_id, location)` - 檢查指定工作和位置是否沒有衝突任務
- `no_active_task_to_specific_location(target_location)` - 檢查具體位置是否無衝突
- `no_active_task_to_destination(destination_type, work_id)` - 檢查目的地類型是否無衝突
- `no_active_task_from_source(source_type, source_location, work_id)` - 檢查來源地是否無衝突

#### 任務建立
- `create_rack_rotation_task(rack_id, room_id, location_type, nodes)` - 建立 Rack 旋轉任務
- `create_rack_transport_task(rack_id, source_location, destination_type, target_location, nodes)` - 建立 Rack 運輸任務

## 🔍 AI Agent 配置管理

### yq 工具整合
Simple WCS 完全支援 AI Agent 使用 `yq` 工具進行配置管理：

```bash
# 查詢業務流程配置
yq '.name' flows/rack_rotation_inlet.yaml
yq '.priority' flows/rack_rotation_inlet.yaml
yq '.trigger_conditions[].condition' flows/rack_rotation_inlet.yaml

# 修改配置
yq '.priority = 120' flows/rack_rotation_inlet.yaml > updated_flow.yaml
yq '.enabled = false' flows/rack_rotation_inlet.yaml > disabled_flow.yaml

# 新增觸發條件
yq '.trigger_conditions += [{"condition": "new_condition", "description": "新條件", "parameters": {}}]' flows/rack_rotation_inlet.yaml
```

### 配置管理工作流程
1. **AI Agent 掃描** flows/ 目錄發現所有業務流程
2. **分析現有配置**使用 `yq` 解析各流程參數
3. **動態調整配置**根據系統狀態修改 YAML 檔案
4. **驗證配置變更**Simple WCS 自動載入新配置
5. **監控執行結果**透過 ROS 2 訊息監控決策效果

## 📊 部署和運行

### ROS 2 部署配置
```bash
# 在 AGVC 容器內執行 (需要資料庫存取)
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入環境
all_source  # 或 agvc_source

# 建置 Simple WCS 套件
cd /app/simple_wcs_ws
colcon build --packages-select simple_wcs

# 環境變數設定 (通常已自動設定)
export RMW_IMPLEMENTATION=rmw_zenohd
export PYTHONPATH=/opt/pyvenv_env/lib/python3.12/site-packages:/app/db_proxy_ws/src:$PYTHONPATH

# 啟動 Simple WCS Engine
source install/setup.bash
ros2 run simple_wcs simple_wcs_node

# 或使用 Launch 檔案
ros2 launch simple_wcs simple_wcs_launch.py
```

### 系統監控
```bash
# 檢查系統狀態
ros2 topic echo /simple_wcs/system_status
ros2 topic echo /simple_wcs/task_decisions

# 查看節點資訊
ros2 node info /simple_wcs_engine
ros2 node list | grep simple_wcs

# 確認 Zenoh 通訊狀態
check_zenoh_status
ros2 topic list  # 確認跨容器主題發現

# 檢查資料庫連接
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT COUNT(*) FROM rack;"
```

### 測試檔案
```bash
# 在 simple_wcs_ws 目錄下有測試檔案
python3 test_simple_wcs.py          # 基本功能測試
python3 test_exit_rotation.py       # 出口旋轉測試  
python3 create_exit_test_data.py    # 建立出口測試資料
```

## 🔧 擴展指導

### 新增業務流程
1. 在 `flows/` 目錄建立新的 `.yaml` 檔案
2. 使用標準的業務流程格式
3. 定義觸發條件和執行動作
4. 系統自動載入新流程

### 新增觸發條件
1. 在 `database_client.py` 中實作條件檢查方法
2. 在 `wcs_engine.py` 中新增條件評估邏輯
3. 在業務流程 YAML 中配置新條件

### 系統監控和調試
- 使用 `debug.enabled: true` 啟用詳細日誌
- 使用 `debug.dry_run: true` 進行模擬測試
- 透過 ROS 2 主題監控系統狀態

## 📋 最佳實踐

### 配置管理
1. **檔案命名**: 使用描述性名稱 (如 `rack_rotation_inlet.yaml`)
2. **優先級設定**: 關鍵流程使用高優先級 (100+)
3. **房間範圍**: 明確指定 `applicable_rooms` 避免衝突
4. **調試配置**: 開發時啟用詳細日誌和條件追蹤

### 效能最佳化
1. **決策週期**: 根據業務需求調整決策循環間隔
2. **條件最佳化**: 將最常見的條件放在前面
3. **資料庫查詢**: 使用連接池和適當的查詢最佳化
4. **Zenoh RMW 通訊**: 合理設定 QoS 和主題設計，利用 Zenoh 的高效能跨容器通訊
5. **Zenoh 配置**: 確保 `/app/routerconfig.json5` 配置正確，Zenoh Router 運行穩定

### 故障排除
1. **配置驗證**: 定期檢查 YAML 語法和結構完整性
2. **條件日誌**: 使用 `log_conditions: true` 追蹤條件評估
3. **Zenoh 通訊**: 使用 `check_zenoh_status` 檢查 Zenoh Router 狀態
4. **系統監控**: 監控 ROS 2 主題和節點狀態，確認跨容器通訊正常
5. **資料庫連接**: 檢查資料庫連接和查詢效能

## 🔗 交叉引用

### 🎯 主要 WCS 系統
- **Simple WCS 統一架構**: @docs-ai/knowledge/agv-domain/wcs-system-design.md - **主要 WCS 系統設計**
- **Simple WCS 實作**: `app/simple_wcs_ws/CLAUDE.md` - **生產環境使用的 WCS 核心**
- **Flow Designer**: `app/web_api_ws/src/agvcui/CLAUDE.md` - **可視化流程設計器**

### 🎯 Simple WCS 相關
- **Simple WCS 開發**: @docs-ai/operations/development/simple-wcs-development.md - 生產系統開發指導
- **Simple WCS 實作**: `app/simple_wcs_ws/` - 主要 WCS 實作代碼

### 🔬 AI WCS (實驗性)
- **AI WCS 實作**: `app/ai_wcs_ws/CLAUDE.md` - 實驗性決策引擎

### 📚 共用資源
- **資料庫設計**: @docs-ai/knowledge/agv-domain/wcs-database-design.md
- **Work ID 系統**: @docs-ai/knowledge/agv-domain/wcs-workid-system.md
- **ROS 2 開發**: @docs-ai/operations/development/ros2-development.md
- **資料庫操作**: @docs-ai/operations/development/database-operations.md
- **容器開發**: @docs-ai/operations/development/docker-development.md

### ⚠️ 選擇指導
- **🏭 生產環境**: 使用 simple_wcs_ws + Flow Designer
- **🔬 研究實驗**: 使用 ai_wcs_ws 進行演算法研究和對比測試
- **📖 學習理解**: 主要學習 simple_wcs_ws，ai_wcs_ws 可作為對比參考