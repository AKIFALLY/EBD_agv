# WCS 統一系統架構設計

## 🎯 適用場景
- 理解 RosAGV 中以 simple_wcs_ws 為核心的 WCS 統一架構
- 掌握 Simple WCS 配置驅動決策引擎和業務流程調度邏輯
- 了解 Flow Designer 可視化配置與 Simple WCS 的整合關係

## 📋 統一系統概述

**WCS (Warehouse Control System)** 是 RosAGV 系統的核心決策引擎，以 **simple_wcs_ws** 為統一實作核心，負責管理 Rack 車的搬移需求決策和任務調度。

### 🎯 Simple WCS 核心定位
- **配置驅動決策**: simple_wcs_ws 作為主要的 WCS 核心邏輯實作，基於 YAML/JSON 檔案配置
- **可視化配置**: AGVCUI Flow Designer 提供流程設計和配置介面，產生 flow 檔案
- **靈活調度**: 基於 flow 檔案的動態業務流程調度系統
- **易於維護**: 配置檔案驅動，無需修改程式碼即可調整業務邏輯

### 基本環境配置
- **房間數量**: 支援 1-10 個房間動態擴展
- **每個房間配置**: 1個入口 + 1個出口 (均可停靠 Rack 車)
- **Rack 位置管理**: 由 WCS 掌控並儲存在資料庫中
- **實體搬運**: 由 KUKA AGV 透過 KUKA API 執行

## 🏗️ 統一系統架構分工

### 🎯 Simple WCS (simple_wcs_ws) - 核心決策引擎
**核心職責**:
- **配置驅動決策引擎**: 基於 YAML/JSON flow 檔案執行業務流程邏輯
- **動態流程調度**: 依據 flow 檔案內容動態調整業務流程和優先度
- **Python 邏輯執行**: 根據 flow 檔案配置執行對應的 Python 程式邏輯
- **資料庫整合**: 與 db_proxy_ws 無縫整合，執行實際的資料庫操作
- **ROS 2 節點架構**: SimpleWCSEngine ROS 2 節點提供系統整合

### 🎨 AGVCUI Flow Designer - 可視化配置介面
**核心職責**:
- **流程設計器**: 可視化拖拽式流程設計 (/flow-designer 路由)
- **節點配置系統**: 提供 action_nodes.yaml, condition_nodes.yaml, logic_nodes.yaml
- **Flow 檔案產生**: FlowFileManager 管理並產生 /app/config/wcs/flows/ 中的 flow 檔案
- **配置轉換**: 將視覺化設計轉換為 simple_wcs_ws 可執行的 YAML/JSON 配置

### 🤖 RCS (rcs_ws) - 執行層
**支援職責**:
- 讀取 Simple WCS 產生的任務
- 透過 KUKA API 向 KUKA Fleet 下達搬運指令
- 管理 AGV 調度和交通控制

### 🌐 Web API (web_api_ws) - 整合層
**支援職責**:
- 任務狀態監控和更新
- 與 KUKA Fleet 介面整合
- 外部系統 API Gateway

### 🔬 AI WCS (ai_wcs_ws) - 實驗性系統
**定位說明**:
- **實驗性質**: ai_wcs_ws 為實驗性或舊版 WCS 系統
- **研究用途**: 可用於 WCS 演算法研究和對比測試
- **非主要系統**: 生產環境建議使用 simple_wcs_ws

## 🔄 完整的 Rack 生命週期流程

### 階段1：上料階段
1. **作業區上貨** → 作業員在作業區(4個作業區，每個有2個停車格)將貨物放上空Rack
   - 停車格位置記錄在 `machine.parking_space_1/2` 欄位
2. **OPUI操作** → 作業員透過OPUI後端直連資料庫，產生`status=0`的task
   - 目標房間資訊寫入 `task.parameters`
   - 任務類型：**叫車**(需要空Rack) 或 **派車**(將滿車移走)
3. **WCS監控與執行**：
   - **派車**：WCS將滿載Rack送到"系統準備派車區"等待
   - **叫車**：WCS挑選空Rack，產生任務讓AGV搬運到叫車的作業區

### 階段2：生產階段
4. **進入房間入口** → WCS依優先度將Rack從準備區送到房間入口
5. **機器手臂取料** → 房間入口機器手臂將Rack上貨物送入房間處理
6. **A面清空與翻面** → Rack有2面，A面carrier全部搬完時：
   - WCS產生**旋轉任務**，讓AGV執行Rack翻面動作
   - 機器手臂繼續作業B面
7. **NG檢查與處理** → 機器手臂OCR檢查，如果NG：
   - 記錄NG但不取出carrier
   - **有NG carrier的Rack → 送到NG區人員處理**
   - **正常空Rack → 送到房間出口(有空位)或空Rack暫存區**

### 階段3：收料階段
8. **機器手臂放料** → 房間出口機器手臂將處理完貨物放到空Rack上
9. **A面放滿與翻面** → Rack A面放滿但B面未放時：
   - WCS產生**旋轉任務**(節點路徑與入口旋轉不同)
   - 機器手臂繼續作業B面
10. **送到收料區** → Rack滿載或房間尾批完成時送到"人工收料區"

### 階段4：回收階段
11. **人工收料** → 作業員從人工收料區取走完成品
    - 作業員在系統輸入Rack已空
    - 作業員將Rack搬到空車回收位置(2格Node)
12. **空車回收** → WCS檢查到Rack在空車回收位置，派任務送回"系統空料車停車區"

## 🚀 Simple WCS 配置驅動決策引擎設計

### 核心架構組件 (基於 simple_wcs_ws 實作)
```
simple_wcs_ws/src/simple_wcs/
├── wcs_engine.py                 # 核心決策引擎 (ROS 2 節點)
├── flow_parser.py                # Flow 檔案解析器 (YAML/JSON)
├── database_client.py            # 資料庫客戶端 (整合 db_proxy)
└── simple_wcs_node.py           # ROS 2 節點主程式
```

### Flow 檔案驅動的業務流程架構

基於 simple_wcs_ws 的 flow 檔案配置，WCS系統支援靈活的業務流程定義，包含但不限於：

#### 🔴 第1級：AGV旋轉檢查 (Priority: 100)
**業務流程**：檢查等待旋轉狀態的AGV，使用3個節點移動方式執行旋轉

**觸發條件**：
- AGV處於 'wait_rotation_state' 狀態
- 對應任務無子任務存在
- 無重複執行任務 (work_id='220001')

#### 🟠 第2級：NG料架回收 (Priority: 90)
**業務流程**：將NG料架從房間入口傳送箱搬運到NG回收區

**觸發條件**：
- NG回收區有空位 (位置71-72, status=2)
- 房間入口傳送箱有NG料架 (location_id=X0001, status=7)
- 無重複執行任務 (work_id='220001', node_id=location_id)

#### 🟡 第3級：滿料架到人工收料區 (Priority: 80)
**業務流程**：滿料架從各房間搬運到人工收料區

**觸發條件**：
- 系統空架區有空料架 (位置31-34, status=3)
- 房間內有carrier需要搬運
- 無重複執行任務 (work_id='220001')

#### 🟡 第4級：人工收料區搬運 (Priority: 80)
**業務流程**：人工收料區滿料架搬運到房間入口傳送箱

**觸發條件**：
- 人工收料區有空位 (位置51-55, status=2)
- 房間出口傳送箱有滿料架 (status=[2,3,6]) 或 cargo任務已完成
- 無重複執行任務 (work_id='220001')

#### 🟢 第5級：系統準備區到房間 (Priority: 60)
**業務流程**：系統準備區料架送往房間入口傳送箱

**觸發條件**：
- 系統準備區有料架 (位置11-18, status=3)
- 房間入口傳送箱無料架佔用
- 無重複執行任務 (work_id='220001')

#### 🔵 第6級：空料架搬運 (Priority: 40)
**業務流程**：入口傳送箱空料架搬運到出口傳送箱

**觸發條件**：
- 房間入口傳送箱有空料架 (status=1)
- 房間出口傳送箱無料架佔用
- 無重複執行任務 (work_id='220001')

#### 🔵 第7級：人工回收空料架 (Priority: 40)
**業務流程**：人工回收空料架區搬運到系統空料架區

**觸發條件**：
- 人工回收空料架區有料架 (位置91-92, status=3)
- 空料架回收區有空位 (位置51-54, status=2)
- 無重複執行任務 (work_id='230001', status IN (0,1,2)) ⭐特殊work_id使用流程觸發

### Simple WCS 配置驅動決策引擎架構 (基於 simple_wcs_ws 實作)

```python
# 基於 simple_wcs_ws/src/simple_wcs/wcs_engine.py 實作
class SimpleWCSEngine(Node):
    """Simple WCS 配置驅動決策引擎 - Flow 檔案驅動"""
    
    def __init__(self):
        super().__init__('simple_wcs_engine')
        
        # 初始化組件
        self.db_client = DatabaseClient()
        self.flow_parser = FlowParser('/app/config/wcs/flows/')
        
        # 載入 Flow 檔案
        self.business_flows = self.flow_parser.parse_flows()
        
        # 設定決策循環定時器
        self.decision_timer = self.create_timer(5.0, self.decision_cycle_callback)
        
        # ROS 2 發布者
        self.task_publisher = self.create_publisher(String, '/simple_wcs/task_decisions', 10)
    
    def decision_cycle_callback(self):
        """決策循環回調 - 基於 Flow 檔案配置執行"""
        # 依 Flow 檔案中定義的優先度執行業務流程
        for flow in sorted(self.business_flows, key=lambda x: x.priority, reverse=True):
            if self._evaluate_flow_conditions(flow):
                task_decision = self._execute_flow_actions(flow)
                if task_decision:
                    self.task_publisher.publish(String(data=json.dumps(task_decision)))
                    break  # 一次只執行一個最高優先度的流程
    
    def _evaluate_flow_conditions(self, flow: Flow) -> bool:
        """評估 Flow 檔案中定義的觸發條件"""
        for condition in flow.trigger_conditions:
            if not self._check_condition(condition):
                return False
        return True
    
    def _execute_flow_actions(self, flow: Flow) -> dict:
        """執行 Flow 檔案中定義的動作"""
        # 根據 Flow 檔案中的 action 配置執行對應的 Python 程式邏輯
        action = flow.action
        if action.type == "create_task":
            return self._create_task_from_flow(flow, action)
        # 其他動作類型...
        
    def _create_task_from_flow(self, flow: Flow, action: FlowAction) -> dict:
        """根據 Flow 配置建立任務"""
        return {
            "name": flow.name,
            "work_id": flow.work_id,
            "priority": flow.priority,
            "task_type": action.task_type,
            "function": action.function,
            "model": action.model,
            # 其他任務參數從 Flow 檔案中提取
        }
```

### Flow Designer 與 Simple WCS 整合架構

**完整整合流程**: Flow Designer (可視化設計) → FlowFileManager (檔案產生) → Simple WCS (執行引擎)

```python
# 實際整合架構 (基於 agvcui.routers.flow_designer 和 agvcui.database.flow_ops)
class FlowDesignerSimpleWCSIntegration:
    """Flow Designer 與 Simple WCS 的完整整合系統"""
    
    def __init__(self):
        # Flow Designer 檔案管理器 (實際存在的實作)
        self.flow_manager = FlowFileManager()  # 來自 agvcui.database.flow_ops
        
        # Simple WCS 配置目錄
        self.flows_dir = Path("/app/config/wcs/flows")
        self.nodes_dir = Path("/app/config/wcs/nodes")
    
    def create_flow_from_designer(self, flow_design: Dict) -> str:
        """從 Flow Designer 視覺化設計建立 Simple WCS flow 檔案"""
        
        # 1. Flow Designer 提供視覺化節點設計
        flow_definition = FlowDefinition(
            id=flow_design["id"],
            name=flow_design["name"],
            description=flow_design.get("description", ""),
            nodes=flow_design.get("nodes", []),           # 視覺化節點
            connections=flow_design.get("connections", []), # 節點連接
            metadata=flow_design.get("metadata", {})
        )
        
        # 2. 轉換為 Simple WCS 可執行的 YAML 格式
        simple_wcs_config = self._convert_designer_to_simple_wcs(flow_definition)
        
        # 3. 儲存到 Simple WCS 讀取目錄
        flow_file_path = self.flows_dir / f"{flow_design['name']}.yaml"
        
        # 使用 FlowFileOperations 儲存為 YAML 格式 (Simple WCS 格式)
        success = FlowFileOperations.save_flow_data(flow_design['name'], simple_wcs_config)
        
        if success:
            logger.info(f"Flow Designer 流程已轉換為 Simple WCS 配置: {flow_file_path}")
            return str(flow_file_path)
        else:
            raise Exception("Flow 檔案產生失敗")
    
    def get_available_designer_nodes(self) -> Dict:
        """獲取 Flow Designer 可用的節點定義"""
        return {
            'action_nodes': self._load_node_config("action_nodes.yaml"),
            'condition_nodes': self._load_node_config("condition_nodes.yaml"), 
            'logic_nodes': self._load_node_config("logic_nodes.yaml")
        }
    
    def _convert_designer_to_simple_wcs(self, flow_def: FlowDefinition) -> Dict:
        """將 Flow Designer 設計轉換為 Simple WCS 配置格式"""
        
        # 解析視覺化節點為 Simple WCS 觸發條件
        trigger_conditions = []
        for node in flow_def.nodes:
            if node.get("type") == "condition":
                trigger_conditions.append({
                    "condition": node.get("condition_type"),
                    "description": node.get("description", ""),
                    "parameters": node.get("parameters", {})
                })
        
        # 解析視覺化節點為 Simple WCS 動作
        action = {"type": "create_task", "task_type": "custom"}
        for node in flow_def.nodes:
            if node.get("type") == "action":
                action = {
                    "type": node.get("action_type", "create_task"),
                    "task_type": node.get("task_type", "custom"),
                    "function": node.get("function", ""),
                    "model": node.get("model", ""),
                    "api": node.get("api", ""),
                    "parameters": node.get("parameters", {})
                }
                break
        
        # 產生 Simple WCS 標準配置
        return {
            "name": flow_def.name,
            "description": flow_def.description,
            "priority": flow_def.metadata.get("priority", 50),
            "work_id": flow_def.metadata.get("work_id", "220001"),
            "enabled": flow_def.metadata.get("enabled", True),
            "trigger_conditions": trigger_conditions,
            "action": action,
            "applicable_rooms": flow_def.metadata.get("applicable_rooms", [1, 2, 3, 4, 5]),
            "debug": {
                "enabled": False,
                "log_conditions": True,
                "dry_run": False
            }
        }
    
    def _load_node_config(self, filename: str) -> Dict:
        """載入節點配置檔案"""
        config_file = self.nodes_dir / filename
        if config_file.exists():
            with open(config_file, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        return {}
```

### 實際系統整合點

**1. Flow Designer API 端點** (agvcui.routers.flow_designer):
- `/flow-designer` - 可視化流程設計頁面
- `/api/flow-designer/flows` - 流程 CRUD 操作
- `/api/flows/list` - 列出 Simple WCS flow 檔案
- `/api/nodes/list` - 獲取可用節點配置

**2. 檔案產生系統** (agvcui.database.flow_ops.FlowFileManager):
- 儲存目錄: `/app/config/wcs/flows/` (Simple WCS 讀取目錄)
- 檔案格式: YAML/JSON (Simple WCS 支援格式)
- 備份機制: 自動備份到 `/app/config/wcs/backups/`

**3. Simple WCS 執行引擎** (simple_wcs_ws):
- 自動掃描: `/app/config/wcs/flows/` 目錄中的所有 flow 檔案
- 解析執行: FlowParser 解析 YAML 配置並執行對應 Python 邏輯
- 決策循環: 每 5 秒執行一次，按優先度處理 flow 檔案

## 🎯 Simple WCS 核心技術特點

### 配置驅動決策引擎 (simple_wcs_ws)
- **配置檔案驅動**: simple_wcs_ws 作為主要的 WCS 核心實作，基於 YAML/JSON 配置
- **靈活性**: Flow 檔案定義業務邏輯，無需修改程式碼即可調整流程
- **Python 邏輯執行**: 根據 flow 檔案配置執行對應的 Python 程式邏輯
- **資料庫整合**: 直接整合 db_proxy_ws，執行實際的資料庫操作

### 可視化配置系統 (Flow Designer)
- **拖拽式設計**: AGVCUI 提供直觀的流程設計介面
- **節點化配置**: action_nodes.yaml, condition_nodes.yaml, logic_nodes.yaml
- **Flow 檔案產生**: FlowFileManager 產生 /app/config/wcs/flows/ 中的 flow 檔案
- **格式支援**: 支援 YAML 和 JSON 兩種 flow 檔案格式

### 動態優先度調度
- **檔案定義優先度**: 每個 flow 檔案可定義自己的優先度
- **靈活調整**: 透過修改 flow 檔案即可調整業務流程優先度
- **條件驅動**: 基於 trigger_conditions 決定是否執行流程
- **動作執行**: 根據 action 配置執行對應的 Python 邏輯

### 智能衝突解決
- **條件檢查**: flow 檔案中定義的 trigger_conditions 自動檢測衝突
- **位置衝突**: 在 flow 檔案中配置位置相關的檢查條件
- **任務去重**: 透過 work_id 和相關條件避免重複任務
- **自定義邏輯**: 可在 flow 檔案中定義任何自定義的檢查邏輯

## 🔄 完整系統使用流程

### 管理員使用 Flow Designer (可視化設計階段)
1. **存取設計器**: 在 AGVCUI 存取 `/flow-designer` 頁面進行可視化流程設計
2. **節點化設計**: 使用 action_nodes, condition_nodes, logic_nodes 拖拽組成完整業務邏輯
3. **視覺化連接**: 透過連接線定義節點間的邏輯關係和資料流
4. **參數配置**: 為各個節點設定具體的參數和條件

### Flow 檔案產生階段 (FlowFileManager)
5. **格式轉換**: FlowFileManager 將視覺化設計轉換為 Simple WCS 標準 YAML 格式
6. **檔案儲存**: 自動儲存到 `/app/config/wcs/flows/` 目錄 (Simple WCS 讀取位置)
7. **備份管理**: 自動建立備份到 `/app/config/wcs/backups/` 避免配置遺失
8. **版本控制**: 記錄建立時間、修改時間、操作人員等元數據

### Simple WCS 自動執行階段 (主要 WCS 引擎)
9. **目錄掃描**: SimpleWCSEngine 啟動時掃描 `/app/config/wcs/flows/` 所有 YAML 檔案
10. **配置解析**: FlowParser 解析每個 flow 檔案的 trigger_conditions 和 action 配置
11. **決策循環**: 每 5 秒執行決策循環，按 priority 順序檢查所有 flow 檔案
12. **條件評估**: 呼叫對應的 Python 邏輯檢查每個 trigger_condition 是否符合
13. **動作執行**: 符合條件的 flow 執行對應的 action，建立實際的 AGV 任務
14. **狀態發布**: 透過 ROS 2 主題 `/simple_wcs/task_decisions` 發布決策結果

### 系統監控和調整階段
15. **即時監控**: 透過 ROS 2 主題 `/simple_wcs/system_status` 監控系統運行狀態
16. **動態調整**: 管理員可隨時透過 Flow Designer 修改流程，無需重啟系統
17. **配置生效**: Simple WCS 自動檢測 flow 檔案變更並重新載入配置
18. **調試支援**: 使用 `debug.enabled: true` 啟用詳細日誌追蹤流程執行

## 🔗 交叉引用
- **Simple WCS 實作**: `app/simple_wcs_ws/CLAUDE.md` - 配置驅動決策引擎詳細實作
- **Flow Designer**: `app/web_api_ws/src/agvcui/CLAUDE.md` - 可視化流程設計器
- **Simple WCS 開發**: @docs-ai/operations/development/simple-wcs-development.md - 開發指導
- **AI WCS (實驗性)**: `app/ai_wcs_ws/CLAUDE.md` - 實驗性決策引擎
- **資料庫設計**: @docs-ai/knowledge/agv-domain/wcs-database-design.md
- **Work ID 系統**: @docs-ai/knowledge/agv-domain/wcs-workid-system.md
- **業務流程**: @docs-ai/knowledge/business/eyewear-production-process.md
- **系統架構**: @docs-ai/context/system/dual-environment.md