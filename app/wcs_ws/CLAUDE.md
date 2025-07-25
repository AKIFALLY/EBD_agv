# wcs_ws CLAUDE.md

## 模組概述  
倉庫控制系統(Warehouse Control System)，負責AGV車隊的任務調度、路徑規劃與倉庫作業管理，整合KUKA Fleet系統

## 專案結構
```
src/
├── kuk<function>a_wcs/        # KUKA WCS核心節點
├── wcs/            # WCS通用模組  
└── wcs_base/       # WCS基礎架構
```

## 核心功能

### 任務管理
- **任務調度**: AGV任務排程與分配
- **路徑規劃**: 多AGV路徑最佳化
- **交通管制**: AGV交通流量控制
- **任務優先權**: 緊急任務與一般任務管理

### KUKA Fleet整合
- **KUKA API**: 與KUKA Fleet Manager通訊
- **任務轉換**: WCS任務轉換為KUKA指令
- **狀態同步**: AGV狀態與KUKA系統同步
- **錯誤處理**: KUKA系統錯誤處理與恢復

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/wcs_ws
```

### 服務啟動
```bash
# 啟動WCS主節點
ros2 run wcs wcs_node

# 啟動KUKA WCS整合
ros2 run kuka_wcs kuka_wcs_node

# WCS基礎服務
ros2 run wcs_base wcs_base_node
```

### 構建與測試
```bash
build_ws wcs_ws
ros2 test wcs  # WCS系統測試
```

## 任務調度開發

### 任務管理器
```python
# wcs/task_manager.py
class TaskManager:
    def __init__(self):
        self.task_queue = PriorityQueue()
        self.agv_assignments = {}
        self.task_history = []
        
    async def submit_task(self, task: Task):
        """提交新任務到系統"""
        validated_task = await self.validate_task(task)
        priority = self.calculate_priority(validated_task)
        await self.task_queue.put((priority, validated_task))
        
    async def assign_tasks(self):
        """分配任務給可用AGV"""
        available_agvs = await self.get_available_agvs()
        while not self.task_queue.empty() and available_agvs:
            priority, task = await self.task_queue.get()
            best_agv = self.select_optimal_agv(task, available_agvs)
            await self.assign_task_to_agv(task, best_agv)
```

### 路徑規劃整合
```python
# wcs/path_coordinator.py
class PathCoordinator:
    def __init__(self):
        self.path_planner = AStarPlanner()
        self.traffic_manager = TrafficManager()
        
    async def plan_multi_agv_paths(self, tasks: List[Task]):
        """多AGV路徑協調規劃"""
        paths = {}
        for task in tasks:
            agv_id = task.assigned_agv
            path = await self.path_planner.plan_path(
                task.start_position, task.end_position
            )
            # 檢查路徑衝突並調整
            conflict_free_path = await self.resolve_path_conflicts(
                agv_id, path, paths
            )
            paths[agv_id] = conflict_free_path
        return paths
```

### KUKA Fleet整合
```python
# kuka_wcs/kuka_fleet_interface.py
class KukaFleetInterface:
    def __init__(self):
        self.kuka_client = KukaFleetClient()
        self.task_converter = TaskConverter()
        
    async def send_task_to_kuka(self, wcs_task: Task):
        """將WCS任務轉換為KUKA指令"""
        kuka_order = self.task_converter.wcs_to_kuka(wcs_task)
        response = await self.kuka_client.submit_order(kuka_order)
        return self.process_kuka_response(response)
        
    async def sync_agv_status(self):
        """同步AGV狀態與KUKA系統"""
        kuka_status = await self.kuka_client.get_fleet_status()
        for agv_status in kuka_status.vehicles:
            await self.update_local_agv_status(agv_status)
```

## 倉庫作業管理

### 作業類型
```python
# wcs/warehouse_operations.py
class WarehouseOperations:
    OPERATION_TYPES = {
        'TRANSPORT': '運輸作業',
        'PICK': '拾取作業', 
        'PLACE': '放置作業',
        'CHARGE': '充電作業',
        'MAINTENANCE': '維護作業'
    }
    
    async def create_transport_task(self, from_location, to_location, cargo):
        """創建運輸任務"""
        task = Task(
            task_type='TRANSPORT',
            start_position=from_location,
            end_position=to_location,
            cargo_info=cargo,
            priority=self.calculate_transport_priority(cargo)
        )
        return await self.task_manager.submit_task(task)
```

### 庫存管理整合
```python
# wcs/inventory_manager.py
class InventoryManager:
    def __init__(self):
        self.db_proxy = DatabaseProxy()
        
    async def allocate_storage_location(self, cargo_info):
        """分配儲存位置"""
        available_locations = await self.get_available_locations()
        optimal_location = self.select_optimal_location(
            cargo_info, available_locations
        )
        await self.reserve_location(optimal_location, cargo_info)
        return optimal_location
```

## 交通管制系統

### 交通規則
```python
# wcs/traffic_manager.py
class TrafficManager:
    def __init__(self):
        self.traffic_rules = self.load_traffic_rules()
        self.agv_positions = {}
        
    async def check_path_conflicts(self, agv_id, planned_path):
        """檢查路徑衝突"""
        for other_agv, other_path in self.active_paths.items():
            if agv_id != other_agv:
                conflicts = self.detect_conflicts(planned_path, other_path)
                if conflicts:
                    return await self.resolve_conflicts(conflicts)
        return planned_path
        
    def detect_intersection_conflicts(self, path1, path2):
        """檢測路徑交叉點衝突"""
        intersections = self.find_path_intersections(path1, path2)
        return [i for i in intersections if self.is_time_conflict(i)]
```

### 優先權管理
```python
# 緊急任務優先權處理
class PriorityManager:
    PRIORITY_LEVELS = {
        'EMERGENCY': 1,    # 緊急情況
        'HIGH': 2,         # 高優先權
        'NORMAL': 3,       # 一般任務
        'LOW': 4           # 低優先權
    }
    
    def calculate_dynamic_priority(self, task):
        """動態計算任務優先權"""
        base_priority = task.priority
        time_factor = self.calculate_time_urgency(task)
        resource_factor = self.calculate_resource_availability(task)
        return base_priority * time_factor * resource_factor
```

## 系統配置

### WCS配置
```yaml
# /app/config/agvc/wcs_config.yaml
wcs:
  task_scheduling:
    max_concurrent_tasks: 50
    task_timeout: 600        # 秒
    retry_attempts: 3
    
  path_planning:
    algorithm: "A_STAR"
    conflict_resolution: "TIME_BASED"
    safety_margin: 1.0       # 公尺
    
  kuka_integration:
    fleet_manager_url: "http://kuka-fleet:8080"
    sync_interval: 5.0       # 秒
    timeout: 30.0           # 秒
```

### 倉庫地圖
```yaml
# 倉庫地圖配置
warehouse_map:
  dimensions:
    width: 100.0    # 公尺
    height: 50.0    # 公尺
    
  zones:
    - name: "STORAGE_A"
      type: "storage"
      bounds: {x: [0, 30], y: [0, 25]}
      
    - name: "PICKUP_ZONE"  
      type: "pickup"
      bounds: {x: [30, 50], y: [0, 25]}
      
    - name: "CHARGE_AREA"
      type: "charging"
      bounds: {x: [80, 100], y: [40, 50]}
```

## 測試與調試

### 任務測試
```bash
# 提交測試任務
ros2 service call /wcs/submit_task wcs_msgs/srv/SubmitTask "{task: {type: 'TRANSPORT', start: {x: 0, y: 0}, end: {x: 10, y: 10}}}"

# 查看任務佇列
ros2 topic echo /wcs/task_queue_status

# AGV分配狀態
ros2 topic echo /wcs/agv_assignments
```

### KUKA整合測試
```bash
# 測試KUKA連線
ros2 service call /kuka_wcs/test_connection

# 同步AGV狀態
ros2 service call /kuka_wcs/sync_fleet_status

# 查看KUKA任務狀態
ros2 topic echo /kuka_wcs/order_status
```

## 性能監控

### 系統指標
```python
# wcs/performance_monitor.py
class PerformanceMonitor:
    def collect_metrics(self):
        return {
            'tasks_completed_per_hour': self.calculate_throughput(),
            'average_task_completion_time': self.calculate_avg_completion(),
            'agv_utilization_rate': self.calculate_utilization(),
            'path_planning_efficiency': self.calculate_path_efficiency()
        }
```

### 效能最佳化
- **任務批次處理**: 合併相似任務提高效率
- **路徑快取**: 常用路徑快取減少計算時間
- **預測性調度**: 基於歷史數據預測任務需求
- **負載均衡**: 平衡AGV工作負載

## 故障排除

### 常見問題
1. **任務分配失敗**: 檢查AGV可用性與任務驗證
2. **路徑規劃錯誤**: 驗證地圖數據與障礙物信息
3. **KUKA通訊異常**: 檢查網路連線與API端點
4. **交通衝突**: 檢查交通規則與衝突解決邏輯

### 診斷工具
```bash
# WCS系統狀態
ros2 service call /wcs/get_system_status

# 任務執行統計
ros2 topic echo /wcs/execution_statistics

# KUKA連線診斷
ros2 run kuka_wcs connection_diagnostics
```

### 緊急處理
```bash
# 暫停所有任務
ros2 service call /wcs/pause_all_tasks

# 清空任務佇列
ros2 service call /wcs/clear_task_queue

# 重置WCS系統
ros2 service call /wcs/reset_system
```

## 安全與可靠性

### 安全機制
- **任務驗證**: 嚴格的任務合法性檢查
- **路徑安全**: 確保規劃路徑的安全性
- **緊急停止**: 全車隊緊急停止機制
- **故障隔離**: 故障AGV自動隔離

### 可靠性保證
- **任務持久化**: 重要任務數據持久化存儲
- **狀態恢復**: 系統重啟後狀態恢復
- **冗餘設計**: 關鍵組件冗餘配置
- **容錯機制**: 自動錯誤檢測與恢復

## 重要提醒
- WCS決策影響整個車隊效率，演算法調整需謹慎
- KUKA整合需考慮系統版本相容性
- 路徑規劃需考慮實際倉庫佈局約束
- 必須在AGVC容器內運行