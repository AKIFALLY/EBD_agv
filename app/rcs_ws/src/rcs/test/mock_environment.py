"""
KUKA 測試環境模擬工具
提供完整的測試環境模擬，包括資料庫、KUKA Fleet API 等
"""
import json
import sqlite3
import threading
import time
from datetime import datetime, timezone
from typing import Dict, List, Any, Optional
from unittest.mock import Mock, MagicMock
from dataclasses import dataclass, asdict


@dataclass
class MockAGV:
    """模擬 AGV 資料結構"""
    id: int
    name: str
    model: str
    enable: int
    status_id: int
    x: float
    y: float
    heading: float
    battery: int
    last_node_id: Optional[int] = None


@dataclass
class MockRack:
    """模擬 Rack 資料結構"""
    id: int
    name: str
    agv_id: Optional[int]
    location_id: int
    product_id: Optional[int]
    is_carry: int
    is_in_map: int
    is_docked: int
    status_id: int
    direction: int


@dataclass
class MockTask:
    """模擬 Task 資料結構"""
    id: int
    name: str
    description: str
    work_id: int
    status_id: int
    priority: int
    room_id: Optional[int]
    agv_id: Optional[int]
    mission_code: Optional[str]
    parameters: Dict[str, Any]
    created_at: datetime
    updated_at: datetime


class MockDatabase:
    """模擬資料庫"""
    
    def __init__(self):
        self.agvs: Dict[int, MockAGV] = {}
        self.racks: Dict[int, MockRack] = {}
        self.tasks: Dict[int, MockTask] = {}
        self.task_counter = 1
        self.rack_counter = 1
        self._initialize_sample_data()
    
    def _initialize_sample_data(self):
        """初始化範例資料"""
        # 添加 KUKA AGV
        self.agvs = {
            101: MockAGV(101, "KUKA101", "KUKA400i", 1, 3, 100.0, 200.0, 45.0, 85),
            102: MockAGV(102, "KUKA102", "KUKA400i", 1, 3, 150.0, 250.0, 90.0, 70),
            103: MockAGV(103, "KUKA103", "KUKA400i", 1, 4, 200.0, 300.0, 135.0, 60),  # busy
        }
        
        # 添加 Rack
        self.racks = {
            1: MockRack(1, "RACK001", None, 10, None, 0, 1, 0, 1, 0),
            2: MockRack(2, "RACK002", 103, 15, None, 1, 1, 0, 1, 0),  # being carried
            3: MockRack(3, "RACK003", None, 20, None, 0, 1, 0, 1, 0),
        }
    
    def get_agvs(self, **filters) -> List[MockAGV]:
        """取得 AGV 列表"""
        agvs = list(self.agvs.values())
        
        # 應用篩選條件
        if 'model' in filters:
            agvs = [agv for agv in agvs if agv.model == filters['model']]
        if 'enable' in filters:
            agvs = [agv for agv in agvs if agv.enable == filters['enable']]
        if 'status_id' in filters:
            agvs = [agv for agv in agvs if agv.status_id == filters['status_id']]
        if 'id_in' in filters:
            agvs = [agv for agv in agvs if agv.id in filters['id_in']]
            
        return agvs
    
    def get_racks(self, **filters) -> List[MockRack]:
        """取得 Rack 列表"""
        racks = list(self.racks.values())
        
        # 應用篩選條件
        if 'name' in filters:
            racks = [rack for rack in racks if rack.name == filters['name']]
        if 'is_carry' in filters:
            racks = [rack for rack in racks if rack.is_carry == filters['is_carry']]
        if 'is_in_map' in filters:
            racks = [rack for rack in racks if rack.is_in_map == filters['is_in_map']]
            
        return racks
    
    def get_tasks(self, **filters) -> List[MockTask]:
        """取得 Task 列表"""
        tasks = list(self.tasks.values())
        
        # 應用篩選條件
        if 'status_id' in filters:
            tasks = [task for task in tasks if task.status_id == filters['status_id']]
        if 'model' in filters:
            tasks = [task for task in tasks if task.parameters.get('model') == filters['model']]
        if 'mission_code' in filters:
            if filters['mission_code'] is None:
                tasks = [task for task in tasks if task.mission_code is None]
            else:
                tasks = [task for task in tasks if task.mission_code == filters['mission_code']]
                
        return tasks
    
    def add_task(self, task_data: Dict[str, Any]) -> MockTask:
        """添加任務"""
        task = MockTask(
            id=self.task_counter,
            **task_data,
            created_at=datetime.now(timezone.utc),
            updated_at=datetime.now(timezone.utc)
        )
        self.tasks[self.task_counter] = task
        self.task_counter += 1
        return task
    
    def update_agv(self, agv_id: int, **updates):
        """更新 AGV"""
        if agv_id in self.agvs:
            for key, value in updates.items():
                if hasattr(self.agvs[agv_id], key):
                    setattr(self.agvs[agv_id], key, value)
    
    def update_rack(self, rack_id: int, **updates):
        """更新 Rack"""
        if rack_id in self.racks:
            for key, value in updates.items():
                if hasattr(self.racks[rack_id], key):
                    setattr(self.racks[rack_id], key, value)
    
    def update_task(self, task_id: int, **updates):
        """更新 Task"""
        if task_id in self.tasks:
            for key, value in updates.items():
                if hasattr(self.tasks[task_id], key):
                    setattr(self.tasks[task_id], key, value)
            self.tasks[task_id].updated_at = datetime.now(timezone.utc)


class MockKukaFleetAPI:
    """模擬 KUKA Fleet API"""
    
    def __init__(self, database: MockDatabase):
        self.db = database
        self.missions: Dict[str, Dict] = {}
        self.mission_counter = 1000
        self.is_online = True
        self.response_delay = 0.1  # 模擬網路延遲
    
    def get_robots(self) -> List[Dict]:
        """取得所有機器人狀態"""
        if not self.is_online:
            raise ConnectionError("KUKA Fleet API is offline")
        
        time.sleep(self.response_delay)
        
        robots = []
        for agv in self.db.get_agvs(model="KUKA400i", enable=1):
            robots.append({
                "robotId": str(agv.id),
                "x": agv.x * 12.5,  # 轉換為 KUKA 單位
                "y": agv.y * 12.5,
                "robotOrientation": self._map_angle_to_kuka_angle(agv.heading),
                "batteryLevel": agv.battery,
                "status": agv.status_id,
                "nodeNumber": str(agv.last_node_id) if agv.last_node_id else ""
            })
        
        return robots
    
    def get_containers(self) -> List[Dict]:
        """取得所有容器狀態"""
        if not self.is_online:
            raise ConnectionError("KUKA Fleet API is offline")
        
        time.sleep(self.response_delay)
        
        containers = []
        for rack in self.db.get_racks():
            containers.append({
                "containerCode": rack.name,
                "isCarry": bool(rack.is_carry),
                "inMapStatus": bool(rack.is_in_map)
            })
        
        return containers
    
    def select_agv(self, status: int) -> List[Dict]:
        """選擇指定狀態的 AGV"""
        if not self.is_online:
            raise ConnectionError("KUKA Fleet API is offline")
        
        time.sleep(self.response_delay)
        
        agvs = self.db.get_agvs(model="KUKA400i", enable=1, status_id=status)
        return [{"id": str(agv.id)} for agv in agvs]
    
    def move(self, nodes: List, agv_id: int, mission_code: str) -> Dict:
        """執行移動任務"""
        return self._create_mission("move", {
            "nodes": nodes,
            "agv_id": agv_id,
            "mission_code": mission_code
        })
    
    def rack_move(self, nodes: List, agv_id: int, mission_code: str) -> Dict:
        """執行貨架移動任務"""
        return self._create_mission("rack_move", {
            "nodes": nodes,
            "agv_id": agv_id,
            "mission_code": mission_code
        })
    
    def workflow(self, template_code: str, agv_id: int, mission_code: str) -> Dict:
        """執行工作流程任務"""
        return self._create_mission("workflow", {
            "template_code": template_code,
            "agv_id": agv_id,
            "mission_code": mission_code
        })
    
    def _create_mission(self, mission_type: str, params: Dict) -> Dict:
        """創建任務"""
        if not self.is_online:
            return {"success": False, "error": "KUKA Fleet API is offline"}
        
        time.sleep(self.response_delay)
        
        mission_id = str(self.mission_counter)
        self.mission_counter += 1
        
        mission = {
            "missionId": mission_id,
            "type": mission_type,
            "params": params,
            "status": "created",
            "created_at": datetime.now(timezone.utc).isoformat()
        }
        
        self.missions[mission_id] = mission
        
        # 更新 AGV 狀態為忙碌
        if "agv_id" in params:
            self.db.update_agv(params["agv_id"], status_id=4)  # busy
        
        return {"success": True, "missionId": mission_id}
    
    def _map_angle_to_kuka_angle(self, map_angle: float) -> float:
        """將地圖角度轉換為 KUKA 角度"""
        # 反向轉換 kuka_angle_2_map_angle
        return (-1 * (map_angle + 180 - 540 % 360)) + 90
    
    def set_offline(self):
        """設置 API 離線"""
        self.is_online = False
    
    def set_online(self):
        """設置 API 上線"""
        self.is_online = True
    
    def set_response_delay(self, delay: float):
        """設置響應延遲"""
        self.response_delay = delay


class MockTestEnvironment:
    """完整的模擬測試環境"""
    
    def __init__(self):
        self.database = MockDatabase()
        self.kuka_api = MockKukaFleetAPI(self.database)
        self.rcs_core = self._create_mock_rcs_core()
        self.scenarios = {}
    
    def _create_mock_rcs_core(self):
        """創建模擬 RCS Core"""
        rcs_core = Mock()
        rcs_core.get_logger.return_value = self._create_mock_logger()
        rcs_core.db_pool = self._create_mock_db_pool()
        return rcs_core
    
    def _create_mock_logger(self):
        """創建模擬 Logger"""
        logger = Mock()
        logger.debug = Mock()
        logger.info = Mock()
        logger.warning = Mock()
        logger.error = Mock()
        return logger
    
    def _create_mock_db_pool(self):
        """創建模擬資料庫連線池"""
        db_pool = Mock()
        db_session = self._create_mock_db_session()
        db_pool.get_session.return_value.__enter__ = Mock(return_value=db_session)
        db_pool.get_session.return_value.__exit__ = Mock(return_value=None)
        return db_pool
    
    def _create_mock_db_session(self):
        """創建模擬資料庫會話"""
        session = Mock()
        
        # 模擬 exec 方法
        def mock_exec(query):
            mock_result = Mock()
            
            # 根據查詢類型返回不同結果
            if hasattr(query, 'where'):
                # 這是一個 select 查詢
                mock_result.all = Mock(return_value=self._handle_select_query(query))
                mock_result.first = Mock(return_value=self._handle_select_query(query, first=True))
            else:
                # 這是一個聚合查詢
                mock_result.first = Mock(return_value=0)
            
            return mock_result
        
        session.exec = Mock(side_effect=mock_exec)
        session.add = Mock()
        session.commit = Mock()
        session.rollback = Mock()
        
        return session
    
    def _handle_select_query(self, query, first=False):
        """處理 select 查詢"""
        # 這裡簡化處理，實際應該解析 SQLModel 查詢
        # 返回預設資料
        if first:
            agvs = self.database.get_agvs(model="KUKA400i", enable=1)
            return agvs[0] if agvs else None
        else:
            return self.database.get_agvs(model="KUKA400i", enable=1)
    
    def add_test_scenario(self, name: str, scenario_data: Dict[str, Any]):
        """添加測試場景"""
        self.scenarios[name] = scenario_data
    
    def load_scenario(self, name: str):
        """載入測試場景"""
        if name not in self.scenarios:
            raise ValueError(f"Unknown scenario: {name}")
        
        scenario = self.scenarios[name]
        
        # 重置資料庫
        self.database = MockDatabase()
        
        # 載入場景資料
        if 'agvs' in scenario:
            for agv_data in scenario['agvs']:
                agv = MockAGV(**agv_data)
                self.database.agvs[agv.id] = agv
        
        if 'tasks' in scenario:
            for task_data in scenario['tasks']:
                task = MockTask(**task_data)
                self.database.tasks[task.id] = task
        
        if 'racks' in scenario:
            for rack_data in scenario['racks']:
                rack = MockRack(**rack_data)
                self.database.racks[rack.id] = rack
    
    def simulate_network_issues(self, duration: float = 5.0):
        """模擬網路問題"""
        self.kuka_api.set_offline()
        
        def restore_network():
            time.sleep(duration)
            self.kuka_api.set_online()
        
        thread = threading.Thread(target=restore_network)
        thread.daemon = True
        thread.start()
    
    def simulate_high_latency(self, delay: float = 2.0):
        """模擬高延遲"""
        self.kuka_api.set_response_delay(delay)
    
    def get_system_state(self) -> Dict[str, Any]:
        """取得系統狀態"""
        return {
            "agvs": [asdict(agv) for agv in self.database.agvs.values()],
            "racks": [asdict(rack) for rack in self.database.racks.values()],
            "tasks": [asdict(task) for task in self.database.tasks.values()],
            "missions": self.kuka_api.missions,
            "api_online": self.kuka_api.is_online,
            "api_delay": self.kuka_api.response_delay
        }
    
    def export_state(self, filepath: str):
        """匯出系統狀態到檔案"""
        state = self.get_system_state()
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(state, f, indent=2, default=str)
    
    def import_state(self, filepath: str):
        """從檔案匯入系統狀態"""
        with open(filepath, 'r', encoding='utf-8') as f:
            state = json.load(f)
        
        # 重建資料庫狀態
        self.database = MockDatabase()
        self.database.agvs = {}
        self.database.racks = {}
        self.database.tasks = {}
        
        for agv_data in state.get('agvs', []):
            agv = MockAGV(**agv_data)
            self.database.agvs[agv.id] = agv
        
        for rack_data in state.get('racks', []):
            rack = MockRack(**rack_data)
            self.database.racks[rack.id] = rack
        
        for task_data in state.get('tasks', []):
            # 處理日期時間字串
            if isinstance(task_data['created_at'], str):
                task_data['created_at'] = datetime.fromisoformat(task_data['created_at'])
            if isinstance(task_data['updated_at'], str):
                task_data['updated_at'] = datetime.fromisoformat(task_data['updated_at'])
            
            task = MockTask(**task_data)
            self.database.tasks[task.id] = task
        
        # 重建 API 狀態
        self.kuka_api.missions = state.get('missions', {})
        if state.get('api_online', True):
            self.kuka_api.set_online()
        else:
            self.kuka_api.set_offline()
        
        self.kuka_api.set_response_delay(state.get('api_delay', 0.1))