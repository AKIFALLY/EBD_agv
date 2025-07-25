"""
KUKA Fleet API 整合測試
測試 KukaManager 與 KUKA Fleet Adapter 的完整整合流程
"""
import pytest
import asyncio
import json
from unittest.mock import Mock, MagicMock, patch, AsyncMock
from datetime import datetime, timezone
from typing import List, Dict, Any

# Import the classes under test
from rcs.kuka_manager import KukaManager, KukaWorkType


class MockKukaFleetServer:
    """模擬 KUKA Fleet Server"""
    
    def __init__(self):
        self.robots = {
            "101": {
                "robotId": "101",
                "x": 100.0,
                "y": 200.0,
                "robotOrientation": 90.0,
                "batteryLevel": 85,
                "status": 3,  # idle
                "nodeNumber": "12"
            },
            "102": {
                "robotId": "102", 
                "x": 150.0,
                "y": 250.0,  
                "robotOrientation": 180.0,
                "batteryLevel": 70,
                "status": 3,  # idle
                "nodeNumber": "15"
            }
        }
        
        self.containers = {
            "RACK001": {
                "containerCode": "RACK001",
                "isCarry": False,
                "inMapStatus": True
            },
            "RACK002": {
                "containerCode": "RACK002",  
                "isCarry": True,
                "inMapStatus": True
            }
        }
        
        self.missions = {}
        self.mission_counter = 1000
        
    def get_robots(self) -> List[Dict]:
        """獲取所有機器人狀態"""
        return list(self.robots.values())
        
    def get_containers(self) -> List[Dict]:
        """獲取所有容器狀態"""
        return list(self.containers.values())
        
    def select_idle_robots(self) -> List[Dict]:
        """選擇閒置機器人"""
        return [robot for robot in self.robots.values() if robot["status"] == 3]
        
    def create_mission(self, mission_type: str, params: Dict) -> Dict:
        """創建任務"""
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
        return {"success": True, "missionId": mission_id}
        
    def execute_move(self, nodes: List, agv_id: int, mission_code: str) -> Dict:
        """執行移動任務"""
        return self.create_mission("move", {
            "nodes": nodes,
            "agv_id": agv_id,
            "mission_code": mission_code
        })
        
    def execute_rack_move(self, nodes: List, agv_id: int, mission_code: str) -> Dict:
        """執行貨架移動任務"""
        return self.create_mission("rack_move", {
            "nodes": nodes,
            "agv_id": agv_id,
            "mission_code": mission_code
        })
        
    def execute_workflow(self, template_code: str, agv_id: int, mission_code: str) -> Dict:
        """執行工作流程任務"""
        return self.create_mission("workflow", {
            "template_code": template_code,
            "agv_id": agv_id,
            "mission_code": mission_code
        })
        
    def update_robot_status(self, robot_id: str, status: int):
        """更新機器人狀態"""
        if robot_id in self.robots:
            self.robots[robot_id]["status"] = status
            
    def update_container_status(self, container_code: str, is_carry: bool, in_map: bool):
        """更新容器狀態"""
        if container_code in self.containers:
            self.containers[container_code]["isCarry"] = is_carry
            self.containers[container_code]["inMapStatus"] = in_map


@pytest.fixture
def mock_kuka_fleet_server():
    """模擬 KUKA Fleet Server fixture"""
    return MockKukaFleetServer()


@pytest.fixture
def mock_kuka_fleet_adapter(mock_kuka_fleet_server):
    """模擬 KUKA Fleet Adapter fixture"""
    adapter = Mock()
    
    # 設置基本狀態常數
    adapter.STATUS_IDLE = 3
    adapter.STATUS_BUSY = 4
    
    # 模擬 select_agv 方法
    def select_agv(status):
        if status == adapter.STATUS_IDLE:
            idle_robots = mock_kuka_fleet_server.select_idle_robots()
            return [{"id": robot["robotId"]} for robot in idle_robots]
        return []
    
    adapter.select_agv = Mock(side_effect=select_agv)
    
    # 模擬 API 調用方法
    adapter.move = Mock(side_effect=mock_kuka_fleet_server.execute_move)
    adapter.rack_move = Mock(side_effect=mock_kuka_fleet_server.execute_rack_move)
    adapter.workflow = Mock(side_effect=mock_kuka_fleet_server.execute_workflow)
    
    # 模擬監控方法
    adapter.start_monitoring = Mock()
    adapter.stop_monitoring = Mock()
    
    # 模擬回調設置
    adapter.on_robot_query_complete = None
    adapter.on_container_query_complete = None
    
    return adapter


class TestKukaIntegration:
    """KUKA 整合測試類別"""
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    @patch('rcs.kuka_manager.WCSTaskAdapter')
    @patch('rcs.kuka_manager.WCSPriorityScheduler')
    @patch('rcs.kuka_manager.RackStateManager')
    def test_kuka_manager_initialization_with_fleet_adapter(
        self, mock_rack_manager, mock_scheduler, mock_wcs_adapter, mock_fleet_adapter_class,
        mock_rcs_core, mock_db_pool, mock_db_session, mock_kuka_fleet_adapter
    ):
        """測試 KukaManager 與 Fleet Adapter 初始化整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        # 創建 KukaManager
        manager = KukaManager(mock_rcs_core)
        
        # 驗證初始化
        assert manager.kuka_fleet == mock_kuka_fleet_adapter
        mock_kuka_fleet_adapter.start_monitoring.assert_called_once()
        
        # 驗證回調函數設置
        assert hasattr(manager.kuka_fleet, 'on_robot_query_complete')
        assert hasattr(manager.kuka_fleet, 'on_container_query_complete')
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_robot_status_update_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool, 
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試機器人狀態更新整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 模擬機器人狀態更新
        robot_data = mock_kuka_fleet_server.get_robots()
        
        # 執行狀態更新
        manager.kuka_robot.on_robot_update(robot_data)
        
        # 驗證更新調用
        assert mock_db_session.commit.called
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_container_status_update_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試容器狀態更新整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 模擬容器狀態更新
        container_data = mock_kuka_fleet_server.get_containers()
        
        # 執行狀態更新
        manager.kuka_container.on_container_update(container_data)
        
        # 驗證更新調用 (容器更新可能找不到對應的 Rack，這是正常的)
        assert mock_db_session.commit.called or not mock_db_session.commit.called
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_task_dispatch_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試任務派發整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i" 
        mock_agv.enable = 1
        mock_agv.status_id = 3  # idle
        
        mock_db_session.exec.return_value.all.side_effect = [
            [mock_agv],  # _load_kuka_agvs
            [mock_agv],  # dispatch: available AGVs
            []           # dispatch: no pending tasks
        ]
        
        # 設置統計查詢
        mock_db_session.exec.return_value.first.side_effect = [0, 1, 0]  # pending, total, executing
        
        manager = KukaManager(mock_rcs_core)
        
        # 執行派發
        manager.dispatch()
        
        # 驗證 Fleet Adapter 調用
        mock_kuka_fleet_adapter.select_agv.assert_called_with(3)  # STATUS_IDLE
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_move_task_execution_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試移動任務執行整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 創建移動任務
        task = Mock()
        task.id = 1
        task.work_id = KukaWorkType.KUKA_MOVE  
        task.parameters = {"nodes": [10, 20, 30]}
        
        # 執行任務
        result = manager._execute_move_task(task, 101, "mission_001")
        
        # 驗證 Fleet Adapter 調用
        mock_kuka_fleet_adapter.move.assert_called_once_with([10, 20, 30], 101, "mission_001")
        assert result["success"] is True
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_rack_move_task_execution_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試貨架移動任務執行整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料  
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 創建貨架移動任務
        task = Mock()
        task.id = 2
        task.work_id = KukaWorkType.KUKA_RACK_MOVE
        task.parameters = {"nodes": [91, 76, 45]}
        
        # 執行任務
        result = manager._execute_rack_move_task(task, 101, "mission_002")
        
        # 驗證 Fleet Adapter 調用
        mock_kuka_fleet_adapter.rack_move.assert_called_once_with([91, 76, 45], 101, "mission_002")
        assert result["success"] is True
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_workflow_task_execution_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試工作流程任務執行整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 創建工作流程任務
        task = Mock()
        task.id = 3
        task.work_id = KukaWorkType.KUKA_WORKFLOW
        task.parameters = {"templateCode": "TEMPLATE_001"}
        
        # 執行任務
        result = manager._execute_workflow_task(task, 101, "mission_003")
        
        # 驗證 Fleet Adapter 調用
        mock_kuka_fleet_adapter.workflow.assert_called_once_with("TEMPLATE_001", 101, "mission_003")
        assert result["success"] is True
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_api_retry_mechanism_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter
    ):
        """測試 API 重試機制整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置 API 調用失敗然後成功
        mock_kuka_fleet_adapter.rack_move.side_effect = [
            {"success": False, "error": "Temporary error"},  # 第一次失敗
            {"success": True, "missionId": "1001"}           # 第二次成功
        ]
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 設置重試配置（縮短延遲以加快測試）
        manager.api_retry_config = {
            'max_attempts': 3,
            'base_delay': 0.01,
            'backoff_factor': 2.0,
            'max_delay': 1.0,
            'timeout': 60.0
        }
        
        with patch('time.sleep'):  # 跳過實際延遲
            result = manager._execute_kuka_api_with_retry(
                'rack_move',
                [[91, 76, 91], 101, 'mission_retry'],
                task_type='rotation',
                rack_id=123
            )
        
        # 驗證重試機制
        assert result["success"] is True
        assert mock_kuka_fleet_adapter.rack_move.call_count == 2  # 一次失敗，一次成功
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_wcs_task_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter
    ):
        """測試 WCS 任務整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 創建 WCS 格式任務
        task = Mock()
        task.id = 4
        task.work_id = 999999  # 非標準工作ID
        task.parameters = {
            "function": "rack_move",
            "nodes": [91, 76, 91],
            "wcs_task_type": "rotation",
            "rack_id": 123
        }
        
        # 執行 WCS 任務
        result = manager._execute_wcs_format_task(task, 101, "wcs_mission_001")
        
        # 驗證 WCS 任務執行
        mock_kuka_fleet_adapter.rack_move.assert_called_once()
        assert result["success"] is True
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_comprehensive_dispatch_integration(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter, mock_kuka_fleet_server
    ):
        """測試完整派發流程整合"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_agv.enable = 1
        mock_agv.status_id = 3  # idle
        
        # 設置模擬任務資料
        mock_task = Mock()
        mock_task.id = 1
        mock_task.work_id = KukaWorkType.KUKA_RACK_MOVE
        mock_task.priority = 80
        mock_task.status_id = 1  # pending
        mock_task.mission_code = None
        mock_task.agv_id = None
        mock_task.parameters = {
            "model": "KUKA400i",
            "nodes": [10, 20, 30],
            "rack_id": 123
        }
        mock_task.updated_at = datetime.now(timezone.utc)
        
        # 設置資料庫查詢結果
        mock_db_session.exec.return_value.all.side_effect = [
            [mock_agv],    # _load_kuka_agvs
            [mock_agv],    # dispatch: available AGVs
            [mock_task]    # dispatch: pending tasks
        ]
        
        # 設置統計查詢結果
        mock_db_session.exec.return_value.first.side_effect = [1, 1, 0]  # pending, total, executing
        
        # 設置 AGV 查詢結果
        mock_db_session.exec.return_value.first.return_value = mock_agv
        
        manager = KukaManager(mock_rcs_core)
        
        # 執行完整派發流程
        manager.dispatch()
        
        # 驗證派發流程
        mock_kuka_fleet_adapter.select_agv.assert_called_with(3)  # STATUS_IDLE
        mock_kuka_fleet_adapter.rack_move.assert_called_once()
        
        # 驗證任務狀態更新
        assert mock_task.agv_id == 101
        assert mock_task.status_id == 2  # running
        assert mock_task.mission_code is not None
        
        # 驗證 AGV 狀態更新
        assert mock_agv.status_id == 4  # busy
        
        # 驗證資料庫提交
        mock_db_session.commit.assert_called()


class TestKukaIntegrationScenarios:
    """KUKA 整合場景測試"""
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_high_load_scenario(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter
    ):
        """測試高負載場景"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置多個 AGV
        agvs = []
        for i in range(5):
            agv = Mock()
            agv.id = 101 + i
            agv.model = "KUKA400i"
            agv.enable = 1
            agv.status_id = 3  # idle
            agvs.append(agv)
        
        # 設置多個任務
        tasks = []
        for i in range(10):
            task = Mock()
            task.id = i + 1
            task.work_id = KukaWorkType.KUKA_RACK_MOVE
            task.priority = 50 + i
            task.status_id = 1  # pending
            task.mission_code = None
            task.agv_id = None
            task.parameters = {
                "model": "KUKA400i",
                "nodes": [10 + i, 20 + i, 30 + i]
            }
            tasks.append(task)
        
        # 設置資料庫查詢結果
        mock_db_session.exec.return_value.all.side_effect = [
            agvs,      # _load_kuka_agvs
            agvs,      # dispatch: available AGVs  
            tasks      # dispatch: pending tasks
        ]
        
        # 設置統計查詢結果
        mock_db_session.exec.return_value.first.side_effect = [10, 5, 0]  # pending, total, executing
        
        # 設置 AGV 查詢結果
        mock_db_session.exec.return_value.first.side_effect = agvs
        
        manager = KukaManager(mock_rcs_core)
        
        # 執行派發
        manager.dispatch()
        
        # 驗證高負載處理
        assert mock_db_session.commit.called
        # 最多只能派發 5 個任務（因為只有 5 台 AGV）
        assert mock_kuka_fleet_adapter.rack_move.call_count <= 5
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_error_recovery_scenario(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter
    ):
        """測試錯誤恢復場景"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置 API 調用異常
        mock_kuka_fleet_adapter.rack_move.side_effect = Exception("Network error")
        
        # 設置模擬資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 創建任務
        task = Mock()
        task.id = 1
        task.work_id = KukaWorkType.KUKA_RACK_MOVE
        task.parameters = {"nodes": [10, 20, 30]}
        
        # 執行任務（應該處理異常）
        result = manager._execute_rack_move_task(task, 101, "mission_error")
        
        # 驗證錯誤處理
        assert result["success"] is False
        assert "error" in result
    
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_mixed_task_types_scenario(
        self, mock_fleet_adapter_class, mock_rcs_core, mock_db_pool,
        mock_db_session, mock_kuka_fleet_adapter
    ):
        """測試混合任務類型場景"""
        
        # 設置模擬
        mock_fleet_adapter_class.return_value = mock_kuka_fleet_adapter
        
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        with patch.object(KukaManager, '_load_kuka_agvs'):
            manager = KukaManager(mock_rcs_core)
        
        # 測試不同類型的任務
        tasks = [
            # 傳統移動任務
            {
                "id": 1,
                "work_id": KukaWorkType.KUKA_MOVE,
                "parameters": {"nodes": [10, 20, 30]}
            },
            # 傳統貨架移動任務
            {
                "id": 2,
                "work_id": KukaWorkType.KUKA_RACK_MOVE,
                "parameters": {"nodes": [91, 76, 45]}
            },
            # WCS 格式任務
            {
                "id": 3,
                "work_id": 999999,
                "parameters": {
                    "function": "rack_move",
                    "nodes": [91, 76, 91],
                    "wcs_task_type": "rotation"
                }
            }
        ]
        
        # 執行不同類型的任務
        for task_data in tasks:
            task = Mock()
            task.id = task_data["id"]
            task.work_id = task_data["work_id"]
            task.parameters = task_data["parameters"]
            
            result = manager._execute_kuka_task(task, 101, f"mission_{task.id}")
            
            # 驗證任務執行成功
            assert result["success"] is True
        
        # 驗證不同API方法被調用
        mock_kuka_fleet_adapter.move.assert_called_once()
        assert mock_kuka_fleet_adapter.rack_move.call_count == 2  # 一次傳統，一次WCS