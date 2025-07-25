"""
KukaManager 單元測試
測試 KUKA 統一管理器的所有功能
"""
import pytest
from unittest.mock import Mock, MagicMock, patch
from datetime import datetime, timezone

# Import the class under test
from rcs.kuka_manager import KukaManager, KukaWorkType


class TestKukaWorkType:
    """KukaWorkType 測試類別"""

    def test_get_work_name(self):
        """測試取得工作類型名稱"""
        assert KukaWorkType.get_work_name(KukaWorkType.KUKA_MOVE) == "KUKA移動"
        assert KukaWorkType.get_work_name(KukaWorkType.KUKA_RACK_MOVE) == "KUKA移動貨架"
        assert KukaWorkType.get_work_name(KukaWorkType.KUKA_WORKFLOW) == "KUKA工作流程"
        assert "未知工作類型" in KukaWorkType.get_work_name(999999)

    def test_is_kuka_work(self):
        """測試檢查是否為 KUKA 工作類型"""
        assert KukaWorkType.is_kuka_work(KukaWorkType.KUKA_MOVE) is True
        assert KukaWorkType.is_kuka_work(KukaWorkType.KUKA_RACK_MOVE) is True
        assert KukaWorkType.is_kuka_work(KukaWorkType.KUKA_WORKFLOW) is True
        assert KukaWorkType.is_kuka_work(999999) is False

    def test_get_work_type_by_function(self):
        """測試根據 function 取得工作類型"""
        assert KukaWorkType.get_work_type_by_function("move") == KukaWorkType.KUKA_MOVE
        assert KukaWorkType.get_work_type_by_function("rack_move") == KukaWorkType.KUKA_RACK_MOVE
        assert KukaWorkType.get_work_type_by_function("workflow") == KukaWorkType.KUKA_WORKFLOW
        assert KukaWorkType.get_work_type_by_function("unknown") == KukaWorkType.KUKA_WORKFLOW


class TestKukaManager:
    """KukaManager 測試類別"""

    @patch('rcs.kuka_manager.WCSPriorityScheduler')
    @patch('rcs.kuka_manager.WCSTaskAdapter')
    @patch('rcs.kuka_manager.RackStateManager')
    @patch('rcs.kuka_manager.KukaContainer')
    @patch('rcs.kuka_manager.KukaRobot')
    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_init(self, mock_kuka_fleet_adapter, mock_kuka_robot, mock_kuka_container,
                  mock_rack_state_manager, mock_wcs_adapter, mock_priority_scheduler,
                  mock_rcs_core, mock_db_pool, mock_db_session):
        """測試 KukaManager 初始化"""
        # 設置模擬 AGV 資料
        mock_agv = Mock()
        mock_agv.id = 101
        mock_agv.model = "KUKA400i"
        mock_db_session.exec.return_value.all.return_value = [mock_agv]
        
        manager = KukaManager(mock_rcs_core)
        
        assert manager.rcs_core == mock_rcs_core
        assert manager.logger == mock_rcs_core.get_logger.return_value
        assert manager.db_pool == mock_rcs_core.db_pool
        assert hasattr(manager, 'kuka_fleet')
        assert hasattr(manager, 'kuka_robot')
        assert hasattr(manager, 'kuka_container')
        assert hasattr(manager, 'wcs_adapter')
        assert hasattr(manager, 'priority_scheduler')
        assert hasattr(manager, 'rack_state_manager')

    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_load_kuka_agvs_success(self, mock_kuka_fleet_adapter, mock_rcs_core, 
                                  mock_db_pool, mock_db_session):
        """測試載入 KUKA AGV - 成功"""
        # 設置模擬 AGV 資料
        mock_agv1 = Mock()
        mock_agv1.id = 101
        mock_agv1.model = "KUKA400i"
        mock_agv1.enable = 1
        
        mock_agv2 = Mock()
        mock_agv2.id = 102
        mock_agv2.model = "KUKA400i"
        mock_agv2.enable = 1
        
        mock_db_session.exec.return_value.all.return_value = [mock_agv1, mock_agv2]
        
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.rcs_core = mock_rcs_core
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.db_pool = mock_rcs_core.db_pool
            manager.kuka_agvs = {}
            
            manager._load_kuka_agvs()
            
            assert len(manager.kuka_agvs) == 2
            assert 101 in manager.kuka_agvs
            assert 102 in manager.kuka_agvs

    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_load_kuka_agvs_no_db_pool(self, mock_kuka_fleet_adapter, mock_rcs_core, mock_logger):
        """測試載入 KUKA AGV - 無資料庫連線"""
        mock_rcs_core.db_pool = None
        
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.rcs_core = mock_rcs_core
            manager.logger = mock_logger
            manager.db_pool = None
            
            manager._load_kuka_agvs()
            
            mock_logger.error.assert_called_with("資料庫連線池不可用，無法載入 KUKA AGV。")

    def test_is_wcs_format_task(self, mock_rcs_core):
        """測試檢查是否為 WCS 格式任務"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 測試 WCS 任務標識
            task1 = Mock()
            task1.parameters = {"wcs_task_type": "rotation"}
            assert manager._is_wcs_format_task(task1) is True
            
            task2 = Mock()
            task2.parameters = {"wcs_task_id": "wcs_001"}
            assert manager._is_wcs_format_task(task2) is True
            
            # 測試 function 為 rack_move
            task3 = Mock()
            task3.parameters = {"function": "rack_move"}
            assert manager._is_wcs_format_task(task3) is True
            
            # 測試非 WCS 格式
            task4 = Mock()
            task4.parameters = {"model": "KUKA400i"}
            assert manager._is_wcs_format_task(task4) is False
            
            # 測試無參數
            task5 = Mock()
            task5.parameters = None
            assert manager._is_wcs_format_task(task5) is False

    def test_get_task_description_wcs_format(self, mock_rcs_core):
        """測試取得任務描述 - WCS 格式"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            task = Mock()
            task.id = 1
            task.priority = 80
            task.parameters = {
                "wcs_task_type": "rotation",
                "wcs_priority": 90
            }
            
            description = manager._get_task_description(task)
            assert description == "WCS-rotation (Priority: 90)"

    def test_get_task_description_traditional_format(self, mock_rcs_core):
        """測試取得任務描述 - 傳統格式"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            task = Mock()
            task.id = 1
            task.work_id = KukaWorkType.KUKA_MOVE
            task.parameters = {}
            
            description = manager._get_task_description(task)
            assert description == f"KUKA移動 (Work ID: {KukaWorkType.KUKA_MOVE})"

    def test_get_task_description_function_based(self, mock_rcs_core):
        """測試取得任務描述 - 基於 function"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            task = Mock()
            task.id = 1
            task.work_id = 999999  # 非 KUKA 工作類型
            task.parameters = {"function": "custom_function"}
            
            description = manager._get_task_description(task)
            assert description == "Function: custom_function"

    def test_validate_rotation_path_success(self, mock_rcs_core):
        """測試驗證旋轉路徑 - 成功"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 正確的旋轉路徑: 起點 -> 中間點 -> 回到起點
            nodes = [91, 76, 91]
            result = manager._validate_rotation_path(nodes)
            
            assert result['valid'] is True
            assert result['path_type'] == 'rotation'
            assert result['start_node'] == 91
            assert result['intermediate_node'] == 76
            assert result['end_node'] == 91

    def test_validate_rotation_path_wrong_length(self, mock_rcs_core):
        """測試驗證旋轉路徑 - 錯誤長度"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 錯誤長度 (只有2個節點)
            nodes = [91, 76]
            result = manager._validate_rotation_path(nodes)
            
            assert result['valid'] is False
            assert "需要3個節點" in result['error']

    def test_validate_rotation_path_different_endpoints(self, mock_rcs_core):
        """測試驗證旋轉路徑 - 起點終點不同"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 起點和終點不同
            nodes = [91, 76, 92]
            result = manager._validate_rotation_path(nodes)
            
            assert result['valid'] is False
            assert "起點(91)和終點(92)必須相同" in result['error']

    def test_validate_rotation_path_same_start_intermediate(self, mock_rcs_core):
        """測試驗證旋轉路徑 - 起點與中間點相同"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 起點與中間點相同
            nodes = [91, 91, 91]
            result = manager._validate_rotation_path(nodes)
            
            assert result['valid'] is False
            assert "移出點(91)不能與起點(91)相同" in result['error']

    def test_validate_move_path_success(self, mock_rcs_core):
        """測試驗證搬運路徑 - 成功"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 正確的搬運路徑
            nodes = [10, 20, 30, 40]
            result = manager._validate_move_path(nodes)
            
            assert result['valid'] is True
            assert result['path_type'] == 'move'
            assert result['start_node'] == 10
            assert result['end_node'] == 40
            assert result['intermediate_count'] == 2

    def test_validate_move_path_too_short(self, mock_rcs_core):
        """測試驗證搬運路徑 - 太短"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 路徑太短 (只有1個節點)
            nodes = [10]
            result = manager._validate_move_path(nodes)
            
            assert result['valid'] is False
            assert "至少需要2個節點" in result['error']

    def test_validate_move_path_too_long(self, mock_rcs_core):
        """測試驗證搬運路徑 - 太長"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 路徑太長 (超過20個節點)
            nodes = list(range(25))
            result = manager._validate_move_path(nodes)
            
            assert result['valid'] is False
            assert "路徑過長" in result['error']

    def test_build_system_context(self, mock_rcs_core, mock_db_session):
        """測試構建系統上下文"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            
            # 設置模擬資料庫查詢結果
            mock_db_session.exec.return_value.first.side_effect = [5, 3, 2]  # pending, total, executing
            
            context = manager._build_system_context(mock_db_session, 2)
            
            assert context['available_agvs'] == 2
            assert context['total_agvs'] == 3
            assert context['pending_tasks_count'] == 5
            assert context['executing_tasks_count'] == 2
            assert context['system_load_ratio'] == 2/3  # executing/total
            assert context['agv_utilization_ratio'] == 1/3  # (total-available)/total

    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_execute_kuka_api_with_retry_success(self, mock_kuka_fleet_adapter, mock_rcs_core):
        """測試 KUKA API 重試執行 - 成功"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.kuka_fleet = Mock()
            manager.kuka_fleet.rack_move.return_value = {"success": True}
            manager.api_retry_config = {
                'max_attempts': 3,
                'base_delay': 1.0,
                'backoff_factor': 2.0,
                'max_delay': 30.0,
                'timeout': 60.0
            }
            manager.api_stats = {
                'total_calls': 0,
                'successful_calls': 0,
                'failed_calls': 0,
                'retry_calls': 0,
                'average_response_time': 0.0
            }
            
            result = manager._execute_kuka_api_with_retry(
                'rack_move', 
                [[10, 20, 30], 101, 'mission_001'],
                task_type='normal',
                rack_id=123
            )
            
            assert result['success'] is True
            manager.kuka_fleet.rack_move.assert_called_once()

    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_execute_kuka_api_with_retry_failure(self, mock_kuka_fleet_adapter, mock_rcs_core):
        """測試 KUKA API 重試執行 - 失敗"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.kuka_fleet = Mock()
            manager.kuka_fleet.rack_move.return_value = {"success": False, "error": "API Error"}
            manager.api_retry_config = {
                'max_attempts': 2,  # 減少重試次數以加快測試
                'base_delay': 0.01,  # 縮短延遲
                'backoff_factor': 2.0,
                'max_delay': 1.0,
                'timeout': 60.0
            }
            manager.api_stats = {
                'total_calls': 0,
                'successful_calls': 0,
                'failed_calls': 0,
                'retry_calls': 0,
                'average_response_time': 0.0
            }
            
            with patch('time.sleep'):  # 跳過實際延遲
                result = manager._execute_kuka_api_with_retry(
                    'rack_move', 
                    [[10, 20, 30], 101, 'mission_001'],
                    task_type='normal',
                    rack_id=123
                )
            
            assert result['success'] is False
            assert "API Error" in result['error']
            assert manager.kuka_fleet.rack_move.call_count == 2  # 應該重試一次

    def test_update_api_stats_success(self, mock_rcs_core):
        """測試更新 API 統計 - 成功"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.api_stats = {
                'total_calls': 1,
                'successful_calls': 0,
                'failed_calls': 0,
                'retry_calls': 0,
                'average_response_time': 0.0
            }
            
            manager._update_api_stats(True, 2.5)
            
            assert manager.api_stats['successful_calls'] == 1
            assert manager.api_stats['failed_calls'] == 0
            assert manager.api_stats['average_response_time'] == 2.5

    def test_update_api_stats_failure(self, mock_rcs_core):
        """測試更新 API 統計 - 失敗"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.api_stats = {
                'total_calls': 1,
                'successful_calls': 0,
                'failed_calls': 0,
                'retry_calls': 0,
                'average_response_time': 0.0
            }
            
            manager._update_api_stats(False, 1.5)
            
            assert manager.api_stats['successful_calls'] == 0
            assert manager.api_stats['failed_calls'] == 1
            assert manager.api_stats['average_response_time'] == 1.5

    def test_get_api_statistics(self, mock_rcs_core):
        """測試獲取 API 統計資訊"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.api_stats = {
                'total_calls': 10,
                'successful_calls': 8,
                'failed_calls': 2,
                'retry_calls': 3,
                'average_response_time': 2.5
            }
            
            stats = manager.get_api_statistics()
            
            assert stats['total_calls'] == 10
            assert stats['successful_calls'] == 8
            assert stats['failed_calls'] == 2
            assert stats['success_rate'] == 80.0  # 8/10 * 100
            assert stats['retry_rate'] == 30.0    # 3/10 * 100

    def test_get_api_statistics_no_calls(self, mock_rcs_core):
        """測試獲取 API 統計資訊 - 無調用"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.api_stats = {
                'total_calls': 0,
                'successful_calls': 0,
                'failed_calls': 0,
                'retry_calls': 0,
                'average_response_time': 0.0
            }
            
            stats = manager.get_api_statistics()
            
            assert stats['success_rate'] == 0.0
            assert stats['retry_rate'] == 0.0

    @patch('rcs.kuka_manager.KukaFleetAdapter')
    def test_stop_monitoring(self, mock_kuka_fleet_adapter, mock_rcs_core):
        """測試停止監控"""
        with patch.object(KukaManager, '__init__', return_value=None):
            manager = KukaManager.__new__(KukaManager)
            manager.logger = mock_rcs_core.get_logger.return_value
            manager.kuka_fleet = Mock()
            
            manager.stop_monitoring()
            
            manager.kuka_fleet.stop_monitoring.assert_called_once()