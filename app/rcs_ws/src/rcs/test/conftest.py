"""
pytest 配置檔案
提供測試用的 fixtures 和共用設定
"""
import pytest
import os
import sys
from unittest.mock import MagicMock, Mock
from datetime import datetime, timezone

# 添加必要的路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

@pytest.fixture
def mock_logger():
    """模擬日誌記錄器"""
    logger = MagicMock()
    logger.debug = Mock()
    logger.info = Mock()
    logger.warning = Mock()
    logger.error = Mock()
    return logger

@pytest.fixture
def mock_db_session():
    """模擬資料庫會話"""
    session = MagicMock()
    session.exec = Mock()
    session.add = Mock()
    session.commit = Mock()
    session.rollback = Mock()
    return session

@pytest.fixture
def mock_db_pool(mock_db_session):
    """模擬資料庫連線池"""
    pool = MagicMock()
    pool.get_session.return_value.__enter__ = Mock(return_value=mock_db_session)
    pool.get_session.return_value.__exit__ = Mock(return_value=None)
    return pool

@pytest.fixture
def mock_rcs_core(mock_logger, mock_db_pool):
    """模擬 RCS Core 節點"""
    rcs_core = MagicMock()
    rcs_core.get_logger.return_value = mock_logger
    rcs_core.db_pool = mock_db_pool
    return rcs_core

@pytest.fixture
def mock_kuka_fleet():
    """模擬 KUKA Fleet Adapter"""
    kuka_fleet = MagicMock()
    kuka_fleet.select_agv = Mock(return_value=[])
    kuka_fleet.move = Mock(return_value={"success": True})
    kuka_fleet.rack_move = Mock(return_value={"success": True})
    kuka_fleet.workflow = Mock(return_value={"success": True})
    kuka_fleet.start_monitoring = Mock()
    kuka_fleet.stop_monitoring = Mock()
    return kuka_fleet

@pytest.fixture
def sample_robot_data():
    """標準機器人資料範例"""
    return {
        "robotId": "101",
        "x": 10.5,
        "y": 20.3,
        "robotOrientation": 90.0,
        "batteryLevel": 85,
        "status": 3,  # idle
        "nodeNumber": "12"
    }

@pytest.fixture
def sample_container_data():
    """標準容器資料範例"""
    return {
        "containerCode": "RACK001",
        "isCarry": True,
        "inMapStatus": True
    }

@pytest.fixture
def sample_task_data():
    """標準任務資料範例"""
    return {
        "id": 1,
        "work_id": 220001,  # KUKA_RACK_MOVE
        "priority": 50,
        "status_id": 1,  # pending
        "mission_code": None,
        "agv_id": None,
        "parameters": {
            "model": "KUKA400i",
            "nodes": [10, 20, 30],
            "rack_id": 123
        },
        "created_at": datetime.now(timezone.utc),
        "updated_at": datetime.now(timezone.utc)
    }

@pytest.fixture
def sample_agv_data():
    """標準 AGV 資料範例"""
    return {
        "id": 101,
        "name": "KUKA101",
        "model": "KUKA400i",
        "enable": 1,
        "status_id": 3,  # idle
        "x": 100.0,
        "y": 200.0,
        "heading": 45.0,
        "battery": 80,
        "last_node_id": 12
    }

@pytest.fixture
def sample_rack_data():
    """標準 Rack 資料範例"""
    return {
        "id": 1,
        "name": "RACK001",
        "agv_id": None,
        "location_id": 10,
        "product_id": None,
        "is_carry": 0,
        "is_in_map": 1,
        "is_docked": 0,
        "status_id": 1,
        "direction": 0
    }