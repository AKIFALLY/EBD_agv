"""
OPUI 測試配置檔案
提供測試所需的 fixtures 和配置
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from sqlmodel import Session, create_engine, SQLModel
from sqlalchemy.pool import StaticPool
from fastapi.testclient import TestClient
import socketio

from opui.op_ui_server import OpUiServer
from opui.op_ui_socket import OpUiSocket
from opui.db import connection_pool
from db_proxy.models import Client, Product, Machine, Room, Rack, Task, Work, TaskStatus


@pytest.fixture(scope="session")
def event_loop():
    """創建事件循環用於異步測試"""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def mock_db_session():
    """模擬資料庫會話"""
    session = Mock()
    # 設定 exec 方法的返回值
    mock_result = Mock()
    mock_result.first = Mock()
    session.exec.return_value = mock_result
    yield session


@pytest.fixture
def mock_connection_pool(mock_db_session):
    """模擬連線池"""
    with patch('opui.db.connection_pool') as mock_pool:
        mock_pool.get_session.return_value.__enter__.return_value = mock_db_session
        mock_pool.get_session.return_value.__exit__.return_value = None
        yield mock_pool


@pytest.fixture
def sample_client_data():
    """範例客戶端資料"""
    return {
        "clientId": 1,
        "userAgent": "Mozilla/5.0 Test Browser",
        "op": {
            "left": {"productSelected": 0, "product": ["產品A", "產品B"]},
            "right": {"productSelected": 1, "product": ["產品C", "產品D"]}
        },
        "machineId": 1
    }


@pytest.fixture
def sample_product_data():
    """範例產品資料"""
    return {
        "name": "測試產品",
        "size": "大",
        "process_settings_id": 1
    }


@pytest.fixture
def sample_machine_data():
    """範例機台資料"""
    return {
        "id": 1,
        "name": "機台1",
        "parking_space_1": 101,
        "parking_space_2": 102,
        "parking_space_1_status": 0,
        "parking_space_2_status": 0
    }


@pytest.fixture
def sample_task_data():
    """範例任務資料"""
    return {
        "name": "測試任務",
        "description": "測試任務描述",
        "work_id": 1,
        "status_id": 1,
        "node_id": 101,
        "priority": 1,
        "parameters": {
            "node_id": 101,
            "machine_id": 1,
            "client_id": 1,
            "task_type": "call_empty"
        }
    }


@pytest.fixture
def mock_sio():
    """模擬 Socket.IO 伺服器"""
    sio = Mock(spec=socketio.AsyncServer)
    sio.emit = AsyncMock()
    sio.emit.call_count = 0  # 添加 call_count 屬性
    sio.on = Mock(return_value=lambda func: func)  # 修正 on 方法返回值
    sio.on.call_count = 0  # 添加 call_count 屬性
    return sio


@pytest.fixture
def op_ui_socket(mock_sio):
    """創建 OpUiSocket 實例用於測試"""
    return OpUiSocket(mock_sio)


@pytest.fixture
def op_ui_server():
    """創建 OpUiServer 實例用於測試"""
    return OpUiServer()


@pytest.fixture
def test_client(op_ui_server):
    """創建 FastAPI 測試客戶端"""
    return TestClient(op_ui_server.app)


@pytest.fixture
def mock_db_operations():
    """模擬資料庫操作函數"""
    # 創建獨立的 Mock 物件
    mock_get_client = Mock(return_value={"clientId": 1, "machineId": 1, "updated_at": "2023-01-01"})
    mock_get_or_create_or_update_client = Mock(return_value={"clientId": 1, "machineId": 1})
    mock_product_all = Mock(return_value=[{"id": 1, "name": "產品A"}])
    mock_machine_all = Mock(return_value=[{"id": 1, "name": "機台1"}])
    mock_room_all = Mock(return_value=[{"id": 1, "name": "房間1"}])
    mock_create_task = Mock(return_value={"id": 1, "name": "測試任務"})
    mock_get_call_empty_work_id = Mock(return_value=1)
    mock_get_dispatch_full_work_id = Mock(return_value=2)
    from opui.constants.task_status import TaskStatus
    mock_get_default_task_status_id = Mock(return_value=TaskStatus.PENDING)
    mock_delete_task_by_parking = Mock(return_value=True)

    mocks = {
        'get_client': mock_get_client,
        'get_or_create_or_update_client': mock_get_or_create_or_update_client,
        'product_all': mock_product_all,
        'machine_all': mock_machine_all,
        'room_all': mock_room_all,
        'create_task': mock_create_task,
        'get_call_empty_work_id': mock_get_call_empty_work_id,
        'get_dispatch_full_work_id': mock_get_dispatch_full_work_id,
        'get_default_task_status_id': mock_get_default_task_status_id,
        'delete_task_by_parking': mock_delete_task_by_parking,
    }

    with patch.multiple('opui.db', **mocks):
        yield mocks


@pytest.fixture
def mock_crud_operations():
    """模擬 CRUD 操作"""
    with patch.multiple(
        'opui.db',
        client_crud=Mock(),
        product_crud=Mock(),
        machine_crud=Mock(),
        room_crud=Mock(),
        rack_crud=Mock(),
        task_crud=Mock(),
        work_crud=Mock(),
        task_status_crud=Mock(),
    ) as mocks:
        yield mocks


# 測試資料庫模型的工廠函數
def create_test_client(**kwargs):
    """創建測試用的 Client 模型"""
    defaults = {
        "id": 1,
        "user_agent": "Test Browser",
        "op": {},
        "machine_id": 1
    }
    defaults.update(kwargs)
    return Client(**defaults)


def create_test_product(**kwargs):
    """創建測試用的 Product 模型"""
    defaults = {
        "id": 1,
        "name": "測試產品",
        "size": "大",
        "process_settings_id": 1
    }
    defaults.update(kwargs)
    return Product(**defaults)


def create_test_machine(**kwargs):
    """創建測試用的 Machine 模型"""
    defaults = {
        "id": 1,
        "name": "測試機台",
        "parking_space_1": 101,
        "parking_space_2": 102,
        "parking_space_1_status": 0,
        "parking_space_2_status": 0
    }
    defaults.update(kwargs)
    return Machine(**defaults)


def create_test_task(**kwargs):
    """創建測試用的 Task 模型"""
    defaults = {
        "id": 1,
        "name": "測試任務",
        "description": "測試任務描述",
        "work_id": 1,
        "status_id": 1,
        "node_id": 101,
        "priority": 1,
        "parameters": {}
    }
    defaults.update(kwargs)
    return Task(**defaults)


# 將工廠函數加入到 pytest 命名空間
pytest.create_test_client = create_test_client
pytest.create_test_product = create_test_product
pytest.create_test_machine = create_test_machine
pytest.create_test_task = create_test_task
