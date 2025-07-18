"""
Loader AGV 測試配置檔案
提供測試所需的 fixtures 和配置
"""

import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch, MagicMock

# 模擬缺失的模組
import sys
from unittest.mock import MagicMock

# 模擬 plc_proxy 模組
sys.modules['plc_proxy'] = MagicMock()
sys.modules['plc_proxy.plc_client'] = MagicMock()

# 模擬 rclpy 模組
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.logging'] = MagicMock()

# 模擬 std_msgs 模組
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()

# 現在可以安全地導入
try:
    from rclpy.node import Node
    from rclpy.logging import get_logger
except ImportError:
    Node = MagicMock
    get_logger = MagicMock

try:
    from std_msgs.msg import Bool, Int32
except ImportError:
    Bool = MagicMock
    Int32 = MagicMock

try:
    from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
    from agv_base.robot import Robot
except ImportError:
    HokuyoDMS8Bit = MagicMock
    Robot = MagicMock

try:
    from db_proxy.carrier_query_client import CarrierQueryClient
    from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
    from db_proxy.agvc_database_client import AGVCDatabaseClient
    from db_proxy.models import CarrierMsg
except ImportError:
    CarrierQueryClient = MagicMock
    EqpSignalQueryClient = MagicMock
    AGVCDatabaseClient = MagicMock
    CarrierMsg = MagicMock

try:
    from loader_agv.robot_context import RobotContext
except ImportError:
    RobotContext = MagicMock


@pytest.fixture
def mock_node():
    """模擬 ROS2 節點"""
    node = Mock(spec=Node)
    node.get_logger.return_value = get_logger('test_node')
    node.room_id = 1  # 預設房間 ID
    return node


@pytest.fixture
def mock_hokuyo_dms_8bit():
    """模擬 Hokuyo DMS 8-bit 設備"""
    hokuyo = Mock(spec=HokuyoDMS8Bit)

    # 設置預設屬性
    hokuyo.valid_success = False
    hokuyo.valid_failed = False
    hokuyo.port_number_success = False
    hokuyo.port_number_failed = False
    hokuyo.tr_req_success = False
    hokuyo.tr_req_failed = False
    hokuyo.load_req = False
    hokuyo.unload_req = False
    hokuyo.ready = False

    # 設置方法
    hokuyo.write_valid = Mock()
    hokuyo.write_port_number = Mock()
    hokuyo.write_tr_req = Mock()

    return hokuyo


@pytest.fixture
def mock_robot():
    """模擬機器人"""
    robot = Mock(spec=Robot)

    # 設置機器人常數
    robot.IDLE = 0
    robot.ACTION_TO = 100
    robot.NONE_POSITION = 0
    robot.AGV_POSITION = 10
    robot.PHOTO_BOX_IN = 200

    # 設置讀取響應
    robot.read_pgno_response = Mock()
    robot.read_pgno_response.value = robot.IDLE
    robot.read_pgno = Mock()

    # 設置寫入方法
    robot.write_pgno = Mock()
    robot.write_chg_para = Mock()

    return robot


@pytest.fixture
def mock_carrier_query_client():
    """模擬 Carrier 查詢客戶端"""
    client = Mock(spec=CarrierQueryClient)

    # 設置靜態方法
    CarrierQueryClient.carrier_port_id_carrier_id = Mock(return_value=None)

    return client


@pytest.fixture
def mock_eqp_signal_query_client():
    """模擬 EQP 信號查詢客戶端"""
    client = Mock(spec=EqpSignalQueryClient)

    # 設置靜態方法
    EqpSignalQueryClient.eqp_signal_port = Mock(return_value=False)

    return client


@pytest.fixture
def mock_agvc_database_client():
    """模擬 AGVC 資料庫客戶端"""
    client = Mock(spec=AGVCDatabaseClient)
    client.async_update_carrier = Mock()
    return client


@pytest.fixture
def mock_robot_context(mock_node):
    """模擬機器人上下文"""
    # 創建一個簡單的 mock state 來初始化 context
    mock_state = Mock()
    context = RobotContext(mock_state)
    context.node = mock_node

    # 設置預設值
    context.take_transfer_continue = False
    context.boxin_port1 = False
    context.boxin_port2 = False
    context.boxin_port3 = False
    context.boxin_port4 = False
    context.agv_port1 = False
    context.agv_port2 = False
    context.agv_port3 = False
    context.agv_port4 = False
    context.boxin_number = 0
    context.agv_port_number = 0
    context.get_loader_agv_port_front = 1
    context.get_boxin_port = 1
    context.carrier_id = 0
    context.boxin_buffer = 0

    return context


@pytest.fixture
def sample_carrier_response():
    """範例 Carrier 響應資料"""
    response = Mock()
    response.success = True
    response.carriers = []
    return response


@pytest.fixture
def sample_eqp_signal_response():
    """範例 EQP 信號響應資料"""
    response = Mock()
    response.success = True
    response.eqp_signals = []
    return response


@pytest.fixture
def sample_carrier_msg():
    """範例 Carrier 訊息"""
    carrier = CarrierMsg()
    carrier.id = 123
    carrier.room_id = 1
    carrier.rack_id = 0
    carrier.port_id = 1101
    carrier.rack_index = 0
    carrier.status_id = Robot.CARRIER_STATUS_IN_USE  # 使用中
    return carrier


# 測試資料工廠函數
def create_test_port_states(port1=False, port2=False, port3=False, port4=False):
    """創建測試用的 port 狀態"""
    return [port1, port2, port3, port4]


def create_test_eqp_response(port_states):
    """創建測試用的 EQP 響應"""
    response = Mock()
    response.success = True
    response.eqp_signals = []

    # 模擬 eqp_signal_port 方法的行為
    def mock_eqp_signal_port(response, port_id):
        # port_id 從 1101-1104，對應 index 0-3
        index = (port_id - 1101) % 4
        if 0 <= index < len(port_states):
            return port_states[index]
        return False

    EqpSignalQueryClient.eqp_signal_port.side_effect = mock_eqp_signal_port
    return response


def create_test_carrier_response(carrier_id=None, success=True):
    """創建測試用的 Carrier 響應"""
    response = Mock()
    response.success = success

    def mock_carrier_port_id_carrier_id(response, port_id):
        return carrier_id

    CarrierQueryClient.carrier_port_id_carrier_id.side_effect = mock_carrier_port_id_carrier_id
    return response


# 將工廠函數加入到 pytest 命名空間
pytest.create_test_port_states = create_test_port_states
pytest.create_test_eqp_response = create_test_eqp_response
pytest.create_test_carrier_response = create_test_carrier_response
