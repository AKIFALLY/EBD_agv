"""
測試 AgvPortCheckEmptyState 類別
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import pytest
import sys

# 模擬缺失的模組
sys.modules['plc_proxy'] = MagicMock()
sys.modules['plc_proxy.plc_client'] = MagicMock()
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.logging'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['agv_base'] = MagicMock()
sys.modules['agv_base.hokuyo_dms_8bit'] = MagicMock()
sys.modules['agv_base.robot'] = MagicMock()
sys.modules['agv_base.states'] = MagicMock()
sys.modules['agv_base.states.state'] = MagicMock()
sys.modules['agv_base.base_context'] = MagicMock()
sys.modules['db_proxy'] = MagicMock()
sys.modules['db_proxy.carrier_query_client'] = MagicMock()
sys.modules['db_proxy.eqp_signal_query_client'] = MagicMock()
sys.modules['db_proxy.agvc_database_client'] = MagicMock()
sys.modules['db_proxy.models'] = MagicMock()

try:
    from loader_agv.robot_states.take_transfer.agv_port_check_empty_state import AgvPortCheckEmptyState
    from loader_agv.robot_context import RobotContext
    from db_proxy.carrier_query_client import CarrierQueryClient
    from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
except ImportError as e:
    print(f"Import error: {e}")
    # 創建 mock 類別

    class AgvPortCheckEmptyState:
        SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_NONE = 1, 2, 3, 4, 0
        def __init__(self, node): pass

    class RobotContext:
        IDLE = 0
        CHECK_IDLE = 1
        WRITE_CHG_PARAMTER = 21
        CHECK_CHG_PARA = 3
        WRITE_PGNO = 4
        CHECK_PGNO = 5
        ACTING = 6
        FINISH = 99
        UPDATE_DATABASE = 100
        def __init__(self, state): pass

    CarrierQueryClient = MagicMock
    EqpSignalQueryClient = MagicMock


class TestAgvPortCheckEmptyState(unittest.TestCase):
    """AgvPortCheckEmptyState 測試類別"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 1

        # 創建測試實例
        self.state = AgvPortCheckEmptyState(self.mock_node)

        # 創建 mock context
        mock_initial_state = Mock()
        self.context = RobotContext(mock_initial_state)
        self.context.node = self.mock_node

    def test_init_dynamic_calculations(self):
        """測試動態計算參數"""
        # 測試 room_id = 1 的情況
        self.assertEqual(self.state.port_address, 1100)  # 1 * 1000 + 100
        self.assertEqual(self.state.eqp_id, 110)  # 1 * 100 + 10

        # 測試不同 room_id
        self.mock_node.room_id = 2
        state2 = AgvPortCheckEmptyState(self.mock_node)
        self.assertEqual(state2.port_address, 2100)  # 2 * 1000 + 100
        self.assertEqual(state2.eqp_id, 210)  # 2 * 100 + 10

    def test_port_selection_table(self):
        """測試 port 選擇表邏輯"""
        # 測試所有 port 都空的情況 - 選擇 PORT01
        port_states = (0, 0, 0, 0)
        expected = self.state.SELECT_PORT01
        self.assertEqual(self.state.select_agv_port_table[port_states], expected)

        # 測試 PORT01 有貨，選擇 PORT02
        port_states = (1, 0, 0, 0)
        expected = self.state.SELECT_PORT02
        self.assertEqual(self.state.select_agv_port_table[port_states], expected)

        # 測試 PORT01, PORT02 有貨，選擇 PORT03
        port_states = (1, 1, 0, 0)
        expected = self.state.SELECT_PORT03
        self.assertEqual(self.state.select_agv_port_table[port_states], expected)

        # 測試 PORT01, PORT02, PORT03 有貨，選擇 PORT04
        port_states = (1, 1, 1, 0)
        expected = self.state.SELECT_PORT04
        self.assertEqual(self.state.select_agv_port_table[port_states], expected)

        # 測試所有 port 都有貨的情況 - 不在表中，應該返回 SELECT_NONE
        port_states = (1, 1, 1, 1)
        self.assertNotIn(port_states, self.state.select_agv_port_table)

    def test_reset_state(self):
        """測試狀態重置功能"""
        # 設置一些狀態
        self.state.check_ok = True
        self.state.sent = True
        self.state.search_eqp_signal_ok = True
        self.state.carrier_query_sended = True
        self.state.carrier_query_success = True
        self.state.select_agv_port = self.state.SELECT_PORT02
        self.state.carrier_id = 123

        # 執行重置
        self.state._reset_state()

        # 驗證重置結果
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)
        self.assertFalse(self.state.search_eqp_signal_ok)
        self.assertFalse(self.state.carrier_query_sended)
        self.assertFalse(self.state.carrier_query_success)
        self.assertEqual(self.state.port_carriers, [True] * 4)
        self.assertEqual(self.state.select_agv_port, self.state.SELECT_NONE)
        self.assertIsNone(self.state.carrier_id)

    def test_enter_and_leave(self):
        """測試進入和離開狀態"""
        # 設置一些狀態
        self.state.check_ok = True
        self.state.sent = True

        # 測試進入狀態
        self.state.enter()
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)

        # 設置一些狀態
        self.state.check_ok = True
        self.state.sent = True

        # 測試離開狀態
        self.state.leave()
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)

    @patch.object(EqpSignalQueryClient, 'eqp_signal_port')
    def test_eqp_signal_query_callback(self, mock_eqp_signal_port):
        """測試 EQP 信號查詢回調"""
        # 設置 mock 返回值 - port 1,3 有貨，port 2,4 沒貨
        mock_eqp_signal_port.side_effect = [True, False, True, False]

        # 創建 mock response
        mock_response = Mock()

        # 執行回調
        self.state.eqp_signal_query_callback(mock_response)

        # 驗證結果
        self.assertTrue(self.state.search_eqp_signal_ok)
        self.assertEqual(self.state.port_carriers, [True, False, True, False])

        # 驗證 port 選擇邏輯 - (1,0,1,0) 應該選擇 PORT02
        self.assertEqual(self.state.select_agv_port, self.state.SELECT_PORT02)

        # 驗證 EqpSignalQueryClient.eqp_signal_port 被正確調用
        expected_calls = [
            ((mock_response, 1101),),  # port_address + 1
            ((mock_response, 1102),),  # port_address + 2
            ((mock_response, 1103),),  # port_address + 3
            ((mock_response, 1104),),  # port_address + 4
        ]
        self.assertEqual(mock_eqp_signal_port.call_args_list, expected_calls)

    @patch.object(CarrierQueryClient, 'carrier_port_id_carrier_id')
    def test_carrier_callback(self, mock_carrier_port_id_carrier_id):
        """測試 Carrier 查詢回調"""
        # 設置 mock 返回值
        mock_carrier_port_id_carrier_id.return_value = 456

        # 創建 mock response
        mock_response = Mock()
        mock_response.success = True

        # 設置 select_agv_port_number
        self.state.select_agv_port_number = 2

        # 執行回調
        self.state.carrier_callback(mock_response)

        # 驗證結果
        self.assertTrue(self.state.carrier_query_success)
        self.assertEqual(self.state.carrier_id, 456)

        # 驗證 CarrierQueryClient.carrier_port_id_carrier_id 被正確調用
        mock_carrier_port_id_carrier_id.assert_called_once_with(
            mock_response, 1102)  # port_address + 2

    def test_update_context_states(self):
        """測試更新 context 狀態"""
        # 設置 port_carriers
        self.state.port_carriers = [True, False, True, False]
        self.state.search_eqp_signal_ok = True

        # 執行更新
        self.state._update_context_states(self.context)

        # 驗證 context 狀態被正確更新
        self.assertTrue(self.context.agv_port1)
        self.assertFalse(self.context.agv_port2)
        self.assertTrue(self.context.agv_port3)
        self.assertFalse(self.context.agv_port4)

    def test_update_context_states_not_ready(self):
        """測試當 EQP 信號查詢未完成時不更新 context"""
        # 設置 search_eqp_signal_ok = False
        self.state.search_eqp_signal_ok = False
        self.state.port_carriers = [True, False, True, False]

        # 保存原始狀態
        original_agv_port1 = self.context.agv_port1

        # 執行更新
        self.state._update_context_states(self.context)

        # 驗證 context 狀態沒有被更新
        self.assertEqual(self.context.agv_port1, original_agv_port1)

    def test_handle_port_selection_success(self):
        """測試成功的 port 選擇處理"""
        # 設置條件
        self.state.search_eqp_signal_ok = True
        self.state.check_ok = False
        self.state.select_agv_port = self.state.SELECT_PORT02

        # 執行 port 選擇
        self.state._handle_port_selection(self.context)

        # 驗證結果
        self.assertTrue(self.state.check_ok)
        self.assertEqual(self.context.get_loader_agv_port_front, 2)

    def test_handle_port_selection_all_ports_full(self):
        """測試所有 port 都滿的情況"""
        # 設置條件
        self.state.search_eqp_signal_ok = True
        self.state.check_ok = False
        self.state.select_agv_port = self.state.SELECT_NONE

        # 執行 port 選擇
        self.state._handle_port_selection(self.context)

        # 驗證結果
        self.assertFalse(self.state.check_ok)
        self.assertIsNone(self.context.get_loader_agv_port_front)

    def test_handle_port_selection_already_checked(self):
        """測試已經檢查過的情況"""
        # 設置條件
        self.state.check_ok = True
        original_get_loader_agv_port_front = self.context.get_loader_agv_port_front

        # 執行 port 選擇
        self.state._handle_port_selection(self.context)

        # 驗證結果 - 不應該改變
        self.assertEqual(self.context.get_loader_agv_port_front, original_get_loader_agv_port_front)

    @patch('loader_agv.robot_states.take_transfer.agv_port_check_empty_state.TakeTransferState')
    def test_handle_complete_flow_success(self, mock_take_transfer_state):
        """測試完整的成功流程"""
        # 設置 mock
        mock_eqp_response = Mock()
        mock_carrier_response = Mock()
        mock_carrier_response.success = True

        # 模擬 EQP 查詢回調 - port 1 有貨，選擇 port 2
        with patch.object(EqpSignalQueryClient, 'eqp_signal_port') as mock_eqp_signal_port:
            mock_eqp_signal_port.side_effect = [True, False, False, False]
            self.state.eqp_signal_query_callback(mock_eqp_response)

        # 模擬 Carrier 查詢回調 - port 2 沒有 carrier
        with patch.object(CarrierQueryClient, 'carrier_port_id_carrier_id') as mock_carrier_port_id:
            mock_carrier_port_id.return_value = None
            self.state.carrier_callback(mock_carrier_response)

        # 設置必要的狀態
        self.state.check_ok = True
        self.state.carrier_query_sended = True
        self.context.get_loader_agv_port_front = 2

        # 執行 handle
        self.state.handle(self.context)

        # 驗證狀態轉換
        self.context.set_state.assert_called_once()
        args = self.context.set_state.call_args[0]
        self.assertIsInstance(args[0], type(mock_take_transfer_state.return_value))

    def test_handle_carrier_exists_reset(self):
        """測試當 carrier 存在時重置狀態"""
        # 設置條件
        self.state.check_ok = True
        self.state.carrier_query_success = True
        self.state.carrier_id = 123  # 有 carrier

        # 執行 handle
        self.state.handle(self.context)

        # 驗證狀態被重置
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)

    @patch.object(EqpSignalQueryClient, 'search_eqp_signal_eqp_id')
    def test_handle_eqp_query_trigger(self, mock_search_eqp_signal):
        """測試 EQP 查詢觸發"""
        # 設置條件
        self.state.search_eqp_signal_ok = False
        self.state.sent = False

        # 執行 handle
        self.state.handle(self.context)

        # 驗證 EQP 查詢被觸發
        mock_search_eqp_signal.assert_called_once_with(
            self.state.eqp_id, self.state.eqp_signal_query_callback)
        self.assertTrue(self.state.sent)

    @patch.object(CarrierQueryClient, 'search_carrier_port_id')
    def test_handle_carrier_query_trigger(self, mock_search_carrier):
        """測試 Carrier 查詢觸發"""
        # 設置條件
        self.state.check_ok = True
        self.state.carrier_query_sended = False
        self.context.get_loader_agv_port_front = 3

        # 執行 handle
        self.state.handle(self.context)

        # 驗證 Carrier 查詢被觸發
        mock_search_carrier.assert_called_once_with(
            port_id=1103, callback=self.state.carrier_callback)  # port_address + 3
        self.assertTrue(self.state.carrier_query_sended)

    def test_port_selection_all_combinations(self):
        """測試所有 port 選擇組合"""
        test_cases = [
            # (port1, port2, port3, port4) -> expected_selection
            ((0, 0, 0, 0), self.state.SELECT_PORT01),
            ((0, 0, 0, 1), self.state.SELECT_PORT01),
            ((0, 0, 1, 0), self.state.SELECT_PORT01),
            ((0, 0, 1, 1), self.state.SELECT_PORT01),
            ((0, 1, 0, 0), self.state.SELECT_PORT01),
            ((0, 1, 0, 1), self.state.SELECT_PORT01),
            ((0, 1, 1, 0), self.state.SELECT_PORT01),
            ((0, 1, 1, 1), self.state.SELECT_PORT01),
            ((1, 0, 0, 0), self.state.SELECT_PORT02),
            ((1, 0, 0, 1), self.state.SELECT_PORT02),
            ((1, 0, 1, 0), self.state.SELECT_PORT02),
            ((1, 0, 1, 1), self.state.SELECT_PORT02),
            ((1, 1, 0, 0), self.state.SELECT_PORT03),
            ((1, 1, 0, 1), self.state.SELECT_PORT03),
            ((1, 1, 1, 0), self.state.SELECT_PORT04),
        ]

        for port_states, expected in test_cases:
            with self.subTest(port_states=port_states):
                self.assertEqual(
                    self.state.select_agv_port_table[port_states],
                    expected,
                    f"Port states {port_states} should select {expected}"
                )

    def test_dynamic_calculations_edge_cases(self):
        """測試動態計算的邊界情況"""
        # 測試 room_id = 0
        self.mock_node.room_id = 0
        state = AgvPortCheckEmptyState(self.mock_node)
        self.assertEqual(state.port_address, 100)  # 0 * 1000 + 100
        self.assertEqual(state.eqp_id, 10)  # 0 * 100 + 10

        # 測試大的 room_id
        self.mock_node.room_id = 999
        state = AgvPortCheckEmptyState(self.mock_node)
        self.assertEqual(state.port_address, 999100)  # 999 * 1000 + 100
        self.assertEqual(state.eqp_id, 99910)  # 999 * 100 + 10


if __name__ == '__main__':
    unittest.main()
