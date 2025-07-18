"""
測試 TransferCheckHaveState 類別
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import pytest

from loader_agv.robot_states.take_transfer.transfer_check_have_state import TransferCheckHaveState
from loader_agv.robot_context import RobotContext
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient


class TestTransferCheckHaveState(unittest.TestCase):
    """TransferCheckHaveState 測試類別"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 1

        # 創建 mock hokuyo
        self.mock_hokuyo = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo

        # 創建測試實例
        self.state = TransferCheckHaveState(self.mock_node)

        # 創建 mock context
        mock_initial_state = Mock()
        self.context = RobotContext(mock_initial_state)
        self.context.node = self.mock_node

    def test_init_dynamic_calculations(self):
        """測試動態計算參數"""
        # 測試 room_id = 1 的情況
        self.assertEqual(self.state.port_address, 1010)  # 1 * 1000 + 10
        self.assertEqual(self.state.eqp_id, 101)  # 1 * 100 + 1

        # 測試不同 room_id
        self.mock_node.room_id = 3
        state2 = TransferCheckHaveState(self.mock_node)
        self.assertEqual(state2.port_address, 3010)  # 3 * 1000 + 10
        self.assertEqual(state2.eqp_id, 301)  # 3 * 100 + 1

    def test_init_state_variables(self):
        """測試初始狀態變數"""
        self.assertEqual(self.state.step, self.state.IDLE)
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)
        self.assertEqual(self.state.queries_completed, {
            'carrier': False,
            'eqp_signal': False
        })
        self.assertIsNone(self.state.earliest_carrier)
        self.assertEqual(self.state.select_boxin_port, 0)
        self.assertFalse(self.state.port_have_carrier)
        self.assertFalse(self.state.search_eqp_signal_ok)
        self.assertEqual(self.state.port_carriers, [False, False, False, False])

    def test_enter_and_leave(self):
        """測試進入和離開狀態"""
        # 設置一些狀態
        self.state.check_ok = True
        self.state.sent = True
        self.state.step = self.state.WRITE_VAILD

        # 測試進入狀態
        self.state.enter()
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)
        self.assertEqual(self.state.step, self.state.IDLE)

        # 設置一些狀態
        self.state.check_ok = True
        self.state.sent = True
        self.state.step = self.state.WRITE_VAILD

        # 測試離開狀態
        self.state.leave()
        self.assertFalse(self.state.check_ok)
        self.assertFalse(self.state.sent)
        self.assertEqual(self.state.step, self.state.IDLE)

    def test_carrier_query_callback_with_carriers(self):
        """測試有 carrier 的查詢回調"""
        # 創建 mock carriers
        mock_carrier1 = Mock()
        mock_carrier1.id = 101
        mock_carrier1.port_id = 1011
        mock_carrier1.created_at = "2023-01-01 10:00:00"

        mock_carrier2 = Mock()
        mock_carrier2.id = 102
        mock_carrier2.port_id = 1012
        mock_carrier2.created_at = "2023-01-01 09:00:00"  # 更早的時間

        # 創建 mock response
        mock_response = Mock()
        mock_response.success = True
        mock_response.carriers = [mock_carrier1, mock_carrier2]

        # 執行回調
        self.state.carrier_query_callback(mock_response)

        # 驗證結果
        self.assertTrue(self.state.queries_completed['carrier'])
        self.assertTrue(self.state.port_have_carrier)
        self.assertEqual(self.state.earliest_carrier, mock_carrier2)  # 應該選擇更早的
        self.assertEqual(self.state.select_boxin_port, 2)  # port_id 1012 -> port 2

    def test_carrier_query_callback_no_carriers(self):
        """測試沒有 carrier 的查詢回調"""
        # 創建 mock response
        mock_response = Mock()
        mock_response.success = True
        mock_response.carriers = []

        # 執行回調
        self.state.carrier_query_callback(mock_response)

        # 驗證結果
        self.assertTrue(self.state.queries_completed['carrier'])
        self.assertFalse(self.state.port_have_carrier)
        self.assertIsNone(self.state.earliest_carrier)
        self.assertEqual(self.state.select_boxin_port, 0)

    def test_carrier_query_callback_failed(self):
        """測試查詢失敗的回調"""
        # 創建 mock response
        mock_response = Mock()
        mock_response.success = False

        # 執行回調
        self.state.carrier_query_callback(mock_response)

        # 驗證結果
        self.assertTrue(self.state.queries_completed['carrier'])
        self.assertFalse(self.state.port_have_carrier)

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
        self.assertTrue(self.state.queries_completed['eqp_signal'])
        self.assertTrue(self.state.search_eqp_signal_ok)
        self.assertEqual(self.state.port_carriers, [True, False, True, False])

    def test_update_context_states(self):
        """測試更新 context 狀態"""
        # 設置 port_carriers
        self.state.port_carriers = [True, False, True, False]
        self.state.search_eqp_signal_ok = True

        # 執行更新
        self.state._update_context_states(self.context)

        # 驗證 context 狀態被正確更新
        self.assertTrue(self.context.boxin_port1)
        self.assertFalse(self.context.boxin_port2)
        self.assertTrue(self.context.boxin_port3)
        self.assertFalse(self.context.boxin_port4)

    def test_check_take_transfer_continue_port1_port2(self):
        """測試 transfer continuation logic - port 1 選擇且 port 2 有貨"""
        # 設置條件
        self.state.search_eqp_signal_ok = True
        self.state.select_boxin_port = 1
        self.context.boxin_port2 = True
        self.context.boxin_port4 = False

        # 執行檢查
        self.state._check_take_transfer_continue(self.context)

        # 驗證結果
        self.assertTrue(self.context.take_transfer_continue)

    def test_check_take_transfer_continue_port3_port4(self):
        """測試 transfer continuation logic - port 3 選擇且 port 4 有貨"""
        # 設置條件
        self.state.search_eqp_signal_ok = True
        self.state.select_boxin_port = 3
        self.context.boxin_port2 = False
        self.context.boxin_port4 = True

        # 執行檢查
        self.state._check_take_transfer_continue(self.context)

        # 驗證結果
        self.assertTrue(self.context.take_transfer_continue)

    def test_check_take_transfer_continue_false_cases(self):
        """測試 transfer continuation logic - 不能繼續的情況"""
        test_cases = [
            # (select_port, port2, port4, expected)
            (1, False, False, False),  # port 1 選擇但 port 2 沒貨
            (1, False, True, False),   # port 1 選擇但 port 2 沒貨（port 4 有貨不影響）
            (3, True, False, False),   # port 3 選擇但 port 4 沒貨
            (3, False, False, False),  # port 3 選擇但 port 4 沒貨
            (2, True, True, False),    # port 2 選擇（不在規則中）
            (4, True, True, False),    # port 4 選擇（不在規則中）
            (0, True, True, False),    # 沒有選擇 port
        ]

        for select_port, port2, port4, expected in test_cases:
            with self.subTest(select_port=select_port, port2=port2, port4=port4):
                # 設置條件
                self.state.search_eqp_signal_ok = True
                self.state.select_boxin_port = select_port
                self.context.boxin_port2 = port2
                self.context.boxin_port4 = port4

                # 執行檢查
                self.state._check_take_transfer_continue(self.context)

                # 驗證結果
                self.assertEqual(self.context.take_transfer_continue, expected)

    def test_check_take_transfer_continue_not_ready(self):
        """測試當查詢未完成時不設置 continuation"""
        # 設置條件 - 查詢未完成
        self.state.search_eqp_signal_ok = False
        self.state.select_boxin_port = 1
        self.context.boxin_port2 = True

        # 執行檢查
        self.state._check_take_transfer_continue(self.context)

        # 驗證結果 - 應該設為 False
        self.assertFalse(self.context.take_transfer_continue)

    def test_handle_hokuyo_write_success(self):
        """測試 Hokuyo 寫入成功"""
        # 設置條件
        self.state.sent = False
        self.mock_hokuyo.valid_success = True
        self.mock_hokuyo.valid_failed = False

        # 執行寫入處理
        self.state._handle_hokuyo_write(
            "write_valid", "1", "valid_success", "valid_failed", self.state.WRITE_PORT_NUMBER)

        # 驗證結果
        self.mock_hokuyo.write_valid.assert_called_once_with("1")
        self.assertEqual(self.state.step, self.state.WRITE_PORT_NUMBER)
        self.assertFalse(self.state.sent)

    def test_handle_hokuyo_write_failed(self):
        """測試 Hokuyo 寫入失敗"""
        # 設置條件
        self.state.sent = False
        self.mock_hokuyo.valid_success = False
        self.mock_hokuyo.valid_failed = True

        # 執行寫入處理
        self.state._handle_hokuyo_write(
            "write_valid", "1", "valid_success", "valid_failed", self.state.WRITE_PORT_NUMBER)

        # 驗證結果
        self.mock_hokuyo.write_valid.assert_called_once_with("1")
        self.assertEqual(self.state.step, self.state.IDLE)  # 失敗時重置
        self.assertFalse(self.state.sent)

    def test_handle_hokuyo_write_not_sent(self):
        """測試 Hokuyo 尚未發送時的處理"""
        # 設置條件
        self.state.sent = True  # 已發送

        # 執行寫入處理
        self.state._handle_hokuyo_write(
            "write_valid", "1", "valid_success", "valid_failed", self.state.WRITE_PORT_NUMBER)

        # 驗證結果 - 不應該再次調用寫入
        self.mock_hokuyo.write_valid.assert_not_called()

    def test_8bit_steps_idle_to_write_vaild(self):
        """測試 8-bit 步驟：IDLE -> WRITE_VAILD"""
        # 設置條件
        self.state.check_ok = True
        self.state.step = self.state.IDLE

        # 執行 handle
        self.state.handle(self.context)

        # 驗證結果
        self.assertEqual(self.state.step, self.state.WRITE_VAILD)
        self.assertFalse(self.state.sent)

    def test_8bit_steps_write_port_number(self):
        """測試 8-bit 步驟：WRITE_PORT_NUMBER"""
        # 設置條件
        self.state.check_ok = True
        self.state.step = self.state.WRITE_PORT_NUMBER
        self.state.sent = False
        self.context.boxin_number = 3

        # 執行 handle
        self.state.handle(self.context)

        # 驗證結果
        self.mock_hokuyo.write_port_number.assert_called_once_with(3)
        self.assertTrue(self.state.sent)

    def test_8bit_steps_wait_unload_req_received(self):
        """測試 8-bit 步驟：等待 unload_req 並收到"""
        # 設置條件
        self.state.check_ok = True
        self.state.step = self.state.WAIT_UNLOAD_REQ
        self.mock_hokuyo.unload_req = True

        # 執行 handle
        self.state.handle(self.context)

        # 驗證結果
        self.assertEqual(self.state.step, self.state.WRITE_TR_REQ)

    def test_8bit_steps_wait_unload_req_not_received(self):
        """測試 8-bit 步驟：等待 unload_req 但未收到"""
        # 設置條件
        self.state.check_ok = True
        self.state.step = self.state.WAIT_UNLOAD_REQ
        self.mock_hokuyo.unload_req = False

        # 執行 handle
        self.state.handle(self.context)

        # 驗證結果 - 步驟不應該改變
        self.assertEqual(self.state.step, self.state.WAIT_UNLOAD_REQ)

    def test_8bit_steps_wait_ready_received(self):
        """測試 8-bit 步驟：等待 ready 並收到"""
        # 設置條件
        self.state.check_ok = True
        self.state.step = self.state.WAIT_READY
        self.mock_hokuyo.ready = True

        # 執行 handle
        self.state.handle(self.context)

        # 驗證結果
        self.assertEqual(self.state.step, self.state.IDLE)
        # 應該轉換到下一個狀態（這裡需要 mock 狀態轉換）

    @patch('loader_agv.robot_states.take_transfer.transfer_check_have_state.TakeTransferState')
    def test_complete_flow_success(self, mock_take_transfer_state):
        """測試完整的成功流程"""
        # 設置 carrier 查詢結果
        mock_carrier = Mock()
        mock_carrier.id = 123
        mock_carrier.port_id = 1011
        mock_carrier.created_at = "2023-01-01 10:00:00"

        mock_carrier_response = Mock()
        mock_carrier_response.success = True
        mock_carrier_response.carriers = [mock_carrier]

        self.state.carrier_query_callback(mock_carrier_response)

        # 設置 EQP 查詢結果
        with patch.object(EqpSignalQueryClient, 'eqp_signal_port') as mock_eqp_signal_port:
            mock_eqp_signal_port.side_effect = [True, False, False, False]
            mock_eqp_response = Mock()
            self.state.eqp_signal_query_callback(mock_eqp_response)

        # 設置 Hokuyo ready
        self.mock_hokuyo.ready = True
        self.state.step = self.state.WAIT_READY
        self.state.check_ok = True

        # 執行 handle
        self.state.handle(self.context)

        # 驗證狀態轉換
        self.context.set_state.assert_called_once()

    @patch.object(CarrierQueryClient, 'search_carrier_port_id')
    def test_handle_carrier_query_trigger(self, mock_search_carrier):
        """測試 Carrier 查詢觸發"""
        # 設置條件
        self.state.queries_completed['carrier'] = False

        # 執行 handle
        self.state.handle(self.context)

        # 驗證 Carrier 查詢被觸發
        mock_search_carrier.assert_called_once_with(
            port_id_min=1011,  # port_address + 1
            port_id_max=1014,  # port_address + 4
            callback=self.state.carrier_query_callback
        )

    @patch.object(EqpSignalQueryClient, 'search_eqp_signal_eqp_id')
    def test_handle_eqp_query_trigger(self, mock_search_eqp_signal):
        """測試 EQP 查詢觸發"""
        # 設置條件
        self.state.queries_completed['carrier'] = True
        self.state.queries_completed['eqp_signal'] = False

        # 執行 handle
        self.state.handle(self.context)

        # 驗證 EQP 查詢被觸發
        mock_search_eqp_signal.assert_called_once_with(
            self.state.eqp_id, self.state.eqp_signal_query_callback)

    def test_handle_query_reset_on_failure(self):
        """測試查詢失敗時的重置邏輯"""
        # 設置條件 - 查詢完成但沒有找到有效的 port
        self.state.queries_completed['eqp_signal'] = True
        self.state.port_have_carrier = False
        self.state.select_boxin_port = 0

        # 執行 handle
        self.state.handle(self.context)

        # 驗證查詢被重置
        self.assertFalse(self.state.queries_completed['carrier'])
        self.assertFalse(self.state.queries_completed['eqp_signal'])

    def test_port_id_to_port_number_conversion(self):
        """測試 port_id 到 port 編號的轉換"""
        test_cases = [
            (1011, 1),  # port_address + 1 -> port 1
            (1012, 2),  # port_address + 2 -> port 2
            (1013, 3),  # port_address + 3 -> port 3
            (1014, 4),  # port_address + 4 -> port 4
        ]

        for port_id, expected_port in test_cases:
            with self.subTest(port_id=port_id):
                # 創建 mock carrier
                mock_carrier = Mock()
                mock_carrier.id = 123
                mock_carrier.port_id = port_id
                mock_carrier.created_at = "2023-01-01 10:00:00"

                # 創建 mock response
                mock_response = Mock()
                mock_response.success = True
                mock_response.carriers = [mock_carrier]

                # 執行回調
                self.state.carrier_query_callback(mock_response)

                # 驗證結果
                self.assertEqual(self.state.select_boxin_port, expected_port)


if __name__ == '__main__':
    unittest.main()
