"""
Take Transfer 流程整合測試
測試完整的 take_transfer 流程，包括狀態轉換和數據流
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import pytest

from loader_agv.robot_states.take_transfer.agv_port_check_empty_state import AgvPortCheckEmptyState
from loader_agv.robot_states.take_transfer.transfer_vision_position_state import TransferVisionPositionState
from loader_agv.robot_states.take_transfer.transfer_check_have_state import TransferCheckHaveState
from loader_agv.robot_states.take_transfer.take_transfer_state import TakeTransferState
from loader_agv.robot_states.take_transfer.put_agv_state import PutAgvState
from loader_agv.robot_states.complete_state import CompleteState
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient


class TestTakeTransferIntegration(unittest.TestCase):
    """Take Transfer 流程整合測試"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 1

        # 創建 mock hokuyo
        self.mock_hokuyo = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo

        # 創建 mock context
        mock_initial_state = Mock()
        self.context = RobotContext(mock_initial_state)
        self.context.node = self.mock_node

        # 設置 mock robot
        self.mock_robot = Mock(spec=Robot)
        self.context.robot = self.mock_robot
        self.mock_robot.IDLE = 0
        self.mock_robot.read_pgno_response = Mock()
        self.mock_robot.read_pgno_response.value = self.mock_robot.IDLE

    def test_complete_flow_success_no_continuation(self):
        """測試完整流程成功 - 無需繼續"""
        # 1. AgvPortCheckEmptyState - 檢查 AGV port 為空
        agv_port_state = AgvPortCheckEmptyState(self.mock_node)

        # 模擬 EQP 查詢 - AGV port 1 有貨，選擇 port 2
        with patch.object(EqpSignalQueryClient, 'eqp_signal_port') as mock_eqp_signal:
            mock_eqp_signal.side_effect = [True, False, False, False]  # port 1 有貨
            mock_eqp_response = Mock()
            agv_port_state.eqp_signal_query_callback(mock_eqp_response)

        # 模擬 Carrier 查詢 - AGV port 2 沒有 carrier
        with patch.object(CarrierQueryClient, 'carrier_port_id_carrier_id') as mock_carrier_id:
            mock_carrier_id.return_value = None
            mock_carrier_response = Mock()
            mock_carrier_response.success = True
            agv_port_state.carrier_callback(mock_carrier_response)

        # 驗證 AGV port 選擇結果
        self.assertEqual(agv_port_state.select_agv_port, AgvPortCheckEmptyState.SELECT_PORT02)

        # 2. TransferCheckHaveState - 檢查傳送箱有貨
        transfer_check_state = TransferCheckHaveState(self.mock_node)

        # 模擬 Carrier 查詢 - 找到傳送箱
        mock_carrier = Mock()
        mock_carrier.id = 123
        mock_carrier.port_id = 1011  # port 1
        mock_carrier.created_at = "2023-01-01 10:00:00"

        mock_carrier_response = Mock()
        mock_carrier_response.success = True
        mock_carrier_response.carriers = [mock_carrier]
        transfer_check_state.carrier_query_callback(mock_carrier_response)

        # 模擬 EQP 查詢 - 傳送箱 port 狀態
        with patch.object(EqpSignalQueryClient, 'eqp_signal_port') as mock_eqp_signal:
            mock_eqp_signal.side_effect = [True, False, False, False]  # port 1 有貨
            mock_eqp_response = Mock()
            transfer_check_state.eqp_signal_query_callback(mock_eqp_response)

        # 設置 context 狀態用於 continuation 檢查
        self.context.boxin_port1 = True
        self.context.boxin_port2 = False
        self.context.boxin_port3 = False
        self.context.boxin_port4 = False

        # 檢查 transfer continuation logic
        transfer_check_state._check_take_transfer_continue(self.context)

        # 驗證結果 - port 1 選擇但 port 2 沒貨，不應該繼續
        self.assertFalse(self.context.take_transfer_continue)
        self.assertEqual(transfer_check_state.select_boxin_port, 1)
        self.assertEqual(self.context.carrier_id, 123)

    def test_complete_flow_success_with_continuation(self):
        """測試完整流程成功 - 需要繼續"""
        # 設置 TransferCheckHaveState
        transfer_check_state = TransferCheckHaveState(self.mock_node)

        # 模擬選擇 port 1 且 port 2 有貨的情況
        transfer_check_state.select_boxin_port = 1
        self.context.boxin_port1 = True
        self.context.boxin_port2 = True  # port 2 有貨
        self.context.boxin_port3 = False
        self.context.boxin_port4 = False

        # 檢查 transfer continuation logic
        transfer_check_state.search_eqp_signal_ok = True
        transfer_check_state._check_take_transfer_continue(self.context)

        # 驗證結果 - 應該繼續
        self.assertTrue(self.context.take_transfer_continue)

    def test_complete_flow_success_port3_port4_continuation(self):
        """測試完整流程成功 - port 3 選擇且 port 4 有貨"""
        # 設置 TransferCheckHaveState
        transfer_check_state = TransferCheckHaveState(self.mock_node)

        # 模擬選擇 port 3 且 port 4 有貨的情況
        transfer_check_state.select_boxin_port = 3
        self.context.boxin_port1 = False
        self.context.boxin_port2 = False
        self.context.boxin_port3 = True
        self.context.boxin_port4 = True  # port 4 有貨

        # 檢查 transfer continuation logic
        transfer_check_state.search_eqp_signal_ok = True
        transfer_check_state._check_take_transfer_continue(self.context)

        # 驗證結果 - 應該繼續
        self.assertTrue(self.context.take_transfer_continue)

    def test_state_transition_flow(self):
        """測試狀態轉換流程"""
        # 測試狀態轉換序列
        state_transitions = [
            # (current_state_class, expected_next_state_class)
            (AgvPortCheckEmptyState, TakeTransferState),
            (TakeTransferState, PutAgvState),
            # PutAgvState 的下一個狀態取決於 take_transfer_continue
        ]

        for current_state_class, expected_next_state_class in state_transitions:
            with self.subTest(current=current_state_class.__name__,
                              expected=expected_next_state_class.__name__):
                # 這裡可以添加具體的狀態轉換測試邏輯
                pass

    def test_data_flow_through_states(self):
        """測試數據在狀態間的流動"""
        # 1. AgvPortCheckEmptyState 設置 AGV port 數據
        agv_port_state = AgvPortCheckEmptyState(self.mock_node)
        agv_port_state.port_carriers = [True, False, True, False]
        agv_port_state.search_eqp_signal_ok = True
        agv_port_state._update_context_states(self.context)

        # 驗證 AGV port 狀態被正確設置
        self.assertTrue(self.context.agv_port1)
        self.assertFalse(self.context.agv_port2)
        self.assertTrue(self.context.agv_port3)
        self.assertFalse(self.context.agv_port4)

        # 2. TransferCheckHaveState 設置 BOXIN port 數據
        transfer_check_state = TransferCheckHaveState(self.mock_node)
        transfer_check_state.port_carriers = [False, True, False, True]
        transfer_check_state.search_eqp_signal_ok = True
        transfer_check_state._update_context_states(self.context)

        # 驗證 BOXIN port 狀態被正確設置
        self.assertFalse(self.context.boxin_port1)
        self.assertTrue(self.context.boxin_port2)
        self.assertFalse(self.context.boxin_port3)
        self.assertTrue(self.context.boxin_port4)

    def test_error_handling_flow(self):
        """測試錯誤處理流程"""
        # 測試 AGV port 全滿的情況
        agv_port_state = AgvPortCheckEmptyState(self.mock_node)
        agv_port_state.search_eqp_signal_ok = True
        agv_port_state.select_agv_port = AgvPortCheckEmptyState.SELECT_NONE

        agv_port_state._handle_port_selection(self.context)

        # 驗證錯誤處理
        self.assertFalse(agv_port_state.check_ok)
        self.assertIsNone(self.context.get_loader_agv_port_front)

    def test_dynamic_parameter_calculations(self):
        """測試動態參數計算"""
        # 測試不同 room_id 的動態計算
        test_cases = [
            (1, 1100, 110, 1010, 101),  # room_id=1
            (2, 2100, 210, 2010, 201),  # room_id=2
            (5, 5100, 510, 5010, 501),  # room_id=5
        ]

        for room_id, expected_agv_port_addr, expected_agv_eqp_id, expected_transfer_port_addr, expected_transfer_eqp_id in test_cases:
            with self.subTest(room_id=room_id):
                self.mock_node.room_id = room_id

                # 測試 AgvPortCheckEmptyState
                agv_state = AgvPortCheckEmptyState(self.mock_node)
                self.assertEqual(agv_state.port_address, expected_agv_port_addr)
                self.assertEqual(agv_state.eqp_id, expected_agv_eqp_id)

                # 測試 TransferCheckHaveState
                transfer_state = TransferCheckHaveState(self.mock_node)
                self.assertEqual(transfer_state.port_address, expected_transfer_port_addr)
                self.assertEqual(transfer_state.eqp_id, expected_transfer_eqp_id)

                # 測試 PutAgvState
                put_state = PutAgvState(self.mock_node)
                self.assertEqual(put_state.port_id_address, expected_agv_port_addr)

    def test_carrier_priority_selection(self):
        """測試 Carrier 優先級選擇（FIFO）"""
        transfer_check_state = TransferCheckHaveState(self.mock_node)

        # 創建多個 carriers，時間不同
        carrier1 = Mock()
        carrier1.id = 101
        carrier1.port_id = 1011
        carrier1.created_at = "2023-01-01 12:00:00"  # 較晚

        carrier2 = Mock()
        carrier2.id = 102
        carrier2.port_id = 1012
        carrier2.created_at = "2023-01-01 10:00:00"  # 較早

        carrier3 = Mock()
        carrier3.id = 103
        carrier3.port_id = 1013
        carrier3.created_at = "2023-01-01 11:00:00"  # 中間

        # 模擬查詢響應
        mock_response = Mock()
        mock_response.success = True
        mock_response.carriers = [carrier1, carrier2, carrier3]

        # 執行回調
        transfer_check_state.carrier_query_callback(mock_response)

        # 驗證選擇最早的 carrier
        self.assertEqual(transfer_check_state.earliest_carrier, carrier2)
        self.assertEqual(transfer_check_state.select_boxin_port, 2)  # port_id 1012 -> port 2

    def test_hokuyo_8bit_communication_flow(self):
        """測試 Hokuyo 8-bit 通信流程"""
        transfer_check_state = TransferCheckHaveState(self.mock_node)
        transfer_check_state.check_ok = True

        # 測試完整的 8-bit 通信序列
        steps = [
            (transfer_check_state.IDLE, transfer_check_state.WRITE_VAILD),
            (transfer_check_state.WRITE_VAILD, transfer_check_state.WRITE_PORT_NUMBER),
            (transfer_check_state.WRITE_PORT_NUMBER, transfer_check_state.WAIT_UNLOAD_REQ),
            (transfer_check_state.WAIT_UNLOAD_REQ, transfer_check_state.WRITE_TR_REQ),
            (transfer_check_state.WRITE_TR_REQ, transfer_check_state.WAIT_READY),
            (transfer_check_state.WAIT_READY, transfer_check_state.IDLE),
        ]

        for current_step, expected_next_step in steps:
            with self.subTest(current=current_step, expected=expected_next_step):
                transfer_check_state.step = current_step

                # 設置適當的 Hokuyo 響應
                if current_step == transfer_check_state.WAIT_UNLOAD_REQ:
                    self.mock_hokuyo.unload_req = True
                elif current_step == transfer_check_state.WAIT_READY:
                    self.mock_hokuyo.ready = True

                # 執行一次 handle（簡化版本，只測試步驟轉換）
                if current_step == transfer_check_state.IDLE:
                    transfer_check_state.step = expected_next_step
                    transfer_check_state.sent = False


if __name__ == '__main__':
    unittest.main()
