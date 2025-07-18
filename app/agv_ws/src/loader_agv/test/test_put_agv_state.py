from loader_agv.robot_states.base_robot_state import BaseRobotState

"""
測試 PutAgvState 類別
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import pytest

from loader_agv.robot_states.take_transfer.put_agv_state import PutAgvState
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from db_proxy.models import CarrierMsg


class TestPutAgvState(unittest.TestCase):
    """PutAgvState 測試類別"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 1
        
        # 創建 mock hokuyo
        self.mock_hokuyo = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo
        
        # 創建測試實例
        self.state = PutAgvState(self.mock_node)
        
        # 創建 mock context
        mock_initial_state = Mock()
        self.context = RobotContext(mock_initial_state)
        self.context.node = self.mock_node
        
        # 設置 mock robot
        self.mock_robot = Mock(spec=Robot)
        self.context.robot = self.mock_robot
        
        # 設置機器人常數和響應
        self.mock_robot.IDLE = 0
        self.mock_robot.ACTION_TO = 100
        self.mock_robot.NONE_POSITION = 0
        self.mock_robot.AGV_POSITION = 10
        self.mock_robot.read_pgno_response = Mock()
        self.mock_robot.read_pgno_response.value = self.mock_robot.IDLE

    def test_init_dynamic_calculations(self):
        """測試動態計算參數"""
        # 測試 room_id = 1 的情況
        self.assertEqual(self.state.port_id_address, 1100)  # 1 * 1000 + 100
        
        # 測試不同 room_id
        self.mock_node.room_id = 3
        state2 = PutAgvState(self.mock_node)
        self.assertEqual(state2.port_id_address, 3100)  # 3 * 1000 + 100

    def test_init_state_variables(self):
        """測試初始狀態變數"""
        self.assertEqual(self.state.step, RobotContext.IDLE)
        self.assertFalse(self.state.update_carrier_success)
        self.assertFalse(self.state.sent)
        self.assertFalse(self.state.hokuyo_input_updated)

    def test_enter_and_leave(self):
        """測試進入和離開狀態"""
        # 設置一些狀態
        self.state.update_carrier_success = True
        self.state.sent = True
        self.state.hokuyo_input_updated = True
        
        # 測試進入狀態
        self.state.enter()
        self.assertFalse(self.state.update_carrier_success)
        self.assertFalse(self.state.sent)
        self.assertFalse(self.state.hokuyo_input_updated)
        
        # 設置一些狀態
        self.state.update_carrier_success = True
        self.state.sent = True
        self.state.hokuyo_input_updated = True
        
        # 測試離開狀態
        self.state.leave()
        self.assertFalse(self.state.update_carrier_success)
        self.assertFalse(self.state.sent)
        self.assertFalse(self.state.hokuyo_input_updated)

    def test_update_carrier_database(self):
        """測試 carrier 資料庫更新"""
        # 設置 context
        self.context.carrier_id = 456
        self.context.get_room_id = 2
        self.context.get_loader_agv_port_front = 3
        
        # 執行更新
        self.state.update_carrier_database(self.context)
        
        # 驗證 agvc_client.async_update_carrier 被調用
        self.state.agvc_client.async_update_carrier.assert_called_once()
        
        # 檢查傳遞的 carrier 參數
        call_args = self.state.agvc_client.async_update_carrier.call_args
        carrier = call_args[0][0]
        callback = call_args[0][1]
        
        self.assertEqual(carrier.id, 456)
        self.assertEqual(carrier.room_id, 2)
        self.assertEqual(carrier.rack_id, 0)
        self.assertEqual(carrier.port_id, 1103)  # port_id_address + get_loader_agv_port_front
        self.assertEqual(carrier.rack_index, 0)
        self.assertEqual(carrier.status_id, 2)
        self.assertEqual(callback, self.state.update_carrier_database_callback)

    def test_update_carrier_database_callback_success(self):
        """測試 carrier 資料庫更新成功回調"""
        # 創建 mock result
        mock_result = Mock()
        mock_result.success = True
        mock_result.message = "Update successful"
        
        # 執行回調
        self.state.update_carrier_database_callback(mock_result)
        
        # 驗證結果
        self.assertTrue(self.state.update_carrier_success)

    def test_update_carrier_database_callback_failed(self):
        """測試 carrier 資料庫更新失敗回調"""
        # 執行回調 - None result
        self.state.update_carrier_database_callback(None)
        
        # 驗證結果
        self.assertFalse(self.state.update_carrier_success)

    def test_handle_step_idle(self):
        """測試 IDLE 步驟"""
        # 設置條件
        self.state.step = RobotContext.IDLE
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.CHECK_IDLE)

    def test_handle_step_check_idle_robot_idle(self):
        """測試 CHECK_IDLE 步驟 - 機器人處於 IDLE"""
        # 設置條件
        self.state.step = RobotContext.CHECK_IDLE
        self.mock_robot.read_pgno_response.value = Robot.IDLE
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.WRITE_CHG_PARAMTER)

    def test_handle_step_acting_completed(self):
        """測試 ACTING 步驟 - 動作完成"""
        # 設置條件
        self.state.step = RobotContext.ACTING
        self.mock_robot.read_pgno_response.value = Robot.IDLE
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.FINISH)

    def test_handle_step_finish_success(self):
        """測試 FINISH 步驟 - 成功"""
        # 設置條件
        self.state.step = RobotContext.FINISH
        self.mock_robot.read_pgno_response.value = Robot.IDLE
        self.context.get_loader_agv_port_front = 2
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.UPDATE_DATABASE)
        self.assertEqual(self.context.boxin_buffer, 2)

    def test_handle_step_update_database_first_call(self):
        """測試 UPDATE_DATABASE 步驟 - 首次調用"""
        # 設置條件
        self.state.step = RobotContext.UPDATE_DATABASE
        self.state.sent = False
        self.context.carrier_id = 789
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertTrue(self.state.sent)
        self.state.agvc_client.async_update_carrier.assert_called_once()

    @patch('loader_agv.robot_states.take_transfer.put_agv_state.TransferCheckHaveState')
    def test_handle_step_update_database_continue_transfer(self, mock_transfer_check_have_state):
        """測試 UPDATE_DATABASE 步驟 - 繼續 transfer"""
        # 設置條件
        self.state.step = RobotContext.UPDATE_DATABASE
        self.state.sent = True
        self.state.update_carrier_success = True
        self.context.take_transfer_continue = True
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertFalse(self.state.sent)
        self.context.set_state.assert_called_once()
        # 驗證轉換到 TransferCheckHaveState
        args = self.context.set_state.call_args[0]
        self.assertIsInstance(args[0], type(mock_transfer_check_have_state.return_value))

    @patch('loader_agv.robot_states.take_transfer.put_agv_state.CompleteState')
    def test_handle_step_update_database_complete_transfer(self, mock_complete_state):
        """測試 UPDATE_DATABASE 步驟 - 完成 transfer"""
        # 設置條件
        self.state.step = RobotContext.UPDATE_DATABASE
        self.state.sent = True
        self.state.update_carrier_success = True
        self.context.take_transfer_continue = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertFalse(self.state.sent)
        self.context.set_state.assert_called_once()
        # 驗證轉換到 CompleteState
        args = self.context.set_state.call_args[0]
        self.assertIsInstance(args[0], type(mock_complete_state.return_value))

    def test_handle_step_update_database_not_success(self):
        """測試 UPDATE_DATABASE 步驟 - 更新未成功"""
        # 設置條件
        self.state.step = RobotContext.UPDATE_DATABASE
        self.state.sent = True
        self.state.update_carrier_success = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果 - 不應該轉換狀態
        self.context.set_state.assert_not_called()

    def test_transfer_continuation_decision_logic(self):
        """測試 transfer continuation 決策邏輯"""
        test_cases = [
            # (take_transfer_continue, expected_state_type)
            (True, 'TransferCheckHaveState'),
            (False, 'CompleteState'),
        ]
        
        for continue_flag, expected_state in test_cases:
            with self.subTest(continue_flag=continue_flag):
                # 重置 context
                self.context.set_state.reset_mock()
                
                # 設置條件
                self.state.step = RobotContext.UPDATE_DATABASE
                self.state.sent = True
                self.state.update_carrier_success = True
                self.context.take_transfer_continue = continue_flag
                
                # 根據預期狀態設置 patch
                if expected_state == 'TransferCheckHaveState':
                    with patch('loader_agv.robot_states.take_transfer.put_agv_state.TransferCheckHaveState') as mock_state:
                        self.state.handle(self.context)
                        self.context.set_state.assert_called_once()
                        args = self.context.set_state.call_args[0]
                        self.assertIsInstance(args[0], type(mock_state.return_value))
                else:
                    with patch('loader_agv.robot_states.take_transfer.put_agv_state.CompleteState') as mock_state:
                        self.state.handle(self.context)
                        self.context.set_state.assert_called_once()
                        args = self.context.set_state.call_args[0]
                        self.assertIsInstance(args[0], type(mock_state.return_value))

    def test_robot_pgno_calculation(self):
        """測試機器人 PGNO 計算"""
        # 設置條件
        self.state.step = RobotContext.WRITE_PGNO
        self.state.sent = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證 PGNO 計算
        expected_pgno = (self.mock_robot.ACTION_TO + 
                        self.mock_robot.NONE_POSITION + 
                        self.mock_robot.AGV_POSITION)
        self.mock_robot.write_pgno.assert_called_once_with(expected_pgno)

    def test_hokuyo_input_continuous_update(self):
        """測試 Hokuyo Input 持續更新"""
        # 設置條件
        self.state.hokuyo_input_updated = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證 Hokuyo Input 被更新
        self.mock_hokuyo.update_input.assert_called_once()
        # 注意：根據記憶，應該是持續更新，不設置 hokuyo_input_updated flag

    def test_edge_case_room_id_zero(self):
        """測試邊界情況 - room_id 為 0"""
        self.mock_node.room_id = 0
        state = PutAgvState(self.mock_node)
        self.assertEqual(state.port_id_address, 100)  # 0 * 1000 + 100

    def test_edge_case_large_room_id(self):
        """測試邊界情況 - 大的 room_id"""
        self.mock_node.room_id = 999
        state = PutAgvState(self.mock_node)
        self.assertEqual(state.port_id_address, 999100)  # 999 * 1000 + 100


if __name__ == '__main__':
    unittest.main()
