from loader_agv.robot_states.base_robot_state import BaseRobotState

"""
測試 TakeTransferState 類別
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import pytest

from loader_agv.robot_states.take_transfer.take_transfer_state import TakeTransferState
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot


class TestTakeTransferState(unittest.TestCase):
    """TakeTransferState 測試類別"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        
        # 創建 mock hokuyo
        self.mock_hokuyo = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo
        
        # 創建測試實例
        self.state = TakeTransferState(self.mock_node)
        
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
        self.mock_robot.BOX_IN_POSITION = 20
        self.mock_robot.read_pgno_response = Mock()
        self.mock_robot.read_pgno_response.value = self.mock_robot.IDLE

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

    def test_update_carrier_database(self):
        """測試 carrier 資料庫更新"""
        # 設置 context
        self.context.carrier_id = 123
        self.context.get_room_id = 2
        self.context.get_boxin_port = 3
        
        # 執行更新
        self.state.update_carrier_database(self.context)
        
        # 驗證 agvc_client.async_update_carrier 被調用
        self.state.agvc_client.async_update_carrier.assert_called_once()
        
        # 檢查傳遞的 carrier 參數
        call_args = self.state.agvc_client.async_update_carrier.call_args
        carrier = call_args[0][0]
        callback = call_args[0][1]
        
        self.assertEqual(carrier.id, 123)
        self.assertEqual(carrier.room_id, 2)
        self.assertEqual(carrier.rack_id, 0)
        self.assertEqual(carrier.port_id, 1003)  # 1000 + 3
        self.assertEqual(carrier.rack_index, 0)
        self.assertEqual(carrier.status_id, 1)
        self.assertEqual(callback, self.state.update_carrier_database_callback)

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

    def test_handle_step_check_idle_robot_not_idle(self):
        """測試 CHECK_IDLE 步驟 - 機器人不處於 IDLE"""
        # 設置條件
        self.state.step = RobotContext.CHECK_IDLE
        self.mock_robot.read_pgno_response.value = 999  # 非 IDLE 狀態
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果 - 步驟不應該改變
        self.assertEqual(self.state.step, RobotContext.CHECK_IDLE)

    def test_handle_step_write_chg_parameter(self):
        """測試 WRITE_CHG_PARAMTER 步驟"""
        # 設置條件
        self.state.step = RobotContext.WRITE_CHG_PARAMTER
        self.state.sent = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.mock_robot.write_chg_para.assert_called_once()
        self.assertTrue(self.state.sent)
        self.assertEqual(self.state.step, RobotContext.CHECK_CHG_PARA)

    def test_handle_step_write_chg_parameter_already_sent(self):
        """測試 WRITE_CHG_PARAMTER 步驟 - 已發送"""
        # 設置條件
        self.state.step = RobotContext.WRITE_CHG_PARAMTER
        self.state.sent = True
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果 - 不應該再次調用
        self.mock_robot.write_chg_para.assert_not_called()

    def test_handle_step_check_chg_para_success(self):
        """測試 CHECK_CHG_PARA 步驟 - 成功"""
        # 設置條件
        self.state.step = RobotContext.CHECK_CHG_PARA
        self.mock_robot.chg_para_success = True
        self.mock_robot.chg_para_failed = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.WRITE_PGNO)
        self.assertFalse(self.state.sent)

    def test_handle_step_check_chg_para_failed(self):
        """測試 CHECK_CHG_PARA 步驟 - 失敗"""
        # 設置條件
        self.state.step = RobotContext.CHECK_CHG_PARA
        self.mock_robot.chg_para_success = False
        self.mock_robot.chg_para_failed = True
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.IDLE)
        self.assertFalse(self.state.sent)

    def test_handle_step_write_pgno(self):
        """測試 WRITE_PGNO 步驟"""
        # 設置條件
        self.state.step = RobotContext.WRITE_PGNO
        self.state.sent = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        expected_pgno = (self.mock_robot.ACTION_TO + 
                        self.mock_robot.NONE_POSITION + 
                        self.mock_robot.BOX_IN_POSITION)
        self.mock_robot.write_pgno.assert_called_once_with(expected_pgno)
        self.assertTrue(self.state.sent)
        self.assertEqual(self.state.step, RobotContext.CHECK_PGNO)

    def test_handle_step_check_pgno_success(self):
        """測試 CHECK_PGNO 步驟 - 成功"""
        # 設置條件
        self.state.step = RobotContext.CHECK_PGNO
        self.mock_robot.pgno_success = True
        self.mock_robot.pgno_failed = False
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.ACTING)
        self.assertFalse(self.state.sent)

    def test_handle_step_check_pgno_failed(self):
        """測試 CHECK_PGNO 步驟 - 失敗"""
        # 設置條件
        self.state.step = RobotContext.CHECK_PGNO
        self.mock_robot.pgno_success = False
        self.mock_robot.pgno_failed = True
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.IDLE)
        self.assertFalse(self.state.sent)

    def test_handle_step_acting_in_progress(self):
        """測試 ACTING 步驟 - 動作進行中"""
        # 設置條件
        self.state.step = RobotContext.ACTING
        expected_pgno = (self.mock_robot.ACTION_TO + 
                        self.mock_robot.NONE_POSITION + 
                        self.mock_robot.BOX_IN_POSITION)
        self.mock_robot.read_pgno_response.value = expected_pgno
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果 - 步驟不應該改變
        self.assertEqual(self.state.step, RobotContext.ACTING)

    def test_handle_step_acting_completed(self):
        """測試 ACTING 步驟 - 動作完成"""
        # 設置條件
        self.state.step = RobotContext.ACTING
        self.mock_robot.read_pgno_response.value = Robot.IDLE
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, RobotContext.FINISH)

    def test_handle_step_acting_failed(self):
        """測試 ACTING 步驟 - 動作失敗"""
        # 設置條件
        self.state.step = RobotContext.ACTING
        self.mock_robot.read_pgno_response.value = 999  # 非預期值
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果 - 步驟不應該改變，但會記錄錯誤
        self.assertEqual(self.state.step, RobotContext.ACTING)

    @patch('loader_agv.robot_states.take_transfer.take_transfer_state.PutAgvState')
    def test_handle_step_finish_success(self, mock_put_agv_state):
        """測試 FINISH 步驟 - 成功"""
        # 設置條件
        self.state.step = RobotContext.FINISH
        self.mock_robot.read_pgno_response.value = Robot.IDLE
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果
        self.assertEqual(self.state.step, 0)
        self.context.set_state.assert_called_once()

    def test_handle_step_finish_not_ready(self):
        """測試 FINISH 步驟 - 尚未完成"""
        # 設置條件
        self.state.step = RobotContext.FINISH
        self.mock_robot.read_pgno_response.value = 999  # 非 IDLE
        
        # 執行 handle
        self.state.handle(self.context)
        
        # 驗證結果 - 步驟不應該改變
        self.assertEqual(self.state.step, RobotContext.FINISH)


if __name__ == '__main__':
    unittest.main()
