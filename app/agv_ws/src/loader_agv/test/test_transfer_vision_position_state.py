"""
測試 TransferVisionPositionState 類別
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import pytest

from loader_agv.robot_states.take_transfer.transfer_vision_position_state import TransferVisionPositionState
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot


class TestTransferVisionPositionState(unittest.TestCase):
    """TransferVisionPositionState 測試類別"""

    def setUp(self):
        """測試前設置"""
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()

        # 創建測試實例
        self.state = TransferVisionPositionState(self.mock_node)

        # 創建 mock context
        mock_initial_state = Mock()
        self.context = RobotContext(mock_initial_state)
        self.context.node = self.mock_node

        # 設置 mock robot
        self.mock_robot = Mock(spec=Robot)
        self.context.robot = self.mock_robot

        # 設置機器人常數
        self.mock_robot.PHOTO_BOX_IN = 200

    def test_enter(self):
        """測試進入狀態"""
        # 執行進入
        self.state.enter()

        # 驗證日誌記錄
        self.mock_node.get_logger().info.assert_called_with(
            "Robot Take Transfer 目前狀態: TranferVisionPosition")

    def test_leave(self):
        """測試離開狀態"""
        # 執行離開
        self.state.leave()

        # 驗證日誌記錄
        self.mock_node.get_logger().info.assert_called_with(
            "Robot Take Transfer 離開 TranferVisionPosition 狀態")

    @patch('loader_agv.robot_states.take_transfer.transfer_vision_position_state.TransferCheckHaveState')
    def test_handle_vision_steps(self, mock_transfer_check_have_state):
        """測試視覺定位步驟處理"""
        # 執行 handle
        self.state.handle(self.context)

        # 驗證日誌記錄
        self.mock_node.get_logger().info.assert_called_with(
            "Robot Take Transfer 傳送箱視覺定位中 狀態")

    def test_handle_calls_base_vision_steps(self):
        """測試 handle 調用基礎視覺步驟"""
        # 由於 TransferVisionPositionState 繼承自 BaseVisionPositionState
        # 我們需要 mock _handle_vision_steps 方法
        with patch.object(self.state, '_handle_vision_steps') as mock_handle_vision_steps:
            # 執行 handle
            self.state.handle(self.context)

            # 驗證 _handle_vision_steps 被調用
            mock_handle_vision_steps.assert_called_once()

            # 檢查調用參數
            call_args = mock_handle_vision_steps.call_args[0]
            self.assertEqual(call_args[0], self.context)
            self.assertEqual(call_args[1], self.mock_robot.PHOTO_BOX_IN)
            # 第三個參數應該是 TransferCheckHaveState 類

    def test_context_buffer_reset(self):
        """測試 context buffer 重置 - 此測試已移除因為 rack_photo_up_or_down_buffer 不再使用"""
        # 執行 handle
        self.state.handle(self.context)

        # 驗證日誌記錄正常
        self.mock_node.get_logger().info.assert_called_with(
            "Robot Take Transfer 傳送箱視覺定位中 狀態")

    def test_inheritance_from_base_vision_position_state(self):
        """測試繼承自 BaseVisionPositionState"""
        from loader_agv.robot_states.base_robot_state import BaseVisionPositionState

        # 驗證繼承關係
        self.assertIsInstance(self.state, BaseVisionPositionState)

    def test_state_transition_target(self):
        """測試狀態轉換目標"""
        # 這個測試驗證 handle 方法中指定的下一個狀態類型
        with patch.object(self.state, '_handle_vision_steps') as mock_handle_vision_steps:
            self.state.handle(self.context)

            # 檢查傳遞給 _handle_vision_steps 的第三個參數
            call_args = mock_handle_vision_steps.call_args[0]
            next_state_class = call_args[2]

            # 驗證下一個狀態是 TransferCheckHaveState
            from loader_agv.robot_states.take_transfer.transfer_check_have_state import TransferCheckHaveState
            self.assertEqual(next_state_class, TransferCheckHaveState)

    def test_robot_photo_constant_usage(self):
        """測試機器人照片常數的使用"""
        with patch.object(self.state, '_handle_vision_steps') as mock_handle_vision_steps:
            self.state.handle(self.context)

            # 檢查傳遞給 _handle_vision_steps 的第二個參數
            call_args = mock_handle_vision_steps.call_args[0]
            photo_constant = call_args[1]

            # 驗證使用的是 PHOTO_BOX_IN 常數
            self.assertEqual(photo_constant, self.context.robot.PHOTO_BOX_IN)

    def test_multiple_handle_calls(self):
        """測試多次調用 handle"""
        # 第一次調用
        self.state.handle(self.context)

        # 驗證第一次調用的日誌
        self.mock_node.get_logger().info.assert_called_with(
            "Robot Take Transfer 傳送箱視覺定位中 狀態")

        # 重置 mock
        self.mock_node.get_logger().info.reset_mock()

        # 第二次調用
        self.state.handle(self.context)

        # 驗證第二次調用的日誌
        self.mock_node.get_logger().info.assert_called_with(
            "Robot Take Transfer 傳送箱視覺定位中 狀態")

    def test_logging_consistency(self):
        """測試日誌記錄的一致性"""
        # 測試進入狀態的日誌
        self.state.enter()
        enter_calls = [call for call in self.mock_node.get_logger().info.call_args_list
                       if "目前狀態: TranferVisionPosition" in str(call)]
        self.assertEqual(len(enter_calls), 1)

        # 重置 mock
        self.mock_node.get_logger().info.reset_mock()

        # 測試離開狀態的日誌
        self.state.leave()
        leave_calls = [call for call in self.mock_node.get_logger().info.call_args_list
                       if "離開 TranferVisionPosition 狀態" in str(call)]
        self.assertEqual(len(leave_calls), 1)

        # 重置 mock
        self.mock_node.get_logger().info.reset_mock()

        # 測試 handle 的日誌
        self.state.handle(self.context)
        handle_calls = [call for call in self.mock_node.get_logger().info.call_args_list
                        if "傳送箱視覺定位中 狀態" in str(call)]
        self.assertEqual(len(handle_calls), 1)

    def test_state_name_consistency(self):
        """測試狀態名稱的一致性"""
        # 檢查類名和日誌中的狀態名稱是否一致
        self.assertEqual(self.state.__class__.__name__, "TransferVisionPositionState")

        # 檢查日誌中使用的狀態名稱
        self.state.enter()
        enter_log = self.mock_node.get_logger().info.call_args[0][0]
        self.assertIn("TranferVisionPosition", enter_log)  # 注意：原代碼中是 "Tranfer" 而不是 "Transfer"

    def test_base_class_method_calls(self):
        """測試基類方法調用"""
        # 驗證 enter 和 leave 方法調用基類的 _reset_common_state
        with patch.object(self.state, '_reset_common_state') as mock_reset:
            self.state.enter()
            mock_reset.assert_called_once()

            mock_reset.reset_mock()

            self.state.leave()
            mock_reset.assert_called_once()


if __name__ == '__main__':
    unittest.main()
