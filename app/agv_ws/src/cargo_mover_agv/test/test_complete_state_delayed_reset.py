#!/usr/bin/env python3
"""
測試 CompleteState 的延遲重置功能
"""

from cargo_mover_agv.robot_context import RobotContext
from cargo_mover_agv.robot_states.complete_state import CompleteState
import unittest
from unittest.mock import Mock, MagicMock, patch
import time
import sys
import os

# 添加路徑以便導入模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


class TestCompleteStateDelayedReset(unittest.TestCase):
    """測試 CompleteState 的延遲重置功能"""

    def setUp(self):
        """設置測試環境"""
        # 創建 mock node
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()

        # 創建 mock Hokuyo 物件
        self.mock_hokuyo_1 = Mock()
        self.mock_hokuyo_2 = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo_1
        self.mock_node.hokuyo_dms_8bit_2 = self.mock_hokuyo_2

        # 創建 CompleteState 實例
        self.state = CompleteState(self.mock_node)

        # 創建 mock context
        self.mock_context = Mock(spec=RobotContext)
        self.mock_context.rack_photo_up_or_down_buffer = None

    def test_initial_state(self):
        """測試初始狀態"""
        self.assertFalse(self.state.hokuyo_write_completed)
        self.assertFalse(self.state.reset_timer_started)
        self.assertIsNone(self.state.reset_start_time)
        self.assertFalse(self.state.reset_completed)

    def test_hokuyo_parameters_reset_success(self):
        """測試 Hokuyo 參數重置成功"""
        # 執行參數重置
        self.state._reset_hokuyo_parameters()

        # 驗證兩個 Hokuyo 物件都被正確調用
        # 檢查 hokuyo_dms_8bit_1
        self.mock_hokuyo_1.write_vaild.assert_called_once_with("1")
        self.mock_hokuyo_1.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_1.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_1.write_complete.assert_called_once_with("1")

        # 檢查 hokuyo_dms_8bit_2
        self.mock_hokuyo_2.write_vaild.assert_called_once_with("1")
        self.mock_hokuyo_2.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_2.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("1")

        # 驗證狀態標誌
        self.assertTrue(self.state.hokuyo_write_completed)
        self.assertTrue(self.state.reset_timer_started)
        self.assertIsNotNone(self.state.reset_start_time)
        self.assertFalse(self.state.reset_completed)

    def test_hokuyo_parameters_reset_exception(self):
        """測試 Hokuyo 參數重置異常處理"""
        # 設置 mock 拋出異常
        self.mock_hokuyo_1.write_valid.side_effect = Exception("Test exception")

        # 執行參數重置
        self.state._reset_hokuyo_parameters()

        # 驗證即使異常也標記為完成
        self.assertTrue(self.state.hokuyo_write_completed)

    @patch('time.time')
    def test_delayed_reset_not_ready(self, mock_time):
        """測試延遲重置未到時間"""
        # 設置時間
        start_time = 1000.0
        current_time = 1003.0  # 只過了 3 秒
        mock_time.side_effect = [start_time, current_time]

        # 先執行參數重置以啟動計時器
        self.state._reset_hokuyo_parameters()

        # 重置 mock 調用記錄
        self.mock_hokuyo_1.reset_mock()
        self.mock_hokuyo_2.reset_mock()

        # 執行延遲重置檢查
        self.state._handle_delayed_reset()

        # 驗證延遲重置尚未執行
        self.mock_hokuyo_1.write_vaild.assert_not_called()
        self.mock_hokuyo_2.write_vaild.assert_not_called()
        self.assertFalse(self.state.reset_completed)

    @patch('time.time')
    def test_delayed_reset_ready(self, mock_time):
        """測試延遲重置時間到達"""
        # 設置時間
        start_time = 1000.0
        current_time = 1006.0  # 過了 6 秒
        mock_time.side_effect = [start_time, current_time]

        # 先執行參數重置以啟動計時器
        self.state._reset_hokuyo_parameters()

        # 重置 mock 調用記錄
        self.mock_hokuyo_1.reset_mock()
        self.mock_hokuyo_2.reset_mock()

        # 執行延遲重置檢查
        self.state._handle_delayed_reset()

        # 驗證延遲重置已執行
        # 檢查 hokuyo_dms_8bit_1
        self.mock_hokuyo_1.write_vaild.assert_called_once_with("0")
        self.mock_hokuyo_1.write_complete.assert_called_once_with("0")

        # 檢查 hokuyo_dms_8bit_2
        self.mock_hokuyo_2.write_vaild.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("0")

        # 驗證完成標誌
        self.assertTrue(self.state.reset_completed)

    @patch('time.time')
    def test_delayed_reset_exception(self, mock_time):
        """測試延遲重置異常處理"""
        # 設置時間
        start_time = 1000.0
        current_time = 1006.0  # 過了 6 秒
        mock_time.side_effect = [start_time, current_time]

        # 先執行參數重置以啟動計時器
        self.state._reset_hokuyo_parameters()

        # 設置 mock 拋出異常
        self.mock_hokuyo_1.write_valid.side_effect = Exception("Test exception")

        # 執行延遲重置檢查
        self.state._handle_delayed_reset()

        # 驗證即使異常也標記為完成
        self.assertTrue(self.state.reset_completed)

    def test_delayed_reset_not_started(self):
        """測試延遲重置未啟動時的處理"""
        # 不執行參數重置，直接檢查延遲重置
        self.state._handle_delayed_reset()

        # 驗證延遲重置未執行
        self.mock_hokuyo_1.write_valid.assert_not_called()
        self.mock_hokuyo_2.write_valid.assert_not_called()
        self.assertFalse(self.state.reset_completed)

    def test_delayed_reset_already_completed(self):
        """測試延遲重置已完成時的處理"""
        # 手動設置狀態
        self.state.reset_timer_started = True
        self.state.reset_start_time = time.time()
        self.state.reset_completed = True

        # 執行延遲重置檢查
        self.state._handle_delayed_reset()

        # 驗證延遲重置未重複執行
        self.mock_hokuyo_1.write_vaild.assert_not_called()
        self.mock_hokuyo_2.write_vaild.assert_not_called()

    def test_handle_method_integration(self):
        """測試 handle 方法的整合功能"""
        # 執行 handle 方法
        self.state.handle(self.mock_context)

        # 驗證 Hokuyo 參數重置被調用
        self.assertTrue(self.state.hokuyo_write_completed)
        self.assertTrue(self.state.reset_timer_started)

        # 驗證 context 清理
        self.assertIsNone(self.mock_context.rack_photo_up_or_down_buffer)

    def test_multiple_handle_calls(self):
        """測試多次調用 handle 方法"""
        # 第一次調用
        self.state.handle(self.mock_context)

        # 重置 mock 調用記錄
        self.mock_hokuyo_1.reset_mock()
        self.mock_hokuyo_2.reset_mock()

        # 第二次調用
        self.state.handle(self.mock_context)

        # 驗證 Hokuyo 參數重置不會重複執行
        self.mock_hokuyo_1.write_valid.assert_not_called()
        self.mock_hokuyo_2.write_valid.assert_not_called()


if __name__ == '__main__':
    unittest.main()
