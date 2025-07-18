#!/usr/bin/env python3
"""
測試 CompleteState 中的 Hokuyo 寫入功能（並行版本）
"""

import unittest
from unittest.mock import Mock, MagicMock
import rclpy
from cargo_mover_agv.robot_states.complete_state import CompleteState
from cargo_mover_agv.robot_context import RobotContext


class TestCompleteStateHokuyo(unittest.TestCase):
    def setUp(self):
        """設置測試環境"""
        # 初始化 ROS2
        if not rclpy.ok():
            rclpy.init()

        # 創建模擬節點
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()

        # 創建兩個模擬 Hokuyo 物件
        self.mock_hokuyo_1 = Mock()
        self.mock_hokuyo_2 = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo_1
        self.mock_node.hokuyo_dms_8bit_2 = self.mock_hokuyo_2

        # 創建模擬 task
        self.mock_task = Mock()
        self.mock_task.id = 1
        self.mock_task.work_id = 123
        self.mock_task.room_id = 2
        self.mock_task.node_id = "test_node"
        self.mock_task.description = "test description"
        self.mock_task.agv_id = 1
        self.mock_task.priority = 1
        self.mock_task.parameters = "{}"
        self.mock_node.task = self.mock_task

        # 創建 CompleteState 實例
        self.state = CompleteState(self.mock_node)

        # 創建模擬 context
        self.mock_context = Mock(spec=RobotContext)

    def test_hokuyo_write_initialization(self):
        """測試 Hokuyo 寫入初始化"""
        self.assertFalse(self.state.hokuyo_write_completed)

    def test_reset_hokuyo_parameters_success(self):
        """測試 Hokuyo 參數重置成功"""
        # 確保初始狀態
        self.assertFalse(self.state.hokuyo_write_completed)

        # 執行參數重置
        self.state._reset_hokuyo_parameters()

        # 驗證兩個 Hokuyo 物件都被正確調用
        # 檢查 hokuyo_dms_8bit_1
        self.mock_hokuyo_1.write_vaild.assert_called_once_with("0")
        self.mock_hokuyo_1.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_1.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_1.write_complete.assert_called_once_with("1")

        # 檢查 hokuyo_dms_8bit_2
        self.mock_hokuyo_2.write_vaild.assert_called_once_with("0")
        self.mock_hokuyo_2.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_2.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("1")

        # 驗證完成標誌
        self.assertTrue(self.state.hokuyo_write_completed)

    def test_reset_hokuyo_parameters_already_completed(self):
        """測試 Hokuyo 參數重置已完成的情況"""
        # 設置為已完成
        self.state.hokuyo_write_completed = True

        # 執行參數重置
        self.state._reset_hokuyo_parameters()

        # 驗證不會重複調用
        self.mock_hokuyo_1.write_vaild.assert_not_called()
        self.mock_hokuyo_1.write_tr_req.assert_not_called()
        self.mock_hokuyo_1.write_busy.assert_not_called()
        self.mock_hokuyo_1.write_complete.assert_not_called()

        self.mock_hokuyo_2.write_vaild.assert_not_called()
        self.mock_hokuyo_2.write_tr_req.assert_not_called()
        self.mock_hokuyo_2.write_busy.assert_not_called()
        self.mock_hokuyo_2.write_complete.assert_not_called()

    def test_reset_hokuyo_parameters_exception_handling(self):
        """測試 Hokuyo 參數重置異常處理"""
        # 設置異常
        self.mock_hokuyo_1.write_valid.side_effect = Exception("Test exception")

        # 執行參數重置
        self.state._reset_hokuyo_parameters()

        # 驗證異常被處理且標記為完成
        self.assertTrue(self.state.hokuyo_write_completed)

    def test_handle_with_parallel_hokuyo_reset(self):
        """測試 handle 方法包含並行 Hokuyo 重置"""
        # 模擬 update_task_success 為 True 以跳過 update_task
        self.state.update_task_success = True

        # 調用 handle 方法
        self.state.handle(self.mock_context)

        # 驗證兩個 Hokuyo 物件都被調用
        self.mock_hokuyo_1.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_1.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_1.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_1.write_complete.assert_called_once_with("1")

        self.mock_hokuyo_2.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_2.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_2.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("1")

        # 驗證完成標誌
        self.assertTrue(self.state.hokuyo_write_completed)

    def tearDown(self):
        """清理測試環境"""
        pass


if __name__ == '__main__':
    unittest.main()
