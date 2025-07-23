#!/usr/bin/env python3
"""
測試 IdleState 中的 Hokuyo 寫入功能
"""

import unittest
from unittest.mock import Mock, MagicMock
import rclpy
from cargo_mover_agv.robot_states.idle_state import IdleState
from cargo_mover_agv.robot_context import RobotContext


class TestIdleStateHokuyo(unittest.TestCase):
    def setUp(self):
        """設置測試環境"""
        # 初始化 ROS2
        if not rclpy.ok():
            rclpy.init()

        # 創建模擬節點
        self.mock_node = Mock()
        self.mock_node.get_logger.return_value = Mock()
        self.mock_node.room_id = 2
        self.mock_node.work_id = 2000201  # 修正：對應正確的 exit_work (room_id=2, "00", "02", "01")

        # 創建兩個模擬 Hokuyo 物件
        self.mock_hokuyo_1 = Mock()
        self.mock_hokuyo_2 = Mock()
        self.mock_node.hokuyo_dms_8bit_1 = self.mock_hokuyo_1
        self.mock_node.hokuyo_dms_8bit_2 = self.mock_hokuyo_2

        # 創建 IdleState 實例
        self.state = IdleState(self.mock_node)

        # 創建模擬 context
        self.mock_context = Mock(spec=RobotContext)

    def test_hokuyo_write_initialization(self):
        """測試 Hokuyo 寫入初始化"""
        self.assertFalse(self.state.hokuyo_write_completed)

    def test_initialize_hokuyo_parameters_success(self):
        """測試 Hokuyo 參數初始化成功"""
        # 確保初始狀態
        self.assertFalse(self.state.hokuyo_write_completed)

        # 執行參數初始化
        self.state._initialize_hokuyo_parameters()

        # 驗證兩個 Hokuyo 物件都被正確調用
        # 檢查 hokuyo_dms_8bit_1
        self.mock_hokuyo_1.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_1.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_1.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_1.write_complete.assert_called_once_with("0")  # IdleState 設定為 0

        # 檢查 hokuyo_dms_8bit_2
        self.mock_hokuyo_2.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_2.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_2.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("0")  # IdleState 設定為 0

        # 驗證完成標誌
        self.assertTrue(self.state.hokuyo_write_completed)

    def test_initialize_hokuyo_parameters_already_completed(self):
        """測試 Hokuyo 參數初始化已完成的情況"""
        # 設置為已完成
        self.state.hokuyo_write_completed = True

        # 執行參數初始化
        self.state._initialize_hokuyo_parameters()

        # 驗證不會重複調用
        self.mock_hokuyo_1.write_valid.assert_not_called()
        self.mock_hokuyo_1.write_tr_req.assert_not_called()
        self.mock_hokuyo_1.write_busy.assert_not_called()
        self.mock_hokuyo_1.write_complete.assert_not_called()

        self.mock_hokuyo_2.write_valid.assert_not_called()
        self.mock_hokuyo_2.write_tr_req.assert_not_called()
        self.mock_hokuyo_2.write_busy.assert_not_called()
        self.mock_hokuyo_2.write_complete.assert_not_called()

    def test_initialize_hokuyo_parameters_exception_handling(self):
        """測試 Hokuyo 參數初始化異常處理"""
        # 設置異常
        self.mock_hokuyo_1.write_valid.side_effect = Exception("Test exception")

        # 執行參數初始化
        self.state._initialize_hokuyo_parameters()

        # 驗證異常被處理且標記為完成
        self.assertTrue(self.state.hokuyo_write_completed)

    def test_handle_with_parallel_hokuyo_initialization(self):
        """測試 handle 方法包含並行 Hokuyo 初始化"""
        # 調用 handle 方法
        self.state.handle(self.mock_context)

        # 驗證兩個 Hokuyo 物件都被調用
        self.mock_hokuyo_1.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_1.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_1.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_1.write_complete.assert_called_once_with("0")  # IdleState 特有

        self.mock_hokuyo_2.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_2.write_tr_req.assert_called_once_with("0")
        self.mock_hokuyo_2.write_busy.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("0")  # IdleState 特有

        # 驗證完成標誌
        self.assertTrue(self.state.hokuyo_write_completed)

    def test_work_id_calculation(self):
        """測試工作 ID 計算"""
        # 驗證動態工作 ID 計算
        # 修正：實際計算邏輯是 str(room_id) + "00" + ENTRANCE/EXIT + PUT/TAKE
        expected_entrance_work = 2000102  # room_id(2) + "00" + "01" + "02"
        expected_exit_work = 2000201      # room_id(2) + "00" + "02" + "01"

        self.assertEqual(self.state.entrance_work, expected_entrance_work)
        self.assertEqual(self.state.exit_work, expected_exit_work)

    def test_difference_from_complete_state(self):
        """測試與 CompleteState 的差異（write_complete 參數）"""
        # 執行參數初始化
        self.state._initialize_hokuyo_parameters()

        # 驗證 write_complete 被設定為 "0"（與 CompleteState 的 "1" 不同）
        self.mock_hokuyo_1.write_complete.assert_called_once_with("0")
        self.mock_hokuyo_2.write_complete.assert_called_once_with("0")

    def test_handle_execution_order_hokuyo_not_completed(self):
        """測試 handle 方法執行順序 - Hokuyo 尚未完成時"""
        # 確保 Hokuyo 尚未完成
        self.state.hokuyo_write_completed = False

        # 調用 handle 方法
        self.state.handle(self.mock_context)

        # 驗證 Hokuyo 初始化被執行
        self.mock_hokuyo_1.write_valid.assert_called_once_with("0")
        self.mock_hokuyo_2.write_valid.assert_called_once_with("0")

        # 修正：即使 Hokuyo 尚未完成，_initialize_hokuyo_parameters() 也會將 hokuyo_write_completed 設為 True
        # 因此狀態切換會被執行
        self.mock_context.set_state.assert_called_once()

    def test_handle_execution_order_hokuyo_completed(self):
        """測試 handle 方法執行順序 - Hokuyo 已完成時"""
        # 設置 Hokuyo 已完成
        self.state.hokuyo_write_completed = True

        # 調用 handle 方法
        self.state.handle(self.mock_context)

        # 驗證 work_id 檢查和狀態切換被執行
        self.mock_context.set_state.assert_called_once()

    def test_handle_execution_order_two_calls(self):
        """測試 handle 方法兩次調用的執行順序"""
        # 修正測試邏輯：模擬 Hokuyo 初始化失敗的情況
        # 讓第一次調用不會完成 Hokuyo 初始化

        # 模擬 Hokuyo 初始化會拋出異常
        self.mock_hokuyo_1.write_valid.side_effect = Exception("模擬初始化失敗")

        # 第一次調用 - Hokuyo 初始化失敗但會被標記為完成
        self.state.hokuyo_write_completed = False
        self.state.handle(self.mock_context)

        # 驗證第一次調用時，即使失敗也會觸發狀態切換（因為 hokuyo_write_completed 被設為 True）
        self.mock_context.set_state.assert_called_once()

        # 重置 mock
        self.mock_context.reset_mock()

        # 移除異常模擬
        self.mock_hokuyo_1.write_valid.side_effect = None

        # 第二次調用 - Hokuyo 已完成
        self.assertTrue(self.state.hokuyo_write_completed)
        self.state.handle(self.mock_context)

        # 驗證第二次調用時狀態切換也被執行
        self.mock_context.set_state.assert_called_once()

    def tearDown(self):
        """清理測試環境"""
        pass


if __name__ == '__main__':
    unittest.main()
