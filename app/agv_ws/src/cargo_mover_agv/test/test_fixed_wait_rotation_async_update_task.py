#!/usr/bin/env python3
"""
測試修正後的 WaitRotationState 中 async_update_task 方法調用
驗證修正是否成功解決了發現的問題
"""

import unittest
from unittest.mock import Mock, MagicMock, patch
import rclpy
from rclpy.node import Node
import json

# 導入待測試的類
from cargo_mover_agv.robot_states.entrance.wait_rotation_state import WaitRotationState as EntranceWaitRotationState
from cargo_mover_agv.robot_context import RobotContext
from db_proxy_interfaces.msg import Task as TaskMsg


class TestFixedWaitRotationAsyncUpdateTask(unittest.TestCase):
    """測試修正後的 WaitRotationState 的 async_update_task 功能"""

    def setUp(self):
        """測試前的設置"""
        rclpy.init()
        self.mock_node = Mock(spec=Node)
        self.mock_node.get_logger.return_value = Mock()

        # 模擬 task 物件
        self.mock_task = Mock()
        self.mock_task.id = 123
        self.mock_task.work_id = 456
        self.mock_task.room_id = 1
        self.mock_task.node_id = 2001  # 重要：確保 node_id 與 room_id 不同
        self.mock_task.name = "測試任務"
        self.mock_task.description = "測試描述"
        # 移除 agv_name，因為 Task.msg 中沒有此欄位
        self.mock_task.priority = 1
        self.mock_task.parameters = '{"rack_id": 789, "port": 5}'

        self.mock_node.task = self.mock_task
        self.mock_node.AGV_id = 123

        # 模擬 context
        self.mock_context = Mock(spec=RobotContext)
        self.mock_context.rack_id = 789
        self.mock_context.get_rack_port = 5

    def tearDown(self):
        """測試後的清理"""
        rclpy.shutdown()

    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.RackQueryClient')
    def test_fixed_node_id_assignment(self, mock_rack_client, mock_agvc_client):
        """測試修正後的 node_id 賦值是否正確"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務就緒
        mock_agvc_instance.task_client.service_is_ready.return_value = True

        # 模擬 async_update_task 返回 future
        mock_future = Mock()
        mock_agvc_instance.async_update_task.return_value = mock_future

        # 創建狀態實例
        state = EntranceWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證 async_update_task 被調用
        self.assertTrue(mock_agvc_instance.async_update_task.called)

        # 獲取調用參數
        call_args = mock_agvc_instance.async_update_task.call_args
        task_arg = call_args[0][0]  # 第一個參數是 task

        # 驗證 node_id 現在正確使用 task.node_id 而不是 task.room_id
        self.assertEqual(task_arg.node_id, self.mock_task.node_id)  # 應該是 2001
        self.assertNotEqual(task_arg.node_id, self.mock_task.room_id)  # 不應該是 1

        # 驗證其他字段仍然正確
        self.assertEqual(task_arg.room_id, self.mock_task.room_id)  # room_id 應該是 1

    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.RackQueryClient')
    def test_service_ready_check_added(self, mock_rack_client, mock_agvc_client):
        """測試是否添加了服務就緒檢查"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務未就緒
        mock_agvc_instance.task_client.service_is_ready.return_value = False

        # 創建狀態實例
        state = EntranceWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證服務就緒檢查被調用
        mock_agvc_instance.task_client.service_is_ready.assert_called()

        # 驗證 async_update_task 沒有被調用（因為服務未就緒）
        self.assertFalse(mock_agvc_instance.async_update_task.called)

        # 驗證警告日誌被記錄
        self.mock_node.get_logger().warn.assert_called_with(
            "⚠️ /agvc/update_task 服務尚未就緒，跳過 update_task"
        )

    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.RackQueryClient')
    def test_return_value_check_added(self, mock_rack_client, mock_agvc_client):
        """測試是否添加了返回值檢查"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務就緒但 async_update_task 返回 None
        mock_agvc_instance.task_client.service_is_ready.return_value = True
        mock_agvc_instance.async_update_task.return_value = None

        # 創建狀態實例
        state = EntranceWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證錯誤日誌被記錄
        self.mock_node.get_logger().error.assert_called_with(
            "❌ update_task 請求發送失敗"
        )

    def test_improved_callback_function(self):
        """測試改進後的回調函數是否正確處理 result.success"""
        state = EntranceWaitRotationState(self.mock_node)

        # 測試成功情況
        mock_result_success = Mock()
        mock_result_success.success = True
        mock_result_success.message = "更新成功"

        state.update_task_callback(mock_result_success)

        # 驗證狀態更新
        self.assertTrue(state.update_task_success)

        # 重置狀態
        state.update_task_success = False

        # 測試失敗情況
        mock_result_failure = Mock()
        mock_result_failure.success = False
        mock_result_failure.message = "更新失敗"

        state.update_task_callback(mock_result_failure)

        # 驗證狀態保持 False
        self.assertFalse(state.update_task_success)

        # 驗證警告日誌被記錄
        self.mock_node.get_logger().warn.assert_called_with(
            "⚠️ Task 更新回應失敗: 更新失敗"
        )

    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.RackQueryClient')
    def test_debug_logging_added(self, mock_rack_client, mock_agvc_client):
        """測試是否添加了調試日誌"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務就緒
        mock_agvc_instance.task_client.service_is_ready.return_value = True
        mock_agvc_instance.async_update_task.return_value = Mock()

        # 創建狀態實例
        state = EntranceWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證調試日誌被記錄
        expected_calls = [
            unittest.mock.call(f"🔍 檢查 update_task_success 狀態: {state.update_task_success}"),
            unittest.mock.call(
                f"🔍 檢查服務就緒狀態: {mock_agvc_instance.task_client.service_is_ready.return_value}"),
            unittest.mock.call("🚀 準備發送 update_task 請求"),
            unittest.mock.call("✅ 已發送 update_task 請求")
        ]

        # 檢查是否包含預期的調用
        for expected_call in expected_calls:
            self.assertIn(expected_call, self.mock_node.get_logger().info.call_args_list)

    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.RackQueryClient')
    def test_duplicate_call_prevention(self, mock_rack_client, mock_agvc_client):
        """測試重複調用防護是否正常工作"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 創建狀態實例
        state = EntranceWaitRotationState(self.mock_node)

        # 設置 update_task_success 為 True
        state.update_task_success = True

        # 執行測試
        state.handle(self.mock_context)

        # 驗證 async_update_task 沒有被調用
        self.assertFalse(mock_agvc_instance.async_update_task.called)

        # 驗證信息日誌被記錄
        self.mock_node.get_logger().info.assert_any_call(
            "ℹ️ update_task 已經成功，跳過重複呼叫"
        )

    def test_comprehensive_error_handling(self):
        """測試全面的錯誤處理"""
        state = EntranceWaitRotationState(self.mock_node)

        # 測試 None 結果的處理
        state.update_task_callback(None)

        # 驗證狀態更新
        self.assertFalse(state.update_task_success)

        # 驗證錯誤日誌被記錄
        self.mock_node.get_logger().error.assert_called_with(
            "❌ Task 更新失敗 - 回調結果為 None"
        )


if __name__ == '__main__':
    unittest.main()
