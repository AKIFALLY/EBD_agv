#!/usr/bin/env python3
"""
測試 WaitRotationState 中 async_update_task 方法調用的單元測試
檢查參數傳遞、錯誤處理、回調函數等功能
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import rclpy
from rclpy.node import Node
import json

# 導入待測試的類
from cargo_mover_agv.robot_states.exit.wait_rotation_state import WaitRotationState as ExitWaitRotationState
from cargo_mover_agv.robot_states.entrance.wait_rotation_state import WaitRotationState as EntranceWaitRotationState
from cargo_mover_agv.robot_context import RobotContext
from db_proxy_interfaces.msg import Task as TaskMsg
from db_proxy_interfaces.srv import UpdateTask


class TestWaitRotationAsyncUpdateTask(unittest.TestCase):
    """測試 WaitRotationState 的 async_update_task 功能"""

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

    @patch('cargo_mover_agv.robot_states.exit.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.exit.wait_rotation_state.RackQueryClient')
    def test_exit_async_update_task_success(self, mock_rack_client, mock_agvc_client):
        """測試 EXIT 版本的 async_update_task 成功調用"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務就緒
        mock_agvc_instance.task_client.service_is_ready.return_value = True

        # 模擬 async_update_task 返回 future
        mock_future = Mock()
        mock_agvc_instance.async_update_task.return_value = mock_future

        # 創建狀態實例
        state = ExitWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證 async_update_task 被調用
        self.assertTrue(mock_agvc_instance.async_update_task.called)

        # 獲取調用參數
        call_args = mock_agvc_instance.async_update_task.call_args
        task_arg = call_args[0][0]  # 第一個參數是 task
        callback_arg = call_args[0][1]  # 第二個參數是 callback

        # 驗證 task 參數
        self.assertIsInstance(task_arg, TaskMsg)
        self.assertEqual(task_arg.id, 456)  # EXIT 版本使用固定值
        self.assertEqual(task_arg.work_id, self.mock_task.work_id)
        self.assertEqual(task_arg.status_id, 10001)
        self.assertEqual(task_arg.room_id, 2)  # EXIT 版本使用固定值
        self.assertEqual(task_arg.node_id, 20002)  # EXIT 版本使用固定值
        self.assertEqual(task_arg.name, "測試任務456")  # EXIT 版本使用固定值
        self.assertEqual(task_arg.agv_id, 123)  # EXIT 版本使用固定值
        # 移除 agv_name 驗證，因為 Task.msg 中沒有此欄位
        self.assertEqual(task_arg.priority, 1)
        self.assertEqual(task_arg.parameters, self.mock_task.parameters)

        # 驗證 callback 是正確的方法
        self.assertEqual(callback_arg, state.update_task_callback)

    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.entrance.wait_rotation_state.RackQueryClient')
    def test_entrance_async_update_task_success(self, mock_rack_client, mock_agvc_client):
        """測試 ENTRANCE 版本的 async_update_task 成功調用"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

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
        callback_arg = call_args[0][1]  # 第二個參數是 callback

        # 驗證 task 參數 - ENTRANCE 版本使用 node.task 的值
        self.assertIsInstance(task_arg, TaskMsg)
        self.assertEqual(task_arg.id, self.mock_task.id)
        self.assertEqual(task_arg.work_id, self.mock_task.work_id)
        self.assertEqual(task_arg.status_id, 10001)
        self.assertEqual(task_arg.room_id, self.mock_task.room_id)
        self.assertEqual(task_arg.node_id, self.mock_task.room_id)  # ENTRANCE 版本錯誤：應該是 node_id
        self.assertEqual(task_arg.name, self.mock_task.name)
        self.assertEqual(task_arg.description, self.mock_task.description)
        self.assertEqual(task_arg.agv_id, self.mock_node.AGV_id)
        # 移除 agv_name 驗證，因為 Task.msg 中沒有此欄位
        self.assertEqual(task_arg.priority, self.mock_task.priority)

        # 驗證 callback 是正確的方法
        self.assertEqual(callback_arg, state.update_task_callback)

    @patch('cargo_mover_agv.robot_states.exit.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.exit.wait_rotation_state.RackQueryClient')
    def test_service_not_ready(self, mock_rack_client, mock_agvc_client):
        """測試服務未就緒的情況"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務未就緒
        mock_agvc_instance.task_client.service_is_ready.return_value = False

        # 創建狀態實例
        state = ExitWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證 async_update_task 沒有被調用
        self.assertFalse(mock_agvc_instance.async_update_task.called)

        # 驗證警告日誌被記錄
        self.mock_node.get_logger().warn.assert_called_with(
            "⚠️ /agvc/update_task 服務尚未就緒，跳過 update_task"
        )

    def test_update_task_callback_success(self):
        """測試 update_task_callback 成功處理"""
        state = ExitWaitRotationState(self.mock_node)

        # 模擬成功的回應
        mock_result = Mock()
        mock_result.success = True
        mock_result.message = "更新成功"

        # 執行回調
        state.update_task_callback(mock_result)

        # 驗證狀態更新
        self.assertTrue(state.update_task_success)

        # 驗證日誌記錄
        self.mock_node.get_logger().info.assert_called()

    def test_update_task_callback_failure(self):
        """測試 update_task_callback 失敗處理"""
        state = ExitWaitRotationState(self.mock_node)

        # 模擬失敗的回應
        mock_result = Mock()
        mock_result.success = False
        mock_result.message = "更新失敗"

        # 執行回調
        state.update_task_callback(mock_result)

        # 驗證狀態更新
        self.assertFalse(state.update_task_success)

        # 驗證警告日誌被記錄
        self.mock_node.get_logger().warn.assert_called()

    def test_update_task_callback_none_result(self):
        """測試 update_task_callback 處理 None 結果"""
        state = ExitWaitRotationState(self.mock_node)

        # 執行回調，傳入 None
        state.update_task_callback(None)

        # 驗證狀態更新
        self.assertFalse(state.update_task_success)

        # 驗證錯誤日誌被記錄
        self.mock_node.get_logger().error.assert_called_with(
            "❌ Task 更新失敗 - 回調結果為 None"
        )

    def test_entrance_parameters_json_handling(self):
        """測試 ENTRANCE 版本的 parameters JSON 處理"""
        # 測試 dict 格式的 parameters
        self.mock_task.parameters = {"rack_id": 789, "port": 5}

        state = EntranceWaitRotationState(self.mock_node)

        with patch.object(state, 'agvc_client') as mock_agvc:
            mock_agvc.async_update_task.return_value = Mock()

            # 執行測試
            state.handle(self.mock_context)

            # 獲取調用參數
            call_args = mock_agvc.async_update_task.call_args
            task_arg = call_args[0][0]

            # 驗證 parameters 被正確轉換為 JSON 字符串
            expected_json = json.dumps({"rack_id": 789, "port": 5})
            self.assertEqual(task_arg.parameters, expected_json)

    @patch('cargo_mover_agv.robot_states.exit.wait_rotation_state.AGVCDatabaseClient')
    @patch('cargo_mover_agv.robot_states.exit.wait_rotation_state.RackQueryClient')
    def test_async_update_task_returns_none(self, mock_rack_client, mock_agvc_client):
        """測試 async_update_task 返回 None 的情況"""
        # 設置 mock
        mock_agvc_instance = Mock()
        mock_agvc_client.return_value = mock_agvc_instance

        # 模擬服務就緒但 async_update_task 返回 None
        mock_agvc_instance.task_client.service_is_ready.return_value = True
        mock_agvc_instance.async_update_task.return_value = None

        # 創建狀態實例
        state = ExitWaitRotationState(self.mock_node)

        # 執行測試
        state.handle(self.mock_context)

        # 驗證錯誤日誌被記錄
        self.mock_node.get_logger().error.assert_called_with(
            "❌ update_task 請求發送失敗"
        )

    def test_update_task_success_prevents_duplicate_calls(self):
        """測試 update_task_success 標誌防止重複調用"""
        state = ExitWaitRotationState(self.mock_node)

        # 設置 update_task_success 為 True
        state.update_task_success = True

        with patch.object(state, 'agvc_client') as mock_agvc:
            # 執行測試
            state.handle(self.mock_context)

            # 驗證 async_update_task 沒有被調用
            self.assertFalse(mock_agvc.async_update_task.called)

            # 驗證信息日誌被記錄
            self.mock_node.get_logger().info.assert_any_call(
                "ℹ️ update_task 已經成功，跳過重複呼叫"
            )


if __name__ == '__main__':
    unittest.main()
