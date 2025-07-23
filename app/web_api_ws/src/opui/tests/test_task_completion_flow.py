"""
測試AGV任務完成後的操作員確認流程
"""

import pytest
from unittest.mock import Mock, AsyncMock, patch
import asyncio
from opui.op_ui_socket import OpUiSocket
from opui.task_monitor import TaskMonitor


class TestTaskCompletionFlow:
    """測試任務完成流程"""

    @pytest.fixture
    def mock_sio(self):
        """模擬Socket.IO服務器"""
        sio = Mock()
        sio.emit = AsyncMock()
        return sio

    @pytest.fixture
    def op_ui_socket(self, mock_sio):
        """創建OpUiSocket實例"""
        socket = OpUiSocket(mock_sio)
        socket.user_sid_map = {1: "test_sid_123"}
        return socket

    @pytest.fixture
    def mock_loop(self):
        """模擬事件循環"""
        return asyncio.new_event_loop()

    @pytest.fixture
    def task_monitor(self, mock_loop, op_ui_socket):
        """創建TaskMonitor實例"""
        with patch('rclpy.init'), \
             patch('opui.task_monitor.Node') as mock_node_class:
            
            mock_node = Mock()
            mock_node.get_logger.return_value = Mock()
            mock_node.create_subscription = Mock()
            mock_node_class.return_value = mock_node
            
            monitor = TaskMonitor(mock_loop, op_ui_socket)
            monitor.node = mock_node
            return monitor

    def test_parking_status_message_generation(self, op_ui_socket):
        """測試停車格狀態訊息生成"""
        # 測試已叫車狀態
        msg = op_ui_socket._get_parking_status_message(1, 101)
        assert "已叫車" in msg
        assert "101" in msg

        # 測試已送達狀態
        msg = op_ui_socket._get_parking_status_message(2, 102)
        assert "已送達" in msg
        assert "102" in msg

        # 測試異常狀態
        msg = op_ui_socket._get_parking_status_message(99, 103)
        assert "狀態異常" in msg
        assert "103" in msg

    @pytest.mark.asyncio
    async def test_confirm_delivery_success(self, op_ui_socket):
        """測試確認送達成功"""
        sid = "test_sid_123"
        
        # 模擬必要的方法
        op_ui_socket._require_client_and_machine = Mock(return_value=(1, 1, None))
        op_ui_socket._update_machine_parking_status = Mock()
        op_ui_socket.notify_machines = AsyncMock()
        op_ui_socket.notify_parking_list = AsyncMock()
        op_ui_socket.notify_message = AsyncMock()

        # 模擬資料庫操作
        with patch('opui.db.machine_crud') as mock_machine_crud, \
             patch('opui.db.connection_pool') as mock_pool:
            
            # 模擬機台資料
            mock_machine = Mock()
            mock_machine.parking_space_1 = 101
            mock_machine.parking_space_1_status = 2  # 已送達狀態
            mock_machine.parking_space_2 = 102
            mock_machine.parking_space_2_status = 0
            
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session
            mock_machine_crud.get_by_id.return_value = mock_machine

            # 執行確認送達
            data = {"parkingSpace": "101"}
            result = await op_ui_socket.confirm_delivery(sid, data)

            # 驗證結果
            assert result["success"] is True
            assert "已確認" in result["message"]

            # 驗證狀態更新被調用
            op_ui_socket._update_machine_parking_status.assert_called_once_with(1, 101, 0)

            # 驗證通知被發送
            op_ui_socket.notify_machines.assert_called_once_with(sid)
            op_ui_socket.notify_parking_list.assert_called_once_with(sid)
            op_ui_socket.notify_message.assert_called_once()

    @pytest.mark.asyncio
    async def test_confirm_delivery_wrong_status(self, op_ui_socket):
        """測試確認送達但狀態不正確"""
        sid = "test_sid_123"
        
        op_ui_socket._require_client_and_machine = Mock(return_value=(1, 1, None))

        # 模擬資料庫操作
        with patch('opui.db.machine_crud') as mock_machine_crud, \
             patch('opui.db.connection_pool') as mock_pool:
            
            # 模擬機台資料 - 狀態不是已送達(2)
            mock_machine = Mock()
            mock_machine.parking_space_1 = 101
            mock_machine.parking_space_1_status = 1  # 已叫車狀態，不是已送達
            
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session
            mock_machine_crud.get_by_id.return_value = mock_machine

            # 執行確認送達
            data = {"parkingSpace": "101"}
            result = await op_ui_socket.confirm_delivery(sid, data)

            # 驗證結果
            assert result["success"] is False
            assert "狀態不正確" in result["message"]

    def test_is_opui_task_with_parameters(self, task_monitor):
        """測試OPUI任務識別 - 通過參數"""
        # 模擬任務對象
        mock_task = Mock()
        mock_task.parameters = '{"task_type": "call_empty", "node_id": 101}'
        mock_task.work_id = 1

        result = task_monitor._is_opui_task(mock_task)
        assert result is True

        # 測試非OPUI任務
        mock_task.parameters = '{"task_type": "other_task", "node_id": 101}'
        result = task_monitor._is_opui_task(mock_task)
        assert result is False

    def test_is_opui_task_with_work_type(self, task_monitor):
        """測試OPUI任務識別 - 通過工作類型"""
        mock_task = Mock()
        mock_task.parameters = None
        mock_task.work_id = 1

        # 模擬工作類型檢查
        with patch.object(task_monitor, '_check_work_type', return_value=True):
            result = task_monitor._is_opui_task(mock_task)
            assert result is True

        with patch.object(task_monitor, '_check_work_type', return_value=False):
            result = task_monitor._is_opui_task(mock_task)
            assert result is False

    @pytest.mark.asyncio
    async def test_handle_task_completion(self, task_monitor):
        """測試任務完成處理"""
        # 模擬任務對象
        mock_task = Mock()
        mock_task.id = 123
        mock_task.node_id = 101

        # 模擬相關方法
        task_monitor._find_machine_by_node_id = AsyncMock(return_value=1)
        task_monitor._update_parking_status_to_delivered = AsyncMock()
        task_monitor._notify_all_clients_for_machine = AsyncMock()

        # 執行任務完成處理
        await task_monitor._handle_task_completion(mock_task)

        # 驗證方法被調用
        task_monitor._find_machine_by_node_id.assert_called_once_with(101)
        task_monitor._update_parking_status_to_delivered.assert_called_once_with(1, 101)
        task_monitor._notify_all_clients_for_machine.assert_called_once_with(1)

    @pytest.mark.asyncio
    async def test_find_machine_by_node_id(self, task_monitor):
        """測試根據node_id查找機台"""
        with patch('opui.db.machine_crud') as mock_machine_crud, \
             patch('opui.db.connection_pool') as mock_pool:
            
            # 模擬機台資料
            mock_machine1 = Mock()
            mock_machine1.id = 1
            mock_machine1.parking_space_1 = 101
            mock_machine1.parking_space_2 = 102
            
            mock_machine2 = Mock()
            mock_machine2.id = 2
            mock_machine2.parking_space_1 = 103
            mock_machine2.parking_space_2 = 104
            
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session
            mock_machine_crud.get_all.return_value = [mock_machine1, mock_machine2]

            # 測試找到機台
            result = await task_monitor._find_machine_by_node_id(101)
            assert result == 1

            result = await task_monitor._find_machine_by_node_id(104)
            assert result == 2

            # 測試找不到機台
            result = await task_monitor._find_machine_by_node_id(999)
            assert result is None

    def test_check_task_completion_new_task(self, task_monitor):
        """測試新任務的監控"""
        # 模擬任務對象
        mock_task = Mock()
        mock_task.id = 123
        mock_task.status_id = 1  # 已選擇
        mock_task.node_id = 101
        mock_task.work_id = 1

        # 模擬OPUI任務
        with patch.object(task_monitor, '_is_opui_task', return_value=True):
            task_monitor._check_task_completion(mock_task)

            # 驗證任務被加入監控
            assert 123 in task_monitor.monitored_tasks
            assert task_monitor.monitored_tasks[123]['previous_status'] == 1

    def test_check_task_completion_status_change(self, task_monitor):
        """測試任務狀態變更"""
        # 先添加一個監控中的任務
        task_monitor.monitored_tasks[123] = {
            'task': Mock(),
            'previous_status': 2,  # 執行中
            'node_id': 101,
            'work_id': 1
        }

        # 模擬任務完成
        mock_task = Mock()
        mock_task.id = 123
        mock_task.status_id = 3  # 完成
        mock_task.node_id = 101
        mock_task.work_id = 1

        with patch.object(task_monitor, '_is_opui_task', return_value=True), \
             patch('asyncio.run_coroutine_threadsafe') as mock_run_coroutine:
            
            task_monitor._check_task_completion(mock_task)

            # 驗證任務完成處理被觸發
            mock_run_coroutine.assert_called_once()
            
            # 驗證任務被標記為已完成
            assert 123 in task_monitor.completed_tasks
            
            # 驗證狀態被更新
            assert task_monitor.monitored_tasks[123]['previous_status'] == 3


if __name__ == "__main__":
    pytest.main([__file__])
