#!/usr/bin/env python3
"""
簡單測試腳本，驗證AGV任務完成流程的核心功能
"""

import asyncio
from unittest.mock import Mock, AsyncMock
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


def test_parking_status_messages():
    """測試停車格狀態訊息生成"""
    print("🧪 測試停車格狀態訊息生成...")

    # 模擬OpUiSocket
    from opui.op_ui_socket import OpUiSocket
    mock_sio = Mock()
    socket = OpUiSocket(mock_sio)

    # 測試已叫車狀態
    msg = socket._get_parking_status_message(1, 101)
    assert "已叫車" in msg and "101" in msg
    print(f"✅ 已叫車狀態訊息: {msg}")

    # 測試已送達狀態
    msg = socket._get_parking_status_message(2, 102)
    assert "已送達" in msg and "102" in msg
    print(f"✅ 已送達狀態訊息: {msg}")

    # 測試異常狀態
    msg = socket._get_parking_status_message(99, 103)
    assert "狀態異常" in msg and "103" in msg
    print(f"✅ 異常狀態訊息: {msg}")

    print("✅ 停車格狀態訊息測試通過\n")


def test_parking_status_check():
    """測試停車格狀態檢查邏輯"""
    print("🧪 測試停車格狀態檢查邏輯...")

    from opui.op_ui_socket import OpUiSocket
    mock_sio = Mock()
    socket = OpUiSocket(mock_sio)

    # 模擬資料庫操作
    from unittest.mock import patch

    with patch('opui.db.machine_crud') as mock_machine_crud, \
            patch('opui.db.connection_pool') as mock_pool:

        # 模擬機台資料
        from opui.constants.parking_status import ParkingStatus

        mock_machine = Mock()
        mock_machine.parking_space_1 = 101
        mock_machine.parking_space_1_status = ParkingStatus.AVAILABLE  # 可用
        mock_machine.parking_space_2 = 102
        mock_machine.parking_space_2_status = ParkingStatus.TASK_ACTIVE  # 任務進行中

        mock_session = Mock()
        mock_pool.get_session.return_value.__enter__.return_value = mock_session
        mock_machine_crud.get_by_id.return_value = mock_machine

        # 測試可用狀態 - 應該可以叫車
        ok, msg = socket._check_parking_space_status(1, 101)
        assert ok is True
        assert msg is None
        print("✅ 可用狀態檢查通過")

        # 測試任務進行中狀態 - 不能再叫車
        ok, msg = socket._check_parking_space_status(1, 102)
        assert ok is False
        assert "已叫車" in msg
        print(f"✅ 任務進行中狀態檢查通過: {msg}")

        # 測試已送達狀態
        mock_machine.parking_space_2_status = 2  # 已送達
        ok, msg = socket._check_parking_space_status(1, 102)
        assert ok is False
        assert "已送達" in msg
        print(f"✅ 已送達狀態檢查通過: {msg}")

    print("✅ 停車格狀態檢查測試通過\n")


async def test_confirm_delivery():
    """測試確認送達功能"""
    print("🧪 測試確認送達功能...")

    from opui.op_ui_socket import OpUiSocket
    mock_sio = Mock()
    mock_sio.emit = AsyncMock()
    socket = OpUiSocket(mock_sio)
    socket.user_sid_map = {1: "test_sid_123"}

    # 模擬必要的方法
    socket._require_client_and_machine = Mock(return_value=(1, 1, None))
    socket._update_machine_parking_status = Mock()
    socket.notify_machines = AsyncMock()
    socket.notify_parking_list = AsyncMock()
    socket.notify_message = AsyncMock()

    from unittest.mock import patch

    with patch('opui.db.machine_crud') as mock_machine_crud, \
            patch('opui.db.connection_pool') as mock_pool:

        # 模擬機台資料 - 停車格狀態為已送達(2)
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
        result = await socket.confirm_delivery("test_sid_123", data)

        # 驗證結果
        assert result["success"] is True
        assert "已確認" in result["message"]
        print(f"✅ 確認送達成功: {result['message']}")

        # 驗證狀態更新被調用
        socket._update_machine_parking_status.assert_called_once_with(1, 101, 0)
        print("✅ 停車格狀態更新調用正確")

        # 驗證通知被發送
        socket.notify_machines.assert_called_once()
        socket.notify_parking_list.assert_called_once()
        socket.notify_message.assert_called_once()
        print("✅ 通知發送調用正確")

    print("✅ 確認送達功能測試通過\n")


async def test_confirm_delivery_wrong_status():
    """測試確認送達但狀態不正確的情況"""
    print("🧪 測試確認送達狀態不正確的情況...")

    from opui.op_ui_socket import OpUiSocket
    mock_sio = Mock()
    socket = OpUiSocket(mock_sio)

    socket._require_client_and_machine = Mock(return_value=(1, 1, None))

    from unittest.mock import patch

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
        result = await socket.confirm_delivery("test_sid_123", data)

        # 驗證結果
        assert result["success"] is False
        assert "狀態不正確" in result["message"]
        print(f"✅ 狀態不正確檢查通過: {result['message']}")

    print("✅ 確認送達狀態檢查測試通過\n")


def test_task_monitoring_functions():
    """測試任務監聽功能"""
    print("🧪 測試任務監聽功能...")

    from opui.op_ui_socket import OpUiSocket
    mock_sio = Mock()
    socket = OpUiSocket(mock_sio)

    # 測試新增任務監聽
    socket.add_task_monitoring(123, 1, 101, 0)
    assert 123 in socket.monitored_tasks
    assert socket.monitored_tasks[123]['machine_id'] == 1
    assert socket.monitored_tasks[123]['node_id'] == 101
    assert socket.monitored_tasks[123]['previous_status'] == 0
    print("✅ 新增任務監聽功能通過")

    # 測試任務參數解析
    mock_task = Mock()
    mock_task.parameters = '{"task_type": "call_empty", "machine_id": 1, "node_id": 101}'
    mock_task.node_id = 101

    machine_id, node_id = socket._extract_task_info(mock_task)
    assert machine_id == 1
    assert node_id == 101
    print("✅ 任務參數解析功能通過")

    # 測試停車格狀態獲取
    from unittest.mock import patch
    with patch('opui.db.machine_crud') as mock_machine_crud, \
            patch('opui.db.connection_pool') as mock_pool:

        mock_machine = Mock()
        mock_machine.parking_space_1 = 101
        mock_machine.parking_space_1_status = 2
        mock_machine.parking_space_2 = 102
        mock_machine.parking_space_2_status = 0

        mock_session = Mock()
        mock_pool.get_session.return_value.__enter__.return_value = mock_session
        mock_machine_crud.get_by_id.return_value = mock_machine

        status = socket._get_current_parking_status(1, 101)
        assert status == 2
        print("✅ 停車格狀態獲取功能通過")

    print("✅ 任務監聽功能測試通過\n")


async def test_task_monitoring_loop():
    """測試任務監聽循環"""
    print("🧪 測試任務監聽循環...")

    from opui.op_ui_socket import OpUiSocket
    mock_sio = Mock()
    mock_sio.emit = AsyncMock()
    socket = OpUiSocket(mock_sio)

    # 模擬一個監聽中的任務
    socket.monitored_tasks[123] = {
        'machine_id': 1,
        'node_id': 101,
        'previous_status': 2,  # 執行中
        'created_at': 0
    }

    from unittest.mock import patch
    with patch('opui.db.task_crud') as mock_task_crud, \
            patch('opui.db.connection_pool') as mock_pool:

        # 模擬任務已完成
        mock_task = Mock()
        mock_task.id = 123
        mock_task.status_id = 3  # 完成
        mock_task.node_id = 101

        mock_session = Mock()
        mock_pool.get_session.return_value.__enter__.return_value = mock_session
        mock_task_crud.get_by_id.return_value = mock_task

        # 模擬任務完成處理方法
        socket._handle_task_completion = AsyncMock()

        # 執行檢查
        await socket._check_monitored_tasks()

        # 驗證任務完成處理被調用
        socket._handle_task_completion.assert_called_once()

        # 驗證任務從監聽列表中移除
        assert 123 not in socket.monitored_tasks

        print("✅ 任務狀態變更檢測通過")

    print("✅ 任務監聽循環測試通過\n")


async def main():
    """主測試函數"""
    print("🚀 開始測試AGV任務完成流程...\n")

    try:
        # 運行所有測試
        test_parking_status_messages()
        test_parking_status_check()
        await test_confirm_delivery()
        await test_confirm_delivery_wrong_status()
        test_task_monitoring_functions()
        await test_task_monitoring_loop()

        print("🎉 所有測試通過！")
        print("\n📋 測試摘要:")
        print("✅ 停車格狀態訊息生成")
        print("✅ 停車格狀態檢查邏輯")
        print("✅ 確認送達功能")
        print("✅ 確認送達狀態檢查")
        print("✅ 任務監聽功能")
        print("✅ 任務監聽循環")
        print("\n🔧 實現的功能:")
        print("1. 停車格狀態從二元擴展為三元 (0:未佔用, 1:已叫車, 2:已送達待確認)")
        print("2. 新增確認送達API和WebSocket事件")
        print("3. 前端按鈕支援三種狀態顯示")
        print("4. 資料庫任務監聽機制 (每秒查詢)")
        print("5. 客戶端連線時自動同步任務狀態")

        return True

    except Exception as e:
        print(f"❌ 測試失敗: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
