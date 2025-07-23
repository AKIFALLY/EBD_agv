"""
OPUI 基礎測試
只測試不依賴真實資料庫的基本功能
"""

import pytest
from unittest.mock import Mock, AsyncMock, patch

from opui.op_ui_server import OpUiServer
from opui.op_ui_socket import OpUiSocket


class TestBasicServer:
    """測試基本伺服器功能"""

    def test_server_creation(self):
        """測試伺服器創建"""
        server = OpUiServer()
        assert server is not None
        assert server.app is not None
        assert server.sio is not None

    def test_server_with_custom_config(self):
        """測試自定義配置的伺服器"""
        server = OpUiServer(host="127.0.0.1", port=8080)
        assert server.host == "127.0.0.1"
        assert server.port == 8080


class TestBasicSocket:
    """測試基本 Socket 功能"""

    def test_socket_creation(self):
        """測試 Socket 創建"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)
        assert socket_handler is not None
        assert socket_handler.sio == mock_sio

    @pytest.mark.asyncio
    async def test_basic_connect(self):
        """測試基本連線功能"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)

        # 測試連線
        await socket_handler.connect("test_sid", {})
        # 連線成功不應該拋出異常

    @pytest.mark.asyncio
    async def test_basic_disconnect(self):
        """測試基本斷線功能"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)

        # 設定一個使用者
        socket_handler.user_sid_map[1] = "test_sid"

        # 測試斷線
        await socket_handler.disconnect("test_sid")

        # 驗證使用者被移除
        assert 1 not in socket_handler.user_sid_map

    @pytest.mark.asyncio
    async def test_notify_message(self):
        """測試通知訊息功能"""
        mock_sio = AsyncMock()

        # 使用 patch 避免初始化問題
        with patch.object(OpUiSocket, 'init_socketio'):
            socket_handler = OpUiSocket(mock_sio)

            await socket_handler.notify_message("test_sid", "測試訊息")

            # 驗證 emit 被調用
            mock_sio.emit.assert_called_once_with(
                "notify_message",
                {"message": "測試訊息"},
                room="test_sid"
            )

    @pytest.mark.asyncio
    async def test_error_message(self):
        """測試錯誤訊息功能"""
        mock_sio = AsyncMock()

        # 使用 patch 避免初始化問題
        with patch.object(OpUiSocket, 'init_socketio'):
            socket_handler = OpUiSocket(mock_sio)

            await socket_handler.error_message("test_sid", "錯誤訊息")

            # 驗證 emit 被調用
            mock_sio.emit.assert_called_once_with(
                "error_message",
                {"message": "錯誤訊息"},
                room="test_sid"
            )


class TestBasicIntegration:
    """測試基本整合功能"""

    def test_server_socket_integration(self):
        """測試伺服器和 Socket 整合"""
        server = OpUiServer()

        # 驗證 Socket 實例被正確創建
        assert server.op_ui_socket is not None
        assert isinstance(server.op_ui_socket, OpUiSocket)
        assert server.op_ui_socket.sio == server.sio

    @patch('opui.op_ui_server.uvicorn.run')
    def test_server_run_mock(self, mock_uvicorn_run):
        """測試伺服器運行（模擬）"""
        server = OpUiServer()
        server.run()

        # 驗證 uvicorn.run 被調用
        mock_uvicorn_run.assert_called_once()


class TestBasicUtilities:
    """測試基本工具函數"""

    def test_user_session_management(self):
        """測試使用者會話管理"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)

        # 測試新增使用者
        socket_handler.user_sid_map[1] = "sid_1"
        socket_handler.user_sid_map[2] = "sid_2"

        assert len(socket_handler.user_sid_map) == 2
        assert socket_handler.user_sid_map[1] == "sid_1"
        assert socket_handler.user_sid_map[2] == "sid_2"

        # 測試移除使用者
        del socket_handler.user_sid_map[1]
        assert len(socket_handler.user_sid_map) == 1
        assert 1 not in socket_handler.user_sid_map
        assert 2 in socket_handler.user_sid_map

    def test_multiple_users_management(self):
        """測試多使用者管理"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)

        # 新增多個使用者
        for i in range(10):
            socket_handler.user_sid_map[i] = f"sid_{i}"

        assert len(socket_handler.user_sid_map) == 10

        # 移除部分使用者
        for i in range(0, 10, 2):  # 移除偶數編號
            del socket_handler.user_sid_map[i]

        assert len(socket_handler.user_sid_map) == 5

        # 驗證剩餘的是奇數編號
        for i in range(1, 10, 2):
            assert i in socket_handler.user_sid_map


class TestBasicErrorHandling:
    """測試基本錯誤處理"""

    @pytest.mark.asyncio
    async def test_disconnect_nonexistent_user(self):
        """測試斷線不存在的使用者"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)

        # 嘗試斷線不存在的使用者，不應該拋出異常
        await socket_handler.disconnect("nonexistent_sid")

        # 驗證沒有異常被拋出
        assert True

    @pytest.mark.asyncio
    async def test_notify_with_mock_failure(self):
        """測試通知時的模擬失敗"""
        mock_sio = AsyncMock()
        mock_sio.emit.side_effect = Exception("模擬錯誤")

        # 使用 patch 避免初始化問題
        with patch.object(OpUiSocket, 'init_socketio'):
            socket_handler = OpUiSocket(mock_sio)

            # 這個測試主要確保我們的測試框架能正確處理異常
            with pytest.raises(Exception, match="模擬錯誤"):
                await socket_handler.notify_message("test_sid", "測試")


class TestBasicPerformance:
    """測試基本效能"""

    def test_large_user_session_creation(self):
        """測試大量使用者會話創建"""
        mock_sio = Mock()
        socket_handler = OpUiSocket(mock_sio)

        # 創建大量使用者會話
        import time
        start_time = time.time()

        for i in range(1000):
            socket_handler.user_sid_map[i] = f"sid_{i}"

        end_time = time.time()
        duration = end_time - start_time

        # 1000 個會話創建應該很快
        assert duration < 1.0, f"會話創建時間過長: {duration:.2f}秒"
        assert len(socket_handler.user_sid_map) == 1000

    @pytest.mark.asyncio
    async def test_multiple_async_operations(self):
        """測試多個異步操作"""
        import asyncio
        import time

        mock_sio = AsyncMock()

        # 使用 patch 避免初始化問題
        with patch.object(OpUiSocket, 'init_socketio'):
            socket_handler = OpUiSocket(mock_sio)

            # 創建多個異步任務
            tasks = []
            for i in range(50):
                task = socket_handler.notify_message(f"sid_{i}", f"訊息_{i}")
                tasks.append(task)

            # 並發執行
            start_time = time.time()
            await asyncio.gather(*tasks)
            end_time = time.time()

            duration = end_time - start_time

            # 50 個並發通知應該很快
            assert duration < 1.0, f"並發操作時間過長: {duration:.2f}秒"

            # 驗證所有 emit 都被調用
            assert mock_sio.emit.call_count == 50
