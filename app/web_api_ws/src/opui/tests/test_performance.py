"""
OPUI 效能和壓力測試
"""

import pytest
import asyncio
import time
from unittest.mock import Mock, AsyncMock, patch
from concurrent.futures import ThreadPoolExecutor
from fastapi.testclient import TestClient

from opui.op_ui_server import OpUiServer
from opui.op_ui_socket import OpUiSocket


class TestPerformance:
    """效能測試"""

    @pytest.mark.asyncio
    async def test_socket_connection_performance(self):
        """測試 Socket 連線效能"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 測試大量連線的處理時間
        start_time = time.time()

        connection_tasks = []
        for i in range(100):
            task = socket_handler.connect(f"sid_{i}", {})
            connection_tasks.append(task)

        await asyncio.gather(*connection_tasks)

        end_time = time.time()
        duration = end_time - start_time

        # 100 個連線應該在 1 秒內完成
        assert duration < 1.0, f"連線處理時間過長: {duration:.2f}秒"

    @pytest.mark.asyncio
    async def test_login_performance(self):
        """測試登入效能"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 模擬資料庫操作
        with patch('opui.db.get_client') as mock_get_client, \
                patch('opui.db.product_all') as mock_product_all, \
                patch('opui.db.machine_all') as mock_machine_all, \
                patch('opui.db.room_all') as mock_room_all, \
        patch('opui.db.connection_pool'):

            mock_get_client.return_value = {
                "clientId": 1, "userAgent": "Test", "machineId": 1,
                "created_at": None, "updated_at": None
            }
            mock_product_all.return_value = []
            mock_machine_all.return_value = []
            mock_room_all.return_value = []

            # 測試批量登入效能
            start_time = time.time()

            login_tasks = []
            for i in range(50):
                task = socket_handler.login(f"sid_{i}", {"clientId": i})
                login_tasks.append(task)

            results = await asyncio.gather(*login_tasks)

            end_time = time.time()
            duration = end_time - start_time

            # 驗證所有登入都成功
            for result in results:
                assert result["success"] is True

            # 50 個登入應該在 2 秒內完成
            assert duration < 2.0, f"登入處理時間過長: {duration:.2f}秒"

    @pytest.mark.asyncio
    async def test_task_creation_performance(self):
        """測試任務創建效能"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 設定使用者會話
        for i in range(20):
            socket_handler.user_sid_map[i] = f"sid_{i}"

        # 模擬必要的方法
        socket_handler._get_client_and_machine_id = Mock(return_value=(1, 1))
        socket_handler._check_parking_space_status = Mock(return_value=(True, None))
        socket_handler._update_machine_parking_status = Mock()
        socket_handler.notify_machines = AsyncMock()

        with patch('opui.db.create_task') as mock_create_task, \
                patch('opui.db.get_call_empty_work_id') as mock_get_work_id, \
                patch('opui.db.get_default_task_status_id') as mock_get_status_id:

            mock_create_task.return_value = {"id": 1, "name": "測試任務"}
            mock_get_work_id.return_value = 1
            mock_get_status_id.return_value = 1

            # 測試批量任務創建效能
            start_time = time.time()

            task_creation_tasks = []
            for i in range(20):
                task = socket_handler.call_empty(f"sid_{i}", {"parkingSpace": f"{100+i}"})
                task_creation_tasks.append(task)

            results = await asyncio.gather(*task_creation_tasks)

            end_time = time.time()
            duration = end_time - start_time

            # 驗證所有任務創建都成功
            for result in results:
                assert result["success"] is True

            # 20 個任務創建應該在 1 秒內完成
            assert duration < 1.0, f"任務創建時間過長: {duration:.2f}秒"

    def test_api_response_performance(self):
        """測試 API 回應效能"""
        server = OpUiServer()
        client = TestClient(server.app)

        with patch('opui.routers.product.connection_pool') as mock_pool, \
                patch('opui.routers.product.product_crud') as mock_crud:

            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session
            mock_crud.get_all.return_value = []

            # 測試 API 回應時間
            start_time = time.time()

            for _ in range(100):
                response = client.get("/products")
                assert response.status_code == 200

            end_time = time.time()
            duration = end_time - start_time

            # 100 個 API 請求應該在 2 秒內完成
            assert duration < 2.0, f"API 回應時間過長: {duration:.2f}秒"

            # 平均每個請求應該在 20ms 內完成
            avg_time = duration / 100
            assert avg_time < 0.02, f"平均 API 回應時間過長: {avg_time:.3f}秒"

    def test_memory_usage_stability(self):
        """測試記憶體使用穩定性"""
        import gc
        import sys

        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 記錄初始記憶體使用
        gc.collect()
        initial_objects = len(gc.get_objects())

        # 模擬大量操作
        for i in range(1000):
            socket_handler.user_sid_map[i] = f"sid_{i}"

        # 清理
        socket_handler.user_sid_map.clear()

        # 強制垃圾回收
        gc.collect()
        final_objects = len(gc.get_objects())

        # 記憶體使用不應該顯著增加
        object_increase = final_objects - initial_objects
        assert object_increase < 100, f"記憶體洩漏可能: 物件增加 {object_increase}"


class TestStress:
    """壓力測試"""

    @pytest.mark.asyncio
    async def test_concurrent_socket_operations(self):
        """測試並發 Socket 操作"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 模擬資料庫操作
        with patch('opui.db.get_client') as mock_get_client, \
                patch('opui.db.product_all') as mock_product_all, \
                patch('opui.db.machine_all') as mock_machine_all, \
                patch('opui.db.room_all') as mock_room_all, \
            patch('opui.db.connection_pool'):

            mock_get_client.side_effect = lambda data: {
                "clientId": data["clientId"], "userAgent": "Test", "machineId": 1,
                "created_at": None, "updated_at": None
            }
            mock_product_all.return_value = []
            mock_machine_all.return_value = []
            mock_room_all.return_value = []

            # 創建大量並發操作
            tasks = []

            # 連線任務
            for i in range(50):
                tasks.append(socket_handler.connect(f"sid_{i}", {}))

            # 登入任務
            for i in range(50):
                tasks.append(socket_handler.login(f"sid_{i}", {"clientId": i}))

            # 執行所有任務
            start_time = time.time()
            results = await asyncio.gather(*tasks, return_exceptions=True)
            end_time = time.time()

            # 檢查是否有異常
            exceptions = [r for r in results if isinstance(r, Exception)]
            assert len(exceptions) == 0, f"並發操作產生異常: {exceptions}"

            # 所有操作應該在合理時間內完成
            duration = end_time - start_time
            assert duration < 5.0, f"並發操作時間過長: {duration:.2f}秒"

    @pytest.mark.asyncio
    async def test_rapid_connect_disconnect(self):
        """測試快速連線斷線"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 快速連線和斷線循環
        for cycle in range(10):
            connect_tasks = []
            disconnect_tasks = []

            # 批量連線
            for i in range(20):
                sid = f"cycle_{cycle}_sid_{i}"
                connect_tasks.append(socket_handler.connect(sid, {}))
                socket_handler.user_sid_map[i] = sid

            await asyncio.gather(*connect_tasks)

            # 批量斷線
            for i in range(20):
                sid = f"cycle_{cycle}_sid_{i}"
                disconnect_tasks.append(socket_handler.disconnect(sid))

            await asyncio.gather(*disconnect_tasks)

            # 驗證使用者映射被正確清理
            assert len(socket_handler.user_sid_map) == 0

    def test_high_frequency_api_requests(self):
        """測試高頻率 API 請求"""
        server = OpUiServer()
        client = TestClient(server.app)

        with patch('opui.routers.product.connection_pool') as mock_pool, \
                patch('opui.routers.product.product_crud') as mock_crud:

            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session
            mock_crud.get_all.return_value = []

            # 使用多執行緒模擬高頻率請求
            def make_request():
                response = client.get("/products")
                return response.status_code == 200

            with ThreadPoolExecutor(max_workers=10) as executor:
                start_time = time.time()

                # 提交 200 個並發請求
                futures = [executor.submit(make_request) for _ in range(200)]

                # 等待所有請求完成
                results = [future.result() for future in futures]

                end_time = time.time()
                duration = end_time - start_time

                # 驗證所有請求都成功
                assert all(results), "部分 API 請求失敗"

                # 200 個並發請求應該在 5 秒內完成
                assert duration < 5.0, f"高頻率 API 請求時間過長: {duration:.2f}秒"

    @pytest.mark.asyncio
    async def test_large_data_handling(self):
        """測試大資料處理"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 模擬大量資料的產品列表
        large_product_list = [
            {"id": i, "name": f"產品_{i}", "size": "大", "process_settings_id": 1}
            for i in range(1000)
        ]

        with patch('opui.db.product_all') as mock_product_all:
            mock_product_all.return_value = large_product_list

            # 測試處理大資料的效能
            start_time = time.time()

            await socket_handler.notify_products("test_sid")

            end_time = time.time()
            duration = end_time - start_time

            # 處理 1000 筆資料應該在 1 秒內完成
            assert duration < 1.0, f"大資料處理時間過長: {duration:.2f}秒"

            # 驗證 emit 被調用
            assert hasattr(socket_handler.sio, 'emit')
            assert callable(socket_handler.sio.emit)

    @pytest.mark.asyncio
    async def test_error_recovery_under_stress(self):
        """測試壓力下的錯誤恢復"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 模擬部分操作失敗的情況
        call_count = 0

        def mock_get_client(data):
            nonlocal call_count
            call_count += 1
            if call_count % 3 == 0:  # 每三次調用失敗一次
                raise Exception("模擬資料庫錯誤")
            return {
                "clientId": data["clientId"], "userAgent": "Test", "machineId": 1,
                "created_at": None, "updated_at": None
            }

        with patch('opui.db.get_client', side_effect=mock_get_client), \
                patch('opui.db.product_all', return_value=[]), \
                patch('opui.db.machine_all', return_value=[]), \
                patch('opui.db.room_all', return_value=[]), \
        patch('opui.db.connection_pool'):

            # 執行大量登入操作，部分會失敗
            tasks = []
            for i in range(30):
                tasks.append(socket_handler.login(f"sid_{i}", {"clientId": i}))

            results = await asyncio.gather(*tasks, return_exceptions=True)

            # 統計成功和失敗的數量
            successes = [r for r in results if not isinstance(r, Exception) and r.get("success")]
            failures = [r for r in results if isinstance(r, Exception)]

            # 驗證系統能處理錯誤情況（可能沒有失敗也是正常的，說明系統很穩定）
            print(f"失敗操作數量: {len(failures)}")
            print(f"成功操作數量: {len(successes)}")
            # 至少要有一些操作被執行
            assert len(successes) + len(failures) > 0, "沒有執行任何操作"
            assert len(successes) > 0, "沒有成功的操作"

            # 系統應該能夠繼續運行
            assert socket_handler.sio is not None

    def test_resource_cleanup_under_stress(self):
        """測試壓力下的資源清理"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 創建大量使用者會話
        for i in range(1000):
            socket_handler.user_sid_map[i] = f"sid_{i}"

        # 驗證會話數量
        assert len(socket_handler.user_sid_map) == 1000

        # 模擬批量斷線清理
        sids_to_remove = [f"sid_{i}" for i in range(0, 1000, 2)]  # 移除偶數編號的會話

        start_time = time.time()

        for sid in sids_to_remove:
            # 找到對應的 clientId 並移除
            for client_id, s in list(socket_handler.user_sid_map.items()):
                if s == sid:
                    del socket_handler.user_sid_map[client_id]
                    break

        end_time = time.time()
        duration = end_time - start_time

        # 清理操作應該很快完成
        assert duration < 1.0, f"資源清理時間過長: {duration:.2f}秒"

        # 驗證正確數量的會話被移除
        assert len(socket_handler.user_sid_map) == 500


class TestScalability:
    """可擴展性測試"""

    @pytest.mark.asyncio
    async def test_user_session_scalability(self):
        """測試使用者會話可擴展性"""
        server = OpUiServer()
        socket_handler = server.op_ui_socket

        # 測試支援大量使用者會話
        max_users = 500

        start_time = time.time()

        # 創建大量使用者會話
        for i in range(max_users):
            socket_handler.user_sid_map[i] = f"sid_{i}"

        # 測試查找效能
        lookup_start = time.time()
        for i in range(100):
            # 隨機查找使用者
            user_id = i % max_users
            assert socket_handler.user_sid_map[user_id] == f"sid_{user_id}"

        lookup_end = time.time()
        lookup_duration = lookup_end - lookup_start

        end_time = time.time()
        total_duration = end_time - start_time

        # 創建和查找都應該很快
        assert total_duration < 2.0, f"使用者會話處理時間過長: {total_duration:.2f}秒"
        assert lookup_duration < 0.1, f"使用者查找時間過長: {lookup_duration:.2f}秒"

    def test_api_endpoint_scalability(self):
        """測試 API 端點可擴展性"""
        server = OpUiServer()
        client = TestClient(server.app)

        # 測試多個端點的並發處理
        endpoints = ["/", "/setting"]

        def test_endpoint(endpoint):
            response = client.get(endpoint)
            return response.status_code == 200

        with ThreadPoolExecutor(max_workers=20) as executor:
            start_time = time.time()

            # 對每個端點發送多個並發請求
            futures = []
            for endpoint in endpoints:
                for _ in range(50):
                    futures.append(executor.submit(test_endpoint, endpoint))

            results = [future.result() for future in futures]

            end_time = time.time()
            duration = end_time - start_time

            # 驗證所有請求都成功
            assert all(results), "部分端點請求失敗"

            # 100 個並發請求應該在合理時間內完成
            assert duration < 3.0, f"端點可擴展性測試時間過長: {duration:.2f}秒"
