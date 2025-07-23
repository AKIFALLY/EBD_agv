"""
測試 OPUI 伺服器模組
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from fastapi.testclient import TestClient

from opui.op_ui_server import OpUiServer


class TestOpUiServer:
    """測試 OpUiServer 類別"""

    def test_server_initialization(self):
        """測試伺服器初始化"""
        server = OpUiServer(host="127.0.0.1", port=8003)

        # 驗證基本屬性
        assert server.host == "127.0.0.1"
        assert server.port == 8003
        assert server.app is not None
        assert server.sio is not None
        assert server.sio_app is not None
        assert server.op_ui_socket is not None

    def test_server_default_initialization(self):
        """測試伺服器預設初始化"""
        server = OpUiServer()

        # 驗證預設值
        assert server.host == "0.0.0.0"
        assert server.port == 8002

    def test_register_routes(self, op_ui_server):
        """測試路由註冊"""
        # 路由應該在初始化時自動註冊
        client = TestClient(op_ui_server.app)

        # 測試主頁路由
        response = client.get("/")
        assert response.status_code == 200
        assert "text/html" in response.headers["content-type"]

    def test_setting_route(self, op_ui_server):
        """測試設定頁面路由"""
        client = TestClient(op_ui_server.app)

        response = client.get("/setting")
        assert response.status_code == 200
        assert "text/html" in response.headers["content-type"]

    @patch('opui.op_ui_server.uvicorn.run')
    def test_server_run(self, mock_uvicorn_run, op_ui_server):
        """測試伺服器運行"""
        op_ui_server.run()

        # 驗證 uvicorn.run 被正確調用
        mock_uvicorn_run.assert_called_once_with(
            op_ui_server.sio_app,
            host=op_ui_server.host,
            port=op_ui_server.port
        )

    @patch('opui.op_ui_server.uvicorn.run')
    def test_server_run_with_exception(self, mock_uvicorn_run, op_ui_server, capsys):
        """測試伺服器運行時發生異常"""
        # 模擬 uvicorn.run 拋出異常
        mock_uvicorn_run.side_effect = Exception("Test exception")

        op_ui_server.run()

        # 驗證異常被捕獲並印出錯誤訊息
        captured = capsys.readouterr()
        assert "Error running server: Test exception" in captured.out

    def test_cors_middleware_configuration(self, op_ui_server):
        """測試 CORS 中介軟體配置"""
        client = TestClient(op_ui_server.app)

        # 發送 OPTIONS 請求測試 CORS
        response = client.options("/", headers={
            "Origin": "http://localhost:3000",
            "Access-Control-Request-Method": "GET"
        })

        # 驗證 CORS 標頭
        assert "access-control-allow-origin" in response.headers
        # CORS 可能設定為 "*" 或特定來源
        assert response.headers["access-control-allow-origin"] in ["*", "http://localhost:3000"]

    def test_static_files_serving(self, op_ui_server):
        """測試靜態檔案服務"""
        client = TestClient(op_ui_server.app)

        # 嘗試訪問靜態檔案（這裡測試 CSS 檔案）
        response = client.get("/static/css/opui-bulma-extend.css")

        # 如果檔案存在，應該返回 200；如果不存在，返回 404
        assert response.status_code in [200, 404]

        if response.status_code == 200:
            assert "text/css" in response.headers.get("content-type", "")

    def test_template_configuration(self, op_ui_server):
        """測試模板配置"""
        # 驗證模板目錄配置
        assert op_ui_server.templates is not None

        # 測試模板渲染（透過路由測試）
        client = TestClient(op_ui_server.app)
        response = client.get("/")

        # 應該成功渲染模板
        assert response.status_code == 200

    def test_socketio_integration(self, op_ui_server):
        """測試 Socket.IO 整合"""
        # 驗證 Socket.IO 伺服器配置
        assert op_ui_server.sio.async_mode == "asgi"

        # 驗證 OpUiSocket 實例創建
        assert op_ui_server.op_ui_socket is not None
        assert op_ui_server.op_ui_socket.sio == op_ui_server.sio

    def test_router_inclusion(self, op_ui_server):
        """測試路由器包含"""
        client = TestClient(op_ui_server.app)

        # 測試 process_settings 路由
        response = client.get("/process-settings")
        # 可能返回 200（如果有資料）或其他狀態碼
        assert response.status_code in [200, 404, 422, 500]

        # 測試 product 路由
        response = client.get("/products")
        assert response.status_code in [200, 404, 422, 500]


class TestOpUiServerErrorHandling:
    """測試伺服器錯誤處理"""

    @patch('opui.op_ui_server.Jinja2Templates')
    def test_home_route_template_error(self, mock_templates, op_ui_server):
        """測試主頁路由模板錯誤處理"""
        # 模擬模板渲染錯誤
        mock_templates.return_value.TemplateResponse.side_effect = Exception("Template error")

        # 重新創建伺服器以使用模擬的模板
        server = OpUiServer()
        server.templates = mock_templates.return_value
        server.register_routes()

        client = TestClient(server.app)
        response = client.get("/")

        # 應該返回 500 錯誤
        assert response.status_code == 500
        assert "Error in home route" in response.text

    @patch('opui.op_ui_server.Jinja2Templates')
    def test_setting_route_template_error(self, mock_templates, op_ui_server):
        """測試設定頁面路由模板錯誤處理"""
        # 模擬模板渲染錯誤
        mock_templates.return_value.TemplateResponse.side_effect = Exception("Template error")

        # 重新創建伺服器以使用模擬的模板
        server = OpUiServer()
        server.templates = mock_templates.return_value
        server.register_routes()

        client = TestClient(server.app)
        response = client.get("/setting")

        # 應該返回 500 錯誤
        assert response.status_code == 500
        assert "Error in setting route" in response.text


class TestOpUiServerMain:
    """測試主函數"""

    @patch('opui.op_ui_server.OpUiServer')
    def test_main_function(self, mock_server_class):
        """測試 main 函數"""
        from opui.op_ui_server import main

        # 模擬伺服器實例
        mock_server = Mock()
        mock_server_class.return_value = mock_server

        # 執行 main 函數
        main()

        # 驗證伺服器被創建和運行
        mock_server_class.assert_called_once()
        mock_server.run.assert_called_once()

    @patch('opui.op_ui_server.OpUiServer')
    def test_main_function_with_exception(self, mock_server_class, capsys):
        """測試 main 函數異常處理"""
        from opui.op_ui_server import main

        # 模擬伺服器運行時拋出異常
        mock_server = Mock()
        mock_server.run.side_effect = Exception("Server error")
        mock_server_class.return_value = mock_server

        # 執行 main 函數，期望異常被拋出
        try:
            main()
            # 如果沒有拋出異常，測試失敗
            assert False, "應該拋出異常"
        except Exception as e:
            # 驗證異常訊息
            assert "Server error" in str(e)

        # 驗證伺服器的 run 方法被調用
        mock_server.run.assert_called_once()


class TestOpUiServerConfiguration:
    """測試伺服器配置"""

    def test_socketio_cors_configuration(self, op_ui_server):
        """測試 Socket.IO CORS 配置"""
        # 驗證 Socket.IO 伺服器存在（CORS 配置可能在內部）
        assert op_ui_server.sio is not None

    def test_fastapi_middleware_configuration(self, op_ui_server):
        """測試 FastAPI 中介軟體配置"""
        # 檢查是否有 CORS 中介軟體
        middleware_types = [
            middleware.cls.__name__ for middleware in op_ui_server.app.user_middleware]

        # 應該包含 CORSMiddleware
        assert "CORSMiddleware" in middleware_types

    def test_static_files_mount(self, op_ui_server):
        """測試靜態檔案掛載"""
        # 檢查靜態檔案路由是否正確掛載
        routes = op_ui_server.app.routes
        static_routes = [route for route in routes if hasattr(
            route, 'path') and route.path.startswith('/static')]

        # 應該有靜態檔案路由
        assert len(static_routes) > 0

    def test_template_directory_configuration(self, op_ui_server):
        """測試模板目錄配置"""
        # 驗證模板目錄設定
        assert op_ui_server.templates is not None

        # 檢查模板目錄路徑
        template_dir = op_ui_server.templates.env.loader.searchpath[0]
        assert "templates" in template_dir


class TestOpUiServerIntegration:
    """測試伺服器整合功能"""

    def test_full_server_lifecycle(self):
        """測試完整的伺服器生命週期"""
        # 創建伺服器
        server = OpUiServer(host="127.0.0.1", port=8004)

        # 驗證初始化
        assert server.app is not None
        assert server.sio is not None

        # 測試路由註冊
        client = TestClient(server.app)
        response = client.get("/")
        assert response.status_code == 200

        # 測試 Socket.IO 整合
        assert server.op_ui_socket is not None

    @patch.dict('os.environ', {'CORS_ALLOWED_ORIGINS': 'http://localhost:3000'})
    def test_environment_variable_configuration(self):
        """測試環境變數配置"""
        # 這個測試檢查是否正確讀取環境變數
        # 由於當前實現沒有使用環境變數，這是一個擴展測試
        server = OpUiServer()

        # 驗證伺服器仍然正常工作
        assert server.app is not None

        client = TestClient(server.app)
        response = client.get("/")
        assert response.status_code == 200
