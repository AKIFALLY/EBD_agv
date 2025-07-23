"""
測試 OPUI API 路由模組
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from fastapi.testclient import TestClient
from fastapi import FastAPI

from opui.routers import product, process_settings
from db_proxy.models import Product, ProcessSettings


class TestProductRouter:
    """測試產品路由"""

    @pytest.fixture
    def app_with_product_router(self):
        """創建包含產品路由的 FastAPI 應用"""
        app = FastAPI()
        app.include_router(product.router)
        return app

    @pytest.fixture
    def client_with_product_router(self, app_with_product_router):
        """創建包含產品路由的測試客戶端"""
        return TestClient(app_with_product_router)

    def test_list_products_success(self, client_with_product_router):
        """測試獲取產品列表成功"""
        mock_products = [
            Product(id=1, name="產品A", size="大", process_settings_id=1),
            Product(id=2, name="產品B", size="小", process_settings_id=2)
        ]

        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.get_all.return_value = mock_products

                response = client_with_product_router.get("/products")

                assert response.status_code == 200
                data = response.json()
                assert len(data) == 2
                assert data[0]["name"] == "產品A"
                assert data[1]["name"] == "產品B"

    def test_list_products_empty(self, client_with_product_router):
        """測試獲取空產品列表"""
        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.get_all.return_value = []

                response = client_with_product_router.get("/products")

                assert response.status_code == 200
                data = response.json()
                assert len(data) == 0

    def test_list_products_database_error(self, client_with_product_router):
        """測試獲取產品列表時資料庫錯誤"""
        with patch('opui.routers.product.connection_pool') as mock_pool:
            # 直接讓 get_session 拋出異常
            mock_pool.get_session.side_effect = Exception("Database connection error")

            try:
                response = client_with_product_router.get("/products")
                # 如果沒有拋出異常，檢查狀態碼
                assert response.status_code == 500
            except Exception as e:
                # 如果拋出異常，驗證異常訊息
                assert "Database connection error" in str(e)

    def test_create_product_success(self, client_with_product_router):
        """測試創建產品成功"""
        new_product = Product(id=1, name="新產品", size="中", process_settings_id=1)

        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.create.return_value = new_product

                product_data = {
                    "name": "新產品",
                    "size": "中",
                    "process_settings_id": 1
                }

                response = client_with_product_router.post("/products", json=product_data)

                assert response.status_code == 200
                data = response.json()
                assert data["name"] == "新產品"
                assert data["size"] == "中"

    def test_create_product_validation_error(self, client_with_product_router):
        """測試創建產品時驗證錯誤"""
        # 缺少必要欄位
        invalid_data = {
            "name": "產品名稱"
            # 缺少 size 和 process_settings_id
        }

        response = client_with_product_router.post("/products", json=invalid_data)

        # 應該返回 422 驗證錯誤
        assert response.status_code == 422

    def test_get_product_by_id_success(self, client_with_product_router):
        """測試根據 ID 獲取產品成功"""
        test_product = Product(id=1, name="測試產品", size="大", process_settings_id=1)

        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.get_by_id.return_value = test_product

                response = client_with_product_router.get("/products/1")

                assert response.status_code == 200
                data = response.json()
                assert data["id"] == 1
                assert data["name"] == "測試產品"

    def test_get_product_by_id_not_found(self, client_with_product_router):
        """測試根據 ID 獲取產品但找不到"""
        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.get_by_id.return_value = None

                response = client_with_product_router.get("/products/999")

                assert response.status_code == 404
                data = response.json()
                assert "not found" in data["detail"].lower()

    def test_update_product_success(self, client_with_product_router):
        """測試更新產品成功"""
        updated_product = Product(id=1, name="更新產品", size="小", process_settings_id=2)

        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.update.return_value = updated_product

                update_data = {
                    "name": "更新產品",
                    "size": "小",
                    "process_settings_id": 2
                }

                response = client_with_product_router.put("/products/1", json=update_data)

                assert response.status_code == 200
                data = response.json()
                assert data["name"] == "更新產品"
                assert data["size"] == "小"

    def test_delete_product_success(self, client_with_product_router):
        """測試刪除產品成功"""
        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.delete.return_value = True

                response = client_with_product_router.delete("/products/1")

                assert response.status_code == 200
                data = response.json()
                assert "deleted successfully" in data["message"]

    def test_delete_product_not_found(self, client_with_product_router):
        """測試刪除不存在的產品"""
        with patch('opui.routers.product.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.product.product_crud') as mock_crud:
                mock_crud.delete.return_value = False

                response = client_with_product_router.delete("/products/999")

                assert response.status_code == 404
                data = response.json()
                assert "not found" in data["detail"].lower()


class TestProcessSettingsRouter:
    """測試製程設定路由"""

    @pytest.fixture
    def app_with_process_settings_router(self):
        """創建包含製程設定路由的 FastAPI 應用"""
        app = FastAPI()
        app.include_router(process_settings.router)
        return app

    @pytest.fixture
    def client_with_process_settings_router(self, app_with_process_settings_router):
        """創建包含製程設定路由的測試客戶端"""
        return TestClient(app_with_process_settings_router)

    def test_list_process_settings_success(self, client_with_process_settings_router):
        """測試獲取製程設定列表成功"""
        mock_settings = [
            ProcessSettings(id=1, soaking_times=10, description="設定A"),
            ProcessSettings(id=2, soaking_times=20, description="設定B")
        ]

        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.get_all.return_value = mock_settings

                response = client_with_process_settings_router.get("/process-settings")

                assert response.status_code == 200
                data = response.json()
                assert len(data) == 2
                assert data[0]["soaking_times"] == 10
                assert data[1]["soaking_times"] == 20

    def test_get_process_settings_by_id_success(self, client_with_process_settings_router):
        """測試根據 ID 獲取製程設定成功"""
        test_settings = ProcessSettings(id=1, soaking_times=15, description="測試設定")

        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.get_by_id.return_value = test_settings

                response = client_with_process_settings_router.get("/process-settings/1")

                assert response.status_code == 200
                data = response.json()
                assert data["id"] == 1
                assert data["soaking_times"] == 15

    def test_get_process_settings_by_id_not_found(self, client_with_process_settings_router):
        """測試根據 ID 獲取製程設定但找不到"""
        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.get_by_id.return_value = None

                response = client_with_process_settings_router.get("/process-settings/999")

                assert response.status_code == 404
                data = response.json()
                assert "not found" in data["detail"].lower()

    def test_create_process_settings_success(self, client_with_process_settings_router):
        """測試創建製程設定成功"""
        new_settings = ProcessSettings(id=1, soaking_times=25, description="新設定")

        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.create.return_value = new_settings

                settings_data = {
                    "soaking_times": 25,
                    "description": "新設定"
                }

                response = client_with_process_settings_router.post(
                    "/process-settings", json=settings_data)

                assert response.status_code == 200
                data = response.json()
                assert data["soaking_times"] == 25
                assert data["description"] == "新設定"

    def test_create_process_settings_validation_error(self, client_with_process_settings_router):
        """測試創建製程設定時驗證錯誤"""
        # 缺少必要欄位
        invalid_data = {
            "description": "只有描述"
            # 缺少 soaking_times
        }

        response = client_with_process_settings_router.post("/process-settings", json=invalid_data)

        # 應該返回 422 驗證錯誤
        assert response.status_code == 422

    def test_update_process_settings_success(self, client_with_process_settings_router):
        """測試更新製程設定成功"""
        updated_settings = ProcessSettings(id=1, soaking_times=30, description="更新設定")

        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.update.return_value = updated_settings

                update_data = {
                    "soaking_times": 30,
                    "description": "更新設定"
                }

                response = client_with_process_settings_router.put(
                    "/process-settings/1", json=update_data)

                assert response.status_code == 200
                data = response.json()
                assert data["soaking_times"] == 30
                assert data["description"] == "更新設定"

    def test_update_process_settings_not_found(self, client_with_process_settings_router):
        """測試更新不存在的製程設定"""
        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.update.return_value = None

                update_data = {
                    "soaking_times": 30,
                    "description": "更新設定"
                }

                response = client_with_process_settings_router.put(
                    "/process-settings/999", json=update_data)

                assert response.status_code == 404
                data = response.json()
                assert "not found" in data["detail"].lower()

    def test_delete_process_settings_success(self, client_with_process_settings_router):
        """測試刪除製程設定成功"""
        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.delete.return_value = True

                response = client_with_process_settings_router.delete("/process-settings/1")

                assert response.status_code == 200
                data = response.json()
                assert "deleted successfully" in data["message"]

    def test_delete_process_settings_not_found(self, client_with_process_settings_router):
        """測試刪除不存在的製程設定"""
        with patch('opui.routers.process_settings.connection_pool') as mock_pool:
            mock_session = Mock()
            mock_pool.get_session.return_value.__enter__.return_value = mock_session

            with patch('opui.routers.process_settings.process_settings_crud') as mock_crud:
                mock_crud.delete.return_value = False

                response = client_with_process_settings_router.delete("/process-settings/999")

                assert response.status_code == 404
                data = response.json()
                assert "not found" in data["detail"].lower()


class TestRouterIntegration:
    """測試路由整合"""

    def test_multiple_routers_integration(self):
        """測試多個路由器整合"""
        app = FastAPI()
        app.include_router(product.router)
        app.include_router(process_settings.router)

        client = TestClient(app)

        # 測試兩個路由器都正常工作
        with patch('opui.routers.product.connection_pool'), \
                patch('opui.routers.product.product_crud') as mock_product_crud, \
                patch('opui.routers.process_settings.connection_pool'), \
                patch('opui.routers.process_settings.process_settings_crud') as mock_settings_crud:

            mock_product_crud.get_all.return_value = []
            mock_settings_crud.get_all.return_value = []

            # 測試產品路由
            response = client.get("/products")
            assert response.status_code == 200

            # 測試製程設定路由
            response = client.get("/process-settings")
            assert response.status_code == 200

    def test_router_tags(self):
        """測試路由標籤"""
        # 驗證路由器有正確的標籤
        assert "Product" in product.router.tags
        assert "ProcessSettings" in process_settings.router.tags
