"""
測試 OPUI 資料庫操作模組
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime, timezone
from sqlmodel import Session

from opui.db import (
    get_client, get_or_create_or_update_client, create_or_update_product,
    product_all, machine_all, room_all, create_task,
    get_call_empty_work_id, get_dispatch_full_work_id, get_default_task_status_id,
    delete_task_by_parking, delete_task
)
from db_proxy.models import Client, Product, Machine, Room, Task, Work, TaskStatus


class TestClientOperations:
    """測試客戶端相關操作"""

    def test_get_client_existing(self, mock_connection_pool, mock_db_session):
        """測試獲取現有客戶端"""
        # 準備測試資料
        test_client = pytest.create_test_client(id=1, user_agent="Test Browser")
        mock_db_session.exec.return_value.first.return_value = test_client

        # 執行測試
        client_data = {"clientId": 1}
        result = get_client(client_data)

        # 驗證結果
        assert result["clientId"] == 1
        assert result["userAgent"] == "Test Browser"
        # 檢查是否有時間戳欄位（可能是 updated_at 或 updatedAt）
        assert any(key in result for key in ["updated_at", "updatedAt", "createdAt", "created_at"])

    def test_get_client_not_existing(self, mock_connection_pool, mock_db_session):
        """測試獲取不存在的客戶端時創建新客戶端"""
        # 模擬客戶端不存在
        mock_db_session.exec.return_value.first.return_value = None

        # 模擬創建新客戶端
        new_client = pytest.create_test_client(id=1)
        with patch('opui.db.client_crud') as mock_crud:
            mock_crud.create.return_value = new_client

            client_data = {"clientId": 1, "userAgent": "New Browser"}
            result = get_client(client_data)

            # 驗證創建了新客戶端
            mock_crud.create.assert_called_once()
            assert result["clientId"] == 1

    def test_get_or_create_or_update_client(self, mock_connection_pool):
        """測試創建或更新客戶端"""
        test_client = pytest.create_test_client(id=1)

        with patch('opui.db.client_crud') as mock_crud:
            mock_crud.create_or_update.return_value = test_client

            client_data = {
                "clientId": 1,
                "userAgent": "Updated Browser",
                "op": {"left": {"productSelected": 0}},
                "machineId": 2
            }

            result = get_or_create_or_update_client(client_data)

            # 驗證調用了 create_or_update
            mock_crud.create_or_update.assert_called_once()
            assert result["clientId"] == 1

    def test_client_data_field_mapping(self, mock_connection_pool, mock_db_session):
        """測試客戶端資料欄位映射"""
        test_client = pytest.create_test_client(
            id=1,
            user_agent="Test Browser",
            machine_id=2
        )
        mock_db_session.exec.return_value.first.return_value = test_client

        client_data = {
            "clientId": 1,
            "userAgent": "Test Browser",
            "machineId": 2
        }

        result = get_client(client_data)

        # 驗證欄位映射正確
        assert result["clientId"] == 1
        assert result["userAgent"] == "Test Browser"
        assert result["machineId"] == 2

    def test_client_default_values(self, mock_connection_pool):
        """測試客戶端預設值設定"""
        with patch('opui.db.client_crud') as mock_crud:
            mock_crud.create_or_update.return_value = pytest.create_test_client()

            # 測試空資料的預設值處理
            client_data = {"clientId": 1}
            get_or_create_or_update_client(client_data)

            # 驗證預設值被設定
            call_args = mock_crud.create_or_update.call_args[0][1]
            assert call_args.user_agent == ""
            assert call_args.op == {}
            assert call_args.machine_id == 1


class TestProductOperations:
    """測試產品相關操作"""

    def test_create_or_update_product_new(self, mock_connection_pool, mock_db_session):
        """測試創建新產品"""
        # 模擬產品不存在
        mock_db_session.exec.return_value.first.return_value = None

        test_product = pytest.create_test_product(name="新產品")
        with patch('opui.db.product_crud') as mock_crud:
            mock_crud.create.return_value = test_product

            product_data = {
                "product": "新產品",
                "size": "大",
                "process": 1
            }

            result = create_or_update_product(product_data)

            # 驗證創建了新產品
            mock_crud.create.assert_called_once()
            assert result["product"] == "新產品"

    def test_create_or_update_product_existing(self, mock_connection_pool, mock_db_session):
        """測試更新現有產品"""
        existing_product = pytest.create_test_product(id=1, name="現有產品")
        mock_db_session.exec.return_value.first.return_value = existing_product

        updated_product = pytest.create_test_product(id=1, name="現有產品", size="小")
        with patch('opui.db.product_crud') as mock_crud:
            mock_crud.update.return_value = updated_product

            product_data = {
                "product": "現有產品",
                "size": "小",
                "process": 1
            }

            result = create_or_update_product(product_data)

            # 驗證更新了現有產品
            mock_crud.update.assert_called_once()

    def test_product_all(self, mock_connection_pool):
        """測試獲取所有產品"""
        test_products = [
            pytest.create_test_product(id=1, name="產品A"),
            pytest.create_test_product(id=2, name="產品B")
        ]

        with patch('opui.db.product_crud') as mock_crud:
            mock_crud.get_all.return_value = test_products

            result = product_all()

            # 驗證返回所有產品
            assert len(result) == 2
            assert result[0]["name"] == "產品A"
            assert result[1]["name"] == "產品B"


class TestMachineOperations:
    """測試機台相關操作"""

    def test_machine_all(self, mock_connection_pool):
        """測試獲取所有機台"""
        test_machines = [
            pytest.create_test_machine(id=1, name="機台A"),
            pytest.create_test_machine(id=2, name="機台B")
        ]

        with patch('opui.db.machine_crud') as mock_crud:
            mock_crud.get_all.return_value = test_machines

            result = machine_all()

            # 驗證返回所有機台
            assert len(result) == 2
            assert result[0]["name"] == "機台A"
            assert result[1]["name"] == "機台B"


class TestRoomOperations:
    """測試房間相關操作"""

    def test_room_all(self, mock_connection_pool):
        """測試獲取所有房間"""
        test_rooms = [
            Room(id=1, name="房間A"),
            Room(id=2, name="房間B")
        ]

        with patch('opui.db.room_crud') as mock_crud:
            mock_crud.get_all.return_value = test_rooms

            result = room_all()

            # 驗證返回所有房間
            assert len(result) == 2
            assert result[0]["name"] == "房間A"
            assert result[1]["name"] == "房間B"


class TestTaskOperations:
    """測試任務相關操作"""

    def test_create_task(self, mock_connection_pool):
        """測試創建任務"""
        test_task = pytest.create_test_task(id=1, name="測試任務")

        with patch('opui.db.task_crud') as mock_crud:
            mock_crud.create.return_value = test_task

            task_data = {
                "name": "測試任務",
                "description": "測試任務描述",
                "work_id": 1,
                "status_id": 1,
                "node_id": 101,
                "priority": 1,
                "parameters": {"test": "data"}
            }

            result = create_task(task_data)

            # 驗證創建了任務
            mock_crud.create.assert_called_once()
            assert result["name"] == "測試任務"

    def test_delete_task_by_parking_success(self, mock_connection_pool, mock_db_session):
        """測試根據停車格刪除任務成功"""
        test_task = pytest.create_test_task(id=1, node_id=101)
        mock_db_session.exec.return_value.first.return_value = test_task

        with patch('opui.db.task_crud') as mock_crud:
            mock_crud.delete.return_value = True

            result = delete_task_by_parking(101)

            # 驗證刪除成功
            assert result is True
            mock_crud.delete.assert_called_once_with(mock_db_session, 1)

    def test_delete_task_by_parking_not_found(self, mock_connection_pool, mock_db_session):
        """測試根據停車格刪除任務但找不到任務"""
        mock_db_session.exec.return_value.first.return_value = None

        result = delete_task_by_parking(101)

        # 驗證返回 False
        assert result is False

    def test_delete_task(self, mock_connection_pool):
        """測試根據 ID 刪除任務"""
        with patch('opui.db.task_crud') as mock_crud:
            mock_crud.delete.return_value = True

            result = delete_task(1)

            # 驗證刪除成功
            assert result is True
            mock_crud.delete.assert_called_once()


class TestWorkAndStatusOperations:
    """測試工作類型和狀態相關操作"""

    def test_get_call_empty_work_id_existing(self, mock_connection_pool, mock_db_session):
        """測試獲取現有的叫空車工作類型 ID"""
        test_work = Work(id=1, name="opui-call-empty")
        mock_db_session.exec.return_value.first.return_value = test_work

        result = get_call_empty_work_id()

        assert result == 1

    def test_get_call_empty_work_id_create_new(self, mock_connection_pool, mock_db_session):
        """測試創建新的叫空車工作類型"""
        # 模擬工作類型不存在
        mock_db_session.exec.return_value.first.return_value = None

        new_work = Work(id=1, name="opui-call-empty")
        with patch('opui.db.work_crud') as mock_crud:
            mock_crud.create.return_value = new_work

            result = get_call_empty_work_id()

            # 驗證創建了新工作類型
            mock_crud.create.assert_called_once()
            assert result == 1

    def test_get_dispatch_full_work_id(self, mock_connection_pool, mock_db_session):
        """測試獲取派滿車工作類型 ID"""
        test_work = Work(id=2, name="opui-dispatch-full")
        mock_db_session.exec.return_value.first.return_value = test_work

        result = get_dispatch_full_work_id()

        assert result == 2

    def test_get_default_task_status_id(self, mock_connection_pool, mock_db_session):
        """測試獲取預設任務狀態 ID"""
        test_status = TaskStatus(id=1, name="待執行")
        mock_db_session.exec.return_value.first.return_value = test_status

        result = get_default_task_status_id()

        assert result == 1

    def test_get_default_task_status_id_create_new(self, mock_connection_pool, mock_db_session):
        """測試創建新的預設任務狀態"""
        # 模擬狀態不存在
        mock_db_session.exec.return_value.first.return_value = None

        new_status = TaskStatus(id=1, name="待執行")
        with patch('opui.db.task_status_crud') as mock_crud:
            mock_crud.create.return_value = new_status

            result = get_default_task_status_id()

            # 驗證創建了新狀態
            mock_crud.create.assert_called_once()
            assert result == 1
