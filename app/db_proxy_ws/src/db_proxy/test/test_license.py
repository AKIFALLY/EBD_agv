"""
測試 License 模型和 CRUD 操作
"""

import pytest
from sqlmodel import SQLModel, Session, create_engine
from db_proxy.models import License
from db_proxy.crud.license_crud import license_crud
from db_proxy.connection_pool_manager import ConnectionPoolManager


# 測試資料庫連線字串
POSTGRES_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"


@pytest.fixture(scope="session")
def engine():
    """建立測試用的資料庫引擎"""
    engine = create_engine(POSTGRES_URL, echo=False)
    SQLModel.metadata.create_all(engine)
    yield engine


@pytest.fixture
def session(engine):
    """建立測試用的資料庫 session"""
    with Session(engine) as session:
        yield session


@pytest.fixture
def connection_pool():
    """建立測試用的連線池"""
    return ConnectionPoolManager(POSTGRES_URL)


class TestLicenseModel:
    """測試 License 模型"""

    def test_license_creation(self):
        """測試 License 模型建立"""
        license_data = License(device_id="test_device", active=1)
        assert license_data.device_id == "test_device"
        assert license_data.active == 1
        assert license_data.id is None  # 尚未儲存到資料庫

    def test_license_fields(self):
        """測試 License 模型欄位"""
        license_data = License(device_id="test_device_123", active=0)

        # 檢查必要欄位
        assert hasattr(license_data, 'id')
        assert hasattr(license_data, 'device_id')
        assert hasattr(license_data, 'active')

        # 檢查欄位值
        assert license_data.device_id == "test_device_123"
        assert license_data.active == 0


class TestLicenseCRUD:
    """測試 License CRUD 操作"""

    def test_create_license(self, session):
        """測試建立 License"""
        license_data = License(device_id="test_create", active=1)
        created_license = license_crud.create(session, license_data)

        assert created_license.id is not None
        assert created_license.device_id == "test_create"
        assert created_license.active == 1

    def test_get_license_by_id(self, session):
        """測試根據 ID 查詢 License"""
        # 先建立測試資料
        license_data = License(device_id="test_get_by_id", active=1)
        created_license = license_crud.create(session, license_data)

        # 查詢測試
        found_license = license_crud.get_by_id(session, created_license.id)
        assert found_license is not None
        assert found_license.device_id == "test_get_by_id"
        assert found_license.active == 1

    def test_get_license_by_device_id(self, session):
        """測試根據 device_id 查詢 License"""
        # 先建立測試資料
        test_device_id = "test_get_by_device_id"
        license_data = License(device_id=test_device_id, active=0)
        license_crud.create(session, license_data)

        # 查詢測試
        found_license = license_crud.get_by_device_id(session, test_device_id)
        assert found_license is not None
        assert found_license.device_id == test_device_id
        assert found_license.active == 0

    def test_get_license_by_device_id_not_found(self, session):
        """測試查詢不存在的 device_id"""
        found_license = license_crud.get_by_device_id(session, "nonexistent_device")
        assert found_license is None

    def test_get_all_licenses(self, session):
        """測試取得所有 License"""
        # 先建立測試資料
        license1 = License(device_id="test_all_1", active=1)
        license2 = License(device_id="test_all_2", active=0)

        license_crud.create(session, license1)
        license_crud.create(session, license2)

        # 查詢測試
        all_licenses = license_crud.get_all(session)
        assert len(all_licenses) >= 2

        # 檢查是否包含我們建立的資料
        device_ids = [license.device_id for license in all_licenses]
        assert "test_all_1" in device_ids
        assert "test_all_2" in device_ids

    def test_update_license(self, session):
        """測試更新 License"""
        # 先建立測試資料
        license_data = License(device_id="test_update", active=0)
        created_license = license_crud.create(session, license_data)

        # 更新測試 - 建立新的 License 物件用於更新
        update_data = License(device_id="test_update", active=1)
        updated_license = license_crud.update(
            session,
            created_license.id,
            update_data
        )

        assert updated_license is not None
        assert updated_license.active == 1
        assert updated_license.device_id == "test_update"

    def test_delete_license(self, session):
        """測試刪除 License"""
        # 先建立測試資料
        license_data = License(device_id="test_delete", active=1)
        created_license = license_crud.create(session, license_data)

        # 刪除測試
        deleted = license_crud.delete(session, created_license.id)
        assert deleted is True

        # 確認已刪除
        found_license = license_crud.get_by_id(session, created_license.id)
        assert found_license is None


class TestLicenseInitialization:
    """測試 License 初始化功能"""

    def test_initialize_license(self, connection_pool):
        """測試 License 初始化功能"""
        import importlib

        # 動態導入初始化模組
        license_module = importlib.import_module("db_proxy.sql.init_data.20_license")
        initialize_license = getattr(license_module, "initialize_license")

        with connection_pool.get_session() as session:
            # 執行初始化
            initialize_license(session)

            # 驗證初始化結果
            license_data = license_crud.get_by_device_id(session, "ca08777c72096c51")
            assert license_data is not None
            assert license_data.device_id == "ca08777c72096c51"
            assert license_data.active == 1

    def test_initialize_license_idempotent(self, connection_pool):
        """測試 License 初始化的冪等性（重複執行不會重複插入）"""
        import importlib

        license_module = importlib.import_module("db_proxy.sql.init_data.20_license")
        initialize_license = getattr(license_module, "initialize_license")

        with connection_pool.get_session() as session:
            # 第一次初始化
            initialize_license(session)
            first_count = len(license_crud.get_all(session))

            # 第二次初始化
            initialize_license(session)
            second_count = len(license_crud.get_all(session))

            # 確認沒有重複插入
            assert first_count == second_count


if __name__ == "__main__":
    # 可以直接執行此檔案進行測試
    pytest.main([__file__, "-v"])
