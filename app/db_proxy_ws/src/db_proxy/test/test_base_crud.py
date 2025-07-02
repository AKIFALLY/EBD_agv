from typing import Optional
import pytest
from datetime import datetime, timezone
from sqlmodel import SQLModel, Field, Session, create_engine
from db_proxy.crud.base_crud import BaseCRUD


class DummyModel(SQLModel, table=True):
    id: int | None = Field(default=None, primary_key=True)
    name: str
    value: int
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


# 這邊請改成你的 PostgreSQL 連線字串
POSTGRES_URL = "postgresql://agvc:password@192.168.100.254:5432/test_db"


@pytest.fixture(scope="session")
def engine():
    engine = create_engine(POSTGRES_URL, echo=False)
    SQLModel.metadata.create_all(engine)
    yield engine
    # 測試完可加上 drop all (選擇性)
    SQLModel.metadata.drop_all(engine)


@pytest.fixture
def session(engine):
    with Session(engine) as session:
        yield session


@pytest.fixture
def crud():
    return BaseCRUD(DummyModel)


def test_create_and_get_by_id(session, crud):
    obj = DummyModel(name="測試", value=10)
    created = crud.create(session, obj)
    assert created.id is not None
    assert created.created_at is not None
    assert created.updated_at is None

    found = crud.get_by_id(session, created.id)
    assert found is not None
    assert found.name == "測試"
    assert found.value == 10


def test_get_all(session, crud):
    crud.create(session, DummyModel(name="A", value=1))
    crud.create(session, DummyModel(name="B", value=2))
    all_items = crud.get_all(session)
    assert len(all_items) >= 2
    names = {item.name for item in all_items}
    assert "A" in names and "B" in names


def test_update(session, crud):
    created = crud.create(session, DummyModel(name="Old", value=5))
    updated_obj = DummyModel(name="New", value=20)
    updated = crud.update(session, created.id, updated_obj)
    assert updated.name == "New"
    assert updated.value == 20
    assert updated.updated_at is not None
    assert updated.created_at == created.created_at


def test_delete(session, crud):
    created = crud.create(session, DummyModel(name="DeleteMe", value=99))
    success = crud.delete(session, created.id)
    assert success
    assert crud.get_by_id(session, created.id) is None


def test_create_or_update(session, crud):
    # create
    obj = DummyModel(id=1, name="First", value=10)
    created = crud.create_or_update(session, obj)
    assert created.id == 1
    assert created.name == "First"

    # update
    obj2 = DummyModel(id=1, name="Updated", value=999)
    updated = crud.create_or_update(session, obj2)
    assert updated.id == 1
    assert updated.name == "Updated"
    assert updated.value == 999
    assert updated.updated_at is not None
