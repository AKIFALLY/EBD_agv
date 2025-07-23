"""
TaskCondition 模型和 CRUD 操作的測試檔案
"""

import pytest
from sqlmodel import Session, create_engine, SQLModel
from db_proxy.models import TaskCondition
from db_proxy.crud.task_condition_crud import task_condition_crud


@pytest.fixture
def test_session():
    """建立測試用的資料庫會話"""
    # 使用記憶體資料庫進行測試
    engine = create_engine("sqlite:///:memory:")
    SQLModel.metadata.create_all(engine)
    
    with Session(engine) as session:
        yield session


def test_create_task_condition(test_session):
    """測試建立任務條件"""
    conditions = "AGV_STATUS = 'IDLE' AND LOCATION = 'STATION_A'"
    results = {
        "status": "ready",
        "timestamp": "2024-01-01T00:00:00Z",
        "description": "AGV 在 A 站待機狀態"
    }
    
    # 建立任務條件
    task_condition = task_condition_crud.create_condition(
        test_session, conditions, results, "測試條件描述"
    )
    
    assert task_condition.id is not None
    assert task_condition.conditions == conditions
    assert task_condition.results == results


def test_get_by_conditions(test_session):
    """測試根據條件內容查詢"""
    conditions = "TASK_TYPE = 'TRANSPORT' AND PRIORITY > 5"
    results = {"status": "high_priority"}
    
    # 建立任務條件
    created_condition = task_condition_crud.create_condition(
        test_session, conditions, results, "高優先級任務條件"
    )
    
    # 查詢任務條件
    found_condition = task_condition_crud.get_by_conditions(test_session, conditions)
    
    assert found_condition is not None
    assert found_condition.id == created_condition.id
    assert found_condition.conditions == conditions


def test_search_conditions(test_session):
    """測試搜尋條件"""
    # 建立多個任務條件
    conditions_list = [
        "AGV_STATUS = 'IDLE'",
        "AGV_STATUS = 'BUSY'",
        "TASK_TYPE = 'TRANSPORT'"
    ]
    
    for i, conditions in enumerate(conditions_list):
        task_condition_crud.create_condition(test_session, conditions, {}, f"測試條件 {i+1}")
    
    # 搜尋包含 'AGV_STATUS' 的條件
    results = task_condition_crud.search_conditions(test_session, "AGV_STATUS")
    
    assert len(results) == 2
    for result in results:
        assert "AGV_STATUS" in result.conditions


def test_update_results(test_session):
    """測試更新結果資料"""
    conditions = "CARRIER_STATUS = 'LOADED'"
    initial_results = {"status": "loading"}
    
    # 建立任務條件
    task_condition = task_condition_crud.create_condition(
        test_session, conditions, initial_results, "載具狀態條件"
    )
    
    # 更新結果資料
    new_results = {
        "status": "loaded",
        "weight": 150.5,
        "destination": "WAREHOUSE"
    }
    
    updated_condition = task_condition_crud.update_results(
        test_session, task_condition.id, new_results
    )
    
    assert updated_condition is not None
    assert updated_condition.results == new_results


def test_get_all_with_results(test_session):
    """測試取得所有有結果資料的條件"""
    # 建立有結果的條件
    task_condition_crud.create_condition(
        test_session,
        "CONDITION_1",
        {"status": "active"},
        "有結果的條件"
    )

    # 建立沒有結果的條件
    task_condition_crud.create_condition(
        test_session,
        "CONDITION_2",
        None,
        "沒有結果的條件"
    )
    
    # 查詢有結果的條件
    conditions_with_results = task_condition_crud.get_all_with_results(test_session)
    
    assert len(conditions_with_results) == 1
    assert conditions_with_results[0].conditions == "CONDITION_1"


def test_delete_by_conditions(test_session):
    """測試根據條件內容刪除"""
    conditions = "TEST_CONDITION"
    
    # 建立任務條件
    task_condition_crud.create_condition(test_session, conditions, {}, "測試刪除條件")
    
    # 確認條件存在
    found_condition = task_condition_crud.get_by_conditions(test_session, conditions)
    assert found_condition is not None
    
    # 刪除條件
    deleted = task_condition_crud.delete_by_conditions(test_session, conditions)
    assert deleted is True
    
    # 確認條件已被刪除
    found_condition = task_condition_crud.get_by_conditions(test_session, conditions)
    assert found_condition is None


if __name__ == "__main__":
    """直接執行測試"""
    print("🧪 開始執行 TaskCondition 測試...")
    
    # 這裡可以加入簡單的測試執行邏輯
    # 實際使用時建議使用 pytest 執行測試
    
    print("✅ 測試檔案載入成功！")
    print("💡 請使用 'pytest test_task_condition.py' 執行完整測試")
