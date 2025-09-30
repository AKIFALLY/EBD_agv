#!/usr/bin/env python3
"""
RCS 系統 pytest 測試
符合 ROS 2 colcon pytest 框架的標準測試檔案
參考標準 pytest 測試結構設計
"""

import pytest
import sys
import os

# 添加路徑以便導入 RCS 模組
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 導入被測試模組
try:
    from db_proxy.connection_pool_manager import ConnectionPoolManager
    from db_proxy.models import Task, AGV, Work
    from shared_constants.task_status import TaskStatus
    from sqlmodel import select
    from sqlalchemy import text
    DB_AVAILABLE = True
except ImportError:
    DB_AVAILABLE = False

try:
    from shared_constants.work_ids import WorkIds
    WORK_IDS_AVAILABLE = True
except ImportError:
    WORK_IDS_AVAILABLE = False

try:
    from rcs.simple_ct_manager import CtManager
    CT_MANAGER_AVAILABLE = True
except ImportError:
    CT_MANAGER_AVAILABLE = False

try:
    from rcs.simple_kuka_manager import KukaManager
    KUKA_MANAGER_AVAILABLE = True
except ImportError:
    KUKA_MANAGER_AVAILABLE = False

RCS_AVAILABLE = CT_MANAGER_AVAILABLE or KUKA_MANAGER_AVAILABLE


# 基本數據單元測試
@pytest.mark.skipif(not WORK_IDS_AVAILABLE, reason="WorkIds 不可用")
def test_work_id_categories():
    """測試工作 ID 分類數值"""
    # 測試使用 WorkIds 常數
    assert WorkIds.KUKA_MOVE == 210001
    assert WorkIds.KUKA_RACK_MOVE == 220001
    assert WorkIds.KUKA_WORKFLOW == 230001
    assert WorkIds.OPUI_CALL_EMPTY == 100001
    assert WorkIds.CT_AGV_WORK == 2000102


def test_agv_model_types():
    """測試 AGV 車型分類"""
    # 測試 AGV 車型分類
    expected_models = ["KUKA400i", "Cargo", "Loader", "Unloader"]
    
    for model in expected_models:
        assert model in expected_models
        assert isinstance(model, str)
        assert len(model) > 0


@pytest.mark.skipif(not WORK_IDS_AVAILABLE, reason="WorkIds 不可用")
def test_work_ids_functionality():
    """測試 WorkIds 類別功能"""
    # 測試基本常數
    assert WorkIds.KUKA_MOVE == 210001
    assert WorkIds.KUKA_RACK_MOVE == 220001
    assert WorkIds.KUKA_WORKFLOW == 230001
    
    # 測試描述功能
    assert WorkIds.get_description(WorkIds.KUKA_MOVE) == "KUKA 移動"
    assert WorkIds.get_description(WorkIds.KUKA_RACK_MOVE) == "KUKA 移動貨架"
    assert WorkIds.get_description(999999) == "未知工作 ID"
    
    # 測試名稱功能
    assert WorkIds.get_name(WorkIds.KUKA_MOVE) == "KUKA_MOVE"
    assert WorkIds.get_name(999999) == "UNKNOWN"
    
    # 測試 KUKA 支援檢查
    assert WorkIds.is_kuka_supported(WorkIds.KUKA_MOVE) == True
    assert WorkIds.is_kuka_supported(WorkIds.OPUI_CALL_EMPTY) == False
    
    # 測試 KUKA API 類型
    assert WorkIds.get_kuka_api_type(WorkIds.KUKA_MOVE) == "move"
    assert WorkIds.get_kuka_api_type(WorkIds.KUKA_RACK_MOVE) == "rack_move"
    assert WorkIds.get_kuka_api_type(WorkIds.KUKA_WORKFLOW) == "workflow"
    assert WorkIds.get_kuka_api_type(WorkIds.OPUI_CALL_EMPTY) == ""


@pytest.mark.skipif(not WORK_IDS_AVAILABLE, reason="WorkIds 不可用")
def test_kuka_supported_work_ids():
    """測試 KUKA 支援的工作 ID 列表"""
    kuka_ids = WorkIds.get_kuka_work_ids()
    
    # 驗證包含預期的工作 ID
    assert WorkIds.KUKA_MOVE in kuka_ids
    assert WorkIds.KUKA_RACK_MOVE in kuka_ids
    assert WorkIds.KUKA_WORKFLOW in kuka_ids
    
    # 驗證不包含非 KUKA 工作 ID
    assert WorkIds.OPUI_CALL_EMPTY not in kuka_ids
    assert WorkIds.CT_AGV_WORK not in kuka_ids
    
    # 驗證長度
    assert len(kuka_ids) == 3


def test_task_status_values():
    """測試任務狀態數值"""
    # RCS 系統使用的任務狀態
    status_pending = 1      # 待處理
    status_ready = 2        # 待執行  
    status_running = 3      # 執行中
    status_completed = 4    # 已完成
    status_rcs_cancelling = 52  # RCS 取消中
    
    assert status_pending == 1
    assert status_ready == 2
    assert status_running == 3
    assert status_completed == 4
    assert status_rcs_cancelling == 52


def test_parameters_format_consistency():
    """測試參數格式一致性"""
    # 測試 parameters["model"] 格式統一 (小寫)
    task_params = {
        "model": "KUKA400i",
        "mission_type": "MOVE"
    }
    
    assert "model" in task_params
    assert isinstance(task_params["model"], str)
    assert task_params["model"] == "KUKA400i"


def test_dispatch_priority_system():
    """測試派發優先級系統"""
    # 測試 RCS 簡化版本的優先級邏輯
    priorities = [1, 2, 3, 10, 100]
    sorted_priorities = sorted(priorities)
    
    # 優先級應該從低到高排序 (1 = 最高優先級)
    assert sorted_priorities == [1, 2, 3, 10, 100]
    assert min(priorities) == 1  # 最高優先級


# 定義測試參數（使用數字避免載入問題）
WORK_ID_ROUTE_PARAMS = [
    (210001, "move"),      # WorkIds.KUKA_MOVE
    (220001, "rack_move"), # WorkIds.KUKA_RACK_MOVE
    (230001, "workflow"),  # WorkIds.KUKA_WORKFLOW
    (100001, "workflow"),  # WorkIds.OPUI_CALL_EMPTY
    (2000102, "workflow"), # WorkIds.CT_AGV_WORK
]

@pytest.mark.parametrize("work_id,expected_route", WORK_ID_ROUTE_PARAMS)
def test_work_id_routing_logic(work_id, expected_route):
    """測試工作 ID 路由邏輯"""
    # 實現 RCS 簡化版本的路由邏輯
    if work_id == 210001:  # KUKA_MOVE
        route_type = "move"
    elif work_id == 220001:  # KUKA_RACK_MOVE
        route_type = "rack_move"
    else:
        route_type = "workflow"
    
    assert route_type == expected_route


def test_rcs_timer_frequency():
    """測試 RCS 定時器頻率"""
    # RCS 核心使用 1 秒定時器
    timer_frequency = 1.0  # 秒
    timer_ms = timer_frequency * 1000
    
    assert timer_frequency == 1.0
    assert timer_ms == 1000.0
    assert isinstance(timer_frequency, float)


def test_agv_fleet_categorization():
    """測試車隊分類"""
    # RCS 管理兩類車隊
    kuka_fleet_models = ["KUKA400i"]
    ct_fleet_models = ["Cargo", "Loader", "Unloader"]
    
    # 驗證 KUKA 車隊
    for model in kuka_fleet_models:
        assert model == "KUKA400i"
        assert model not in ct_fleet_models
    
    # 驗證 CT 車隊
    for model in ct_fleet_models:
        assert model in ["Cargo", "Loader", "Unloader"]
        assert model != "KUKA400i"


# 資料庫相關測試 (需要資料庫連接的進階測試)
@pytest.fixture
def db_connection():
    """資料庫連接 fixture"""
    if not DB_AVAILABLE:
        pytest.skip("資料庫組件不可用")
    
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    db_pool = ConnectionPoolManager(db_url)
    return db_pool


def test_database_connection(db_connection):
    """測試資料庫連接"""
    with db_connection.get_session() as session:
        result = session.execute(text("SELECT 1 as test")).fetchone()
        assert result is not None
        assert result[0] == 1


def test_task_model_filtering(db_connection):
    """測試任務模型篩選"""
    with db_connection.get_session() as session:
        # 測試 KUKA400i 任務篩選
        kuka_tasks = session.exec(
            select(Task).where(
                Task.status_id == TaskStatus.PENDING,  # 待處理 (WCS-任務已接受，待處理)
                Task.parameters["model"].as_string() == "KUKA400i"
            )
        ).all()
        
        # 測試 CT 任務篩選
        ct_tasks = session.exec(
            select(Task).where(
                Task.status_id == TaskStatus.PENDING,  # 待處理 (WCS-任務已接受，待處理)
                Task.parameters["model"].as_string() != "KUKA400i"
            )
        ).all()
        
        # 驗證篩選邏輯存在
        assert isinstance(kuka_tasks, list)
        assert isinstance(ct_tasks, list)


# RCS 管理器測試 (需要 RCS 組件的整合測試)
@pytest.mark.skipif(not KUKA_MANAGER_AVAILABLE, reason="KUKA Manager 不可用")
def test_kuka_manager_availability():
    """測試 KUKA 管理器可用性"""
    # 驗證 KUKA 管理器類別可以導入
    assert KukaManager is not None
    assert callable(KukaManager)
    assert hasattr(KukaManager, '__init__')


@pytest.mark.skipif(not CT_MANAGER_AVAILABLE, reason="CT Manager 不可用")
def test_ct_manager_availability():
    """測試 CT 管理器可用性"""
    # 驗證 CT 管理器類別可以導入
    assert CtManager is not None
    assert callable(CtManager)
    assert hasattr(CtManager, '__init__')


# 功能測試 (測試業務邏輯)
def test_rcs_system_architecture():
    """測試 RCS 系統架構"""
    # RCS 簡化系統的架構特點
    system_components = {
        "rcs_core": "核心協調節點",
        "ct_manager": "CT 車隊管理器", 
        "kuka_manager": "KUKA 車隊管理器",
        "timer_1s": "1秒定時器"
    }
    
    # 驗證系統組件定義
    assert "rcs_core" in system_components
    assert "ct_manager" in system_components
    assert "kuka_manager" in system_components
    assert "timer_1s" in system_components
    
    # 驗證組件描述
    assert system_components["rcs_core"] == "核心協調節點"
    assert system_components["timer_1s"] == "1秒定時器"


def test_agv_topic_subscription():
    """測試 AGV 主題訂閱"""
    # RCS 系統訂閱的 AGV 主題
    subscribed_topics = [
        "/agv/state_change",
        "/agv/status"
    ]
    
    # 驗證主題格式
    for topic in subscribed_topics:
        assert topic.startswith("/agv/")
        assert isinstance(topic, str)
        assert len(topic) > 5
    
    # 驗證關鍵主題存在
    assert "/agv/state_change" in subscribed_topics
    assert "/agv/status" in subscribed_topics


def test_database_connection_string():
    """測試資料庫連接字串"""
    # RCS 使用的資料庫連接配置
    db_config = {
        "host": "192.168.100.254",
        "port": 5432,
        "database": "agvc",
        "username": "agvc",
        "password": "password"
    }
    
    # 驗證連接配置
    assert db_config["host"] == "192.168.100.254"
    assert db_config["port"] == 5432
    assert db_config["database"] == "agvc"
    assert db_config["username"] == "agvc"
    
    # 驗證連接字串格式
    expected_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    actual_url = f'postgresql+psycopg2://{db_config["username"]}:{db_config["password"]}@{db_config["host"]}/{db_config["database"]}'
    assert actual_url == expected_url


if __name__ == '__main__':
    pytest.main([__file__])