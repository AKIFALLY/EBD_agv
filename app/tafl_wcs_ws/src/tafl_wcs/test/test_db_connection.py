#!/usr/bin/env python3
"""
資料庫連接測試
測試 TAFL WCS 的資料庫連接功能
"""

import pytest
from tafl_wcs.tafl_db_bridge import TAFLDatabaseBridge

DATABASE_URL = "postgresql://agvc:password@192.168.100.254:5432/agvc"

@pytest.fixture
def db_bridge():
    """資料庫橋接 fixture"""
    return TAFLDatabaseBridge(DATABASE_URL)

def test_database_connection(db_bridge):
    """測試資料庫連接"""
    # 測試連接是否正常
    assert db_bridge is not None
    assert db_bridge.pool_manager is not None

def test_query_locations(db_bridge):
    """測試查詢位置"""
    locations = db_bridge.query_locations(limit=5)
    assert locations is not None
    assert isinstance(locations, list)
    assert len(locations) <= 5
    
    # 如果有資料，檢查結構
    if locations:
        first = locations[0]
        assert 'id' in first
        assert 'name' in first
        assert 'room_id' in first

def test_query_racks(db_bridge):
    """測試查詢料架"""
    racks = db_bridge.query_racks(limit=5)
    assert racks is not None
    assert isinstance(racks, list)
    assert len(racks) <= 5
    
    # 如果有資料，檢查結構
    if racks:
        first = racks[0]
        assert 'id' in first
        assert 'name' in first

def test_query_tasks(db_bridge):
    """測試查詢任務"""
    tasks = db_bridge.query_tasks(limit=5)
    assert tasks is not None
    assert isinstance(tasks, list)
    assert len(tasks) <= 5

def test_query_works(db_bridge):
    """測試查詢工作"""
    works = db_bridge.query_works(limit=5)
    assert works is not None
    assert isinstance(works, list)
    assert len(works) <= 5

if __name__ == '__main__':
    pytest.main([__file__, '-v'])