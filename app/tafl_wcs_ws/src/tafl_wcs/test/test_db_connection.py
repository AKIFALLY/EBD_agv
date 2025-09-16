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
    result = db_bridge.query_locations(limit=5)
    assert result is not None
    
    # 驗證返回格式包含 metadata
    assert isinstance(result, dict)
    assert 'data' in result
    assert 'total' in result
    assert 'limit' in result
    assert 'offset' in result
    assert 'has_more' in result
    
    # 取得資料陣列
    locations = result['data']
    assert isinstance(locations, list)
    assert len(locations) <= 5
    
    # 如果有資料，檢查結構
    if locations:
        first = locations[0]
        assert 'id' in first
        assert 'name' in first
        # room_id 可能不一定存在所有記錄中

def test_query_racks(db_bridge):
    """測試查詢料架"""
    result = db_bridge.query_racks(limit=5)
    assert result is not None
    
    # 驗證返回格式包含 metadata
    assert isinstance(result, dict)
    assert 'data' in result
    assert 'total' in result
    assert 'limit' in result
    assert 'offset' in result
    assert 'has_more' in result
    
    # 取得資料陣列
    racks = result['data']
    assert isinstance(racks, list)
    assert len(racks) <= 5
    
    # 如果有資料，檢查結構
    if racks:
        first = racks[0]
        assert 'id' in first
        # name 在 rack 中可能是 rack_name

def test_query_tasks(db_bridge):
    """測試查詢任務"""
    result = db_bridge.query_tasks(limit=5)
    assert result is not None
    
    # 驗證返回格式包含 metadata
    assert isinstance(result, dict)
    assert 'data' in result
    assert 'total' in result
    assert 'limit' in result
    assert 'offset' in result
    assert 'has_more' in result
    
    # 取得資料陣列
    tasks = result['data']
    assert isinstance(tasks, list)
    assert len(tasks) <= 5

def test_query_works(db_bridge):
    """測試查詢工作"""
    result = db_bridge.query_works(limit=5)
    assert result is not None
    
    # 驗證返回格式包含 metadata
    assert isinstance(result, dict)
    assert 'data' in result
    assert 'total' in result
    assert 'limit' in result
    assert 'offset' in result
    assert 'has_more' in result
    
    # 取得資料陣列
    works = result['data']
    assert isinstance(works, list)
    assert len(works) <= 5

if __name__ == '__main__':
    pytest.main([__file__, '-v'])