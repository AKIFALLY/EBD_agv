"""
CT 任務分配器單元測試

測試 CtTaskAllocator 類的各項功能

運行方式:
    cd /app/rcs_ws
    python3 -m pytest src/rcs/test/test_ct_task_allocator.py -v

作者: AI Assistant
創建日期: 2025-10-21
"""

import pytest
import os
import yaml
import tempfile
from unittest.mock import Mock, MagicMock
from rcs.ct_task_allocator import CtTaskAllocator


class TestCtTaskAllocator:
    """測試 CtTaskAllocator 類"""

    @pytest.fixture
    def mock_logger(self):
        """創建 mock logger"""
        logger = Mock()
        return logger

    @pytest.fixture
    def sample_config_content(self):
        """示例配置內容"""
        return {
            'version': '1.0',
            'enabled': True,
            'agv_capabilities': {
                'cargo02': {
                    'model': 'Cargo',
                    'rooms': [1, 2],
                    'max_concurrent_tasks': 1,
                    'enabled': True
                },
                'loader02': {
                    'model': 'Loader',
                    'rooms': [1, 2],
                    'max_concurrent_tasks': 1,
                    'enabled': True
                },
                'unloader02': {
                    'model': 'Unloader',
                    'rooms': [1, 2],
                    'max_concurrent_tasks': 1,
                    'enabled': False  # 測試禁用狀態
                }
            },
            'work_id_allocations': {
                2000102: {
                    'agv_name': 'cargo02',
                    'priority_override': None,
                    'room_override': None
                },
                2010101: {
                    'agv_name': 'loader02',
                    'priority_override': 5,
                    'room_override': None
                },
                2051101: {
                    'agv_name': 'unloader02',
                    'priority_override': 3,
                    'room_override': None
                }
            },
            'default_allocation': {
                'enabled': False,
                'fallback_agv': None,
                'log_unmapped': True
            },
            'reload_config': {
                'enabled': True,
                'check_interval': 10.0,
                'log_on_reload': True
            },
            'debug': {
                'verbose_logging': False,
                'log_allocation_decisions': True,
                'log_skipped_tasks': True
            }
        }

    @pytest.fixture
    def temp_config_file(self, sample_config_content):
        """創建臨時配置文件"""
        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False
        ) as f:
            yaml.dump(sample_config_content, f)
            temp_path = f.name

        yield temp_path

        # 清理臨時文件
        if os.path.exists(temp_path):
            os.remove(temp_path)

    @pytest.fixture
    def allocator(self, temp_config_file, mock_logger):
        """創建 CtTaskAllocator 實例"""
        return CtTaskAllocator(temp_config_file, mock_logger)

    def test_initialization(self, allocator):
        """測試初始化"""
        assert allocator.config is not None
        assert allocator.config.get('version') == '1.0'
        assert allocator.config.get('enabled') is True

    def test_config_validation(self, allocator):
        """測試配置驗證"""
        assert allocator._validate_config() is True

    def test_load_config(self, allocator, mock_logger):
        """測試配置加載"""
        success = allocator.load_config()
        assert success is True
        mock_logger.info.assert_called()

    def test_allocate_task_cargo(self, allocator):
        """測試 Cargo AGV 任務分配"""
        # 創建 mock task
        task = Mock()
        task.work_id = 2000102
        task.room_id = 1
        task.priority = 4

        # 創建 mock available_agvs
        agv1 = Mock()
        agv1.name = 'cargo02'
        agv1.model = 'Cargo'

        available_agvs = [agv1]

        # 執行分配
        agv_name, priority = allocator.allocate_task(task, available_agvs)

        assert agv_name == 'cargo02'
        assert priority is None  # 沒有優先級覆蓋

    def test_allocate_task_loader_with_priority_override(self, allocator):
        """測試 Loader AGV 任務分配（帶優先級覆蓋）"""
        task = Mock()
        task.work_id = 2010101
        task.room_id = 1
        task.priority = 4

        agv1 = Mock()
        agv1.name = 'loader02'
        agv1.model = 'Loader'

        available_agvs = [agv1]

        agv_name, priority = allocator.allocate_task(task, available_agvs)

        assert agv_name == 'loader02'
        assert priority == 5  # 優先級覆蓋為 5

    def test_allocate_task_agv_not_available(self, allocator):
        """測試 AGV 不可用時返回 None"""
        task = Mock()
        task.work_id = 2000102
        task.room_id = 1

        # available_agvs 列表為空
        available_agvs = []

        agv_name, priority = allocator.allocate_task(task, available_agvs)

        assert agv_name is None
        assert priority is None

    def test_allocate_task_agv_disabled(self, allocator):
        """測試 AGV 被禁用時返回 None"""
        task = Mock()
        task.work_id = 2051101  # unloader02 (已禁用)
        task.room_id = 1

        agv1 = Mock()
        agv1.name = 'unloader02'
        agv1.model = 'Unloader'

        available_agvs = [agv1]

        agv_name, priority = allocator.allocate_task(task, available_agvs)

        assert agv_name is None  # 因為 unloader02 在配置中被禁用

    def test_allocate_task_room_restriction(self, allocator):
        """測試房間限制"""
        task = Mock()
        task.work_id = 2000102
        task.room_id = 3  # 房間 3 不在允許列表中

        agv1 = Mock()
        agv1.name = 'cargo02'
        agv1.model = 'Cargo'

        available_agvs = [agv1]

        agv_name, priority = allocator.allocate_task(task, available_agvs)

        assert agv_name is None  # 房間限制導致分配失敗

    def test_allocate_task_unmapped_work_id(self, allocator, mock_logger):
        """測試未映射的 work_id"""
        task = Mock()
        task.work_id = 9999999  # 未配置的 work_id
        task.room_id = 1
        task.id = 123

        agv1 = Mock()
        agv1.name = 'cargo02'

        available_agvs = [agv1]

        agv_name, priority = allocator.allocate_task(task, available_agvs)

        assert agv_name is None
        # 應該記錄警告
        mock_logger.warning.assert_called()

    def test_get_agv_capability(self, allocator):
        """測試獲取 AGV 能力配置"""
        cap = allocator.get_agv_capability('cargo02')

        assert cap is not None
        assert cap['model'] == 'Cargo'
        assert cap['rooms'] == [1, 2]
        assert cap['enabled'] is True

    def test_get_all_configured_agvs(self, allocator):
        """測試獲取所有配置的 AGV"""
        agvs = allocator.get_all_configured_agvs()

        assert len(agvs) == 3
        assert 'cargo02' in agvs
        assert 'loader02' in agvs
        assert 'unloader02' in agvs

    def test_get_work_id_allocation(self, allocator):
        """測試獲取 work_id 分配配置"""
        alloc = allocator.get_work_id_allocation(2000102)

        assert alloc is not None
        assert alloc['agv_name'] == 'cargo02'

    def test_get_config_stats(self, allocator):
        """測試獲取配置統計"""
        stats = allocator.get_config_stats()

        assert stats['version'] == '1.0'
        assert stats['enabled'] is True
        assert stats['total_work_id_mappings'] == 3
        assert stats['total_agvs_configured'] == 3
        assert stats['hot_reload_enabled'] is True

    def test_hot_reload(self, allocator, temp_config_file, sample_config_content):
        """測試熱重載功能"""
        import time

        # 初始配置
        assert allocator.config['version'] == '1.0'

        # 等待1秒確保文件時間戳不同
        time.sleep(1)

        # 修改配置文件
        sample_config_content['version'] = '2.0'
        with open(temp_config_file, 'w') as f:
            yaml.dump(sample_config_content, f)

        # 觸發重載
        reloaded = allocator.check_and_reload()

        assert reloaded is True
        assert allocator.config['version'] == '2.0'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
