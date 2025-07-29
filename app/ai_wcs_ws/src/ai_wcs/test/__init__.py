"""
AI WCS 測試套件
提供單元測試、整合測試、功能測試的完整測試框架
"""

# 測試工具導入
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# 添加模組路徑
test_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(test_dir, '..')
sys.path.insert(0, src_dir)

# 測試基礎設定
TEST_CONFIG = {
    'mock_database': True,
    'enable_logging': False,
    'test_timeout': 30
}