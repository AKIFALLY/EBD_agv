# db.py - 重構後的資料庫操作模組
"""
資料庫操作模組 - 向後兼容性接口

原本的 db.py 已重構為多個模組：
- database/connection.py: 資料庫連線管理
- database/client_ops.py: 客戶端相關操作
- database/product_ops.py: 產品相關操作
- database/rack_ops.py: 貨架相關操作
- database/carrier_ops.py: 載具相關操作
- database/equipment_ops.py: 設備相關操作
- database/log_ops.py: 日誌相關操作
- database/user_ops.py: 用戶相關操作
- database/task_ops.py: 任務相關操作
- database/other_ops.py: 其他操作
- database/utils.py: 通用工具函數

此文件保持向後兼容性，從新的模組結構導入所有函數。
"""

# 從新的 database 模組導入所有功能，保持向後兼容性
from .database import *

# 為了完全向後兼容，也可以直接導入連線池
from .database.connection import connection_pool

# 所有原本的函數現在都從 database 模組導入
# 這個文件只作為向後兼容性的接口
