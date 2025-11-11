# database/__init__.py
"""
資料庫操作模組

將原本的 db.py 拆分為多個模組：
- connection.py: 資料庫連線管理
- client_ops.py: 客戶端相關操作
- product_ops.py: 產品相關操作
- rack_ops.py: 貨架相關操作
- carrier_ops.py: 載具相關操作
- equipment_ops.py: 設備相關操作
- log_ops.py: 日誌相關操作
- user_ops.py: 用戶相關操作
- task_ops.py: 任務相關操作
- utils.py: 通用工具函數
"""

# 從各個模組導入所有函數，保持向後兼容性
from .connection import connection_pool
from .client_ops import *
from .product_ops import *
from .rack_ops import *
# from .carrier_ops import *  # 🔴 已移除 Carrier 表，改用 rack.carrier_bitmap
from .equipment_ops import *
from .log_ops import *
from .audit_log_ops import *
from .user_ops import *
from .task_ops import *
from .agv_ops import *
from .other_ops import *
from .utils import *

# 保持原有的導出接口
__all__ = [
    # 連線管理
    'connection_pool',

    # 通用工具
    'fetch_all',

    # 客戶端操作
    'client_all', 'get_clients', 'count_clients', 'get_client_by_id',
    'update_client', 'reset_client_op_settings', 'delete_client',

    # 產品操作
    'product_all', 'get_products', 'count_products', 'get_product_by_id',
    'create_product', 'update_product', 'delete_product', 'get_all_products',
    'get_all_process_settings',

    # 貨架操作
    'rack_all', 'get_racks', 'count_racks', 'get_rack_by_id',
    'create_rack', 'update_rack', 'delete_rack', 'get_all_racks',
    'get_all_rack_statuses', 'get_rack_grid_info',  # 🔴 移除 count_carriers_by_rack

    # 🔴 載具操作已移除，改用 rack.carrier_bitmap
    # 'carrier_all', 'get_carriers', 'count_carriers', 'get_carriers_grouped',
    # 'get_carrier_status_list', 'get_carrier_by_id', 'update_carrier',
    # 'delete_carrier', 'create_carrier',

    # 設備操作
    'get_eqps', 'count_eqps', 'get_signals', 'count_signals',
    'get_signals_by_eqp_id', 'count_signals_by_eqp_id', 'get_eqps_with_signal_counts',
    'get_eqp_by_id', 'create_eqp', 'update_eqp', 'delete_eqp',
    'get_eqp_ports_by_eqp_id', 'create_eqp_port', 'delete_eqp_port',
    'check_port_has_signals', 'update_eqp_port', 'get_complete_device',
    'create_complete_device', 'update_complete_device', 'delete_complete_device',
    'get_all_available_ports', 'get_available_ports_for_eqp',
    'get_ports_not_assigned_to_any_eqp', 'get_all_eqp_ports',

    # 日誌操作
    'get_rosout_logs', 'count_rosout_logs', 'get_rosout_node_names',
    'get_runtime_logs', 'count_runtime_logs', 'get_runtime_node_names',

    # 審計日誌操作
    'create_audit_log', 'get_audit_logs', 'count_audit_logs',
    'get_user_audit_logs', 'get_resource_audit_logs', 'get_session_audit_logs',
    'get_operation_type_audit_logs',

    # 用戶操作
    'get_user_by_username', 'get_user_by_email', 'create_user',
    'update_user_last_login', 'get_users', 'count_users',

    # 任務操作
    'task_all', 'get_tasks', 'count_tasks', 'get_task_by_id', 'create_task',
    'update_task', 'delete_task', 'work_all', 'task_status_all',

    # Work 操作
    'get_works', 'count_works', 'get_work_by_id', 'create_work',
    'update_work', 'delete_work',

    # AGV 操作
    'get_agvs', 'count_agvs', 'get_agv_by_id', 'update_agv', 'delete_agv', 'create_agv',

    # 其他操作
    'machine_all', 'room_all', 'signal_all', 'node_all', 'edge_all',
    'kuka_node_all', 'kuka_edge_all', 'node_type_all', 'agv_all',
    'modify_log_all', 'modify_log_all_objects', 'get_all_machines',
    'get_all_rooms', 'get_all_agvs', 'get_all_locations',
    'agv_status_all', 'get_all_agv_statuses',
]
