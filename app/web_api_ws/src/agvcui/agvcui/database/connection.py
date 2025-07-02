# database/connection.py
"""
資料庫連線管理模組
"""

from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import (
    Client, Product, Machine, Room, Rack, Task, TaskStatus, Carrier, CarrierStatus,
    User, EqpPort, Eqp, EqpSignal, RosoutLog, RuntimeLog, ModifyLog, Work, AGV,
    Node, ProcessSettings, Location, RackStatus
)
from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.crud.eqp_crud import eqp_signal_crud, eqp_crud, eqp_port_crud
from db_proxy.crud.carrier_crud import carrier_crud
from db_proxy.crud.carrier_status_crud import carrier_status_crud
from db_proxy.crud.room_crud import room_crud
from db_proxy.crud.rack_crud import rack_crud
from db_proxy.crud.agv_crud import agv_crud
from db_proxy.crud.node_crud import node_crud, edge_crud, kuka_node_crud, kuka_edge_crud, node_type_crud
from db_proxy.crud.location_crud import location_crud
from db_proxy.crud.user_crud import user_crud
from db_proxy.crud.process_settings_crud import process_settings_crud

# 資料庫連線 URL
db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc?client_encoding=utf8'

# 初始化 ConnectionPoolManager
connection_pool = ConnectionPoolManager(db_url_agvc)

# 初始化 CRUD 處理器
client_crud = BaseCRUD(Client, id_column="id")
product_crud = BaseCRUD(Product, id_column="id")
machine_crud = BaseCRUD(Machine, id_column="id")
room_crud = BaseCRUD(Room, id_column="id")
rack_crud = BaseCRUD(Rack, id_column="id")
task_crud = BaseCRUD(Task, id_column="id")
task_status_crud = BaseCRUD(TaskStatus, id_column="id")
work_crud = BaseCRUD(Work, id_column="id")
modify_log_crud = BaseCRUD(ModifyLog, id_column="table_name")
