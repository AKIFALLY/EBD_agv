from db_proxy.models import AGV,AGVContext
from db_proxy.crud.base_crud import BaseCRUD

agv_crud = BaseCRUD(AGV, id_column="id")
agv_context_crud = BaseCRUD(AGVContext, id_column="id")
