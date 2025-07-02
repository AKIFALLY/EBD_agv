from db_proxy.models import Node
from db_proxy.crud.base_crud import BaseCRUD

rosout_log_crud = BaseCRUD(Node, id_column="id")
