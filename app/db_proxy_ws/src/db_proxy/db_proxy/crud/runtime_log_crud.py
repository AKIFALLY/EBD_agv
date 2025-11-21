from db_proxy.models import RuntimeLog
from db_proxy.crud.base_crud import BaseCRUD

runtime_log_crud = BaseCRUD(RuntimeLog, id_column="id")
