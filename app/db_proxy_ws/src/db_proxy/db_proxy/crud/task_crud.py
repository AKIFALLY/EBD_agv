from db_proxy.models import Task
from db_proxy.crud.base_crud import BaseCRUD

task_crud = BaseCRUD(Task, id_column="id")
