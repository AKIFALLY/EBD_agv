from db_proxy.models import ProcessSettings
from db_proxy.crud.base_crud import BaseCRUD

process_settings_crud = BaseCRUD(ProcessSettings, id_column="id")
