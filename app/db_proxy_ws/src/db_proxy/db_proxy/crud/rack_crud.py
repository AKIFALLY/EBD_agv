from db_proxy.models import Rack
from db_proxy.crud.base_crud import BaseCRUD

rack_crud = BaseCRUD(Rack, id_column="id")
