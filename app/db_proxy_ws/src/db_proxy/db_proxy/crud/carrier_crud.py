from db_proxy.models import Carrier
from db_proxy.crud.base_crud import BaseCRUD

carrier_crud = BaseCRUD(Carrier, id_column="id")
