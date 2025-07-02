from db_proxy.models.agvc_location import Location
from db_proxy.crud.base_crud import BaseCRUD

location_crud = BaseCRUD(Location, id_column="id")
