from db_proxy.models import TrafficZone
from db_proxy.crud.base_crud import BaseCRUD

traffic_zone_curd = BaseCRUD(TrafficZone, id_column="id")
