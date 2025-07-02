from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.models.carrier_status import CarrierStatus

# 創建 CarrierStatus CRUD 實例
carrier_status_crud = BaseCRUD(CarrierStatus, id_column="id")
