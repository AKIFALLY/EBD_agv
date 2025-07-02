from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.models.room import Room

# 創建 Room CRUD 實例
room_crud = BaseCRUD(Room, id_column="id")
