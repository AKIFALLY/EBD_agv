# 假設 Product 模型有一個名為 'product_id' 的主鍵
from sqlmodel import Session
from db_proxy.models import Product
from db_proxy.crud.base_crud import BaseCRUD

# 初始化 CRUD 物件，並指定 id_column 為 'product_id'
product_crud = BaseCRUD(Product, id_column="id")

# 創建新的產品
# new_product = product_crud.create(session, {"id": 1, "name": "Product1", "size": 5, "process_code": 1})

# 根據 'product_id' 查詢產品
# product = product_crud.get_by_id(session, 1)

# 更新產品資訊
# updated_product = product_crud.update(session, 1, {"name": "Updated Product", "size": 6})

# 刪除產品
# is_deleted = product_crud.delete(session, 1)
