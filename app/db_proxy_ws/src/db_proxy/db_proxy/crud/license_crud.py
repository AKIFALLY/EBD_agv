from db_proxy.models import License
from db_proxy.crud.base_crud import BaseCRUD
from sqlmodel import Session, select
from typing import Optional


class LicenseCRUD(BaseCRUD):
    def get_by_device_id(self, session: Session, device_id: str) -> Optional[License]:
        """根據 device_id 查詢 License"""
        statement = select(License).where(License.device_id == device_id)
        return session.exec(statement).first()


license_crud = LicenseCRUD(License, id_column="id")
