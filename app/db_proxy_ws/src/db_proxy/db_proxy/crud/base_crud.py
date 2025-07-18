# crud/base_crud.py
from datetime import datetime, timezone
from typing import Any, Type, Optional, List
from sqlmodel import SQLModel, Session, select
from typing import get_args


class BaseCRUD:
    def __init__(self, model: Type[SQLModel], id_column: str = "id"):
        """
        通用 CRUD 類別

        Args:
            model: SQLModel 資料模型類別
            id_column: 代表主鍵的欄位名稱，預設 "id"
        """
        self.model = model
        self.id_column = id_column
        # 取得 id 欄位型別，例如 int 或 str
        annotation = model.model_fields[id_column].annotation
        args = get_args(annotation)
        # e.g., Optional[str] -> str
        self.id_type = args[0] if args else annotation

    def get_by_id(self, session: Session, entry_id) -> Optional[SQLModel]:
        statement = select(self.model).where(
            getattr(self.model, self.id_column) == entry_id)
        return session.exec(statement).first()

    def get_by_field(self, session: Session, field: str, value: Any) -> Optional[SQLModel]:
        if not hasattr(self.model, field):
            raise ValueError(f"模型 {self.model.__name__} 沒有欄位 {field}")

        statement = select(self.model).where(getattr(self.model, field) == value)
        return session.exec(statement).first()

    def get_all(self, session: Session) -> List[SQLModel]:
        """取得所有資料"""
        statement = select(self.model)
        results = session.exec(statement).all()
        return results

    def create(self, session: Session, obj: SQLModel) -> SQLModel:
        """新增一筆資料，支援 SQLModel 實例"""
        if not getattr(obj, "created_at", None):
            if hasattr(obj, 'created_at'):
                setattr(obj, "created_at", datetime.now(timezone.utc))
        if hasattr(obj, 'updated_at'):
            setattr(obj, "updated_at", datetime.now(timezone.utc))

        session.add(obj)
        try:
            session.commit()
        except Exception as e:
            session.rollback()
            print('===create error===')
            print(e)
            raise e
        session.refresh(obj)
        return obj

    def update(self, session: Session, entry_id, obj: SQLModel) -> Optional[SQLModel]:
        """更新指定 ID 的資料，使用 SQLAlchemy 的 merge 方法"""
        try:
            entry_id = self.id_type(entry_id)
        except (ValueError, TypeError):
            raise ValueError(f"無法將 id 轉為 {self.id_type}: {entry_id}")

        existing = self.get_by_id(session, entry_id)
        if not existing:
            raise ValueError(
                f"{self.model.__name__} with id={entry_id} not found")

        # 設定 ID 和 created_at
        setattr(obj, self.id_column, entry_id)
        if hasattr(existing, "created_at") and hasattr(obj, "created_at"):
            setattr(obj, "created_at", getattr(existing, "created_at"))

        # 確保 updated_at 欄位被更新
        if hasattr(obj, "updated_at"):
            setattr(obj, "updated_at", datetime.now(timezone.utc))

        # 獲取 obj 中實際設定的欄位（只包含明確設定的欄位）
        obj_data = obj.model_dump(exclude_unset=True)

        # 將 existing 中的欄位值複製到 obj 中，但只複製 obj 中不存在的欄位
        for key, value in existing.model_dump().items():
            if key not in obj_data and key != self.id_column:
                setattr(obj, key, value)

        # 使用 merge 方法更新資料
        result = session.merge(obj)

        try:
            session.commit()
        except Exception as e:
            session.rollback()
            print('===update error===')
            print(e)
            raise e
        session.refresh(result)
        return result

    def delete(self, session: Session, entry_id) -> bool:
        """刪除指定 ID 的資料"""
        try:
            entry_id = self.id_type(entry_id)
        except (ValueError, TypeError):
            raise ValueError(f"無法將 id 轉為 {self.id_type}: {entry_id}")

        obj = self.get_by_id(session, entry_id)
        if not obj:
            return False
        session.delete(obj)
        session.commit()
        return True

    def create_or_update(self, session: Session, obj: SQLModel) -> SQLModel:
        id_value = getattr(obj, self.id_column, None)

        if id_value is not None:
            print(f"id_value:{id_value}")
            try:
                entry_id = self.id_type(id_value)
            except (ValueError, TypeError):
                raise ValueError(f"無法將 id 轉為 {self.id_type}: {id_value}")

            existing = self.get_by_id(session, entry_id)
            if existing:
                return self.update(session, entry_id, obj)

        return self.create(session, obj)
