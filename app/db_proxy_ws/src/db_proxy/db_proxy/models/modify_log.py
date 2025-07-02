from typing import List
from sqlmodel import SQLModel, Field, Session, select
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from pydantic import ConfigDict


class ModifyLog(SQLModel, table=True):
    __tablename__ = "modify_log"
    table_name: str = Field(primary_key=True)
    modified_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型

    @classmethod
    def mark(cls, session: Session, table_name: str):
        existing = session.get(cls, table_name)
        if not existing:
            session.add(cls(table_name=table_name))
        else:
            existing.modified_at = datetime.now(timezone.utc)
        session.commit()

    @classmethod
    def get_all(cls, session: Session) -> List["ModifyLog"]:
        """取得所有資料"""
        statement = select(cls)
        results = session.exec(statement).all()
        return results
